import os
import logging
from typing import Optional, Dict, Any
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uvicorn

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Initialize FastAPI app
app = FastAPI(title="Book RAG Chatbot API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure this properly for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class ChatRequest(BaseModel):
    message: str
    selected_text_context: Optional[str] = None
    history: Optional[list] = []

class ChatResponse(BaseModel):
    response: str
    sources: Optional[list] = []

class RAGChatbot:
    def __init__(self):
        self.collection_name = "book_content"
        self.embedding_model = "models/text-embedding-004"
        self.gemini_model = genai.GenerativeModel('gemini-1.5-flash')

    def get_query_embedding(self, query: str) -> list:
        """Generate embedding for query using Google Gemini"""
        result = genai.embed_content(
            model=self.embedding_model,
            content=query,
            task_type="retrieval_query"
        )
        return result['embedding']

    def retrieve_context(self, query: str, limit: int = 5) -> list:
        """Retrieve relevant context from Qdrant"""
        query_embedding = self.get_query_embedding(query)

        search_result = qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True
        )

        return [
            {
                'text': hit.payload['text'],
                'source': hit.payload['source_file'],
                'chapter': hit.payload['chapter_title'],
                'score': hit.score
            }
            for hit in search_result
        ]

    def generate_response_with_context(self, user_query: str, context: str) -> str:
        """Generate response using Gemini with provided context"""
        prompt = f"""
        You are an expert assistant for a book about physical AI and humanoid robotics.
        Answer the user's question based on the provided context.

        Context: {context}

        User's question: {user_query}

        Provide a helpful and accurate answer based on the context. If the context doesn't contain relevant information, say so.
        """

        response = self.gemini_model.generate_content(prompt)
        return response.text

    def generate_response_selected_text(self, user_query: str, selected_text: str) -> str:
        """Generate response using only the selected text"""
        prompt = f"""
        You are an expert assistant for a book about physical AI and humanoid robotics.
        Answer the user's question based ONLY on the following selected text:

        Selected text: {selected_text}

        User's question: {user_query}

        Answer the question based SOLELY on the provided selected text. Do not use any external knowledge or make assumptions beyond what's in the selected text. If the selected text doesn't contain information to answer the question, say so.
        """

        response = self.gemini_model.generate_content(prompt)
        return response.text

# Initialize the chatbot instance
chatbot = RAGChatbot()

@app.get("/")
def read_root():
    return {"message": "Book RAG Chatbot API is running!"}

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Main chat endpoint that handles both RAG and selected text modes
    """
    try:
        logger.info(f"Received chat request: {request.message[:50]}...")

        # If selected text context is provided, use only that context
        if request.selected_text_context and request.selected_text_context.strip():
            logger.info("Using selected text context mode")
            response = chatbot.generate_response_selected_text(
                request.message,
                request.selected_text_context
            )
            return ChatResponse(response=response, sources=[])
        else:
            logger.info("Using standard RAG mode")
            # Retrieve relevant context from the book
            retrieved_contexts = chatbot.retrieve_context(request.message)

            if not retrieved_contexts:
                response = "I couldn't find relevant information in the book to answer your question."
                return ChatResponse(response=response, sources=[])

            # Combine the most relevant contexts
            combined_context = "\n\n".join([ctx['text'] for ctx in retrieved_contexts[:3]])

            # Generate response using the context
            response = chatbot.generate_response_with_context(
                request.message,
                combined_context
            )

            # Extract source information
            sources = [
                {
                    'text': ctx['text'][:200] + "..." if len(ctx['text']) > 200 else ctx['text'],
                    'source': ctx['source'],
                    'chapter': ctx['chapter'],
                    'relevance_score': ctx['score']
                }
                for ctx in retrieved_contexts[:3]
            ]

            return ChatResponse(response=response, sources=sources)

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=os.getenv("BACKEND_HOST", "0.0.0.0"),
        port=int(os.getenv("BACKEND_PORT", 8000)),
        reload=True
    )