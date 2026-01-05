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
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str

class RAGChatbot:
    def __init__(self):
        self.collection_name = "book_content"
        self.embedding_model = "models/text-embedding-004"
        self.gemini_model = genai.GenerativeModel('gemini-flash-latest')

    def get_query_embedding(self, query: str) -> list:
        """Generate embedding for query using Google Gemini"""
        result = genai.embed_content(
            model=self.embedding_model,
            content=query,
            task_type="retrieval_query"
        )
        return result['embedding']

    def retrieve_context(self, query: str, limit: int = 3) -> list:
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
        You are a helpful AI assistant for a robotics book. Answer using this context: {context}. Question: {user_query}.
        """

        response = self.gemini_model.generate_content(prompt)
        return response.text

    def generate_response_selected_text(self, user_query: str, selected_text: str) -> str:
        """Generate response using only the selected text"""
        prompt = f"""
        You are a helpful AI assistant for a robotics book. Answer using this context: {selected_text}. Question: {user_query}.
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
    Main chat endpoint with the following flow:
    1. Input: Accept JSON { "message": "user question", "selected_text": "optional string" }
    2. Context Strategy:
       - Case A (User Selected Text): If `selected_text` is not empty, use it as the ONLY context. Skip Qdrant search.
       - Case B (General Question): If `selected_text` is empty:
         - Generate embedding for the user's question using Gemini
         - Search Qdrant collection `book_content` (Limit: 3 results)
         - Concatenate result texts to form context
    3. Generation:
       - Construct a prompt with context and question
       - Use Gemini to generate the response
       - Return { "response": "..." }
    """
    try:
        logger.info(f"Received chat request: {request.message[:50]}...")

        # Context Strategy
        if request.selected_text and request.selected_text.strip():
            # Case A: Use selected text as ONLY context
            context = request.selected_text
            response = chatbot.generate_response_selected_text(
                request.message,
                context
            )
        else:
            # Case B: General question - use Qdrant retrieval
            retrieved_contexts = chatbot.retrieve_context(request.message, limit=3)

            if not retrieved_contexts:
                response = "I couldn't find relevant information in the book to answer your question."
            else:
                # Concatenate result texts to form context
                context = " ".join([ctx['text'] for ctx in retrieved_contexts])
                response = chatbot.generate_response_with_context(
                    request.message,
                    context
                )

        return ChatResponse(response=response)

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