# Book RAG Chatbot System

A complete Retrieval-Augmented Generation (RAG) chatbot system for your online book website with contextual text selection feature.

## Features

- **RAG Chatbot**: Answers questions based on your book's content using vector search
- **Contextual Selection**: Answers questions based *only* on selected text
- **Glassmorphism UI**: Modern, frosted glass design for the chat widget
- **Full Stack**: FastAPI backend with React frontend

## Tech Stack

- **Frontend**: React/Vite
- **Backend**: FastAPI (Python)
- **LLM**: Google Gemini API
- **Embeddings**: Google Gemini API
- **Vector Database**: Qdrant Cloud
- **UI**: Glassmorphism CSS

## Setup Instructions

### 1. Environment Setup

1. Clone this repository
2. Create a `.env` file in the root directory with your API keys:

```env
# API Keys
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
GEMINI_API_KEY=your_gemini_api_key

# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
```

### 2. Backend Setup

1. Navigate to the backend directory:
```bash
cd rag-chatbot/backend
```

2. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Start the backend server:
```bash
python main.py
# Or use the start script:
./start.sh
```

The backend will run on `http://localhost:8000`

### 3. Data Ingestion

1. Prepare your book content (markdown, HTML, or other text files)

2. Run the ingestion script:
```bash
cd rag-chatbot/scripts
python ingest.py
```

Follow the prompts to enter the path to your book content directory.

### 4. Frontend Setup

1. Navigate to the frontend directory:
```bash
cd rag-chatbot/frontend
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm run dev
```

The frontend will run on `http://localhost:3000`

### 5. Usage

1. Make sure both backend and frontend are running
2. Open your browser to `http://localhost:3000`
3. Click the chat button (💬) on the left side of the screen to open the chat widget
4. To use contextual selection:
   - Select any text on the book page
   - The chat widget will show a notification that it's using selected text
   - Ask a question related to the selected text
5. For general questions about the book, simply ask without selecting text

## API Endpoints

- `GET /` - Health check
- `POST /chat` - Chat endpoint
  - Request body: `{"message": "your question", "selected_text_context": "optional selected text", "history": []}`
  - Response: `{"response": "answer", "sources": []}`

## Architecture

```
[Book Content Files]
        ↓
[Ingestion Script] → [Qdrant Vector DB]
        ↓
[FastAPI Backend] ←→ [React Frontend]
        ↓
[Google Gemini API]
[Qdrant Cloud]
```

## Note on Embeddings

The system now uses Google Gemini API for both document embeddings (during ingestion) and query embeddings (during retrieval), which helps avoid rate limits that were encountered with the Cohere API.

## Customization

- Modify `rag-chatbot/scripts/ingest.py` to customize how book content is processed
- Modify `rag-chatbot/backend/main.py` to customize RAG logic
- Modify `rag-chatbot/frontend/src/ChatWidget.jsx` and `ChatWidget.css` to customize the UI

## Troubleshooting

- If the chat widget doesn't connect to the backend, ensure the backend is running on port 8000
- Check browser console for frontend errors
- Check backend logs for API errors
- Ensure all API keys in `.env` are valid and have proper permissions