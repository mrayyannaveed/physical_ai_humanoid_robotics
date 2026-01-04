import os
import logging
import hashlib
import time
from typing import List, Dict, Any
from pathlib import Path
import yaml
from dotenv import load_dotenv

import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
import markdown
from bs4 import BeautifulSoup

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configure Google Generative AI
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

class BookContentIngestor:
    def __init__(self):
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

        # Configuration
        self.collection_name = "book_content"
        self.embedding_model = "models/text-embedding-004"
        self.chunk_size = 500
        self.chunk_overlap = 50

        # Initialize Qdrant collection
        self._init_qdrant_collection()

    def _init_qdrant_collection(self):
        """Initialize Qdrant collection with proper vector configuration"""
        try:
            # Check if collection exists
            self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except:
            # Create new collection
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Gemini embedding size for text-embedding-004
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection '{self.collection_name}'")

    def extract_text_from_markdown(self, file_path: str) -> str:
        """Extract text content from markdown file"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Convert markdown to HTML then extract text
        html = markdown.markdown(content)
        soup = BeautifulSoup(html, 'html.parser')
        return soup.get_text(separator=' ', strip=True)

    def extract_text_from_html(self, file_path: str) -> str:
        """Extract text content from HTML file"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        soup = BeautifulSoup(content, 'html.parser')
        return soup.get_text(separator=' ', strip=True)

    def chunk_text(self, text: str) -> List[str]:
        """Split text into overlapping chunks"""
        chunks = []
        start = 0

        while start < len(text):
            # Determine chunk end position
            end = start + self.chunk_size

            # If we're near the end, include the rest
            if end >= len(text):
                end = len(text)
            else:
                # Try to break at sentence boundary
                while end < len(text) and end - start < self.chunk_size + 50:
                    if text[end] in ['.', '!', '?', '\n'] and end > start + self.chunk_size:
                        end += 1
                        break
                    end += 1

            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)

            # Move start position with overlap
            start = end - self.chunk_overlap
            if start < end:  # Ensure we're making progress
                start = end

        return chunks

    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings using Google Gemini API"""
        embeddings = []
        for text in texts:
            # Add a small delay to respect rate limits (15 RPM for free tier)
            time.sleep(1)
            result = genai.embed_content(
                model=self.embedding_model,
                content=text,
                task_type="retrieval_document"
            )
            embeddings.append(result['embedding'])
        return embeddings

    def process_book_content(self, content_dir: str):
        """Process all book content files and store in Qdrant"""
        content_dir = Path(content_dir)
        all_chunks = []

        # Find all markdown and HTML files
        for file_path in content_dir.rglob('*'):
            if file_path.suffix.lower() in ['.md', '.html', '.htm']:
                logger.info(f"Processing file: {file_path}")

                # Extract text based on file type
                if file_path.suffix.lower() == '.md':
                    text = self.extract_text_from_markdown(str(file_path))
                else:
                    text = self.extract_text_from_html(str(file_path))

                # Extract chapter title from filename or directory structure
                chapter_title = file_path.stem
                if 'chapter' in str(file_path).lower():
                    # Try to extract chapter info from path
                    parts = str(file_path).split(os.sep)
                    for part in parts:
                        if 'chapter' in part.lower():
                            chapter_title = part
                            break

                # Chunk the text
                chunks = self.chunk_text(text)

                for i, chunk in enumerate(chunks):
                    chunk_data = {
                        'id': hashlib.md5(f"{file_path}_{i}".encode()).hexdigest(),
                        'text': chunk,
                        'source_file': str(file_path),
                        'chapter_title': chapter_title,
                        'chunk_index': i
                    }
                    all_chunks.append(chunk_data)

        logger.info(f"Processed {len(all_chunks)} chunks from book content")

        # Process in batches to avoid memory issues
        batch_size = 100
        for i in range(0, len(all_chunks), batch_size):
            batch = all_chunks[i:i+batch_size]
            self._store_batch(batch)

    def _store_batch(self, batch: List[Dict[str, Any]]):
        """Store a batch of chunks in Qdrant"""
        texts = [chunk['text'] for chunk in batch]

        # Generate embeddings
        embeddings = self.get_embeddings(texts)

        # Prepare points for Qdrant
        points = []
        for chunk, embedding in zip(batch, embeddings):
            points.append(
                models.PointStruct(
                    id=chunk['id'],
                    vector=embedding,
                    payload={
                        'text': chunk['text'],
                        'source_file': chunk['source_file'],
                        'chapter_title': chunk['chapter_title'],
                        'chunk_index': chunk['chunk_index']
                    }
                )
            )

        # Upload to Qdrant
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        logger.info(f"Stored batch of {len(batch)} chunks in Qdrant")

    def count_points(self) -> int:
        """Count total points in the collection"""
        try:
            count_result = self.qdrant_client.count(collection_name=self.collection_name)
            return count_result.count
        except Exception as e:
            logger.error(f"Error counting points: {e}")
            return 0

def main():
    # Initialize the ingestor
    ingestor = BookContentIngestor()

    # Process book content (adjust path as needed)
    content_dir = input("Enter the path to your book content directory (e.g., ./book): ").strip()
    if not content_dir:
        content_dir = "./book"  # Default path

    logger.info("Starting content ingestion process...")
    ingestor.process_book_content(content_dir)

    # Count final points
    total_points = ingestor.count_points()
    logger.info(f"Ingestion completed! Total points in collection: {total_points}")

if __name__ == "__main__":
    main()