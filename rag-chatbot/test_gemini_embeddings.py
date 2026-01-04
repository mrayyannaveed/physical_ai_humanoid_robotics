#!/usr/bin/env python3
"""
Test script to verify the updated ingestion script with Google Gemini embeddings
"""
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'scripts'))

from dotenv import load_dotenv
import google.generativeai as genai

# Load environment variables
load_dotenv(dotenv_path='../.env')

# Configure Google Generative AI
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

def test_gemini_embedding():
    """Test if Gemini embeddings work correctly"""
    try:
        test_text = "This is a test sentence for embedding."

        result = genai.embed_content(
            model="models/text-embedding-004",
            content=test_text,
            task_type="retrieval_document"
        )

        embedding = result['embedding']

        print("SUCCESS: Gemini embedding successful!")
        print(f"Embedding length: {len(embedding)}")
        print(f"First 5 values: {embedding[:5]}")

        return True
    except Exception as e:
        print(f"ERROR: Error with Gemini embedding: {str(e)}")
        return False

if __name__ == "__main__":
    print("Testing Gemini embedding functionality...")
    success = test_gemini_embedding()

    if success:
        print("\nSUCCESS: The ingestion script should work with Google Gemini embeddings!")
        print("You can now run: python scripts/ingest.py")
    else:
        print("\nERROR: There was an issue with Gemini embeddings.")
        print("Please check your GEMINI_API_KEY in the .env file.")