#!/bin/bash
# Test script to verify the RAG chatbot system

echo "Testing RAG Chatbot System..."
echo

# Check if backend is running
echo "Checking backend status..."
if curl -s http://localhost:8000/health; then
    echo "✓ Backend is running"
else
    echo "✗ Backend is not running. Please start the backend server."
    echo "Run: cd rag-chatbot/backend && python main.py"
fi

echo
echo "System verification complete!"
echo
echo "To fully test the system:"
echo "1. Make sure backend is running on http://localhost:8000"
echo "2. Make sure frontend is running on http://localhost:3000"
echo "3. Open browser to http://localhost:3000"
echo "4. Test the chat widget functionality"
echo "5. Test both RAG mode and selected text mode"