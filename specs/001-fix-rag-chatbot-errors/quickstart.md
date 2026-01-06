# Quickstart Guide for Fixing RAG Chatbot "Technical Difficulties" Error

## Prerequisites

- Python 3.11 or higher
- FastAPI installed
- Qdrant vector database running
- Docusaurus frontend setup

## Setup Instructions

1. **Start the FastAPI Backend**
   ```bash
   cd backend
   pip install -r requirements.txt
   uvicorn main:app --reload --port 8000
   ```

2. **Verify Qdrant Connection**
   ```bash
   # Check that Qdrant is accessible and contains the expected collection
   # Default collection name should match the retriever configuration
   ```

3. **Enable CORS in FastAPI**
   Add CORS middleware to your FastAPI application:
   ```python
   from fastapi.middleware.cors import CORSMiddleware

   app.add_middleware(
       CORSMiddleware,
       allow_origins=["http://localhost:3000"],  # Docusaurus default
       allow_credentials=True,
       allow_methods=["*"],
       allow_headers=["*"],
   )
   ```

4. **Test the /query Endpoint**
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{"question": "What is a humanoid robot?"}'
   ```

## Expected Response Format

The /query endpoint should return JSON in the following format:
```json
{
  "answer": "The answer to your question..."
}
```

## Error Handling

If the RAG agent encounters an error, it should:
1. Log the error to the console
2. Return a fallback response with proper JSON format
3. Never return an empty response or generic error

## Verification Steps

1. Submit a test question to the /query endpoint
2. Verify the response contains a proper "answer" field
3. Check that the Docusaurus frontend receives and displays the answer
4. Test error scenarios to ensure proper error handling