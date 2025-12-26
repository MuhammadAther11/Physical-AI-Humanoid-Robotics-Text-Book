# Quickstart: RAG Agent API

## Prerequisites

- Python 3.11 or higher
- OpenAI API key with Agent SDK access
- FastAPI-compatible environment
- Access to the retrieval system (from the content ingestion feature)
- uv package manager (or pip)

## Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

3. **Install uv package manager** (if not already installed):
   ```bash
   pip install uv
   ```

4. **Create a virtual environment and install dependencies**:
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install openai fastapi uvicorn python-dotenv
   ```

5. **Set up environment variables**:
   Create a `.env` file in the backend directory with the following content:
   ```
   OPENAI_API_KEY=your_openai_api_key_here
   RETRIEVAL_ENDPOINT=your_retrieval_endpoint_url_here
   FASTAPI_ENV=development
   ```

## Running the API

1. **Start the API server**:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

2. **The API will be available at**: `http://localhost:8000`

3. **Access the API documentation at**: `http://localhost:8000/docs`

## API Usage

### Submitting a Query

Send a POST request to `/query` with the following JSON body:

```json
{
  "query": "Your question about the textbook content",
  "session_id": "optional-session-id"
}
```

Example using curl:
```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the fundamentals of ROS2?",
    "session_id": "abc123"
  }'
```

### Expected Response

```json
{
  "id": "response-id",
  "answer": "The answer to your query...",
  "citations": [
    {
      "url": "https://source-url.com/page",
      "section_title": "Section Title",
      "confidence": 0.95
    }
  ],
  "query_id": "query-id",
  "response_time_ms": 1250
}
```

## Example Implementation

The RAG agent API contains these key components:
- `main.py`: FastAPI application entry point
- `agent/`: Agent processing logic using OpenAI Agent SDK
- `retrieval_client/`: Client to interface with the retrieval system
- `models/`: Pydantic models for request/response validation
- `middleware/`: Request processing and validation middleware

## Testing the API

1. **Run the test suite**:
   ```bash
   pytest tests/
   ```

2. **Validate API responses**:
   - Submit test queries and verify that responses are context-grounded
   - Check that citations are properly included
   - Verify that answers are relevant to the textbook content

3. **Performance validation**:
   - Test that API responds to 95% of requests within 2 seconds
   - Verify that the system can handle concurrent requests

## Configuration

The API can be configured with:
- `OPENAI_MODEL`: The OpenAI model to use (default: gpt-4-turbo)
- `MAX_TOKENS`: Maximum tokens for responses (default: 1000)
- `TEMPERATURE`: Temperature setting for agent responses (default: 0.3)
- `RETRIEVAL_K`: Number of content chunks to retrieve (default: 5)

## Troubleshooting

- **API Key Issues**: Ensure your OpenAI API key is correct and has the necessary permissions for Agent SDK
- **Retrieval Issues**: Verify that the retrieval system is accessible and properly configured
- **Rate Limiting**: Monitor API usage to ensure you stay within OpenAI's rate limits
- **Response Quality**: Adjust temperature and other parameters to improve answer quality

## Next Steps

After successful validation:
1. The RAG agent API can be integrated with the frontend
2. Consider implementing caching for frequently asked questions
3. Set up monitoring for API performance and usage metrics