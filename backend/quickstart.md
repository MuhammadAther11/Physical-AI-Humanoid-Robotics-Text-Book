# Quickstart Guide for RAG Chatbot Backend

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to Cohere API key
- Access to Qdrant Cloud or local instance

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd Physical-AI-Humanoid-Robotics-Text-Book
   ```

2. **Set up Python virtual environment**
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure environment variables**
   Create a `.env` file in the backend directory with the following variables:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   OPENAI_API_KEY=your_openai_api_key (optional)
   OPENROUTER_API_KEY=your_openrouter_api_key (optional)
   DOCUSAURUS_SITE_URL=https://your-docusaurus-site.com
   ```

5. **Start the backend server**
   ```bash
   uvicorn main:app --reload --port 8000
   ```

## API Usage

### Query Endpoint
Send a POST request to `http://localhost:8000/query` with the following JSON payload:

```json
{
  "question": "Your question here",
  "selected_text": "Optional selected text for context",
  "session_id": "Optional session ID"
}
```

### Example Request
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key components of a humanoid robot?",
    "selected_text": "",
    "session_id": "session-123"
  }'
```

### Expected Response
```json
{
  "answer": "The answer to your question...",
  "status": "success"
}
```

## Health Check
Check the health of the service at `http://localhost:8000/health`

## Troubleshooting

- If you get CORS errors, make sure the frontend domain is added to the CORS middleware configuration
- If queries return no results, verify that content has been ingested into Qdrant
- If you get API key errors, double-check your environment variables

## Ingesting Content
To populate the knowledge base with content from your Docusaurus site:
```bash
curl -X POST http://localhost:8000/ingest
```