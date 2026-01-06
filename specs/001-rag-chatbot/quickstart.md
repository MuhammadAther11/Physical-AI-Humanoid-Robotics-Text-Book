# Quick Start: RAG Chatbot Development

This guide helps you set up and test the RAG chatbot for the Physical AI & Humanoid Robotics textbook.

## Prerequisites

### 1. Qdrant Cloud Account

1. Create an account at [qdrant.cloud](https://qdrant.cloud)
2. Create a new cluster (free tier is sufficient)
3. Note your cluster URL and API key

**Add to `backend/.env`:**
```bash
QDRANT_URL=https://your-cluster.aws.cloud.qdrant.io:6334
QDRANT_API_KEY=your-api-key-here
```

### 2. OpenAI API Key

1. Get an API key from [platform.openai.com](https://platform.openai.com)
2. Add to `backend/.env`:
   ```bash
   OPENAI_API_KEY=sk-your-openai-key-here
   ```

### 3. Verify Configuration

Run the configuration validation:
```bash
cd backend
python -c "from src.config import Config; errors = Config.validate(); print('Errors:' if errors else 'OK')"
```

## Starting the Services

### Terminal 1: Start the Backend

```bash
cd backend

# Activate virtual environment (Windows)
.venv\Scripts\activate

# Or Linux/Mac
source .venv/bin/activate

# Start FastAPI server
python src/main.py
```

Expected output:
```
INFO: Application startup complete
INFO: API Documentation available at: /docs
INFO: API Redoc available at: /redoc
```

The API will be available at:
- API: http://localhost:8000
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

### Terminal 2: Start the Frontend

```bash
cd book-frontend
npm start
```

The Docusaurus site will open at http://localhost:3000

## Regenerating Embeddings

If embeddings are missing or corrupted, regenerate them:

```bash
cd backend
python content_ingestion.py
```

This script will:
1. Scan the `book-frontend/docs` directory
2. Extract content from all `.md` and `.mdx` files
3. Chunk content by section headers
4. Generate embeddings using OpenAI text-embedding-3-small
5. Store vectors in Qdrant collection `book_content`

### Chunking Strategy

Content is chunked at heading boundaries (H1, H2) to maintain semantic coherence:
- Each chunk represents a logical section
- Metadata includes section path for citation
- Overlap: 50 characters between chunks for continuity

## Testing the Chatbot

### 1. Health Check

```bash
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "RAG Query API",
  "timestamp": "2026-01-03T12:00:00Z",
  "dependencies": {
    "qdrant": "connected",
    "openai": "connected"
  }
}
```

### 2. Submit a Query

```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is a humanoid robot?"}'
```

Expected response:
```json
{
  "queryId": "550e8400-e29b-41d4-a716-446655440000",
  "answer": "A humanoid robot is a robot that...",
  "sources": [
    {
      "url": "/docs/introduction",
      "title": "Introduction to Humanoid Robotics",
      "relevanceScore": 0.92
    }
  ],
  "confidence": 0.92,
  "processingTimeMs": 2450,
  "status": "success"
}
```

### 3. View on Frontend

1. Open http://localhost:3000
2. Look for the chat icon/widget on the page
3. Click to expand the chat interface
4. Type a question about the book content

## Common Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| "Service unavailable" | Qdrant not running | Check cluster URL and API key in `.env` |
| Empty sources returned | No embeddings stored | Run `python content_ingestion.py` |
| Rate limit errors | Too many requests | Wait 60 seconds (limit: 10 req/min) |
| 401 errors | Invalid API key | Verify OpenAI and Qdrant API keys |
| Empty answer | Query not in book content | Ask about topics covered in the book |
| Slow responses | Large embedding index | Normal for initial queries |

### Checking Embedding Count

```bash
# Check how many vectors are stored
curl -X GET "http://localhost:8000/api/collections/book_content" \
  -H "Authorization: Bearer $QDRANT_API_KEY"
```

### Debug Mode

Enable debug logging:
```bash
DEBUG=True python src/main.py
```

## Architecture Overview

```
User Query
    │
    ▼
┌─────────────────┐
│  Frontend (3000)│
│   React/Docusaurus│
└────────┬────────┘
         │ HTTP POST /api/query
         ▼
┌─────────────────┐
│  Backend (8000) │
│  FastAPI        │
│  - Validation   │
│  - Rate Limiting│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Query Processing│
│  - Embedding    │
│  - Retrieval    │
│  - Generation   │
└────────┬────────┘
    ┌────┴────┐
    ▼         ▼
┌───────┐ ┌──────────┐
│Qdrant │ │  OpenAI  │
│Vector │ │  (LLM)   │
│DB     │ │          │
└───────┘ └──────────┘
```

## Next Steps

After confirming the basic setup works:

1. **Test edge cases**: Ask about topics not in the book
2. **Verify sources**: Check that citations link to real content
3. **Performance test**: Measure response times under load
4. **UI polish**: Customize the chat widget appearance
