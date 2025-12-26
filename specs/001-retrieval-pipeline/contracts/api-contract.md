# API Contract: Retrieval Service

## Overview
This document describes the API contract for the content retrieval service that performs semantic search on ingested textbook content.

## Endpoints

### POST /retrieve
Performs semantic search on the ingested content using a user query.

#### Request
```json
{
  "query": "user query text",
  "top_k": 5,
  "similarity_threshold": 0.5
}
```

#### Response (Success)
```json
{
  "query_id": "uuid-string",
  "results": [
    {
      "id": "result-uuid",
      "content_chunk_id": "chunk-uuid",
      "text": "content chunk text...",
      "url": "https://original-url.com/page",
      "section_title": "Section Title",
      "similarity_score": 0.92,
      "rank": 1
    }
  ],
  "retrieval_time_ms": 125
}
```

#### Response (Error)
```json
{
  "error": "query_processing_failed",
  "message": "The query could not be processed"
}
```

### GET /retrieve/{query_id}
Gets the status and results of a retrieval operation.

#### Response
```json
{
  "query_id": "uuid-string",
  "status": "completed",
  "results": [
    {
      "id": "result-uuid",
      "content_chunk_id": "chunk-uuid",
      "text": "content chunk text...",
      "url": "https://original-url.com/page",
      "section_title": "Section Title",
      "similarity_score": 0.92,
      "rank": 1
    }
  ],
  "created_at": "2025-12-23T10:00:00Z",
  "processed_at": "2025-12-23T10:00:00.125Z"
}
```

## Data Models

### QueryRequest
- query: string (required) - The user's input query text
- top_k: integer (default: 5) - Number of top results to return
- similarity_threshold: float (default: 0.5) - Minimum similarity score threshold

### SearchResult
- id: string (uuid)
- content_chunk_id: string (uuid)
- text: string
- url: string (url)
- section_title: string
- similarity_score: float (0.0 - 1.0)
- rank: integer

### RetrievalResponse
- query_id: string (uuid)
- results: array[SearchResult]
- retrieval_time_ms: integer
- status: enum ["pending", "processing", "completed", "failed"]