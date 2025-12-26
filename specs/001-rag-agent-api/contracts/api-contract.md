# API Contract: RAG Agent API

## Overview
This document describes the API contract for the RAG Agent that provides context-grounded answers based on textbook content.

## Base URL
```
https://api.example.com/v1
```

## Authentication
All requests must include an Authorization header with a Bearer token:
```
Authorization: Bearer {API_KEY}
```

## Endpoints

### POST /query
Processes a user query and returns a context-grounded answer with citations.

#### Request
```json
{
  "query": "string, the user's query text (required)",
  "session_id": "string, optional session identifier",
  "temperature": "float, optional temperature for response generation (0.0-1.0)",
  "max_tokens": "integer, optional max tokens for response (default: 1000)"
}
```

#### Response (Success)
```json
{
  "id": "string, unique identifier for the response",
  "answer": "string, the context-grounded answer",
  "query_id": "string, identifier for the original query",
  "citations": [
    {
      "url": "string, source URL for the cited content",
      "section_title": "string, title of the cited section",
      "confidence": "float, confidence score (0.0-1.0)"
    }
  ],
  "response_time_ms": "integer, time taken to process the request",
  "model_used": "string, the model used to generate the response"
}
```

#### Response (Error)
```json
{
  "error": {
    "code": "string, error code (e.g., 'invalid_query', 'retrieval_error', 'agent_error')",
    "message": "string, human-readable error message",
    "details": "object, optional additional error details"
  }
}
```

### GET /health
Checks the health status of the API and its dependencies.

#### Response
```json
{
  "status": "string, overall health status ('healthy', 'degraded', 'unhealthy')",
  "checks": {
    "openai_agent_sdk": {
      "status": "string, status of OpenAI connection",
      "message": "string, optional details"
    },
    "retrieval_system": {
      "status": "string, status of retrieval system connection",
      "message": "string, optional details"
    },
    "database": {
      "status": "string, status of database connection if applicable",
      "message": "string, optional details"
    }
  },
  "timestamp": "string, ISO 8601 timestamp of the health check"
}
```

### POST /sessions
Creates a new agent session for maintaining conversation context.

#### Request
```json
{
  "user_id": "string, optional identifier for the user",
  "metadata": "object, optional session metadata"
}
```

#### Response
```json
{
  "session_id": "string, unique identifier for the new session",
  "created_at": "string, ISO 8601 timestamp of creation",
  "expires_at": "string, ISO 8601 timestamp when session expires"
}
```

## Data Models

### QueryRequest
- query: string (required) - The user's query text
- session_id: string (optional) - Session identifier to maintain context
- temperature: float (optional, default: 0.3) - Controls randomness of response
- max_tokens: integer (optional, default: 1000) - Maximum tokens in response

### QueryResponse
- id: string (uuid) - Unique response identifier
- answer: string - The context-grounded answer
- query_id: string (uuid) - Reference to the original query
- citations: array[Citation] - Sources used in the answer
- response_time_ms: integer - Processing time in milliseconds
- model_used: string - Name of the model used

### Citation
- url: string (url) - Source URL of the cited content
- section_title: string - Title of the cited section
- confidence: float (0.0-1.0) - Confidence score in the citation

### HealthStatus
- status: enum ["healthy", "degraded", "unhealthy"]
- checks: object - Individual service health checks
- timestamp: string (ISO 8601) - Time of the health check

## Error Codes

- `invalid_query`: The query format was invalid
- `retrieval_error`: Error occurred during content retrieval
- `agent_error`: Error occurred during answer generation
- `rate_limit_exceeded`: API rate limit has been exceeded
- `internal_error`: Internal server error occurred
- `service_unavailable`: Dependency service is temporarily unavailable