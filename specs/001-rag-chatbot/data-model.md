# Data Model: RAG Chatbot

## Overview

This document describes the data models used by the RAG chatbot system for queries, responses, source documents, and chat messages.

## Entities

### Query

Represents a user question submitted to the RAG system.

| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| `id` | UUID | Auto-generated, readonly | Unique identifier for this query session |
| `text` | string | Required, 1-2000 chars | The user's question text |
| `userId` | string | Optional, max 100 chars | User identifier for rate limiting |
| `timestamp` | datetime | Auto-generated, readonly | When the query was submitted |
| `context` | array | Optional | Previous messages for conversation continuity |

**Example:**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "text": "What are the main components of a humanoid robot?",
  "userId": "anonymous",
  "timestamp": "2026-01-03T12:00:00Z",
  "context": []
}
```

### Response

Represents the RAG system's answer to a user query.

| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| `queryId` | UUID | Required, foreign key | Links to the original Query |
| `answer` | string | Required | Generated response text |
| `sources` | array | Required, non-empty when status=success | Source citations from retrieval |
| `confidence` | float | 0.0-1.0 | Relevance score of best match |
| `processingTimeMs` | int | Required | Total processing time in milliseconds |
| `status` | enum | Required | success, partial, error |

**Example:**
```json
{
  "queryId": "550e8400-e29b-41d4-a716-446655440000",
  "answer": "Humanoid robots typically consist of several key components...",
  "sources": [
    {
      "url": "/docs/introduction",
      "title": "Introduction to Humanoid Robotics",
      "relevanceScore": 0.92
    },
    {
      "url": "/docs/architecture",
      "title": "Robot Architecture",
      "relevanceScore": 0.85
    }
  ],
  "confidence": 0.88,
  "processingTimeMs": 2450,
  "status": "success"
}
```

### SourceDocument

Represents a chunk of indexed book content stored in the vector database.

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Unique identifier (Qdrant point ID) |
| `url` | string | Source document URL in Docusaurus |
| `title` | string | Section/page title |
| `content` | string | Raw text content (chunked) |
| `chunkIndex` | int | Position within source document |
| `metadata` | object | Additional metadata |

**Metadata Schema:**
```json
{
  "section_path": ["Part I", "Chapter 1", "Introduction"],
  "heading": "Introduction to Humanoid Robotics",
  "char_count": 1500,
  "word_count": 250,
  "content_type": "markdown",
  "last_updated": "2026-01-01T00:00:00Z"
}
```

### ChatMessage

Represents a single message in a chat conversation.

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Message identifier |
| `conversationId` | UUID | Groups messages into conversations |
| `role` | enum | user, assistant, system |
| `content` | string | Message text |
| `sources` | array | Citations for assistant messages |
| `timestamp` | datetime | When message was created |

**Example:**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440001",
  "conversationId": "550e8400-e29b-41d4-a716-446655440000",
  "role": "assistant",
  "content": "Humanoid robots typically consist of several key components...",
  "sources": [
    {
      "url": "/docs/introduction",
      "title": "Introduction to Humanoid Robotics",
      "relevanceScore": 0.92
    }
  ],
  "timestamp": "2026-01-03T12:00:02Z"
}
```

## Relationships

```
Query
  ├── 1:1 Response (one response per query)
  └── 1:N ChatMessage (query initiates conversation)

Response
  └── 1:N SourceDocument (retrieved sources for answer)

SourceDocument
  └── N:1 URL (many chunks per source document)
```

## State Transitions

### Query Processing States

```
idle → validating → embedding → retrieving → generating → success|error
```

| From State | To State | Trigger |
|------------|----------|---------|
| idle | validating | User submits query |
| validating | embedding | Query passes validation |
| embedding | retrieving | Embedding vector generated |
| retrieving | generating | Relevant docs retrieved |
| generating | success | LLM response generated |
| any | error | Exception occurs |

## Validation Rules

### Query Validation

- `text` must be 1-2000 characters
- `text` must not be empty or whitespace only
- `userId` must be string if provided
- Rate limit: max 10 requests per 60 seconds per userId

### Response Validation

- `answer` must be non-empty string
- `sources` must contain at least one source when status=success
- `confidence` must be 0.0-1.0
- `processingTimeMs` must be positive integer

### SourceDocument Validation

- `content` must be non-empty
- `url` must be valid Docusaurus path
- `embedding` must be array of floats (1536 dimensions for text-embedding-3-small)
