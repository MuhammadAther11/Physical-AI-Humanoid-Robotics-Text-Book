# API Contract: Content Ingestion Service

## Overview
This document describes the API contract for the content ingestion service that fetches, processes, and stores Docusaurus site content in vector format.

## Endpoints

### POST /ingest
Initiates the content ingestion process from a Docusaurus site.

#### Request
```json
{
  "site_url": "https://example-docusaurus-site.com",
  "chunk_size": 500,
  "max_chunk_size": 800,
  "collection_name": "rag_embeddings",
  "force_reprocess": false
}
```

#### Response (Success)
```json
{
  "job_id": "uuid-string",
  "status": "running",
  "estimated_completion": "2025-12-22T15:30:00Z",
  "total_pages_found": 120,
  "pages_to_process": 120
}
```

#### Response (Error)
```json
{
  "error": "site_unreachable",
  "message": "The specified site URL is not accessible"
}
```

### GET /ingest/{job_id}
Gets the status of an ingestion job.

#### Response
```json
{
  "job_id": "uuid-string",
  "status": "completed",
  "pages_processed": 120,
  "pages_successful": 118,
  "pages_failed": 2,
  "start_time": "2025-12-22T12:00:00Z",
  "end_time": "2025-12-22T13:45:00Z",
  "completion_percentage": 100
}
```

### POST /search
Performs a semantic search on the ingested content.

#### Request
```json
{
  "query": "search query text",
  "collection_name": "rag_embeddings",
  "limit": 5
}
```

#### Response
```json
{
  "results": [
    {
      "id": "chunk-uuid",
      "text": "content chunk text...",
      "url": "https://original-url.com/page",
      "section_title": "Section Title",
      "similarity_score": 0.92
    }
  ]
}
```

## Data Models

### IngestionJob
- job_id: string (uuid)
- status: enum ["pending", "running", "completed", "failed"]
- site_url: string (url)
- total_pages: integer
- processed_pages: integer
- successful_pages: integer
- failed_pages: integer
- start_time: datetime
- end_time: datetime
- error_log: array[ErrorLog]

### ErrorLog
- timestamp: datetime
- url: string (url)
- error_type: string
- error_message: string

### SearchQuery
- query: string (required)
- collection_name: string (default: "rag_embeddings")
- limit: integer (default: 5, max: 20)

### SearchResult
- id: string (uuid)
- text: string
- url: string (url)
- section_title: string
- similarity_score: float (0.0 - 1.0)