# Implementation Plan: RAG Chatbot Fix & UI Integration

**Branch**: `001-rag-chatbot` | **Date**: 2026-01-03 | **Spec**: [spec.md](../spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

## Summary

The goal is to make the RAG chatbot fully functional for the Physical AI & Humanoid Robotics textbook. This involves:
1. **Re-generating embeddings** for complete book content and storing them correctly in Qdrant
2. **Fixing all RAG pipeline errors** that currently prevent successful embedding storage and retrieval
3. **Building a clean chatbot UI** and integrating it into the Docusaurus homepage

**Technical Approach**: Leverage existing FastAPI backend with Qdrant vector database and OpenAI for LLM responses. The frontend uses a React-based chatbot component that communicates with the backend via REST API.

## Technical Context

**Language/Version**: Python 3.11 (backend), TypeScript/JavaScript (frontend Docusaurus)
**Primary Dependencies**: FastAPI, Qdrant client, OpenAI SDK, React/Docusaurus
**Storage**: Qdrant Cloud (vector database), existing book content (Docusaurus docs)
**Testing**: pytest (Python tests exist in backend/)
**Target Platform**: Local development environment (Docusaurus + FastAPI backend)
**Project Type**: Web application with separate backend/frontend
**Performance Goals**: Response time under 30 seconds per query, 100% embedding storage success
**Constraints**: Local-only development, no production deployment, no authentication
**Scale/Scope**: Single collection in Qdrant, all book content indexed

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| **I. Technical Accuracy** | PASS | All responses derived from indexed book content only |
| **II. Reproducible Code** | PASS | Backend tests exist; frontend component structure defined |
| **IV. Zero Hallucinations** | PASS | System constrained to use only indexed content via RAG pipeline |
| **V. Standardized Toolchain** | PASS | Uses FastAPI, Qdrant, OpenAI as specified in constitution |
| **VI. Realistic Architectures** | PASS | Standard RAG architecture with vector search + LLM |

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (below)
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── api-contract.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   └── api.py              # Query endpoint: POST /api/query
│   ├── models/
│   │   ├── query.py            # Query model
│   │   └── response.py         # Response model
│   ├── services/
│   │   ├── qdrant_service.py   # Vector storage/retrieval
│   │   ├── openai_service.py   # LLM integration
│   │   ├── rag_agent_service.py # RAG orchestration
│   │   └── query_processing_service.py
│   ├── config.py               # Environment configuration
│   └── main.py                 # FastAPI app entry
├── content_ingestion.py        # Book content ingestion script
├── requirements.txt
└── tests/                      # Existing test files

book-frontend/
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── Chatbot.jsx     # Chatbot UI component
│   │       └── Chatbot.css
│   ├── pages/
│   │   └── chatbot.tsx         # Chatbot page
│   └── services/
│       └── api.js              # API service layer
├── docusaurus.config.js/ts
└── package.json
```

**Structure Decision**: Web application with separate Python backend (FastAPI) and Docusaurus frontend (React). The structure is already established and functional - this plan focuses on fixing issues and completing integration.

## Complexity Tracking

> No constitution violations requiring justification.

---

# Phase 0: Research & Technical Decisions

## research.md

This section documents the technical decisions made for the RAG Chatbot implementation based on analysis of existing code and requirements.

### Decision 1: Embedding Storage Strategy

**Decision**: Re-generate embeddings for all book content using the existing embedding model and store in Qdrant with proper metadata.

**Rationale**:
- Existing `content_ingestion.py` provides content extraction from Docusaurus
- Qdrant collection `book_content` already defined in configuration
- Metadata (source URL, title, section) required for citation in responses

**Alternatives Considered**:
- Partial re-ingestion (only new/changed content) - Rejected: Need complete rebuild to ensure all content is properly indexed
- Third-party embedding service - Rejected: Existing embedding model already configured

**Implementation Notes**:
- Use OpenAI text-embedding-3-small model (cost-effective, good quality)
- Chunk content by section headers (H1, H2) for semantic coherence
- Store metadata: `source_url`, `title`, `section_path`, `char_count`

### Decision 2: Error Handling Strategy

**Decision**: Implement comprehensive error handling at each RAG pipeline stage with graceful degradation.

**Rationale**:
- Current pipeline has unhandled exceptions causing failures
- Users need feedback when something goes wrong (per FR-008)
- Must prevent hallucination when retrieval fails

**Error Taxonomy**:
| Error Type | User Message | System Action |
|------------|--------------|---------------|
| Qdrant connection failed | "Service temporarily unavailable" | Return 503, log error |
| No results retrieved | "I couldn't find information about that in the book" | No fabricated answer |
| Embedding generation failed | "Unable to process your question" | Return 400/500 |
| LLM generation failed | "Unable to generate response" | Return 500 with retry hint |
| Query timeout | "Taking too long, please try again" | Return 504 |

### Decision 3: Chatbot UI Integration Strategy

**Decision**: Add chatbot widget to Docusaurus homepage as a persistent floating component.

**Rationale**:
- Homepage already has feature components that can include chatbot
- Users expect chat on homepage per User Story 2
- Docusaurus theme allows custom React components

**Implementation**:
- Add `<Chatbot />` component to homepage index.tsx
- Use floating button design (expandable chat window)
- Show on all pages via Docusaurus layout or homepage only

### Decision 4: API Contract Standardization

**Decision**: Standardize API responses with consistent format, error codes, and source citations.

**Rationale**:
- Frontend needs reliable response structure
- Error handling must be consistent
- Users need to trust answers come from book (FR-005, FR-010)

**Response Format**:
```json
{
  "queryId": "uuid",
  "answer": "Response text...",
  "sources": [
    {"url": "...", "title": "...", "relevance_score": 0.85}
  ],
  "processing_time_ms": 1500,
  "status": "success" | "partial" | "error"
}
```

### Decision 5: Loading State Handling

**Decision**: Implement optimistic UI with loading indicators for all async operations.

**Rationale**:
- RAG queries take time (embedding + LLM generation)
- Users need feedback during processing (FR-007)
- Prevent duplicate submissions

**States**:
- `idle`: Ready for input
- `loading`: Show spinner/progress
- `streaming`: Show partial response as it arrives
- `error`: Show error with retry option
- `success`: Display answer with sources

---

# Phase 1: Design Artifacts

## data-model.md

### Entity: Query

| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| `id` | UUID | Auto-generated | Unique query identifier |
| `text` | string | 1-2000 chars, required | User's question |
| `userId` | string | Optional, max 100 chars | User identifier for rate limiting |
| `timestamp` | datetime | Auto-generated | When query was submitted |
| `context` | array | Optional | Previous messages for continuity |

### Entity: Response

| Field | Type | Validation | Description |
|-------|------|------------|-------------|
| `queryId` | UUID | Required | Links to original query |
| `answer` | string | Required | Generated response text |
| `sources` | array | Required | Source citations from retrieval |
| `confidence` | float | 0-1 | Relevance score of best match |
| `processingTimeMs` | int | Required | Total processing time |
| `status` | enum | Required | success, partial, error |

### Entity: SourceDocument

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Vector point ID |
| `url` | string | Source URL in Docusaurus |
| `title` string | Section/page title |
| `content` | string | Raw text content |
| `embedding` | array[float] | Vector representation |
| `metadata` | object | Section path, char count, etc. |

### Entity: ChatMessage

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Message identifier |
| `role` | enum | user, assistant, system |
| `content` | string | Message text |
| `sources` | array | Citations for assistant messages |
| `timestamp` | datetime | When message was created |

### Relationships

```
Query 1:1 Response
Response 1:N SourceDocument (retrieved sources)
Query 1:N ChatMessage (conversation history)
```

## contracts/api-contract.yaml

```yaml
openapi: 3.0.3
info:
  title: Physical AI & Humanoid Robotics RAG API
  version: 1.0.0
  description: API for querying the textbook chatbot

paths:
  /api/query:
    post:
      summary: Submit a query to the RAG chatbot
      description: Accepts user queries and returns responses based on indexed book content
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required:
                - query
              properties:
                query:
                  type: string
                  minLength: 1
                  maxLength: 2000
                  description: The user's question
                userId:
                  type: string
                  maxLength: 100
                  description: Optional user identifier for rate limiting
                context:
                  type: array
                  description: Previous messages for conversation continuity
      responses:
        '200':
          description: Successful response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/QueryResponse'
        '400':
          description: Invalid request
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '429':
          description: Rate limit exceeded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '500':
          description: Internal server error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

  /api/health:
    get:
      summary: Health check
      description: Returns health status of API and dependencies
      responses:
        '200':
          description: Service is healthy
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/HealthResponse'

components:
  schemas:
    QueryResponse:
      type: object
      properties:
        queryId:
          type: string
          format: uuid
          description: Unique identifier for this query
        answer:
          type: string
          description: Generated response from the RAG system
        sources:
          type: array
          items:
            $ref: '#/components/schemas/Source'
          description: Retrieved source documents
        processingTimeMs:
          type: integer
          description: Processing time in milliseconds
        status:
          type: string
          enum: [success, partial, error]
          description: Response status

    Source:
      type: object
      properties:
        url:
          type: string
          description: Source document URL
        title:
          type: string
          description: Document title
        relevanceScore:
          type: number
          format: float
          description: Similarity score (0-1)

    ErrorResponse:
      type: object
      properties:
        error:
          type: string
          description: Error type
        detail:
          type: string
          description: Human-readable error message
        code:
          type: string
          description: Machine-readable error code

    HealthResponse:
      type: object
      properties:
        status:
          type: string
          enum: [healthy, unhealthy]
        service:
          type: string
        dependencies:
          type: object
          properties:
            qdrant:
              type: string
            openai:
              type: string
```

## quickstart.md

### Quick Start: RAG Chatbot Development

#### Prerequisites

1. **Qdrant Cloud**
   - Create account at qdrant.cloud
   - Get API key and cluster URL
   - Add to `backend/.env`:
     ```
     QDRANT_URL=https://xxx-xxx.aws.cloud.qdrant.io:6334
     QDRANT_API_KEY=your-api-key
     ```

2. **OpenAI API Key**
   - Get key from platform.openai.com
   - Add to `backend/.env`:
     ```
     OPENAI_API_KEY=sk-...
     ```

3. **Start Services**
   ```bash
   # Terminal 1: Start backend
   cd backend
   source .venv/bin/activate  # or .venv\Scripts\activate on Windows
   python src/main.py

   # Terminal 2: Start frontend
   cd book-frontend
   npm start
   ```

#### Regenerate Embeddings

```bash
cd backend
python content_ingestion.py
```

This will:
1. Scan Docusaurus docs directory
2. Extract content from all .md/.mdx files
3. Generate embeddings for each section
4. Store in Qdrant collection `book_content`

#### Test the Chatbot

```bash
# Health check
curl http://localhost:8000/api/health

# Submit a query
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is humanoid robotics?"}'
```

#### View Frontend

Open http://localhost:3000 in your browser. The chatbot widget should be visible on the homepage.

#### Common Issues

| Issue | Solution |
|-------|----------|
| Qdrant connection refused | Check cluster URL and API key in `.env` |
| No embeddings found | Run `python content_ingestion.py` to populate |
| 429 rate limit | Wait 60 seconds, reduce request frequency |
| Empty responses | Check Qdrant has indexed documents |
