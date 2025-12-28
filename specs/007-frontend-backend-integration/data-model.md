# Data Model: Frontend ↔ Backend Integration

## Entities

### Query
Represents a user's question or request sent from the frontend to the backend.

**Fields**:
- `id` (string, required): Unique identifier for the query
- `content` (string, required): The actual text content of the user's query
- `timestamp` (datetime, required): When the query was submitted
- `userId` (string, optional): Identifier for the user who submitted the query
- `metadata` (object, optional): Additional information about the query context

**Validation Rules**:
- Content must be between 1 and 2000 characters
- Content must not be empty or contain only whitespace
- Timestamp must be in ISO 8601 format

### Response
Represents the RAG agent's answer sent from the backend to the frontend.

**Fields**:
- `id` (string, required): Unique identifier for the response
- `queryId` (string, required): Reference to the original query
- `content` (string, required): The text content of the agent's response
- `timestamp` (datetime, required): When the response was generated
- `sources` (array of objects, optional): List of sources used to generate the response
- `metadata` (object, optional): Additional information about the response

**Validation Rules**:
- Content must not be empty
- QueryId must reference an existing query
- Sources must be an array of objects with 'title' and 'url' fields

### API Endpoint
Represents the communication interface between frontend and backend.

**Endpoint**: `/api/query`
**Method**: POST
**Request Format**: JSON
**Response Format**: JSON

**Request Body**:
- Must contain a 'query' field with the user's question
- May contain optional 'userId' and 'context' fields

**Response Body**:
- Contains 'response' field with the agent's answer
- May contain 'sources' field with references to source documents
- Includes 'queryId' to link the response to the original query

## State Transitions

### Query States
1. **Submitted**: Query has been received by the backend
2. **Processing**: Query is being handled by the RAG agent
3. **Completed**: Response has been generated and is ready to return
4. **Failed**: An error occurred during processing

### Response States
1. **Generated**: Response has been created by the RAG agent
2. **Validated**: Response has been checked for quality and relevance
3. **Delivered**: Response has been sent back to the frontend