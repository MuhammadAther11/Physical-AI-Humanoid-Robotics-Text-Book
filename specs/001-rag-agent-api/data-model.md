# Data Model: RAG Agent API

## Entities

### Query
- **id**: string (UUID) - Unique identifier for the query
- **text**: string - The user's input query text
- **created_at**: datetime - Timestamp when query was received
- **session_id**: string (optional) - Identifier for the conversation session
- **metadata**: object - Additional query metadata (user preferences, context, etc.)

### Retrieved Content
- **id**: string (UUID) - Unique identifier for the retrieved content chunk
- **content**: string - The actual content text from the textbook
- **url**: string - Source URL of the content
- **section_title**: string - Title of the section from which the content was extracted
- **similarity_score**: float - Similarity score between query and content (0.0-1.0)
- **source_rank**: integer - Rank of this content among all retrieved results
- **query_id**: string - Reference to the Query that triggered this retrieval

### Context-Grounded Answer
- **id**: string (UUID) - Unique identifier for the answer
- **text**: string - The agent-generated answer text
- **query_id**: string - Reference to the Query that generated this answer
- **retrieved_content_ids**: array<string> - References to the content used in the answer
- **confidence_score**: float - Confidence level in the answer's accuracy (0.0-1.0)
- **citations**: array<object> - Citations to source materials used in the answer
- **created_at**: datetime - Timestamp when answer was generated
- **token_usage**: object - Information about tokens used (input, output, total)

### API Request
- **id**: string (UUID) - Unique identifier for the API request
- **query_text**: string - The query text sent in the request
- **headers**: object - Request headers (authentication, etc.)
- **timestamp**: datetime - Time when the request was received
- **client_info**: object - Information about the client making the request

### API Response
- **id**: string (UUID) - Unique identifier for the API response
- **answer_id**: string - Reference to the generated answer
- **status_code**: integer - HTTP status code of the response
- **response_time_ms**: integer - Time taken to process the request in milliseconds
- **citations**: array<object> - Citations included in the response
- **timestamp**: datetime - Time when the response was sent

### Agent Session
- **id**: string (UUID) - Unique identifier for the agent session
- **user_id**: string (optional) - Identifier for the user if authenticated
- **created_at**: datetime - Time when the session was created
- **last_activity**: datetime - Time of last activity in the session
- **conversation_history**: array<object> - History of query-answer pairs in the session
- **session_metadata**: object - Additional session-specific data

## Relationships

- Query 1:1 Context-Grounded Answer (one query generates one answer)
- Query 1:M Retrieved Content (one query can retrieve multiple content chunks)
- API Request 1:1 Query (one API request creates one query)
- API Response 1:1 Context-Grounded Answer (one API response returns one answer)
- Agent Session 1:M Query (one session can have multiple queries)
- Context-Grounded Answer M:1 Query (one answer corresponds to one query)

## Validation Rules

- Query.text must not exceed 2000 characters
- Retrieved Content.similarity_score must be between 0.0 and 1.0
- Context-Grounded Answer.confidence_score must be between 0.0 and 1.0
- Query.session_id must be a valid UUID format if provided
- API Request.query_text must be at least 1 character long
- Citations in answers must reference actual source content

## State Transitions

### Query
- received → processing (when retrieval begins)
- processing → answered (when agent generates response)
- processing → failed (when errors occur during processing)

### Agent Session
- created → active (when first query is processed)
- active → inactive (after period of inactivity)
- inactive → resumed (when user sends another query)