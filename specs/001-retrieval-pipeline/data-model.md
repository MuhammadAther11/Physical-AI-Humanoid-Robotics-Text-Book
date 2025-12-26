# Data Model: Retrieval Pipeline & Validation

## Entities

### Query
- **id**: string (UUID) - Unique identifier for the query
- **text**: string - The user's input query text
- **created_at**: datetime - Timestamp when query was received
- **processed_at**: datetime - Timestamp when query was processed
- **status**: enum - Status of the query (pending, processing, completed, failed)
- **top_k**: integer - Number of top results requested (default: 5-10)
- **similarity_threshold**: float - Minimum similarity score threshold

### Semantic Search Result
- **id**: string (UUID) - Unique identifier for the search result
- **query_id**: string - Reference to the Query that generated this result
- **content_chunk_id**: string - Reference to the Content Chunk
- **similarity_score**: float - The similarity score between query and content
- **rank**: integer - The rank of this result in the top-k results
- **retrieved_at**: datetime - Timestamp when result was retrieved

### Content Chunk
- **id**: string (UUID) - Unique identifier for the content chunk
- **text**: string - The actual content text
- **url**: string - Source URL of the content
- **section_title**: string - Title of the section from which the chunk was extracted
- **created_at**: datetime - Timestamp when chunk was created
- **word_count**: integer - Number of words in the chunk
- **embedding_id**: string - Reference to the embedding vector

### Metadata
- **id**: string (UUID) - Unique identifier for the metadata record
- **content_chunk_id**: string - Reference to the Content Chunk
- **url**: string - Source URL of the content
- **section_title**: string - Title of the section
- **source_title**: string - Title of the source document/page
- **created_at**: datetime - Timestamp when metadata was created

### Embedding Vector
- **id**: string (UUID) - Unique identifier for the embedding vector
- **content_chunk_id**: string - Reference to the Content Chunk
- **vector**: array<float> - The embedding vector
- **model_used**: string - The model that generated the embedding (e.g., "cohere/embed-multilingual-v2.0")
- **created_at**: datetime - Timestamp when vector was created

## Relationships

- Query 1:M Semantic Search Result (one query can have multiple results)
- Content Chunk 1:1 Metadata (one chunk has one set of metadata)
- Content Chunk 1:1 Embedding Vector (one chunk has one embedding vector)
- Query M:M Content Chunk (via Semantic Search Result - one query can return multiple chunks)

## Validation Rules

- Query.text must not exceed 500 characters
- Query.top_k must be between 1 and 20 (default 5-10)
- Content Chunk.text must be between 100-1000 words
- Content Chunk.url must be a valid URL format
- Semantic Search Result.similarity_score must be between 0 and 1
- Metadata.url and Content Chunk.url must match

## State Transitions

### Query
- pending → processing (when query is received and begins processing)
- processing → completed (when all results are retrieved successfully)
- processing → failed (when errors occur during processing)
- completed → validated (when results are validated for accuracy)