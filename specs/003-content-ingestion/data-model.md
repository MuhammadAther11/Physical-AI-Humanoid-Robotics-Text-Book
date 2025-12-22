# Data Model: Content Ingestion & Vector Storage

## Entities

### Content Chunk
- **id**: string (UUID) - Unique identifier for the chunk
- **text**: string (500-800 words) - The actual content chunk
- **url**: string - Source URL of the content
- **section_title**: string - Title of the section from which the chunk was extracted
- **created_at**: datetime - Timestamp when chunk was created
- **updated_at**: datetime - Timestamp when chunk was last updated
- **word_count**: integer - Number of words in the chunk
- **hash**: string - Hash of the content for change detection

### Semantic Representation Vector
- **id**: string (UUID) - Unique identifier for the vector
- **chunk_id**: string - Reference to the Content Chunk
- **vector**: array<float> - The embedding vector
- **model_used**: string - The model that generated the embedding (e.g., "cohere/embed-multilingual-v2.0")
- **created_at**: datetime - Timestamp when vector was created

### Metadata
- **id**: string (UUID) - Unique identifier for the metadata record
- **chunk_id**: string - Reference to the Content Chunk
- **url**: string - Source URL of the content
- **section_title**: string - Title of the section
- **source_title**: string - Title of the source document/page
- **word_count**: integer - Number of words in the chunk
- **created_at**: datetime - Timestamp when metadata was created

### Ingestion Job
- **id**: string (UUID) - Unique identifier for the ingestion job
- **status**: enum - Status of the job (pending, running, completed, failed)
- **source_url**: string - URL of the Docusaurus site being ingested
- **start_time**: datetime - When the job started
- **end_time**: datetime - When the job completed/failed
- **processed_count**: integer - Number of pages processed
- **successful_count**: integer - Number of pages successfully processed
- **failed_count**: integer - Number of pages that failed processing
- **error_log**: text - Details of any errors encountered

## Relationships

- Content Chunk 1:M Semantic Representation Vector (one chunk can have multiple vector representations over time)
- Content Chunk 1:1 Metadata (one chunk has one set of metadata)
- Ingestion Job 1:M Content Chunk (one job can create many chunks)

## Validation Rules

- Content Chunk.text must be between 500-800 words
- Content Chunk.url must be a valid URL format
- Semantic Representation Vector.vector must have consistent dimensions based on the model used
- Metadata.url and Content Chunk.url must match
- Content Chunk.hash must be calculated using SHA-256 algorithm

## State Transitions

### Ingestion Job
- pending → running (when ingestion starts)
- running → completed (when all content is processed successfully)
- running → failed (when errors occur and threshold is exceeded)