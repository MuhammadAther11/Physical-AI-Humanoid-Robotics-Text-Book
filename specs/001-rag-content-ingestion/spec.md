# Feature Specification: Content Ingestion & Vectorization

**Feature Branch**: `001-rag-content-ingestion`  
**Created**: 2025-12-19
**Status**: Draft  
**Input**: User description: "Spec: Spec 1 – Content Ingestion & Vectorization Target: RAG backend for AI-native textbook on GitHub Pages Focus: Extract book content, generate embeddings, and store them in a vector database Success: - All deployed pages are fetched and parsed - Content is cleanly chunked by section - Embeddings generated using Cohere - Vectors stored and indexed in Qdrant - Metadata includes page, section, and URL Constraints: - Source: Deployed Docusaurus URLs - Embeddings: Cohere models - Vector DB: Qdrant Cloud - Idempotent ingestion - Semantic chunking only Not building: - Retrieval or ranking logic - Agent reasoning - Frontend integration"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Initial Content Ingestion (Priority: P1)

As a developer, I want to trigger a process that fetches all content from the deployed textbook, processes it, and stores it in a vector database, so that the content is ready for use by a Retrieval-Augmented Generation (RAG) system.

**Why this priority**: This is the foundational step required to enable any AI-powered search or question-answering features for the textbook. Without this, there is no data for the AI to work with.

**Independent Test**: Can be fully tested by running the ingestion script/process. The success of the test is verified by checking the vector database to confirm that it contains the expected vectors and metadata corresponding to the textbook content. This delivers the core value of having a queryable knowledge base.

**Acceptance Scenarios**:

1.  **Given** a list of public URLs for a Docusaurus site, **When** the ingestion process is triggered, **Then** the system successfully fetches, chunks, and vectorizes the content from all URLs and stores it in the vector database.
2.  **Given** the content has been ingested, **When** a developer queries the vector database for a known piece of content, **Then** the correct vector and associated metadata (URL, section) are returned.

### User Story 2 - Idempotent Content Updates (Priority: P2)

As a developer, I want to be able to re-run the ingestion process to update the vector database with new or changed content, without creating duplicates or reprocessing unchanged content.

**Why this priority**: Content will inevitably change. This ensures that keeping the AI's knowledge base up-to-date is efficient and reliable, preventing data duplication and drift.

**Independent Test**: Can be tested by running the ingestion process twice. After the second run, the vector database should contain the same number of entries as after the first run (assuming no content has changed), proving idempotency.

**Acceptance Scenarios**:

1.  **Given** an existing vector database with ingested content, **When** the ingestion process is re-run with no changes to the source content, **Then** the number of entries in the vector database remains unchanged.
2.  **Given** an existing vector database and a single updated page on the source site, **When** the ingestion process is re-run, **Then** only the vectors corresponding to the updated page are modified in the database.

### Edge Cases

-   What happens when a URL from the source list is broken (404 Not Found)? The system should log the error and continue processing the remaining URLs.
-   How does the system handle network timeouts when fetching a page? The system should attempt a limited number of retries before logging the failure and moving on.
-   What if a page's HTML structure is unexpected and content extraction fails? The system should log the page URL and the parsing error, then skip to the next page.

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST ingest content from a provided list of Docusaurus URLs.
-   **FR-002**: The system MUST parse HTML from each URL to extract the main textual content, ignoring headers, footers, and navigation sidebars.
-   **FR-003**: The system MUST chunk the extracted text into semantically meaningful sections (e.g., based on headings).
-   **FR-004**: The system MUST generate a vector embedding for each text chunk.
-   **FR-005**: The system MUST store each vector in the designated vector database.
-   **FR-006**: Each stored vector MUST be associated with metadata, including the source URL and the section from which the content was extracted.
-   **FR-007**: The ingestion process MUST be idempotent; re-running it on unchanged content must not create duplicate entries.
-   **FR-008**: The process MUST gracefully handle and log errors for individual pages (e.g., network errors, parsing failures) without halting the entire ingestion process.

### Out of Scope

-   This feature does not include building the retrieval or ranking logic to query the vector database.
-   This feature does not include any agent or reasoning layer that uses the retrieved context.
-   This feature does not include creating any frontend UI for search or display.
-   This feature does not include automatically triggering the ingestion pipeline on content changes; a manual trigger is assumed.

### Constraints

-   **Source Content**: Must originate from publicly accessible, deployed Docusaurus URLs.
-   **Embeddings Model**: Must use a model provided by Cohere.
-   **Vector Database**: Must use Qdrant Cloud.

### Key Entities

-   **Page**: Represents a single document from the Docusaurus site. (Attributes: URL)
-   **Chunk**: A semantic section of text extracted from a Page. (Attributes: text_content, section_heading)
-   **VectorEmbedding**: The numerical representation of a Chunk. (Attributes: vector_data)
-   **Metadata**: Information associated with a VectorEmbedding. (Attributes: source_url, section)

### Assumptions

-   A list of Docusaurus URLs to be ingested will be provided as a configuration or input to the process.
-   Secure credentials for Cohere and Qdrant Cloud will be made available to the ingestion system environment.
-   The HTML structure of the Docusaurus pages is consistent enough to allow for reliable and predictable content extraction.
-   "Semantic chunking" will be based on splitting content by HTML heading levels (e.g., `<h1>`, `<h2>`, etc.) and their subsequent paragraphs.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: 100% of pages from the provided Docusaurus site URLs are processed (either successfully ingested or logged as failed).
-   **SC-002**: A developer can successfully trigger and complete the ingestion process for the entire textbook (approx. 50 pages) in under 15 minutes.
-   **SC-003**: A query against the vector database for a specific, unique phrase from the textbook returns the correct corresponding text chunk and metadata with 100% accuracy.
-   **SC-004**: Re-running the ingestion pipeline on an unchanged site results in zero new vector entries being created in the database.