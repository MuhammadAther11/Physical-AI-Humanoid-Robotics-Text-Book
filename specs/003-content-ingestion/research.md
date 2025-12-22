# Research Summary: Content Ingestion & Vector Storage

## Decision: Backend project structure with UV package manager
**Rationale**: UV is a fast Python package manager that will help manage dependencies for the RAG system. The backend approach aligns with the requirement to create a RAG backend for the AI-native textbook.
**Alternatives considered**: Poetry, Pipenv, plain pip - UV was selected for its speed and compatibility with modern Python projects.

## Decision: Cohere API for embeddings
**Rationale**: The feature specification specifically mentions using Cohere models for embeddings. Cohere provides high-quality semantic representations suitable for RAG systems.
**Alternatives considered**: OpenAI embeddings, Hugging Face transformers, Sentence Transformers - Cohere was specified in the original requirements.

## Decision: Qdrant Cloud for vector storage
**Rationale**: The feature specification specifically mentions using Qdrant Cloud for vector storage. Qdrant is a purpose-built vector database with good similarity search performance.
**Alternatives considered**: Pinecone, Weaviate, ChromaDB - Qdrant was specified in the original requirements.

## Decision: Fetch content from Docusaurus site
**Rationale**: The system needs to extract content from deployed Docusaurus sites, which are typically static HTML pages. Using requests/beautifulsoup approach will work well.
**Alternatives considered**: Using Docusaurus API, direct file access - HTML scraping is the most universal approach for deployed sites.

## Decision: Content chunking with 500-800 word size
**Rationale**: Based on clarification session, 500-800 words is optimal for semantic representation models, balancing context retention with processing efficiency.
**Alternatives considered**: Smaller chunks (200-400 words), larger chunks (900-1200 words) - 500-800 was specified in clarifications.

## Decision: Main.py single-file design
**Rationale**: For initial implementation, a single file approach keeps the system simple and focused on core functionality. The design includes all required functions as specified.
**Alternatives considered**: Multi-file modular approach - deferred to later iterations for simplicity in initial implementation.

## Decision: Deploy link processing
**Rationale**: The system will process the provided Netlify deploy link to extract all URLs and content from the deployed Docusaurus site.
**Alternatives considered**: Processing local files, GitHub repository directly - using deployed site is most practical for the RAG system.