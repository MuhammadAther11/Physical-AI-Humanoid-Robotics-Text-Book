# Quickstart: Content Ingestion & Vector Storage

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key
- Access to a deployed Docusaurus site (or the example site)

## Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

3. **Install UV package manager** (if not already installed):
   ```bash
   pip install uv
   ```

4. **Create a virtual environment and install dependencies**:
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install cohere qdrant-client requests beautifulsoup4 python-dotenv
   ```

5. **Set up environment variables**:
   Create a `.env` file in the backend directory with the following content:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   DOCUSAURUS_SITE_URL=https://6946f8b2df180536939dad08--dulcet-dolphin-395d63.netlify.app/
   ```

   For the example site, you can use:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=https://your-cluster-url.qdrant.tech
   QDRANT_API_KEY=your_qdrant_api_key_here
   DOCUSAURUS_SITE_URL=https://6946f8b2df180536939dad08--dulcet-dolphin-395d63.netlify.app/
   ```

## Running the Ingestion

1. **Execute the main script**:
   ```bash
   python main.py
   ```

2. **Monitor the output**:
   The script will:
   - Fetch all URLs from the specified Docusaurus site
   - Extract text content from each page
   - Chunk the content into 500-800 word segments
   - Generate embeddings using Cohere
   - Create a Qdrant collection named "rag_embeddings"
   - Store the embeddings with metadata in Qdrant

## Example Usage

The main script contains these key functions:
- `get_all_urls()`: Fetches all URLs from the Docusaurus site
- `extract_text_from_url()`: Extracts clean text from a given URL
- `chunk_text()`: Splits text into 500-800 word chunks
- `embed()`: Generates embeddings using Cohere
- `create_collection()`: Creates the Qdrant collection
- `save_chunk_to_qdrant()`: Stores the chunk and embedding in Qdrant
- `main()`: Orchestrates the entire process

## Troubleshooting

- **API Key Issues**: Ensure your Cohere and Qdrant API keys are correct and have the necessary permissions
- **URL Access Issues**: Verify that the Docusaurus site URL is accessible and properly formatted
- **Memory Issues**: For large sites, the process may require significant memory; consider processing in batches
- **Embedding Limits**: Be aware of Cohere's rate limits and token usage when processing large amounts of text

## Next Steps

After successful ingestion:
1. The content will be available in Qdrant for RAG operations
2. You can implement retrieval logic to search and access the stored content
3. Consider setting up scheduled updates to keep content current