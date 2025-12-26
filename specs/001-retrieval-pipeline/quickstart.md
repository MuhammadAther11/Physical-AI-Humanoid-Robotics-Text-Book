# Quickstart: Retrieval Pipeline & Validation

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key
- Access to the ingested content in Qdrant (from the content ingestion feature)

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
   uv pip install cohere qdrant-client requests python-dotenv
   ```

5. **Set up environment variables**:
   Create a `.env` file in the backend directory with the following content:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

## Running the Retrieval Pipeline

1. **Execute the main script**:
   ```bash
   python main.py
   ```

2. **Or run the retrieval pipeline directly**:
   ```bash
   cd retrieval
   python query_processor.py
   ```

## Example Usage

The retrieval pipeline contains these key components:
- `query_processor.py`: Main query processing logic
- `embedding_service.py`: Cohere embedding generation
- `vector_search.py`: Qdrant search functionality
- `validation.py`: Result validation logic
- `main.py`: Orchestrates the entire flow

## Testing the Pipeline

To test the retrieval pipeline functionality:

1. **Run the test suite**:
   ```bash
   pytest tests/
   ```

2. **Validate retrieval accuracy**:
   - Submit test queries and verify that returned content matches the query intent
   - Check that metadata (URLs and section titles) is preserved
   - Verify that top-k results (default 5-10) maintain contextual accuracy

3. **Performance validation**:
   - Test that query response time is under 1 second for 95% of requests
   - Verify that the system can handle concurrent queries

## Configuration

The pipeline can be configured with:
- `top_k`: Number of results to return (default: 5-10)
- `similarity_threshold`: Minimum similarity score for results
- `query_validation_level`: Level of query validation (basic, moderate, strict)

## Troubleshooting

- **API Key Issues**: Ensure your Cohere and Qdrant API keys are correct and have the necessary permissions
- **Empty Results**: Verify that the content ingestion feature has successfully populated the Qdrant database
- **Performance Issues**: Monitor query response times and adjust the top_k parameter if needed
- **Embedding Issues**: Ensure the query embedding model matches the one used for content ingestion

## Next Steps

After successful validation:
1. The retrieval pipeline can be integrated with the agent reasoning system
2. Consider implementing caching for frequently requested content
3. Set up monitoring for query performance and accuracy metrics