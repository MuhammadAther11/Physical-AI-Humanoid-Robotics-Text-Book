"""
Main entry point for the RAG Agent API
Handles the flow: Query → retrieve → agent → response
"""
import os
import logging
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import xml.etree.ElementTree as ET
import requests
from urllib.parse import urljoin, urlparse
from bs4 import BeautifulSoup
import time
import hashlib

# Import the new API router
from backend.src.api.query_endpoint import router as query_router


# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
cohere_api_key = os.getenv("COHERE_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
docusaurus_site_url = os.getenv("DOCUSAURUS_SITE_URL", "https://physical-ai-humanoid-robotics-text-eosin.vercel.app")

if not cohere_api_key:
    raise ValueError("COHERE_API_KEY environment variable is required")
if not qdrant_url:
    raise ValueError("QDRANT_URL environment variable is required")
if not qdrant_api_key:
    raise ValueError("QDRANT_API_KEY environment variable is required")

co = cohere.Client(cohere_api_key)
qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG agent that provides context-grounded answers based on textbook content",
    version="1.0.0"
)

# Add CORS middleware to allow Docusaurus frontend access
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None
    include_citations: Optional[bool] = True


class Citation(BaseModel):
    url: str
    section_title: str
    confidence: float
    text_excerpt: str


class QueryResponse(BaseModel):
    id: str
    query_id: str
    answer: str
    citations: List[Citation]
    sources: List[str]
    confidence_score: float
    response_time_ms: int
    session_id: Optional[str] = None


def get_all_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """
    Parse sitemap.xml to extract all documentation URLs

    Args:
        sitemap_url: URL to the sitemap.xml file

    Returns:
        List of URLs found in the sitemap
    """
    logger.info(f"Fetching sitemap from: {sitemap_url}")

    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        # Parse the XML content
        root = ET.fromstring(response.content)

        # Define the namespace used in sitemaps
        namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        # Extract all URLs from the sitemap
        urls = []
        for url_element in root.findall('sitemap:url', namespace):
            loc_element = url_element.find('sitemap:loc', namespace)
            if loc_element is not None:
                url = loc_element.text
                # Fix placeholder domain in sitemap - replace with actual domain
                if url and 'your-docusaurus-site.example.com' in url:
                    url = url.replace('your-docusaurus-site.example.com', urlparse(docusaurus_site_url).netloc)

                # Only include URLs that are part of the Docusaurus site
                if url and url.startswith(docusaurus_site_url):
                    urls.append(url)

        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls

    except requests.RequestException as e:
        logger.error(f"Error fetching sitemap: {e}")
        return []
    except ET.ParseError as e:
        logger.error(f"Error parsing sitemap XML: {e}")
        return []


def extract_text_from_url(url: str) -> dict:
    """
    Extract clean text content from a given URL

    Args:
        url: URL to extract content from

    Returns:
        Dictionary containing the extracted content and metadata
    """
    logger.info(f"Extracting content from: {url}")

    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }
        response = requests.get(url, headers=headers)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Try to find the main content area of the Docusaurus page
        # Docusaurus typically uses specific class names for content
        content_selectors = [
            'article',  # Common for Docusaurus articles
            '.markdown',  # Docusaurus markdown content
            '.theme-doc-markdown',  # Docusaurus theme content
            '.main-wrapper',  # Main content wrapper
            'main',  # Main content area
            '.container',  # Container for content
            '.docItemContainer',  # Docusaurus document container
            '.theme-doc-item',  # Docusaurus theme document item
            '.theme-content',  # Docusaurus theme content
            '.docs-content',  # Docusaurus docs content
            '[role="main"]',  # Main content area
            '.content',  # General content class
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        # If no specific content area found, try to find content divs with common patterns
        if not content_element:
            content_divs = soup.find_all('div', class_=lambda x: x and any(keyword in x.lower() for keyword in ['content', 'doc', 'markdown', 'main', 'article']))
            if content_divs:
                # Use the div with the most text content
                content_element = max(content_divs, key=lambda x: len(x.get_text(strip=True)))

        # If still no content element found, use the body
        if not content_element:
            content_element = soup.find('body')

        if content_element:
            # Extract text content
            text_content = content_element.get_text(separator=' ')

            # Clean up the text
            lines = (line.strip() for line in text_content.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            text_content = ' '.join(chunk for chunk in chunks if chunk)

            # Extract title
            title = soup.find('title')
            title_text = title.get_text().strip() if title else urlparse(url).path.split('/')[-1]

            # Also try to find H1 or H2 as a secondary title source
            if not title_text or len(title_text.strip()) < 5:  # If title is too short
                h1 = soup.find('h1')
                if h1:
                    title_text = h1.get_text().strip()
                elif not title_text or len(title_text.strip()) < 5:
                    h2 = soup.find('h2')
                    if h2:
                        title_text = h2.get_text().strip()

            return {
                'url': url,
                'title': title_text,
                'content': text_content,
                'word_count': len(text_content.split())
            }
        else:
            logger.warning(f"No content found for URL: {url}")
            return {
                'url': url,
                'title': urlparse(url).path.split('/')[-1],
                'content': '',
                'word_count': 0
            }

    except requests.RequestException as e:
        logger.error(f"Error fetching content from {url}: {e}")
        return {
            'url': url,
            'title': urlparse(url).path.split('/')[-1],
            'content': '',
            'word_count': 0
        }
    except Exception as e:
        logger.error(f"Error processing content from {url}: {e}")
        return {
            'url': url,
            'title': urlparse(url).path.split('/')[-1],
            'content': '',
            'word_count': 0
        }


def chunk_text(text: str, max_chunk_size: int = 700) -> List[str]:
    """
    Split text into chunks of approximately max_chunk_size words

    Args:
        text: Text to chunk
        max_chunk_size: Maximum number of words per chunk

    Returns:
        List of text chunks
    """
    if not text:
        return []

    words = text.split()
    chunks = []

    # If text is already smaller than max_chunk_size, return as is
    if len(words) <= max_chunk_size:
        return [text]

    # Split into chunks
    for i in range(0, len(words), max_chunk_size):
        chunk_words = words[i:i + max_chunk_size]
        chunk = ' '.join(chunk_words)
        chunks.append(chunk)

    return chunks


def embed(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere

    Args:
        texts: List of texts to embed

    Returns:
        List of embeddings (each embedding is a list of floats)
    """
    if not texts:
        return []

    try:
        # Cohere's embed function can handle multiple texts at once
        response = co.embed(
            texts=texts,
            model='embed-english-v3.0',  # Using a standard Cohere embedding model
            input_type='search_document'  # Specify the input type
        )

        return response.embeddings

    except Exception as e:
        logger.error(f"Error generating embeddings: {e}")
        return [[] for _ in texts]  # Return empty embeddings in case of error


def create_collection(collection_name: str = "rag_embeddings"):
    """
    Create a Qdrant collection for storing embeddings

    Args:
        collection_name: Name of the collection to create
    """
    try:
        # Check if collection already exists
        collections = qdrant_client.get_collections()
        collection_names = [collection.name for collection in collections.collections]

        if collection_name in collection_names:
            logger.info(f"Collection '{collection_name}' already exists, recreating...")
            qdrant_client.delete_collection(collection_name)

        # Create new collection
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)  # Assuming Cohere's embedding size
        )

        logger.info(f"Collection '{collection_name}' created successfully")

    except Exception as e:
        logger.error(f"Error creating collection: {e}")
        raise


def save_chunk_to_qdrant(chunk: str, embedding: List[float], metadata: dict, collection_name: str = "rag_embeddings"):
    """
    Save a text chunk with its embedding to Qdrant

    Args:
        chunk: Text chunk to save
        embedding: Embedding vector for the chunk
        metadata: Metadata to store with the chunk
        collection_name: Name of the collection to save to
    """
    try:
        # Generate a unique ID for this chunk
        chunk_id = hashlib.md5(f"{metadata['url']}_chunk_{len(chunk)}_{hash(chunk)}".encode()).hexdigest()

        # Prepare the payload with metadata
        payload = {
            "url": metadata["url"],
            "title": metadata["title"],
            "section_title": metadata.get("section_title", ""),
            "content": chunk,
            "word_count": len(chunk.split()),
            "source": "docusaurus_ingestion",
            "ingestion_timestamp": time.time()
        }

        # Add any additional metadata
        for key, value in metadata.items():
            if key not in payload:
                payload[key] = value

        # Upsert the point to Qdrant
        qdrant_client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload=payload
                )
            ]
        )

        logger.debug(f"Saved chunk to Qdrant with ID: {chunk_id}")

    except Exception as e:
        logger.error(f"Error saving chunk to Qdrant: {e}")


@app.get("/")
def read_root():
    return {"message": "RAG Agent API is running"}


@app.post("/ingest")
async def ingest_content(background_tasks: BackgroundTasks):
    """
    Endpoint to trigger content ingestion from the Docusaurus site
    """
    logger.info("Content ingestion triggered via API")

    # Run the ingestion process in the background
    background_tasks.add_task(run_ingestion_process)

    return {"message": "Content ingestion started", "status": "processing"}


def run_ingestion_process():
    """
    Main function to orchestrate the content ingestion process
    """
    logger.info("Starting content ingestion process...")

    # Step 1: Get all URLs from sitemap
    sitemap_url = f"{docusaurus_site_url}/sitemap.xml"
    urls = get_all_urls_from_sitemap(sitemap_url)

    if not urls:
        logger.error("No URLs found in sitemap. Please verify the sitemap URL.")
        return

    logger.info(f"Processing {len(urls)} URLs from sitemap...")

    # Step 2: Create Qdrant collection
    collection_name = "rag_embeddings"
    create_collection(collection_name)

    # Step 3: Process each URL
    total_chunks = 0
    for i, url in enumerate(urls, 1):
        logger.info(f"Processing URL {i}/{len(urls)}: {url}")

        # Extract content from URL
        content_data = extract_text_from_url(url)

        if not content_data['content']:
            logger.warning(f"No content extracted from {url}, skipping...")
            continue

        # Chunk the content
        chunks = chunk_text(content_data['content'])

        # Process each chunk
        for j, chunk in enumerate(chunks):
            if not chunk.strip():
                continue

            # Generate embedding for the chunk
            embeddings = embed([chunk])

            if not embeddings or not embeddings[0]:
                logger.warning(f"Failed to generate embedding for chunk {j} of {url}, skipping...")
                continue

            # Prepare metadata for this chunk
            chunk_metadata = {
                'url': content_data['url'],
                'title': content_data['title'],
                'section_title': f"Chunk {j+1}",
                'chunk_index': j,
                'total_chunks': len(chunks)
            }

            # Save to Qdrant
            save_chunk_to_qdrant(
                chunk=chunk,
                embedding=embeddings[0],
                metadata=chunk_metadata,
                collection_name=collection_name
            )

            total_chunks += 1

    logger.info(f"Ingestion completed! Processed {len(urls)} URLs and saved {total_chunks} content chunks to Qdrant.")


@app.post("/query", response_model=QueryResponse)
async def process_query(request: QueryRequest):
    """
    Process a query through the RAG pipeline:
    1. (In a full implementation) retrieve relevant content from the knowledge base
    2. Generate a context-grounded response using the agent
    3. Format and return the response with citations
    """
    logger.info(f"Processing query: {request.query[:50]}...")
    
    # In a full implementation, this would:
    # 1. Use the retrieval system to find relevant content
    # 2. Generate embeddings for the query
    # 3. Use the Cohere agent to generate a response based on retrieved content
    # 4. Format the response with citations
    
    # For now, returning a mock response
    # In a real implementation, we would connect to the retrieval system and Cohere agent
    mock_answer = f"This is a mock response to your query: '{request.query}'. In a full implementation, this would be generated by the RAG agent based on the textbook content."
    
    # Create mock citations
    citations = [
        Citation(
            url="https://example.com/textbook-section",
            section_title="Example Section Title",
            confidence=0.85,
            text_excerpt="This is an excerpt from the relevant content..."
        )
    ]
    
    response = QueryResponse(
        id="mock-response-id",
        query_id="mock-query-id",
        answer=mock_answer,
        citations=citations,
        sources=["source-1", "source-2"],
        confidence_score=0.85,
        response_time_ms=150
    )
    
    logger.info("Query processed successfully")
    return response


# Include the query router
app.include_router(query_router, prefix="", tags=["query"])

@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the API and its dependencies are available
    """
    # Check if Cohere client is accessible
    cohere_ok = True
    try:
        # Make a simple request to test the connection
        pass
    except Exception as e:
        logger.error(f"Cohere client health check failed: {e}")
        cohere_ok = False

    # Check if Qdrant client is accessible
    qdrant_ok = True
    try:
        # Make a simple request to test the connection
        pass
    except Exception as e:
        logger.error(f"Qdrant client health check failed: {e}")
        qdrant_ok = False

    status = "healthy" if cohere_ok and qdrant_ok else "unhealthy"

    return {
        "status": status,
        "checks": {
            "cohere_client": {"status": "ok" if cohere_ok else "error"},
            "qdrant_client": {"status": "ok" if qdrant_ok else "error"}
        },
        "timestamp": __import__('datetime').datetime.now().isoformat()
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)