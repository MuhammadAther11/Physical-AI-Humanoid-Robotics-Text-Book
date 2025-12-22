import os
import re
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import time
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=30
)

def get_all_urls(base_url: str) -> List[str]:
    """
    Fetch all URLs from the deployed Docusaurus site
    """
    logger.info(f"Fetching URLs from: {base_url}")
    
    try:
        response = requests.get(base_url)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.text, 'html.parser')
        links = soup.find_all('a', href=True)
        
        urls = set()
        for link in links:
            href = link['href']
            full_url = urljoin(base_url, href)
            
            # Only include URLs from the same domain
            if urlparse(full_url).netloc == urlparse(base_url).netloc:
                urls.add(full_url)
        
        # Also try to find links in the sitemap if it exists
        sitemap_url = urljoin(base_url, "sitemap.xml")
        try:
            sitemap_response = requests.get(sitemap_url)
            if sitemap_response.status_code == 200:
                sitemap_soup = BeautifulSoup(sitemap_response.text, 'xml')
                for loc in sitemap_soup.find_all('loc'):
                    url = loc.text.strip()
                    if urlparse(url).netloc == urlparse(base_url).netloc:
                        urls.add(url)
        except Exception as e:
            logger.warning(f"Could not fetch sitemap: {e}")
        
        logger.info(f"Found {len(urls)} URLs")
        return list(urls)
    
    except Exception as e:
        logger.error(f"Error fetching URLs: {e}")
        return []

def extract_text_from_url(url: str) -> str:
    """
    Extract clean text from a given URL
    """
    try:
        logger.info(f"Extracting text from: {url}")
        response = requests.get(url)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.text, 'html.parser')
        
        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()
        
        # Get main content - try common Docusaurus content selectors
        content_selectors = [
            'main div[class*="docItemContainer"]',
            'article',
            'main',
            'div.main-wrapper',
            'div.container',
            'div#__docusaurus',
            '.markdown',
            '.theme-doc-markdown'
        ]
        
        text_content = ""
        for selector in content_selectors:
            elements = soup.select(selector)
            if elements:
                for element in elements:
                    text_content += element.get_text(separator=' ', strip=True) + "\n\n"
                break
        
        # If no content found with selectors, get all text
        if not text_content.strip():
            text_content = soup.get_text(separator=' ', strip=True)
        
        # Clean up the text
        lines = (line.strip() for line in text_content.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text_content = ' '.join(chunk for chunk in chunks if chunk)
        
        logger.info(f"Extracted {len(text_content)} characters from {url}")
        return text_content
    
    except Exception as e:
        logger.error(f"Error extracting text from {url}: {e}")
        return ""

def chunk_text(text: str, min_chunk_size: int = 500, max_chunk_size: int = 800) -> List[Dict[str, Any]]:
    """
    Split text into 500-800 word chunks
    """
    logger.info(f"Chunking text of {len(text)} characters")
    
    # Split text into sentences
    sentences = re.split(r'[.!?]+\s+', text)
    chunks = []
    current_chunk = ""
    current_word_count = 0
    
    for sentence in sentences:
        # Count words in sentence
        sentence_word_count = len(sentence.split())
        
        # If adding this sentence would exceed max chunk size
        if current_word_count + sentence_word_count > max_chunk_size and current_chunk:
            # Save current chunk if it meets minimum size
            if current_word_count >= min_chunk_size:
                chunks.append({
                    'text': current_chunk.strip(),
                    'word_count': current_word_count
                })
                current_chunk = sentence
                current_word_count = sentence_word_count
            else:
                # Add to current chunk anyway (even if it exceeds max)
                current_chunk += " " + sentence
                current_word_count += sentence_word_count
        else:
            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence
            current_word_count += sentence_word_count
    
    # Add the last chunk if it meets minimum size
    if current_chunk and current_word_count >= min_chunk_size:
        chunks.append({
            'text': current_chunk.strip(),
            'word_count': current_word_count
        })
    elif current_chunk and len(chunks) == 0:
        # If the entire text is smaller than min_chunk_size, add it anyway
        chunks.append({
            'text': current_chunk.strip(),
            'word_count': current_word_count
        })
    
    logger.info(f"Created {len(chunks)} chunks")
    return chunks

def embed(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings using Cohere
    """
    logger.info(f"Generating embeddings for {len(texts)} text chunks")
    
    try:
        response = co.embed(
            texts=texts,
            model='embed-multilingual-v2.0'  # Using Cohere's multilingual model
        )
        
        embeddings = [item for item in response.embeddings]
        logger.info(f"Generated {len(embeddings)} embeddings")
        return embeddings
    
    except Exception as e:
        logger.error(f"Error generating embeddings: {e}")
        return [[] for _ in texts]  # Return empty embeddings in case of error

def create_collection(collection_name: str = "rag_embeddings"):
    """
    Create a Qdrant collection for storing embeddings
    """
    logger.info(f"Creating Qdrant collection: {collection_name}")
    
    try:
        # Check if collection already exists
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]
        
        if collection_name in collection_names:
            logger.info(f"Collection {collection_name} already exists, using existing collection")
            return
        
        # Create new collection
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE)
        )
        
        logger.info(f"Successfully created collection: {collection_name}")
        
    except Exception as e:
        logger.error(f"Error creating collection: {e}")

def save_chunk_to_qdrant(chunk: Dict[str, Any], embedding: List[float], url: str, collection_name: str = "rag_embeddings"):
    """
    Save a text chunk and its embedding to Qdrant with metadata
    """
    try:
        # Create a unique ID for this chunk
        import hashlib
        chunk_id = hashlib.md5((url + chunk['text'][:100]).encode()).hexdigest()
        
        # Prepare the point to insert
        point = models.PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={
                "text": chunk['text'],
                "url": url,
                "word_count": chunk['word_count'],
                "created_at": time.time()
            }
        )
        
        # Insert the point into the collection
        qdrant_client.upsert(
            collection_name=collection_name,
            points=[point]
        )
        
        logger.info(f"Saved chunk to Qdrant: {chunk_id[:8]}... from {url}")
        
    except Exception as e:
        logger.error(f"Error saving chunk to Qdrant: {e}")

def main():
    """
    Main function to orchestrate the ingestion process
    """
    logger.info("Starting content ingestion process")
    
    # Get the Docusaurus site URL from environment or use default
    site_url = os.getenv("DOCUSAURUS_SITE_URL", "https://6946f8b2df180536939dad08--dulcet-dolphin-395d63.netlify.app/")
    
    if not os.getenv("COHERE_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
        logger.error("Missing required environment variables. Please set COHERE_API_KEY, QDRANT_URL, and QDRANT_API_KEY")
        return
    
    # Step 1: Get all URLs from the site
    urls = get_all_urls(site_url)
    if not urls:
        logger.error("No URLs found, exiting")
        return
    
    # Step 2: Create Qdrant collection
    create_collection()
    
    # Process each URL
    processed_count = 0
    successful_count = 0
    failed_count = 0
    
    for url in urls:
        try:
            logger.info(f"Processing URL: {url}")
            
            # Step 3: Extract text from URL
            text = extract_text_from_url(url)
            if not text.strip():
                logger.warning(f"No text extracted from {url}, skipping")
                failed_count += 1
                continue
            
            # Step 4: Chunk the text
            chunks = chunk_text(text)
            if not chunks:
                logger.warning(f"No valid chunks created from {url}, skipping")
                failed_count += 1
                continue
            
            # Step 5: Generate embeddings for chunks
            chunk_texts = [chunk['text'] for chunk in chunks]
            embeddings = embed(chunk_texts)
            
            # Step 6: Save chunks and embeddings to Qdrant
            for chunk, embedding in zip(chunks, embeddings):
                if embedding:  # Only save if embedding was generated successfully
                    save_chunk_to_qdrant(chunk, embedding, url)
                    successful_count += 1
                else:
                    logger.warning(f"Failed to generate embedding for chunk from {url}")
                    failed_count += 1
            
            processed_count += 1
            
        except Exception as e:
            logger.error(f"Error processing {url}: {e}")
            failed_count += 1
            continue
    
    logger.info(f"Ingestion process completed. Processed: {processed_count}, Successful: {successful_count}, Failed: {failed_count}")

if __name__ == "__main__":
    main()