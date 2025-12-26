"""
Vector search module for the retrieval pipeline.
Handles searching in Qdrant for similar content.
"""

import os
import logging
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Filter


class VectorSearch:
    """
    Service class to handle vector similarity search in Qdrant
    """
    
    def __init__(self):
        url = os.getenv("QDRANT_URL")
        api_key = os.getenv("QDRANT_API_KEY")
        
        if not url or not api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")
        
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            timeout=30
        )
        
        # Use the same collection name as in the ingestion pipeline
        self.collection_name = "rag_embeddings"
        self.logger = logging.getLogger(__name__)

    def search(
        self, 
        query_embedding: List[float], 
        top_k: int = 5, 
        similarity_threshold: float = 0.5
    ) -> List[Dict[str, Any]]:
        """
        Perform a similarity search in Qdrant
        
        Args:
            query_embedding: The embedding vector to search for
            top_k: Number of top results to return
            similarity_threshold: Minimum similarity score threshold
            
        Returns:
            A list of search results with content, metadata, and similarity scores
        """
        self.logger.info(f"Performing vector search with top_k={top_k}, threshold={similarity_threshold}")
        
        try:
            # Perform the search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=similarity_threshold,
                with_payload=True
            )
            
            # Format the results to match our expected structure
            formatted_results = []
            for i, hit in enumerate(search_results):
                result = {
                    "id": hit.id,
                    "content_chunk_id": hit.id,
                    "text": hit.payload.get("text", ""),
                    "url": hit.payload.get("url", ""),
                    "section_title": hit.payload.get("section_title", ""),
                    "similarity_score": hit.score,
                    "rank": i + 1
                }
                formatted_results.append(result)
            
            self.logger.info(f"Found {len(formatted_results)} results from vector search")
            return formatted_results
            
        except Exception as e:
            self.logger.error(f"Error performing vector search: {str(e)}")
            raise