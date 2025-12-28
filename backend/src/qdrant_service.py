from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
import logging
from src.config import Config

logger = logging.getLogger(__name__)

class QdrantService:
    """Service class to handle Qdrant client initialization and connection handling."""
    
    def __init__(self):
        """Initialize the Qdrant client with configuration from environment."""
        try:
            # Initialize Qdrant client
            self.client = QdrantClient(
                url=Config.QDRANT_URL,
                api_key=Config.QDRANT_API_KEY,
                # Additional configuration can be added here
            )
            
            # Test the connection
            self.client.get_collections()
            logger.info("Successfully connected to Qdrant")
            
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {str(e)}")
            raise
    
    def search(self, collection_name: str, query_vector: List[float], limit: int = 10) -> List[models.ScoredPoint]:
        """
        Perform a vector search in the specified collection.
        
        Args:
            collection_name: Name of the collection to search in
            query_vector: Vector to search for
            limit: Maximum number of results to return
            
        Returns:
            List of scored points matching the query
        """
        try:
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit
            )
            return results
        except Exception as e:
            logger.error(f"Search operation failed: {str(e)}")
            raise
    
    def get_collection_info(self, collection_name: str) -> Optional[models.CollectionInfo]:
        """
        Get information about a specific collection.
        
        Args:
            collection_name: Name of the collection to get info for
            
        Returns:
            Collection information or None if collection doesn't exist
        """
        try:
            return self.client.get_collection(collection_name)
        except Exception as e:
            logger.error(f"Failed to get collection info: {str(e)}")
            return None
    
    def health_check(self) -> bool:
        """
        Perform a health check on the Qdrant connection.
        
        Returns:
            True if the connection is healthy, False otherwise
        """
        try:
            # Try to get collections to verify connection
            self.client.get_collections()
            return True
        except Exception:
            return False