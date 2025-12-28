from typing import Dict, Any
from src.models.query import Query
from src.models.response import Response
from src.rag_agent_service import RAGAgentService
from src.validation_service import ValidationService
import uuid
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class QueryProcessingService:
    """Service class to handle the complete query processing pipeline."""
    
    def __init__(self):
        """Initialize the query processing service with required dependencies."""
        self.rag_agent_service = RAGAgentService()
        self.validation_service = ValidationService()
    
    def process_query(self, query_data: Dict[str, Any]) -> Response:
        """
        Process a query through the complete pipeline.
        
        Args:
            query_data: Dictionary containing query information
                      Expected keys: 'query', 'userId', 'context'
                      
        Returns:
            Response object with the agent's answer
        """
        try:
            # Generate a unique ID for this query
            query_id = str(uuid.uuid4())
            
            # Create a Query object from the input data
            query = Query(
                id=query_id,
                content=self.validation_service.sanitize_query_content(query_data.get('query', '')),
                userId=query_data.get('userId'),
                metadata=query_data.get('context')
            )
            
            # Validate the query
            self.validation_service.validate_query(query)
            
            # Process the query through the RAG agent
            response = self.rag_agent_service.process_query(query)
            
            return response
            
        except ValueError as ve:
            logger.error(f"Validation error processing query: {str(ve)}")
            raise
        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            raise
    
    def validate_and_format_query(self, raw_query: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate and format the incoming query data.
        
        Args:
            raw_query: Raw query data from the API request
            
        Returns:
            Formatted and validated query data
        """
        # Check if query is provided
        if not raw_query.get('query'):
            raise ValueError("Query content is required")
        
        # Sanitize the query content
        sanitized_query = self.validation_service.sanitize_query_content(raw_query['query'])
        
        # Check query length
        if len(sanitized_query) > 2000:
            raise ValueError("Query exceeds maximum length of 2000 characters")
        
        # Format the query data
        formatted_query = {
            'query': sanitized_query,
            'userId': raw_query.get('userId'),
            'context': raw_query.get('context', {})
        }
        
        return formatted_query
    
    def health_check(self) -> bool:
        """
        Perform a health check on the query processing service.
        
        Returns:
            True if the service is healthy, False otherwise
        """
        try:
            return self.rag_agent_service.health_check()
        except Exception as e:
            logger.error(f"Health check failed: {str(e)}")
            return False