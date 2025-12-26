"""
Query processor module for the retrieval pipeline.
Handles the flow: Query → embed → Qdrant search → top-k results → validation
"""

import logging
from typing import List, Dict, Any, Optional
from .embedding_service import EmbeddingService
from .vector_search import VectorSearch
from .validation import ValidationResult, validate_results


class QueryProcessor:
    """
    Main class to handle query processing from input to validated results
    """
    
    def __init__(self, embedding_service: EmbeddingService, vector_search: VectorSearch):
        self.embedding_service = embedding_service
        self.vector_search = vector_search
        self.logger = logging.getLogger(__name__)

    def process_query(
        self, 
        query_text: str, 
        top_k: int = 5, 
        similarity_threshold: float = 0.5
    ) -> ValidationResult:
        """
        Process a user query through the full pipeline
        
        Args:
            query_text: The user's query text
            top_k: Number of top results to return (default: 5-10 based on spec)
            similarity_threshold: Minimum similarity score threshold
        
        Returns:
            ValidationResult containing the search results and validation status
        """
        self.logger.info(f"Processing query: {query_text[:50]}...")
        
        # Step 1: Validate query format with basic validation (length limits, special character filtering)
        if not self._validate_query_format(query_text):
            error_msg = "Query failed basic validation (length or character filtering)"
            self.logger.warning(error_msg)
            return ValidationResult(
                query_id=None,
                results=[],
                is_valid=False,
                errors=[error_msg]
            )
        
        try:
            # Step 2: Convert query to embedding
            query_embedding = self.embedding_service.generate_embedding(query_text)
            
            # Step 3: Perform similarity search in Qdrant
            search_results = self.vector_search.search(
                query_embedding, 
                top_k=top_k, 
                similarity_threshold=similarity_threshold
            )
            
            # Step 4: Validate results for relevance and metadata preservation
            validation_result = validate_results(
                query_text=query_text,
                search_results=search_results,
                top_k=top_k
            )
            
            self.logger.info(f"Query processed successfully, returning {len(validation_result.results)} results")
            return validation_result
            
        except Exception as e:
            error_msg = f"Error processing query: {str(e)}"
            self.logger.error(error_msg)
            # Continue with warnings approach as specified in clarifications
            return ValidationResult(
                query_id=None,
                results=[],
                is_valid=False,
                errors=[error_msg]
            )

    def _validate_query_format(self, query_text: str) -> bool:
        """
        Perform basic validation on query format
        - Length limits
        - Special character filtering
        """
        # Length validation: queries should be between 5 and 500 characters
        if len(query_text) < 5 or len(query_text) > 500:
            self.logger.warning(f"Query length {len(query_text)} is outside acceptable range (5-500)")
            return False
        
        # Basic special character filtering (example: block certain control characters)
        # In a real implementation, this would be more sophisticated
        if '\x00' in query_text:  # Null bytes
            self.logger.warning("Query contains null bytes, rejecting")
            return False
        
        return True