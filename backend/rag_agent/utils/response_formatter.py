import logging
from typing import List
from ..models.response import APIResponse, Citation
from ..models.query import Query


class ResponseFormatter:
    """
    Utility class to format responses consistently
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
    
    def format_api_response(self, 
                           query_id: str, 
                           answer_text: str, 
                           citations: List[Citation], 
                           response_time_ms: int,
                           model_used: str = None) -> APIResponse:
        """
        Format a proper API response
        
        Args:
            query_id: The ID of the original query
            answer_text: The generated answer text
            citations: List of citations to source materials
            response_time_ms: Time taken to process the request in milliseconds
            model_used: Name of the model used to generate the response (optional)
        
        Returns:
            APIResponse object properly formatted
        """
        self.logger.info(f"Formatting API response for query {query_id}")
        
        response = APIResponse(
            id=f"resp_{query_id}",
            answer=answer_text,
            query_id=query_id,
            citations=citations,
            response_time_ms=response_time_ms,
            model_used=model_used
        )
        
        self.logger.info(f"Successfully formatted response with {len(citations)} citations")
        return response