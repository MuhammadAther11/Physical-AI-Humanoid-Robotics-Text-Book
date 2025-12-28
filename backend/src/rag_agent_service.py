from typing import Dict, Any, List, Optional
from src.openai_service import OpenAIService
from src.qdrant_service import QdrantService
from src.models.query import Query
from src.models.response import Response, Source
from src.validation_service import ValidationService
import logging
import asyncio
from datetime import datetime

logger = logging.getLogger(__name__)

class RAGAgentService:
    """Service class to handle RAG (Retrieval-Augmented Generation) agent functionality."""

    def __init__(self):
        """Initialize the RAG agent with required services."""
        self.openai_service = OpenAIService()
        self.qdrant_service = QdrantService()
        self.validation_service = ValidationService()

    def process_query(self, query: Query) -> Response:
        """
        Process a user query using the RAG approach.

        Args:
            query: The user's query

        Returns:
            Response containing the agent's answer and sources
        """
        try:
            # Validate the query
            self.validation_service.validate_query(query)

            # Retrieve relevant context from Qdrant
            context = self._retrieve_context(query.content)

            # Generate response using OpenAI with timeout handling
            response_data = self.openai_service.generate_response(
                prompt=query.content,
                context=context
            )

            # Create response object
            response = Response(
                id=f"response-{query.id}",
                queryId=query.id,
                content=response_data["response"],
                sources=self._format_sources(context) if context else None,
                metadata={
                    "model_used": response_data["model"],
                    "usage": response_data["usage"]
                }
            )

            # Validate the response
            self.validation_service.validate_response(response)

            return response

        except Exception as e:
            logger.error(f"Error processing query {query.id}: {str(e)}")
            # Return a default response when the RAG agent is unavailable
            return self._create_error_response(query.id, str(e))

    def process_query_with_timeout(self, query: Query, timeout_seconds: int = 30) -> Response:
        """
        Process a user query with a specific timeout to handle long-running requests.

        Args:
            query: The user's query
            timeout_seconds: Maximum time to wait for a response

        Returns:
            Response containing the agent's answer and sources
        """
        import signal

        def timeout_handler(signum, frame):
            raise TimeoutError(f"Query processing timed out after {timeout_seconds} seconds")

        # Set up the timeout mechanism
        old_handler = signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(timeout_seconds)

        try:
            result = self.process_query(query)
            return result
        except TimeoutError:
            logger.error(f"Query {query.id} timed out after {timeout_seconds} seconds")
            return self._create_error_response(
                query.id,
                f"Request timed out after {timeout_seconds} seconds"
            )
        except Exception as e:
            logger.error(f"Error processing query {query.id} with timeout: {str(e)}")
            return self._create_error_response(query.id, str(e))
        finally:
            # Cancel the alarm
            signal.alarm(0)
            signal.signal(signal.SIGALRM, old_handler)

    def _create_error_response(self, query_id: str, error_message: str) -> Response:
        """
        Create a default error response when the RAG agent is unavailable.

        Args:
            query_id: The ID of the original query
            error_message: The error message to include in the response

        Returns:
            A default response indicating an error occurred
        """
        logger.warning(f"Returning error response for query {query_id}: {error_message}")

        return Response(
            id=f"error-response-{query_id}",
            queryId=query_id,
            content="Sorry, I'm currently experiencing technical difficulties. Please try again later.",
            timestamp=datetime.utcnow(),
            sources=None,
            metadata={
                "error": error_message,
                "fallback_response": True
            }
        )

    async def process_query_with_timeout(self, query: Query, timeout_seconds: int = 30) -> Response:
        """
        Process a query with a timeout to handle cases where the RAG agent takes too long.

        Args:
            query: The user's query
            timeout_seconds: Maximum time to wait for a response

        Returns:
            Response containing the agent's answer and sources, or an error response
        """
        try:
            # Use asyncio.wait_for to implement timeout
            response = await asyncio.wait_for(
                asyncio.to_thread(self.process_query, query),
                timeout=timeout_seconds
            )
            return response
        except asyncio.TimeoutError:
            logger.error(f"Query {query.id} timed out after {timeout_seconds} seconds")
            return self._create_error_response(
                query.id,
                f"Request timed out after {timeout_seconds} seconds"
            )
        except Exception as e:
            logger.error(f"Error processing query {query.id} with timeout: {str(e)}")
            return self._create_error_response(query.id, str(e))
    
    def _retrieve_context(self, query_text: str) -> Optional[List[Dict[str, str]]]:
        """
        Retrieve relevant context from the Qdrant vector database.
        
        Args:
            query_text: The text to search for in the vector database
            
        Returns:
            List of context documents or None if retrieval fails
        """
        try:
            # This is a simplified implementation
            # In a real implementation, you would:
            # 1. Convert the query_text to an embedding
            # 2. Search the Qdrant database for similar vectors
            # 3. Return the relevant documents
            
            # For now, returning a mock context
            # In a real implementation, you would use the qdrant_service
            # to perform the actual vector search
            logger.info(f"Retrieving context for query: {query_text[:50]}...")
            
            # Placeholder implementation - in a real system, this would
            # perform actual vector search in Qdrant
            return [
                {
                    "title": "Humanoid Robotics Documentation",
                    "url": "https://example.com/humanoid-robotics",
                    "content": "Information about humanoid robots and their control systems..."
                }
            ]
            
        except Exception as e:
            logger.error(f"Error retrieving context: {str(e)}")
            return None
    
    def _format_sources(self, context: List[Dict[str, str]]) -> List[Source]:
        """
        Format context documents as Source objects.
        
        Args:
            context: List of context documents
            
        Returns:
            List of Source objects
        """
        sources = []
        for item in context:
            source = Source(
                title=item.get("title", ""),
                url=item.get("url", ""),
                excerpt=item.get("content", "")
            )
            sources.append(source)
        return sources
    
    def health_check(self) -> bool:
        """
        Perform a health check on the RAG agent services.
        
        Returns:
            True if all services are healthy, False otherwise
        """
        try:
            # Check if OpenAI service is working
            openai_ok = self.openai_service.validate_api_key()
            
            # Check if Qdrant service is working
            qdrant_ok = self.qdrant_service.health_check()
            
            return openai_ok and qdrant_ok
        except Exception as e:
            logger.error(f"Health check failed: {str(e)}")
            return False