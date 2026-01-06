"""
Query endpoint for the RAG Chatbot API
"""
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel
from typing import Optional
import logging
import time
from backend.src.models.query_request import QueryRequest
from backend.src.models.rag_agent_response import RAGAgentResponse
from backend.src.services.rag_agent import rag_agent_service
from backend.src.services.error_handler import handle_error, log_error


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create router
router = APIRouter()


class QueryResponse(BaseModel):
    """
    Standardized response format for the /query endpoint
    """
    answer: str
    status: Optional[str] = "success"
    error: Optional[str] = None


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Process a query through the RAG pipeline:
    1. Retrieve relevant content from the knowledge base
    2. Generate a context-grounded response using the agent
    3. Format and return the response with citations
    """
    start_time = time.time()

    try:
        logger.info(f"Processing query: {request.question[:50]}...")

        # Process the query using the RAG agent service
        response = rag_agent_service.process_query(request)

        # Log successful processing
        response_time = int((time.time() - start_time) * 1000)
        logger.info(f"Query processed successfully in {response_time}ms")

        # Return the answer in the required format
        return QueryResponse(
            answer=response.answer,
            status="success"
        )

    except Exception as e:
        # Handle any errors that occur during query processing
        error_info = handle_error(e, endpoint="/query")

        # Log the error
        logger.error(f"Error processing query: {error_info['error']}")

        # Return a response with the error message instead of raising an HTTPException
        # This prevents the frontend from showing "technical difficulties"
        return QueryResponse(
            answer="I'm experiencing some technical difficulties processing your request. Please try again in a moment.",
            status="error",
            error=error_info['error']
        )


@router.post("/query-full", response_model=RAGAgentResponse)
async def query_full_endpoint(request: QueryRequest):
    """
    Process a query and return the full RAG agent response with all details
    This endpoint provides the complete response including citations, sources, etc.
    """
    start_time = time.time()

    try:
        logger.info(f"Processing full query: {request.question[:50]}...")

        # Process the query using the RAG agent service
        response = rag_agent_service.process_query(request)

        # Log successful processing
        response_time = int((time.time() - start_time) * 1000)
        logger.info(f"Full query processed successfully in {response_time}ms")

        return response

    except Exception as e:
        # Handle any errors that occur during query processing
        error_info = handle_error(e, endpoint="/query-full")

        # Log the error
        logger.error(f"Error processing full query: {error_info['error']}")

        # Return a fallback response with minimal data to prevent errors
        return RAGAgentResponse(
            answer="I'm experiencing some technical difficulties processing your request. Please try again in a moment.",
            context=[],
            sources=[],
            citations=[],
            query_id=f"error_{int(start_time)}",
            confidence_score=0.0,
            response_time_ms=int((time.time() - start_time) * 1000)
        )


@router.get("/health")
async def health_endpoint():
    """
    Health check endpoint for the query service
    """
    try:
        is_healthy = rag_agent_service.health_check()

        if is_healthy:
            return {"status": "healthy", "service": "query_endpoint"}
        else:
            return {"status": "unhealthy", "service": "query_endpoint"}, 503

    except Exception as e:
        error_info = handle_error(e, endpoint="/health")
        logger.error(f"Health check failed: {error_info['error']}")
        return {"status": "error", "service": "query_endpoint", "message": str(e)}, 503