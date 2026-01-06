"""
Error handling infrastructure for the RAG Chatbot backend
"""
import logging
from typing import Optional
from fastapi import HTTPException, status
from pydantic import BaseModel


class ErrorLogEntry(BaseModel):
    """
    Represents a logged error for debugging purposes
    """
    timestamp: str
    level: str
    message: str
    stack_trace: Optional[str] = None
    endpoint: Optional[str] = None
    error_code: Optional[str] = None


class RAGError(Exception):
    """
    Base exception class for RAG-related errors
    """
    def __init__(self, message: str, error_code: str = "RAG_ERROR", status_code: int = 500):
        self.message = message
        self.error_code = error_code
        self.status_code = status_code
        super().__init__(self.message)


class QueryProcessingError(RAGError):
    """
    Exception raised when query processing fails
    """
    def __init__(self, message: str = "Query processing failed", status_code: int = 500):
        super().__init__(message, "QUERY_PROCESSING_ERROR", status_code)


class RetrievalError(RAGError):
    """
    Exception raised when content retrieval fails
    """
    def __init__(self, message: str = "Content retrieval failed", status_code: int = 500):
        super().__init__(message, "RETRIEVAL_ERROR", status_code)


class AgentError(RAGError):
    """
    Exception raised when the RAG agent fails
    """
    def __init__(self, message: str = "RAG agent processing failed", status_code: int = 500):
        super().__init__(message, "AGENT_ERROR", status_code)


class ValidationError(RAGError):
    """
    Exception raised when validation fails
    """
    def __init__(self, message: str = "Validation failed", status_code: int = 400):
        super().__init__(message, "VALIDATION_ERROR", status_code)


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def log_error(message: str, level: str = "ERROR", stack_trace: Optional[str] = None, endpoint: Optional[str] = None):
    """
    Log an error with proper formatting
    """
    import datetime
    timestamp = datetime.datetime.now().isoformat()

    error_entry = ErrorLogEntry(
        timestamp=timestamp,
        level=level,
        message=message,
        stack_trace=stack_trace,
        endpoint=endpoint
    )

    # Log to console/file
    logger.log(
        logging.getLevelName(level.upper()),
        f"[{error_entry.timestamp}] {error_entry.level}: {error_entry.message}"
    )

    # In a real implementation, you might also save this to a database or external logging service
    return error_entry


def handle_error(exception: Exception, endpoint: str = None, default_status_code: int = 500):
    """
    Centralized error handling function
    """
    import traceback

    # Determine the status code based on the exception type
    if isinstance(exception, RAGError):
        status_code = exception.status_code
        error_message = exception.message
        error_code = exception.error_code
    elif isinstance(exception, HTTPException):
        status_code = exception.status_code
        error_message = str(exception.detail)
        error_code = f"HTTP_{status_code}"
    else:
        status_code = default_status_code
        error_message = f"An unexpected error occurred: {str(exception)}"
        error_code = "UNEXPECTED_ERROR"

    # Log the error
    stack_trace = traceback.format_exc() if status_code >= 500 else None
    log_error(
        message=error_message,
        level="ERROR" if status_code >= 500 else "WARNING",
        stack_trace=stack_trace,
        endpoint=endpoint
    )

    # Return error response structure
    return {
        "error": error_message,
        "error_code": error_code,
        "status_code": status_code
    }


def validate_query_input(query: str) -> bool:
    """
    Validate query input to prevent common issues
    """
    if not query or not query.strip():
        raise ValidationError("Query cannot be empty")

    if len(query.strip()) < 3:
        raise ValidationError("Query must be at least 3 characters long")

    if len(query) > 1000:  # Arbitrary limit to prevent very large queries
        raise ValidationError("Query is too long (max 1000 characters)")

    return True