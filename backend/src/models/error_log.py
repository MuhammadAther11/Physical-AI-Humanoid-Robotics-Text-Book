"""
Error Log Entry model representing logged errors for debugging purposes
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class ErrorLogEntry(BaseModel):
    """
    Represents a logged error for debugging purposes
    """
    timestamp: datetime
    level: str  # e.g., "ERROR", "WARNING", "INFO"
    message: str
    stack_trace: Optional[str] = None
    endpoint: Optional[str] = None
    error_code: Optional[str] = None
    query_id: Optional[str] = None