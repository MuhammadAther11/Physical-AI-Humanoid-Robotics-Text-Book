"""
Backend Endpoint Response model representing standardized response format for the /query endpoint
"""
from pydantic import BaseModel
from typing import Optional


class BackendEndpointResponse(BaseModel):
    """
    Represents standardized response format for the /query endpoint
    """
    answer: str
    status: str = "success"
    error: Optional[str] = None
    query_id: Optional[str] = None
    response_time_ms: Optional[int] = None