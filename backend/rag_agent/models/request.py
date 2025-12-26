"""
API Request and Response models for the RAG Agent API
Defines the structure for API communication with the frontend
"""

from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import uuid
from datetime import datetime


class APIRequest(BaseModel):
    """
    Model representing an API request to the RAG Agent
    """
    id: str = str(uuid.uuid4())
    query_text: str
    headers: Optional[Dict[str, Any]] = {}
    timestamp: datetime = datetime.now()
    client_info: Optional[Dict[str, Any]] = {}


class APIResponse(BaseModel):
    """
    Model representing an API response from the RAG Agent
    """
    id: str = str(uuid.uuid4())
    answer_id: str
    status_code: int = 200
    response_time_ms: int
    citations: List[Dict[str, Any]] = []
    timestamp: datetime = datetime.now()