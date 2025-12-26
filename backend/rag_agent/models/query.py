"""
Query Request model for the RAG Agent API
Defines the structure for incoming query requests from the frontend
"""

from pydantic import BaseModel
from typing import Optional
import uuid
from datetime import datetime


class QueryRequest(BaseModel):
    """
    Model representing a query request from the frontend
    """
    id: str = str(uuid.uuid4())
    text: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None
    metadata: Optional[dict] = {}
    timestamp: datetime = datetime.now()


class QueryResponse(BaseModel):
    """
    Model representing a response to a query
    """
    id: str = str(uuid.uuid4())
    query_id: str
    text: str
    citations: list = []
    sources: list = []
    confidence_score: float = 0.0
    token_usage: Optional[dict] = {}
    timestamp: datetime = datetime.now()


class SessionContext(BaseModel):
    """
    Model representing session context for maintaining conversation state
    """
    id: str = str(uuid.uuid4())
    user_id: Optional[str] = None
    created_at: datetime = datetime.now()
    last_activity: datetime = datetime.now()
    query_history: list = []
    preferences: dict = {}