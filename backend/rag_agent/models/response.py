from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime


class Citation(BaseModel):
    """
    Model representing a citation in the response
    """
    url: str
    section_title: str
    confidence: float


class TokenUsage(BaseModel):
    """
    Model representing token usage information
    """
    input_tokens: int
    output_tokens: int
    total_tokens: int


class ContextGroundedAnswer(BaseModel):
    """
    Model representing a context-grounded answer from the RAG agent
    """
    id: Optional[str] = None
    text: str
    query_id: str
    retrieved_content_ids: List[str] = []
    confidence_score: float = 0.0
    citations: List[Citation] = []
    created_at: datetime = datetime.now()
    token_usage: Optional[TokenUsage] = None


class APIResponse(BaseModel):
    """
    Model representing the API response
    """
    id: str
    answer: str
    query_id: str
    citations: List[Citation] = []
    response_time_ms: int
    model_used: Optional[str] = None


class APIErrorResponse(BaseModel):
    """
    Model representing an API error response
    """
    error: Dict[str, Any]