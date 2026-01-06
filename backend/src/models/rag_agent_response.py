"""
RAG Agent Response model representing the processed answer from the retrieval-augmented generation system
"""
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class Citation(BaseModel):
    """
    Citation model for tracking sources in the response
    """
    url: str
    section_title: str
    confidence: float
    text_excerpt: str


class RAGAgentResponse(BaseModel):
    """
    Represents the processed answer from the retrieval-augmented generation system
    """
    answer: str
    context: Optional[List[str]] = []
    sources: Optional[List[str]] = []
    citations: Optional[List[Citation]] = []
    timestamp: Optional[datetime] = None
    query_id: Optional[str] = None
    session_id: Optional[str] = None
    confidence_score: Optional[float] = None
    response_time_ms: Optional[int] = None

    def __init__(self, **data):
        super().__init__(**data)
        if self.timestamp is None:
            from datetime import datetime
            self.timestamp = datetime.now()