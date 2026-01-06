"""
Query Request model representing a user's question sent to the RAG chatbot
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class QueryRequest(BaseModel):
    """
    Represents a user's question sent to the RAG chatbot
    """
    question: str
    timestamp: Optional[datetime] = None
    user_id: Optional[str] = None
    session_id: Optional[str] = None
    selected_text: Optional[str] = None
    include_citations: Optional[bool] = True

    def __init__(self, **data):
        super().__init__(**data)
        if self.timestamp is None:
            from datetime import datetime
            self.timestamp = datetime.now()