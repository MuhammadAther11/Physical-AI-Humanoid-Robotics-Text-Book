from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class QueryAnswerPair(BaseModel):
    """
    Model representing a query-answer pair in conversation history
    """
    query: str
    answer: str
    timestamp: datetime = datetime.now()


class AgentSession(BaseModel):
    """
    Model representing an agent session for maintaining conversation context
    """
    id: str
    user_id: Optional[str] = None
    created_at: datetime = datetime.now()
    last_activity: datetime = datetime.now()
    conversation_history: List[QueryAnswerPair] = []
    session_metadata: Optional[Dict[str, Any]] = {}