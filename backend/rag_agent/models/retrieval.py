from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class RetrievedContent(BaseModel):
    """
    Model representing content retrieved from the retrieval system
    """
    id: str
    content: str
    url: str
    section_title: str
    similarity_score: float = 0.0  # Between 0.0 and 1.0
    source_rank: int = 0  # Rank of this content among all retrieved results
    query_id: str
    retrieved_at: datetime = datetime.now()