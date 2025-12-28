from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime

class Query(BaseModel):
    """
    Represents a user's question or request sent from the frontend to the backend.
    """
    id: str = Field(..., description="Unique identifier for the query")
    content: str = Field(
        ..., 
        min_length=1, 
        max_length=2000, 
        description="The actual text content of the user's query"
    )
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the query was submitted")
    userId: Optional[str] = Field(None, description="Identifier for the user who submitted the query")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional information about the query context")
    
    class Config:
        json_schema_extra = {
            "example": {
                "id": "query-123",
                "content": "What are the key components of a humanoid robot's control system?",
                "timestamp": "2025-12-27T12:00:00Z",
                "userId": "user-123",
                "metadata": {"context": "robotics", "source": "textbook"}
            }
        }