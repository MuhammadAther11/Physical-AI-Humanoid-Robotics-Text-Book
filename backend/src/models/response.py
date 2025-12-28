from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from datetime import datetime

class Source(BaseModel):
    """
    Represents a source document used to generate the response.
    """
    title: str = Field(..., description="Title of the source document")
    url: str = Field(..., description="URL of the source document")
    excerpt: Optional[str] = Field(None, description="Relevant excerpt from the source")

class Response(BaseModel):
    """
    Represents the RAG agent's answer sent from the backend to the frontend.
    """
    id: str = Field(..., description="Unique identifier for the response")
    queryId: str = Field(..., description="Reference to the original query")
    content: str = Field(..., min_length=1, description="The text content of the agent's response")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the response was generated")
    sources: Optional[List[Source]] = Field(None, description="List of sources used to generate the response")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional information about the response")
    
    class Config:
        json_schema_extra = {
            "example": {
                "id": "response-456",
                "queryId": "query-123",
                "content": "The key components of a humanoid robot's control system include...",
                "timestamp": "2025-12-27T12:00:05Z",
                "sources": [
                    {
                        "title": "Humanoid Robotics: A Reference",
                        "url": "https://example.com/humanoid-robotics-reference",
                        "excerpt": "The control system of a humanoid robot typically consists of multiple interconnected subsystems..."
                    }
                ],
                "metadata": {"model_used": "gpt-4o", "tokens_used": 150}
            }
        }