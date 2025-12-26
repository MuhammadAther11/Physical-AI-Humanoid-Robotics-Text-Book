import asyncio
import logging
import os
import aiohttp
from typing import List, Optional
from ..models.retrieval import RetrievedContent
from ..models.query import Query


class RetrievalClient:
    """
    Service class to interface with the existing retrieval system
    """
    
    def __init__(self, retrieval_endpoint: Optional[str] = None):
        self.retrieval_endpoint = retrieval_endpoint or os.getenv("RETRIEVAL_ENDPOINT")
        self.logger = logging.getLogger(__name__)
        
        if not self.retrieval_endpoint:
            raise ValueError("RETRIEVAL_ENDPOINT environment variable is required")
    
    async def retrieve_content(self, query_text: str, k: int = 5) -> List[RetrievedContent]:
        """
        Retrieve content from the retrieval system based on the query
        
        Args:
            query_text: The query text to search for
            k: Number of results to retrieve (default 5)
        
        Returns:
            List of RetrievedContent objects
        """
        self.logger.info(f"Retrieving content for query: {query_text[:50]}...")
        
        try:
            # Construct the request payload
            payload = {
                "query": query_text,
                "top_k": k,
                "similarity_threshold": 0.3  # Minimum similarity threshold
            }
            
            # Make the request to the retrieval endpoint
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    self.retrieval_endpoint,
                    json=payload,
                    headers={"Content-Type": "application/json"},
                    timeout=aiohttp.ClientTimeout(total=30)
                ) as response:
                    if response.status != 200:
                        self.logger.error(f"Retrieval request failed with status {response.status}")
                        # Return empty list if retrieval fails, allowing the agent to continue
                        return []
                    
                    data = await response.json()
                    
                    # Convert the response to RetrievedContent objects
                    retrieved_items = []
                    for idx, item in enumerate(data.get("results", [])):
                        retrieved_item = RetrievedContent(
                            id=item.get("id", f"retrieved-{idx}"),
                            content=item.get("text", ""),
                            url=item.get("url", ""),
                            section_title=item.get("section_title", ""),
                            similarity_score=item.get("similarity_score", 0.0),
                            source_rank=item.get("rank", idx),
                            query_id=data.get("query_id", "")
                        )
                        retrieved_items.append(retrieved_item)
                    
                    self.logger.info(f"Successfully retrieved {len(retrieved_items)} content items")
                    return retrieved_items
        
        except asyncio.TimeoutError:
            self.logger.error("Retrieval request timed out")
            return []
        except Exception as e:
            self.logger.error(f"Error retrieving content: {str(e)}")
            return []
    
    async def check_availability(self) -> bool:
        """
        Check if the retrieval system is available
        """
        try:
            async with aiohttp.ClientSession() as session:
                async with session.get(
                    f"{self.retrieval_endpoint}/health",
                    timeout=aiohttp.ClientTimeout(total=10)
                ) as response:
                    return response.status == 200
        except Exception:
            return False