"""
Embedding service module for the retrieval pipeline.
Handles generating embeddings for queries using Cohere API.
"""

import os
import logging
from typing import List
import cohere


class EmbeddingService:
    """
    Service class to handle embedding generation using Cohere
    """
    
    def __init__(self):
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        
        self.client = cohere.Client(api_key)
        self.logger = logging.getLogger(__name__)

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate an embedding for the given text using Cohere
        
        Args:
            text: The text to convert to an embedding
            
        Returns:
            A list of floats representing the embedding vector
        """
        self.logger.info(f"Generating embedding for text: {text[:50]}...")
        
        try:
            response = self.client.embed(
                texts=[text],
                model='embed-multilingual-v2.0'  # Using Cohere's multilingual model
            )
            
            embedding = response.embeddings[0]  # Get the first (and only) embedding
            self.logger.info(f"Generated embedding with {len(embedding)} dimensions")
            return embedding
            
        except Exception as e:
            self.logger.error(f"Error generating embedding: {str(e)}")
            raise