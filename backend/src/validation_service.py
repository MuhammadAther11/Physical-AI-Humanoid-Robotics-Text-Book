from typing import Union
from src.models.query import Query
from src.models.response import Response
import re

class ValidationService:
    """Service class to handle validation logic for Query and Response entities."""
    
    @staticmethod
    def validate_query(query: Query) -> bool:
        """
        Validate a Query entity.

        Args:
            query: The Query entity to validate

        Returns:
            True if the query is valid, False otherwise
        """
        # Check content length
        if not (1 <= len(query.content) <= 2000):
            raise ValueError("Query content must be between 1 and 2000 characters")

        # Check for empty or whitespace-only content
        if not query.content.strip():
            raise ValueError("Query content cannot be empty or contain only whitespace")

        # Check for extremely long input queries (more than 1000 words)
        word_count = len(query.content.split())
        if word_count > 1000:
            raise ValueError(f"Query is too long: {word_count} words. Maximum allowed is 1000 words.")

        # Check for potential injection attacks or harmful content
        if ValidationService._contains_potential_injection(query.content):
            raise ValueError("Query contains potentially harmful content")

        # Check if userId is valid if provided
        if query.userId is not None:
            if not isinstance(query.userId, str) or len(query.userId.strip()) == 0:
                raise ValueError("userId must be a non-empty string if provided")

        # Check timestamp format (should be datetime)
        if not isinstance(query.timestamp, object):  # datetime object
            raise ValueError("Timestamp must be a valid datetime")

        return True

    @staticmethod
    def _contains_potential_injection(text: str) -> bool:
        """
        Check if the text contains potential injection patterns.

        Args:
            text: The text to check

        Returns:
            True if potential injection patterns are found, False otherwise
        """
        # List of potential injection patterns to check
        injection_patterns = [
            r'<script',  # Potential XSS
            r'javascript:',  # Potential XSS
            r'on\w+\s*=',  # Event handlers
            r'eval\s*\(',  # Code evaluation
            r'exec\s*\(',  # Code execution
        ]

        import re
        text_lower = text.lower()
        for pattern in injection_patterns:
            if re.search(pattern, text_lower):
                return True
        return False
    
    @staticmethod
    def validate_response(response: Response) -> bool:
        """
        Validate a Response entity.
        
        Args:
            response: The Response entity to validate
            
        Returns:
            True if the response is valid, False otherwise
        """
        # Check content is not empty
        if not response.content or not response.content.strip():
            raise ValueError("Response content cannot be empty")
        
        # Check queryId reference
        if not response.queryId:
            raise ValueError("Response must reference a valid queryId")
        
        # Validate sources if provided
        if response.sources is not None:
            for source in response.sources:
                if not source.title or not source.title.strip():
                    raise ValueError("Each source must have a non-empty title")
                if not source.url or not source.url.strip():
                    raise ValueError("Each source must have a non-empty URL")
        
        # Check timestamp format (should be datetime)
        if not isinstance(response.timestamp, object):  # datetime object
            raise ValueError("Timestamp must be a valid datetime")
        
        return True
    
    @staticmethod
    def sanitize_query_content(content: str) -> str:
        """
        Sanitize query content to prevent injection attacks.
        
        Args:
            content: The raw query content
            
        Returns:
            Sanitized query content
        """
        # Remove any potentially harmful characters or patterns
        # This is a basic example - implement more comprehensive sanitization as needed
        sanitized = content.strip()
        
        # Additional sanitization can be added here
        # For example, removing script tags, etc.
        
        return sanitized