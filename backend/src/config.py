import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to handle environment variables and settings."""
    
    # Qdrant Configuration
    QDRANT_URL = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
    
    # OpenAI Configuration
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
    OPENROUTER_BASE_URL = os.getenv("OPENROUTER_BASE_URL", "https://api.openai.com/v1")
    OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY", "")
    
    # Application Configuration
    APP_NAME = os.getenv("APP_NAME", "RAG Backend Service")
    DEBUG = os.getenv("DEBUG", "False").lower() == "true"
    HOST = os.getenv("HOST", "0.0.0.0")
    PORT = int(os.getenv("PORT", "8000"))
    
    # RAG Agent Configuration
    RAG_AGENT_MODEL = os.getenv("RAG_AGENT_MODEL", "gpt-4o")
    RAG_AGENT_TEMPERATURE = float(os.getenv("RAG_AGENT_TEMPERATURE", "0.7"))
    
    # Query Processing Configuration
    MAX_QUERY_LENGTH = int(os.getenv("MAX_QUERY_LENGTH", "2000"))
    QUERY_TIMEOUT = int(os.getenv("QUERY_TIMEOUT", "30"))
    
    # Validation
    @classmethod
    def validate(cls):
        """Validate that all required environment variables are set."""
        errors = []
        
        if not cls.QDRANT_URL:
            errors.append("QDRANT_URL is not set")
        if not cls.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is not set")
        if not cls.OPENAI_API_KEY:
            errors.append("OPENAI_API_KEY is not set")
            
        return errors