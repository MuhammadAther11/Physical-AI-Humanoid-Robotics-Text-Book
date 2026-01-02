import openai
from typing import Dict, Any, List, Optional
import logging
from src.config import Config
import asyncio

logger = logging.getLogger(__name__)

class OpenAIService:
    """Service class to handle OpenAI API interactions with proper error handling."""

    def __init__(self):
        """Initialize the OpenAI client with configuration from environment."""
        # Use OpenRouter API key if available, otherwise fall back to OpenAI key
        api_key = Config.OPENROUTER_API_KEY or Config.OPENAI_API_KEY
        self.client = openai.OpenAI(
            api_key=api_key,
            base_url=Config.OPENROUTER_BASE_URL
        )
        self.model = Config.RAG_AGENT_MODEL
        self.temperature = Config.RAG_AGENT_TEMPERATURE

    def generate_response(self, prompt: str, context: Optional[List[Dict[str, str]]] = None) -> Dict[str, Any]:
        """
        Generate a response using the OpenAI API.

        Args:
            prompt: The user's query or prompt
            context: Optional context to provide to the model

        Returns:
            Dictionary containing the response and metadata
        """
        try:
            # Prepare messages for the chat completion
            messages = []

            # Add system message with context if provided
            if context:
                context_str = " ".join([item.get("content", "") for item in context])
                messages.append({
                    "role": "system",
                    "content": f"Use the following context to answer the user's question: {context_str}"
                })

            # Add the user's prompt
            messages.append({
                "role": "user",
                "content": prompt
            })

            # Call the OpenAI API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=1000,  # Adjust as needed
                timeout=Config.QUERY_TIMEOUT
            )

            # Extract the response
            content = response.choices[0].message.content
            usage = response.usage

            return {
                "response": content,
                "usage": {
                    "prompt_tokens": usage.prompt_tokens,
                    "completion_tokens": usage.completion_tokens,
                    "total_tokens": usage.total_tokens
                },
                "model": response.model
            }

        except openai.AuthenticationError:
            logger.error("Authentication error: Invalid API key")
            raise Exception("Authentication failed. Please check your API key. Ensure that your OPENAI_API_KEY is correctly set in the environment variables.")

        except openai.RateLimitError:
            logger.error("Rate limit exceeded")
            raise Exception("Rate limit exceeded. Please try again later. You may need to upgrade your OpenAI API plan if this occurs frequently.")

        except openai.APIConnectionError:
            logger.error("Connection error with OpenAI API")
            raise Exception("Failed to connect to OpenAI API. Please check your internet connection and ensure that the API endpoint is accessible.")

        except openai.APITimeoutError:
            logger.error("Timeout error with OpenAI API")
            raise Exception("Request to OpenAI API timed out. The request took too long to process. Please try again with a shorter query or check your network connection.")

        except openai.APIError as e:
            logger.error(f"OpenAI API error: {str(e)}")
            raise Exception(f"OpenAI API error: {str(e)}. This may be a temporary issue with the OpenAI service. Please try again later.")

        except asyncio.TimeoutError:
            logger.error("Async timeout error")
            raise Exception("Request timed out. The system took too long to process your request. Please try again.")

        except Exception as e:
            logger.error(f"Unexpected error in OpenAI service: {str(e)}")
            raise Exception(f"An unexpected error occurred in the OpenAI service: {str(e)}. Please contact support if this issue persists.")

    def validate_api_key(self) -> bool:
        """
        Validate the OpenAI API key by making a simple test call.

        Returns:
            True if the API key is valid, False otherwise
        """
        try:
            # Make a simple test call
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": "Hello"}],
                max_tokens=5,
                temperature=0,
                timeout=Config.QUERY_TIMEOUT
            )
            return True
        except Exception as e:
            logger.error(f"API key validation failed: {str(e)}")
            return False