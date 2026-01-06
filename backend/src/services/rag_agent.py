"""
RAG Agent service with error handling for processing queries
"""
import os
import logging
import time
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
from pydantic import BaseModel
from backend.src.models.query_request import QueryRequest
from backend.src.models.rag_agent_response import RAGAgentResponse, Citation
from backend.src.services.error_handler import (
    handle_error, log_error, AgentError, RetrievalError, validate_query_input
)
from dotenv import load_dotenv
from urllib.parse import urlparse


# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
cohere_api_key = os.getenv("COHERE_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if not cohere_api_key:
    raise ValueError("COHERE_API_KEY environment variable is required")
if not qdrant_url:
    raise ValueError("QDRANT_URL environment variable is required")
if not qdrant_api_key:
    raise ValueError("QDRANT_API_KEY environment variable is required")

co = cohere.Client(cohere_api_key)
qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)


class RAGAgentService:
    """
    Service class for the RAG agent with comprehensive error handling
    """

    def __init__(self, collection_name: str = "rag_embeddings"):
        self.collection_name = collection_name
        self.co = co
        self.qdrant_client = qdrant_client

    def retrieve_context(self, query: str, top_k: int = 5) -> List[dict]:
        """
        Retrieve relevant context from the vector database
        """
        try:
            # Validate query input
            validate_query_input(query)

            # Generate embedding for the query
            query_embeddings = self.co.embed(
                texts=[query],
                model='embed-english-v3.0',
                input_type='search_query'
            )
            query_embedding = query_embeddings.embeddings[0]

            # Search in Qdrant for similar content
            search_result = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Extract relevant context from search results
            contexts = []
            for result in search_result:
                if result.payload:
                    context = {
                        'content': result.payload.get('content', ''),
                        'url': result.payload.get('url', ''),
                        'title': result.payload.get('title', ''),
                        'section_title': result.payload.get('section_title', ''),
                        'score': result.score
                    }
                    contexts.append(context)

            logger.info(f"Retrieved {len(contexts)} context items for query: {query[:50]}...")
            return contexts

        except Exception as e:
            error_info = handle_error(e, endpoint="/retrieve_context")
            raise RetrievalError(f"Failed to retrieve context: {error_info['error']}")

    def generate_response(self, query: str, context: List[dict], selected_text: Optional[str] = None) -> str:
        """
        Generate a response using the Cohere generative model with retrieved context
        """
        try:
            # Build the context for the generative model
            context_text = "\n".join([ctx['content'] for ctx in context if ctx['content']])

            # If there's selected text, include it in the prompt
            if selected_text:
                prompt = f"Based on the following context, answer the query. The user has selected this specific text: '{selected_text}'\n\nContext: {context_text}\n\nQuery: {query}\n\nAnswer:"
            else:
                prompt = f"Based on the following context, answer the query:\n\nContext: {context_text}\n\nQuery: {query}\n\nAnswer:"

            # Generate response using Cohere
            response = self.co.generate(
                model='command-r-plus',  # Using a reliable model
                prompt=prompt,
                max_tokens=500,
                temperature=0.7,
                stop_sequences=["\n\n"]
            )

            if response.generations and len(response.generations) > 0:
                answer = response.generations[0].text.strip()
                logger.info(f"Generated response for query: {query[:50]}...")
                return answer
            else:
                raise AgentError("No response generated from the agent")

        except Exception as e:
            error_info = handle_error(e, endpoint="/generate_response")
            raise AgentError(f"Failed to generate response: {error_info['error']}")

    def process_query(self, query_request: QueryRequest) -> RAGAgentResponse:
        """
        Process a query request end-to-end with comprehensive error handling
        """
        start_time = time.time()

        try:
            # Validate the query input
            validate_query_input(query_request.question)

            # Retrieve context
            contexts = self.retrieve_context(query_request.question)

            # If no context is found, return a fallback response
            if not contexts or len(contexts) == 0:
                logger.warning(f"No context found for query: {query_request.question}")
                answer = f"I couldn't find specific information about '{query_request.question}' in the knowledge base. Please try rephrasing your question or check if the topic is covered in the documentation."

                return RAGAgentResponse(
                    answer=answer,
                    context=[],
                    sources=[],
                    citations=[],
                    query_id=f"query_{int(start_time)}",
                    confidence_score=0.0,
                    response_time_ms=int((time.time() - start_time) * 1000)
                )

            # Generate response using the retrieved context
            answer = self.generate_response(
                query_request.question,
                contexts,
                query_request.selected_text
            )

            # Create citations from the retrieved contexts
            citations = []
            for ctx in contexts:
                if ctx['url'] and ctx['content']:
                    citation = Citation(
                        url=ctx['url'],
                        section_title=ctx.get('title', ctx.get('section_title', 'Unknown')),
                        confidence=ctx.get('score', 0.0),
                        text_excerpt=ctx['content'][:200] + "..." if len(ctx['content']) > 200 else ctx['content']
                    )
                    citations.append(citation)

            # Extract sources (unique URLs from contexts)
            sources = list(set([ctx['url'] for ctx in contexts if ctx['url']]))

            # Calculate response time
            response_time = int((time.time() - start_time) * 1000)

            # Determine confidence score based on the highest score from retrieved contexts
            confidence_score = max([ctx.get('score', 0.0) for ctx in contexts]) if contexts else 0.0

            # Create and return the response
            response = RAGAgentResponse(
                answer=answer,
                context=[ctx['content'] for ctx in contexts],
                sources=sources,
                citations=citations,
                query_id=f"query_{int(start_time)}",
                confidence_score=confidence_score,
                response_time_ms=response_time,
                session_id=query_request.session_id
            )

            logger.info(f"Successfully processed query in {response_time}ms")
            return response

        except Exception as e:
            # Handle any errors that occur during query processing
            error_info = handle_error(e, endpoint="/process_query")

            # Log the error but return a fallback response to prevent the "technical difficulties" message
            logger.error(f"Error processing query '{query_request.question}': {error_info['error']}")

            # Return a fallback response instead of raising the exception
            response_time = int((time.time() - start_time) * 1000)
            fallback_answer = "I'm experiencing some technical difficulties processing your request. Please try again in a moment. If the issue persists, please contact support."

            return RAGAgentResponse(
                answer=fallback_answer,
                context=[],
                sources=[],
                citations=[],
                query_id=f"query_{int(start_time)}",
                confidence_score=0.0,
                response_time_ms=response_time,
                session_id=query_request.session_id
            )

    def health_check(self) -> bool:
        """
        Check if the RAG agent service is healthy
        """
        try:
            # Test Cohere connection
            self.co.tokenize(text="health check")

            # Test Qdrant connection
            self.qdrant_client.get_collections()

            return True
        except Exception as e:
            logger.error(f"RAG Agent health check failed: {e}")
            return False


# Global instance of the RAG Agent service
rag_agent_service = RAGAgentService()