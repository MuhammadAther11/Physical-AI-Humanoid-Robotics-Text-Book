import asyncio
import logging
from typing import List, Optional
from openai import OpenAI
import os
from .models.query import Query
from .models.response import ContextGroundedAnswer, Citation, TokenUsage
from .models.retrieval import RetrievedContent
from .retrieval_client import RetrievalClient
from .validation_service import ValidationService


class AgentService:
    """
    Service class to handle the core agent processing logic
    """
    
    def __init__(self, retrieval_client: RetrievalClient, validation_service: ValidationService):
        self.retrieval_client = retrieval_client
        self.validation_service = validation_service
        self.logger = logging.getLogger(__name__)
        
        # Initialize OpenAI client
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        
        self.openai_client = OpenAI(api_key=api_key)
        
        # Use a default model, but this could be configurable
        self.model = os.getenv("OPENAI_MODEL", "gpt-4-turbo-preview")
    
    async def process_query(self, query: Query, k: int = 5) -> ContextGroundedAnswer:
        """
        Process a query through the RAG pipeline: retrieve → agent → response
        
        Args:
            query: The user's query
            k: Number of content chunks to retrieve (default 5)
        
        Returns:
            ContextGroundedAnswer with the response and citations
        """
        self.logger.info(f"Processing query: {query.text[:50]}...")
        
        try:
            # Step 1: Retrieve relevant content
            retrieved_content = await self.retrieval_client.retrieve_content(query.text, k=k)
            self.logger.info(f"Retrieved {len(retrieved_content)} content chunks")
            
            # Step 2: Generate context-grounded response using OpenAI
            answer_text, token_usage = await self._generate_answer_with_context(
                query.text, 
                retrieved_content
            )
            
            # Step 3: Validate the response to ensure it's grounded in content
            is_valid, validation_errors = self.validation_service.validate_answer(
                query.text,
                answer_text,
                retrieved_content
            )
            
            if not is_valid:
                self.logger.warning(f"Response validation failed: {validation_errors}")
                # We can either return an error or try to fix the response
                # For now, we'll proceed but log the validation issues
                for error in validation_errors:
                    self.logger.warning(f"Validation issue: {error}")
            
            # Step 4: Create citations from the retrieved content
            citations = [
                Citation(
                    url=item.url,
                    section_title=item.section_title,
                    confidence=item.similarity_score
                )
                for item in retrieved_content
            ]
            
            # Step 5: Calculate confidence based on average similarity score
            avg_similarity = sum(item.similarity_score for item in retrieved_content) / len(retrieved_content) if retrieved_content else 0.0
            
            # Step 6: Create and return the answer
            answer = ContextGroundedAnswer(
                text=answer_text,
                query_id=query.id or "",
                retrieved_content_ids=[item.id for item in retrieved_content],
                confidence_score=avg_similarity,
                citations=citations,
                token_usage=TokenUsage(
                    input_tokens=token_usage.get('input_tokens', 0),
                    output_tokens=token_usage.get('output_tokens', 0),
                    total_tokens=token_usage.get('total_tokens', 0)
                ) if token_usage else None
            )
            
            self.logger.info(f"Successfully processed query, answer length: {len(answer_text)}")
            return answer
            
        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            raise
    
    async def _generate_answer_with_context(self, query_text: str, retrieved_content: List[RetrievedContent]):
        """
        Generate an answer using the OpenAI API with the retrieved context
        """
        self.logger.info("Generating answer with OpenAI API")
        
        # Prepare the context from retrieved content
        context = "\n\n".join([
            f"Section: {item.section_title}\nURL: {item.url}\nContent: {item.content}"
            for item in retrieved_content
        ])
        
        # Prepare the prompt with context
        system_prompt = (
            "You are an AI assistant helping with educational content. "
            "Provide accurate answers based ONLY on the provided context. "
            "Do not hallucinate or provide information not found in the context. "
            "If the context doesn't contain the information needed to answer, say so."
        )
        
        user_prompt = (
            f"Context:\n{context}\n\n"
            f"Question: {query_text}\n\n"
            f"Please provide a detailed answer based on the context provided. "
            f"Reference the specific sections and URLs when possible."
        )
        
        try:
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=float(os.getenv("TEMPERATURE", "0.3")),
                max_tokens=int(os.getenv("MAX_TOKENS", "1000"))
            )
            
            answer_text = response.choices[0].message.content
            token_usage = {
                'input_tokens': response.usage.prompt_tokens,
                'output_tokens': response.usage.completion_tokens,
                'total_tokens': response.usage.total_tokens
            }
            
            self.logger.info(f"OpenAI response generated successfully, tokens used: {token_usage['total_tokens']}")
            return answer_text, token_usage
            
        except Exception as e:
            self.logger.error(f"Error calling OpenAI API: {str(e)}")
            raise