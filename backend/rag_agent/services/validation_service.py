import logging
from typing import List, Tuple
from ..models.query import Query
from ..models.response import RetrievedContent


class ValidationService:
    """
    Service class to validate that answers are properly grounded in content
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)

    def validate_answer(self, query: str, answer: str, retrieved_content: List[RetrievedContent]) -> Tuple[bool, List[str]]:
        """
        Validate that the answer is properly grounded in the retrieved content

        Args:
            query: The original query
            answer: The generated answer
            retrieved_content: The content used to generate the answer

        Returns:
            Tuple of (is_valid, list_of_validation_errors)
        """
        self.logger.info("Validating answer grounding...")

        validation_errors = []

        # Check 1: Does the answer reference the retrieved content?
        has_content_reference = self._check_content_reference(answer, retrieved_content)
        if not has_content_reference:
            validation_errors.append(
                "Answer does not adequately reference or cite the retrieved content"
            )

        # Check 2: Is the answer consistent with the retrieved content?
        is_consistent = self._check_consistency(answer, retrieved_content)
        if not is_consistent:
            validation_errors.append(
                "Answer contains information not supported by the retrieved content"
            )

        # Check 3: Are citations properly formatted?
        has_proper_citations = self._check_citations(answer)
        if not has_proper_citations:
            validation_errors.append(
                "Answer does not include proper citations to source content"
            )

        # Check 4: Does the answer address the original query?
        addresses_query = self._check_query_addressed(query, answer)
        if not addresses_query:
            validation_errors.append(
                "Answer does not adequately address the original query"
            )

        is_valid = len(validation_errors) == 0
        if is_valid:
            self.logger.info("Answer validation passed")
        else:
            self.logger.warning(f"Answer validation failed with {len(validation_errors)} errors: {validation_errors}")

        return is_valid, validation_errors

    def _check_content_reference(self, answer: str, retrieved_content: List[RetrievedContent]) -> bool:
        """
        Check if the answer references information from the retrieved content
        """
        if not retrieved_content:
            return False

        # Look for references to key phrases from retrieved content in the answer
        answer_lower = answer.lower()

        # Check if any significant terms from the retrieved content appear in the answer
        for content_item in retrieved_content:
            content_text = content_item.content.lower()

            # Extract some key terms from the content (simple approach)
            words = content_text.split()
            key_terms = set(words[:20])  # Take first 20 words as key terms

            # Check if any of the key terms appear in the answer
            if any(term in answer_lower for term in key_terms if len(term) > 3):
                return True

        # If no key terms matched, the answer might not be properly grounded
        return False

    def _check_consistency(self, answer: str, retrieved_content: List[RetrievedContent]) -> bool:
        """
        Check if the answer is consistent with the retrieved content
        """
        # This is a simplified check - in a real implementation, this would be more sophisticated
        # For now, we'll just check if the answer contains information that contradicts the retrieved content

        answer_lower = answer.lower()

        # Check for contradictions (simple approach)
        contradiction_indicators = [
            "not mentioned", "not in the text", "not found", "no information"
        ]

        for indicator in contradiction_indicators:
            if indicator in answer_lower:
                # If the answer claims information isn't in the text but we have retrieved content,
                # it might be inconsistent with the fact that we did retrieve relevant content
                if retrieved_content:
                    return False

        return True

    def _check_citations(self, answer: str) -> bool:
        """
        Check if the answer includes proper citations
        """
        # Look for citation patterns in the answer
        citation_patterns = [
            "according to", "cited in", "reference:", "source:", "as stated",
            "see section", "found in", "mentioned in"
        ]

        answer_lower = answer.lower()
        return any(pattern in answer_lower for pattern in citation_patterns)

    def _check_query_addressed(self, query: str, answer: str) -> bool:
        """
        Check if the answer addresses the original query
        """
        # Simple check: see if answer contains relevant terms to the query
        query_lower = query.lower()
        answer_lower = answer.lower()

        # Extract key terms from the query
        query_terms = set(query_lower.split())

        # Check if the answer discusses topics related to the query
        answer_words = set(answer_lower.split())

        # If at least some query terms appear in the answer, consider it addressed
        common_terms = query_terms.intersection(answer_words)

        # Need at least one common term (excluding common stop words)
        stop_words = {"the", "a", "an", "and", "or", "but", "in", "on", "at", "to", "for", "of", "with", "by", "is", "are", "was", "were", "be", "been", "being", "have", "has", "had", "do", "does", "did", "will", "would", "could", "should"}
        meaningful_common_terms = common_terms - stop_words

        return len(meaningful_common_terms) > 0