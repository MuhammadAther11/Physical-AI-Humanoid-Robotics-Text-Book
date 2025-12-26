"""
Validation module for the retrieval pipeline.
Validates that search results meet the required criteria.
"""

import logging
from typing import List, Dict, Any
from dataclasses import dataclass


@dataclass
class ValidationResult:
    """
    Data class to hold the results of validation
    """
    query_id: str
    results: List[Dict[str, Any]]
    is_valid: bool
    errors: List[str]


def validate_results(
    query_text: str, 
    search_results: List[Dict[str, Any]], 
    top_k: int
) -> ValidationResult:
    """
    Validate that search results meet the required criteria:
    - Relevance
    - Metadata intact
    - No errors
    
    Args:
        query_text: The original query text
        search_results: The results from the vector search
        top_k: The number of top results requested
        
    Returns:
        ValidationResult containing validation status and any errors
    """
    logger = logging.getLogger(__name__)
    logger.info(f"Validating {len(search_results)} results for query: {query_text[:50]}...")
    
    errors = []
    
    # Check 1: Ensure we have results
    if not search_results:
        errors.append("No results returned for the query")
    
    # Check 2: Validate metadata is preserved (URL and section title)
    for i, result in enumerate(search_results):
        if not result.get("url"):
            errors.append(f"Result {i} missing URL in metadata")
        if not result.get("section_title"):
            errors.append(f"Result {i} missing section title in metadata")
        if not result.get("text"):
            errors.append(f"Result {i} missing content text")
    
    # Check 3: Validate result relevance (basic check)
    # In a real implementation, this would be more sophisticated
    for i, result in enumerate(search_results):
        similarity_score = result.get("similarity_score", 0)
        if similarity_score < 0.3:  # Arbitrary threshold for low relevance
            logger.warning(f"Result {i} has low similarity score: {similarity_score}")
    
    # Check 4: Ensure result count matches expectations
    if len(search_results) > top_k:
        errors.append(f"Returned {len(search_results)} results, but requested top_k was {top_k}")
    
    # Create a query ID (in a real system, this would be a proper UUID)
    query_id = f"query_{hash(query_text) % 1000000}"
    
    # Determine if the results are valid based on our checks
    is_valid = len(errors) == 0
    
    if is_valid:
        logger.info("Results validation passed")
    else:
        logger.error(f"Results validation failed with {len(errors)} errors: {errors}")
    
    return ValidationResult(
        query_id=query_id,
        results=search_results,
        is_valid=is_valid,
        errors=errors
    )