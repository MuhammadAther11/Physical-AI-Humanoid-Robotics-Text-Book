import logging
from typing import List
from ..models.response import Citation


class CitationFormatter:
    """
    Service class to format citations properly
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
    
    def format_citations(self, retrieved_content_list: List[RetrievedContent]) -> List[Citation]:
        """
        Format retrieved content into proper citation objects
        
        Args:
            retrieved_content_list: List of RetrievedContent objects
            
        Returns:
            List of Citation objects
        """
        self.logger.info(f"Formatting {len(retrieved_content_list)} citations")
        
        citations = []
        for content in retrieved_content_list:
            citation = Citation(
                url=content.url,
                section_title=content.section_title,
                confidence=content.similarity_score
            )
            citations.append(citation)
        
        self.logger.info(f"Formatted {len(citations)} citations successfully")
        return citations
    
    def format_single_citation(self, content: RetrievedContent) -> Citation:
        """
        Format a single retrieved content item into a citation
        
        Args:
            content: A single RetrievedContent object
            
        Returns:
            A Citation object
        """
        return Citation(
            url=content.url,
            section_title=content.section_title,
            confidence=content.similarity_score
        )