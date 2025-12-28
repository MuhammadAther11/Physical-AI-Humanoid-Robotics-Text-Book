"""
Test script to verify the end-to-end flow of the RAG agent system.
This script simulates the complete flow from query submission to response.
"""

import asyncio
import sys
import os

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

from models.query import Query
from rag_agent_service import RAGAgentService
from query_processing_service import QueryProcessingService
import uuid

def test_end_to_end_flow():
    """Test the complete end-to-end flow from query to response."""
    print("Testing end-to-end flow...")
    
    try:
        # Initialize services
        query_service = QueryProcessingService()
        
        # Create a sample query
        sample_query_data = {
            "query": "What are the key components of a humanoid robot's control system?",
            "userId": "test-user-123",
            "context": {"domain": "humanoid robotics", "source": "textbook"}
        }
        
        print("Step 1: Validating and formatting query...")
        formatted_query = query_service.validate_and_format_query(sample_query_data)
        print(f"✓ Query validated: {formatted_query['query'][:50]}...")
        
        print("Step 2: Processing query through RAG agent...")
        response = query_service.process_query(formatted_query)
        
        print(f"Step 3: Received response with ID: {response.id}")
        print(f"✓ Query ID: {response.queryId}")
        print(f"✓ Response content length: {len(response.content)} characters")
        print(f"✓ Sources provided: {response.sources is not None}")
        
        if response.sources:
            print(f"✓ Number of sources: {len(response.sources)}")
        
        print("\n✓ End-to-end flow test completed successfully!")
        return True
        
    except Exception as e:
        print(f"\n✗ End-to-end flow test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def test_error_handling():
    """Test error handling capabilities."""
    print("\nTesting error handling...")
    
    try:
        query_service = QueryProcessingService()
        
        # Test with invalid query
        try:
            invalid_query = {"query": ""}  # Empty query
            query_service.validate_and_format_query(invalid_query)
            print("✗ Error handling test failed: Should have raised an error for empty query")
            return False
        except ValueError:
            print("✓ Correctly caught validation error for empty query")
        
        # Test with too long query
        try:
            long_query = {"query": "A" * 3000}  # Too long query
            query_service.validate_and_format_query(long_query)
            print("✗ Error handling test failed: Should have raised an error for long query")
            return False
        except ValueError:
            print("✓ Correctly caught validation error for long query")
        
        print("✓ Error handling tests completed successfully!")
        return True
        
    except Exception as e:
        print(f"✗ Error handling test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Starting RAG Agent End-to-End Tests\n")
    
    success1 = test_end_to_end_flow()
    success2 = test_error_handling()
    
    if success1 and success2:
        print("\n🎉 All tests passed! The end-to-end flow is working correctly.")
    else:
        print("\n❌ Some tests failed. Please check the implementation.")
        sys.exit(1)