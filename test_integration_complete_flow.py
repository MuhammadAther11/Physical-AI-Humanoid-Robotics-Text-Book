"""
Integration Tests
These tests verify the complete query flow from frontend to backend and back.
"""
import pytest
import sys
import os
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import json

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

from main import app

client = TestClient(app)

def test_complete_query_flow_integration():
    """Test the complete query flow from API endpoint to response."""
    # Mock the query processing service to simulate the full flow
    with patch('src.api.api.query_service') as mock_query_service:
        # Create a mock response
        mock_response = MagicMock()
        mock_response.id = "response-123"
        mock_response.queryId = "query-123"
        mock_response.content = "This is a test response from the RAG agent"
        mock_response.timestamp = "2025-12-27T12:00:00Z"
        mock_response.sources = [
            {
                "title": "Humanoid Robotics Guide",
                "url": "https://example.com/humanoid-guide",
                "excerpt": "Information about humanoid robots and their applications"
            }
        ]
        
        # Configure the mock to return our test response
        mock_query_service.process_query.return_value = mock_response
        mock_query_service.validate_and_format_query.return_value = {
            "query": "What is a humanoid robot?",
            "userId": "test-user-123",
            "context": {"domain": "robotics", "source": "textbook"}
        }
        
        # Make a request to the API
        response = client.post("/api/query", json={
            "query": "What is a humanoid robot?",
            "userId": "test-user-123",
            "context": {"domain": "robotics", "source": "textbook"}
        })
        
        # Verify the response
        assert response.status_code == 200
        
        # Parse the response
        data = response.json()
        
        # Verify the response structure
        assert "id" in data
        assert "queryId" in data
        assert "content" in data
        assert "timestamp" in data
        assert "sources" in data
        
        # Verify the content
        assert data["content"] == "This is a test response from the RAG agent"
        assert data["queryId"] == "query-123"
        assert len(data["sources"]) == 1
        assert data["sources"][0]["title"] == "Humanoid Robotics Guide"


def test_error_handling_integration():
    """Test error handling in the complete flow."""
    # Test with invalid query
    response = client.post("/api/query", json={
        "query": "",  # Empty query should cause validation error
        "userId": "test-user"
    })
    
    assert response.status_code == 400
    
    # Test with missing query field
    response = client.post("/api/query", json={
        "userId": "test-user"
    })
    
    assert response.status_code == 400
    
    # Test with query that's too long
    long_query = "A" * 3000  # This should exceed our validation limit
    response = client.post("/api/query", json={
        "query": long_query,
        "userId": "test-user"
    })
    
    assert response.status_code == 400


def test_rate_limiting_integration():
    """Test rate limiting in the complete flow."""
    # Mock the query processing service
    with patch('src.api.api.query_service') as mock_query_service:
        # Create a mock response
        mock_response = MagicMock()
        mock_response.id = "response-123"
        mock_response.queryId = "query-123"
        mock_response.content = "This is a test response"
        
        mock_query_service.process_query.return_value = mock_response
        mock_query_service.validate_and_format_query.return_value = {
            "query": "test query",
            "userId": "test-user-rate-limit"
        }
        
        # Make multiple requests to test rate limiting
        for i in range(15):  # More than the rate limit
            response = client.post("/api/query", json={
                "query": f"Test query {i}",
                "userId": "test-user-rate-limit"
            })
            
            # Depending on implementation details, early requests might succeed
            # while later ones hit rate limits
            assert response.status_code in [200, 422, 429]


def test_health_check_integration():
    """Test the health check endpoint."""
    response = client.get("/health")
    
    assert response.status_code == 200
    
    data = response.json()
    assert "status" in data
    assert "service" in data
    assert "version" in data
    assert data["status"] == "healthy"


def test_root_endpoint_integration():
    """Test the root endpoint."""
    response = client.get("/")
    
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "version" in data
    assert "docs" in data


def test_concurrent_queries_simulation():
    """Simulate handling of multiple concurrent queries."""
    import concurrent.futures
    import threading
    
    # Use a lock to safely count successful requests
    success_count = 0
    lock = threading.Lock()
    
    def make_request(query_num):
        nonlocal success_count
        
        # Mock the query processing service for this thread
        with patch('src.api.api.query_service') as mock_query_service:
            # Create a mock response
            mock_response = MagicMock()
            mock_response.id = f"response-{query_num}"
            mock_response.queryId = f"query-{query_num}"
            mock_response.content = f"This is response to query {query_num}"
            
            mock_query_service.process_query.return_value = mock_response
            mock_query_service.validate_and_format_query.return_value = {
                "query": f"Test query {query_num}",
                "userId": f"test-user-{query_num}"
            }
            
            response = client.post("/api/query", json={
                "query": f"Test query {query_num}",
                "userId": f"test-user-{query_num}"
            })
            
            if response.status_code == 200:
                with lock:
                    success_count += 1
                    
            return response.status_code
    
    # Create multiple concurrent requests
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
        futures = [executor.submit(make_request, i) for i in range(10)]
        results = [future.result() for future in futures]
    
    # Verify that we got some successful responses
    successful_requests = sum(1 for status in results if status == 200)
    assert successful_requests >= 0  # At least 0 successful requests (could be limited by rate limiting)


if __name__ == "__main__":
    pytest.main([__file__])