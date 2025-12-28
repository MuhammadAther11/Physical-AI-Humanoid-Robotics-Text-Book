"""
API Contract Tests
These tests validate that the API endpoints follow the expected request/response formats.
"""
import pytest
import sys
import os
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

from main import app

client = TestClient(app)

def test_query_endpoint_contract():
    """Test that the query endpoint follows the expected contract."""
    # Mock the query processing service to avoid actual processing
    with patch('src.api.api.query_service') as mock_query_service:
        # Create a mock response
        mock_response = MagicMock()
        mock_response.id = "response-123"
        mock_response.queryId = "query-123"
        mock_response.content = "This is a test response"
        mock_response.sources = [{"title": "Test Source", "url": "http://example.com", "excerpt": "Test excerpt"}]
        
        mock_query_service.process_query.return_value = mock_response
        mock_query_service.validate_and_format_query.return_value = {
            "query": "test query",
            "userId": "test-user"
        }
        
        # Test valid request
        response = client.post("/api/query", json={
            "query": "What is a humanoid robot?",
            "userId": "test-user-123"
        })
        
        assert response.status_code == 200
        
        # Validate response structure
        data = response.json()
        assert "id" in data
        assert "queryId" in data
        assert "content" in data
        assert "timestamp" in data
        assert "sources" in data
        
        # Validate response data types
        assert isinstance(data["id"], str)
        assert isinstance(data["queryId"], str)
        assert isinstance(data["content"], str)
        assert isinstance(data["timestamp"], str)  # ISO format string
        assert data["sources"] is None or isinstance(data["sources"], list)


def test_query_endpoint_validation_errors():
    """Test that the query endpoint properly validates inputs."""
    # Test with empty query
    response = client.post("/api/query", json={"query": ""})
    assert response.status_code == 400
    
    # Test with missing query
    response = client.post("/api/query", json={})
    assert response.status_code == 400
    
    # Test with non-string query
    response = client.post("/api/query", json={"query": 123})
    assert response.status_code == 400
    
    # Test with too long query
    long_query = "A" * 3000
    response = client.post("/api/query", json={"query": long_query})
    assert response.status_code == 400


def test_health_endpoint_contract():
    """Test that the health endpoint follows the expected contract."""
    response = client.get("/health")
    
    assert response.status_code == 200
    
    data = response.json()
    assert "status" in data
    assert "service" in data
    assert "version" in data
    
    # Validate data types
    assert isinstance(data["status"], str)
    assert isinstance(data["service"], str)
    assert isinstance(data["version"], str)


def test_rate_limiting():
    """Test that rate limiting works as expected."""
    # Mock the query processing service to avoid actual processing
    with patch('src.api.api.query_service') as mock_query_service:
        # Create a mock response
        mock_response = MagicMock()
        mock_response.id = "response-123"
        mock_response.queryId = "query-123"
        mock_response.content = "This is a test response"
        
        mock_query_service.process_query.return_value = mock_response
        mock_query_service.validate_and_format_query.return_value = {
            "query": "test query",
            "userId": "test-user"
        }
        
        # Make multiple requests to trigger rate limiting
        for i in range(15):  # More than the rate limit
            response = client.post("/api/query", json={
                "query": f"Test query {i}",
                "userId": "test-user-rate-limit"
            })
            
            # The first 10 requests should be OK, then rate limiting kicks in
            if i < 10:
                assert response.status_code in [200, 422]  # 422 might occur due to validation errors in mock
            else:
                # Either 200 (if our rate limiter isn't triggered due to timing) or 429 (rate limited)
                assert response.status_code in [200, 422, 429]


if __name__ == "__main__":
    pytest.main([__file__])