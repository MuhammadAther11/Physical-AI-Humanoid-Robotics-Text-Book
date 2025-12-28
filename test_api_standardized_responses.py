"""
Comprehensive API Tests
These tests verify the API endpoint with various inputs and ensure standardized JSON responses.
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

def test_api_with_valid_inputs():
    """Test API endpoint with various valid inputs."""
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
        
        # Test with a simple query
        response = client.post("/api/query", json={
            "query": "What is a humanoid robot?"
        })
        assert response.status_code == 200
        data = response.json()
        assert "id" in data
        assert "queryId" in data
        assert "content" in data
        assert data["content"] == "This is a test response"
        
        # Test with query and userId
        response = client.post("/api/query", json={
            "query": "How do humanoid robots move?",
            "userId": "user-456"
        })
        assert response.status_code == 200
        data = response.json()
        assert "id" in data
        assert "queryId" in data
        assert "content" in data
        
        # Test with query, userId, and context
        response = client.post("/api/query", json={
            "query": "Explain inverse kinematics",
            "userId": "user-789",
            "context": {"domain": "robotics", "level": "beginner"}
        })
        assert response.status_code == 200
        data = response.json()
        assert "id" in data
        assert "queryId" in data
        assert "content" in data


def test_api_with_edge_case_inputs():
    """Test API endpoint with edge case inputs."""
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
        
        # Test with a very long query (within limits)
        long_query = "A" * 1000  # Within the 2000 character limit
        response = client.post("/api/query", json={
            "query": long_query
        })
        # Should be either 200 (success) or 422 (validation error in mock)
        assert response.status_code in [200, 422]
        
        # Test with special characters
        response = client.post("/api/query", json={
            "query": "What's the difference between R&D and R&D?",
            "userId": "user-special-chars"
        })
        assert response.status_code in [200, 422]
        
        # Test with Unicode characters
        response = client.post("/api/query", json={
            "query": "¿Qué es un humanoide?",
            "userId": "user-unicode"
        })
        assert response.status_code in [200, 422]


def test_api_response_format_consistency():
    """Test that all API responses follow the same standardized format."""
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
        
        # Make a request
        response = client.post("/api/query", json={
            "query": "Standardization test"
        })
        
        assert response.status_code == 200
        data = response.json()
        
        # Verify the response structure
        required_fields = ["id", "queryId", "content", "timestamp"]
        for field in required_fields:
            assert field in data, f"Field '{field}' is missing from response"
        
        # Verify field types
        assert isinstance(data["id"], str), "ID should be a string"
        assert isinstance(data["queryId"], str), "QueryId should be a string"
        assert isinstance(data["content"], str), "Content should be a string"
        assert isinstance(data["timestamp"], str), "Timestamp should be a string (ISO format)"
        
        # Sources can be null or an array
        assert data["sources"] is None or isinstance(data["sources"], list), "Sources should be null or an array"


def test_api_error_responses():
    """Test that error responses follow standardized format."""
    # Test with empty query
    response = client.post("/api/query", json={"query": ""})
    assert response.status_code == 400
    error_data = response.json()
    assert "detail" in error_data
    
    # Test with missing query
    response = client.post("/api/query", json={})
    assert response.status_code == 400
    error_data = response.json()
    assert "detail" in error_data
    
    # Test with non-object request
    response = client.post("/api/query", data="invalid json")
    # This might result in a validation error depending on FastAPI's handling
    assert response.status_code in [400, 422]


def test_health_endpoint():
    """Test the health endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    
    data = response.json()
    assert "status" in data
    assert "service" in data
    assert "version" in data
    assert data["status"] == "healthy"


def test_root_endpoint():
    """Test the root endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "version" in data
    assert "docs" in data


if __name__ == "__main__":
    pytest.main([__file__])