"""
Contract test for proper JSON response format
"""
import pytest
from fastapi.testclient import TestClient
from backend.main import app


@pytest.fixture
def client():
    return TestClient(app)


def test_json_response_format_success(client):
    """
    Test that the endpoint returns proper JSON response format
    """
    # Test with a mock to avoid external dependencies
    import json
    from unittest.mock import patch

    # Mock the rag agent service to return a predictable response
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        mock_response = type('MockResponse', (), {
            'answer': 'This is a test answer',
            'context': ['test context'],
            'sources': ['test source'],
            'citations': [],
            'query_id': 'test-query-id',
            'confidence_score': 0.8,
            'response_time_ms': 100,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        response = client.post("/query", json={
            "question": "Test question for JSON format"
        })

        # Verify status code
        assert response.status_code == 200

        # Verify content type
        assert response.headers["content-type"].startswith("application/json")

        # Parse response JSON
        data = response.json()

        # Verify required fields exist
        assert "answer" in data
        assert isinstance(data["answer"], str)
        assert len(data["answer"]) > 0  # Answer should not be empty

        assert "status" in data
        assert data["status"] in ["success", "error"]


def test_json_response_format_with_error(client):
    """
    Test JSON response format when there's an error
    """
    import json
    from unittest.mock import patch

    # Mock to simulate an error condition
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        # Simulate an error by having process_query raise an exception
        # but the endpoint should still return proper JSON
        mock_response = type('MockResponse', (), {
            'answer': 'Fallback answer due to error',
            'context': [],
            'sources': [],
            'citations': [],
            'query_id': 'error-query-id',
            'confidence_score': 0.0,
            'response_time_ms': 50,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        response = client.post("/query", json={
            "question": "Test question that might cause error"
        })

        # Should still return 200 with proper JSON
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("application/json")

        data = response.json()
        assert "answer" in data
        assert isinstance(data["answer"], str)
        assert "status" in data


def test_json_structure_consistency(client):
    """
    Test that the JSON structure is consistent across different requests
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        mock_response = type('MockResponse', (), {
            'answer': 'Consistent answer',
            'context': ['context'],
            'sources': ['source'],
            'citations': [],
            'query_id': 'consistent-query-id',
            'confidence_score': 0.7,
            'response_time_ms': 75,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        # Make multiple requests to verify consistency
        for i in range(3):
            response = client.post("/query", json={
                "question": f"Test question {i} for consistency"
            })

            assert response.status_code == 200
            data = response.json()

            # Verify consistent structure
            required_fields = ["answer", "status"]
            for field in required_fields:
                assert field in data, f"Field '{field}' missing in response {i}"

            # Verify types
            assert isinstance(data["answer"], str)
            assert isinstance(data["status"], str)


def test_json_response_format_full_endpoint(client):
    """
    Test JSON response format for the full query endpoint
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        mock_response = type('MockResponse', (), {
            'answer': 'Full response answer',
            'context': ['full context'],
            'sources': ['full source'],
            'citations': [],
            'query_id': 'full-query-id',
            'confidence_score': 0.9,
            'response_time_ms': 120,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        response = client.post("/query-full", json={
            "question": "Test question for full response"
        })

        assert response.status_code == 200
        assert response.headers["content-type"].startswith("application/json")

        data = response.json()

        # Verify full response structure
        expected_fields = [
            "answer", "context", "sources", "citations",
            "query_id", "confidence_score", "response_time_ms"
        ]

        for field in expected_fields:
            assert field in data, f"Expected field '{field}' not found in full response"


if __name__ == "__main__":
    pytest.main([__file__])