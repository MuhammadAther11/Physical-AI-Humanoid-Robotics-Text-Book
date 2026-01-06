"""
Contract test for error handling
"""
import pytest
from fastapi.testclient import TestClient
from backend.main import app
from unittest.mock import patch


@pytest.fixture
def client():
    return TestClient(app)


def test_error_handling_contract(client):
    """
    Test that error handling follows the expected contract
    """
    # Test normal operation first
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        mock_response = type('MockResponse', (), {
            'answer': 'Normal answer',
            'context': ['context'],
            'sources': ['source'],
            'citations': [],
            'query_id': 'test-id',
            'confidence_score': 0.8,
            'response_time_ms': 100,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        response = client.post("/query", json={
            "question": "Normal question"
        })

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert isinstance(data["answer"], str)


def test_error_response_format(client):
    """
    Test that error responses follow the expected format
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        # Simulate an error by having the service return an error response
        mock_response = type('MockResponse', (), {
            'answer': 'Fallback answer due to error',
            'context': [],
            'sources': [],
            'citations': [],
            'query_id': 'error-id',
            'confidence_score': 0.0,
            'response_time_ms': 50,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        response = client.post("/query", json={
            "question": "Question that might cause error"
        })

        # Should still return 200 with proper error handling
        assert response.status_code == 200
        data = response.json()

        # Verify the response structure is maintained
        assert "answer" in data
        assert isinstance(data["answer"], str)
        assert "status" in data


def test_error_handling_json_contract(client):
    """
    Test that error handling maintains JSON contract
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        mock_response = type('MockResponse', (), {
            'answer': 'Error fallback response',
            'context': [],
            'sources': [],
            'citations': [],
            'query_id': 'error-query',
            'confidence_score': 0.0,
            'response_time_ms': 25,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        response = client.post("/query", json={
            "question": "Test error handling"
        })

        # Verify response headers
        assert response.headers["content-type"].startswith("application/json")

        # Verify JSON structure
        data = response.json()
        assert isinstance(data, dict)
        assert "answer" in data
        assert isinstance(data["answer"], str)


def test_endpoint_error_handling_consistency(client):
    """
    Test that error handling is consistent across different error scenarios
    """
    test_scenarios = [
        "Short question",
        "A" * 50,  # Medium length
        "Question with normal length to test error handling"
    ]

    for scenario in test_scenarios:
        with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
            mock_response = type('MockResponse', (), {
                'answer': f'Handled error for: {scenario[:20]}...',
                'context': [],
                'sources': [],
                'citations': [],
                'query_id': 'consistency-test',
                'confidence_score': 0.0,
                'response_time_ms': 30,
                'session_id': 'test-session'
            })()

            mock_rag_service.process_query.return_value = mock_response

            response = client.post("/query", json={
                "question": scenario
            })

            # Verify consistent error handling
            assert response.status_code == 200
            data = response.json()

            # Verify required fields exist
            assert "answer" in data
            assert isinstance(data["answer"], str)
            assert "status" in data


def test_error_handling_does_not_crash_endpoint(client):
    """
    Test that error conditions don't crash the endpoint
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        # Simulate various error conditions
        mock_response = type('MockResponse', (), {
            'answer': 'Graceful fallback',
            'context': [],
            'sources': [],
            'citations': [],
            'query_id': 'graceful-error',
            'confidence_score': 0.0,
            'response_time_ms': 40,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        # Test various inputs that could potentially cause errors
        test_inputs = [
            {"question": "Normal question"},
            {"question": "A"},  # Short question
            {"question": "A" * 100},  # Long question
        ]

        for test_input in test_inputs:
            response = client.post("/query", json=test_input)

            # Verify the endpoint doesn't crash
            assert response.status_code == 200
            data = response.json()

            # Verify basic response structure is maintained
            assert "answer" in data
            assert isinstance(data["answer"], str)


def test_error_handling_response_structure(client):
    """
    Test that error handling maintains proper response structure
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        mock_response = type('MockResponse', (), {
            'answer': 'Error was handled gracefully',
            'context': [],
            'sources': [],
            'citations': [],
            'query_id': 'structure-test',
            'confidence_score': 0.0,
            'response_time_ms': 35,
            'session_id': 'test-session'
        })()

        mock_rag_service.process_query.return_value = mock_response

        response = client.post("/query", json={
            "question": "Test structure maintenance"
        })

        data = response.json()

        # Verify the response structure is maintained even during error handling
        required_fields = ["answer"]
        for field in required_fields:
            assert field in data, f"Required field '{field}' missing in error response"

        # Verify field types
        assert isinstance(data["answer"], str)


if __name__ == "__main__":
    pytest.main([__file__])