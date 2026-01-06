"""
Integration test for malformed request handling
"""
import pytest
from fastapi.testclient import TestClient
from backend.main import app
from unittest.mock import patch


@pytest.fixture
def client():
    return TestClient(app)


def test_malformed_request_missing_question(client):
    """
    Test handling of requests with missing question field
    """
    # Request with missing question
    response = client.post("/query", json={})

    # Should return an error response, not crash
    assert response.status_code == 200  # The endpoint handles errors gracefully
    data = response.json()
    assert "answer" in data
    assert isinstance(data["answer"], str)


def test_malformed_request_empty_question(client):
    """
    Test handling of requests with empty question
    """
    response = client.post("/query", json={
        "question": ""
    })

    # Should return an error response
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert isinstance(data["answer"], str)


def test_malformed_request_very_long_question(client):
    """
    Test handling of requests with extremely long question
    """
    long_question = "A" * 2000  # Much longer than validation limit
    response = client.post("/query", json={
        "question": long_question
    })

    # Should handle gracefully
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert isinstance(data["answer"], str)


def test_malformed_request_invalid_json_structure(client):
    """
    Test handling of requests with invalid JSON structure
    """
    # Test with invalid data types
    response = client.post("/query", json={
        "question": 12345  # Invalid - should be string
    })

    # Should handle gracefully
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data


def test_error_handling_with_mocked_service_error(client):
    """
    Test error handling when the RAG service fails
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        # Configure the mock to raise an exception
        mock_rag_service.process_query.side_effect = Exception("Service unavailable")

        response = client.post("/query", json={
            "question": "What happens when service fails?"
        })

        # Even with service error, endpoint should return a response
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        # Should return a fallback message instead of crashing
        assert isinstance(data["answer"], str)
        # Should not be an empty answer
        assert len(data["answer"]) > 0


def test_error_handling_with_retrieval_error(client):
    """
    Test error handling when retrieval fails
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.process_query') as mock_process:
        mock_process.side_effect = Exception("Retrieval failed")

        response = client.post("/query", json={
            "question": "Question that triggers retrieval error"
        })

        # Should return a graceful response
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert isinstance(data["answer"], str)


def test_error_handling_with_generation_error(client):
    """
    Test error handling when response generation fails
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.process_query') as mock_process:
        mock_process.side_effect = Exception("Generation failed")

        response = client.post("/query", json={
            "question": "Question that triggers generation error"
        })

        # Should return a graceful response
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert isinstance(data["answer"], str)


def test_health_endpoint_error_handling(client):
    """
    Test health endpoint error handling
    """
    with patch('backend.src.services.rag_agent.rag_agent_service') as mock_service:
        mock_service.health_check.return_value = False

        response = client.get("/health")

        # Should return appropriate status
        assert response.status_code in [200, 503]
        data = response.json()
        assert "status" in data


def test_endpoint_robustness_with_various_inputs(client):
    """
    Test endpoint robustness with various malformed inputs
    """
    test_cases = [
        {},  # Empty object
        {"question": ""},  # Empty question
        {"question": "   "},  # Whitespace only
        {"question": "A"},  # Too short
        {"q": "test"},  # Wrong field name
        [],  # Array instead of object
        "not an object",  # String instead of object
    ]

    for test_case in test_cases:
        try:
            response = client.post("/query", json=test_case)
            # Should not crash, should return a response
            assert response.status_code in [200]
            data = response.json()
            assert "answer" in data
            assert isinstance(data["answer"], str)
        except Exception:
            # If it does crash, that's a failure
            assert False, f"Endpoint crashed with input: {test_case}"


if __name__ == "__main__":
    pytest.main([__file__])