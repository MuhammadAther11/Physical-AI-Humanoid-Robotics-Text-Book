"""
Integration test for query processing
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from backend.main import app
from backend.src.services.rag_agent import RAGAgentService


@pytest.fixture
def client():
    return TestClient(app)


def test_query_processing_integration_success(client):
    """
    Test the complete query processing flow end-to-end
    """
    # Mock the RAG agent service to avoid external dependencies during testing
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        # Configure the mock to return a successful response
        mock_response = MagicMock()
        mock_response.answer = "This is a test answer for your question."
        mock_response.context = ["context1", "context2"]
        mock_response.sources = ["source1", "source2"]
        mock_response.confidence_score = 0.9
        mock_response.response_time_ms = 100

        mock_rag_service.process_query.return_value = mock_response

        query_data = {
            "question": "What is a humanoid robot?",
            "session_id": "test-session"
        }

        response = client.post("/query", json=query_data)

        assert response.status_code == 200
        response_data = response.json()
        assert "answer" in response_data
        assert response_data["answer"] == "This is a test answer for your question."
        assert response_data["status"] == "success"


def test_query_processing_integration_error_handling(client):
    """
    Test query processing error handling
    """
    # Mock the RAG agent service to simulate an error
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        # Configure the mock to raise an exception
        mock_rag_service.process_query.side_effect = Exception("Test error")

        query_data = {
            "question": "What happens when there's an error?",
            "session_id": "test-session-error"
        }

        response = client.post("/query", json=query_data)

        # Even with an error, the endpoint should return a response, not crash
        assert response.status_code == 200
        response_data = response.json()
        assert "answer" in response_data
        # The answer should be a fallback response, not crash the system
        assert isinstance(response_data["answer"], str)


def test_query_full_endpoint_integration(client):
    """
    Test the full query endpoint integration
    """
    with patch('backend.src.api.query_endpoint.rag_agent_service') as mock_rag_service:
        # Configure the mock for the full response
        mock_response = MagicMock()
        mock_response.answer = "Detailed answer"
        mock_response.context = ["context1"]
        mock_response.sources = ["source1"]
        mock_response.citations = []
        mock_response.query_id = "test-query-id"
        mock_response.confidence_score = 0.8
        mock_response.response_time_ms = 150
        mock_response.session_id = "test-session"

        mock_rag_service.process_query.return_value = mock_response

        query_data = {
            "question": "Detailed question for full response",
            "session_id": "test-session-full"
        }

        response = client.post("/query-full", json=query_data)

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["answer"] == "Detailed answer"
        assert "context" in response_data
        assert "sources" in response_data
        assert "citations" in response_data


def test_health_endpoint_integration(client):
    """
    Test the health endpoint integration
    """
    with patch('backend.src.services.rag_agent.rag_agent_service') as mock_rag_service:
        mock_rag_service.health_check.return_value = True

        response = client.get("/health")

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["status"] == "healthy"


if __name__ == "__main__":
    pytest.main([__file__])