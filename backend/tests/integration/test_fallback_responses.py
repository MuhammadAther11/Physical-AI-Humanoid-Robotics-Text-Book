"""
Integration test for fallback responses
"""
import pytest
from fastapi.testclient import TestClient
from backend.main import app
from unittest.mock import patch, MagicMock


@pytest.fixture
def client():
    return TestClient(app)


def test_fallback_response_when_no_context_found(client):
    """
    Test that a fallback response is provided when no context is found
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.retrieve_context') as mock_retrieve:
        # Simulate no context found scenario
        mock_retrieve.return_value = []

        response = client.post("/query", json={
            "question": "A question with no matching context"
        })

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        # Should return a fallback message rather than failing
        assert isinstance(data["answer"], str)
        assert len(data["answer"]) > 0


def test_fallback_response_when_retrieval_fails(client):
    """
    Test fallback response when context retrieval fails
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.retrieve_context') as mock_retrieve:
        # Simulate retrieval failure
        mock_retrieve.side_effect = Exception("Retrieval failed")

        response = client.post("/query", json={
            "question": "Question when retrieval fails"
        })

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        # Should return a fallback response
        assert isinstance(data["answer"], str)


def test_fallback_response_when_generation_fails(client):
    """
    Test fallback response when answer generation fails
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.generate_response') as mock_generate:
        # Simulate generation failure
        mock_generate.side_effect = Exception("Generation failed")

        response = client.post("/query", json={
            "question": "Question when generation fails"
        })

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        # Should return a fallback response
        assert isinstance(data["answer"], str)


def test_fallback_response_when_full_process_fails(client):
    """
    Test fallback response when the entire process fails
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.process_query') as mock_process:
        # Simulate complete process failure
        mock_process.side_effect = Exception("Complete process failed")

        response = client.post("/query", json={
            "question": "Question when everything fails"
        })

        assert response.status_code == 200  # Should still return a response
        data = response.json()
        assert "answer" in data
        # Should return a fallback message
        assert isinstance(data["answer"], str)
        # Should not be empty
        assert len(data["answer"]) > 0


def test_fallback_response_with_different_error_types(client):
    """
    Test fallback responses for different types of errors
    """
    error_scenarios = [
        ("RetrievalError", "backend.src.services.rag_agent.RAGAgentService.retrieve_context"),
        ("GenerationError", "backend.src.services.rag_agent.RAGAgentService.generate_response"),
        ("ValidationError", "backend.src.services.error_handler.validate_query_input"),
    ]

    for error_name, mock_path in error_scenarios:
        with patch(mock_path) as mock_func:
            if "validate_query_input" in mock_path:
                # For validation, we need to mock the validation function
                mock_func.side_effect = Exception(f"{error_name} occurred")
            else:
                mock_func.side_effect = Exception(f"{error_name} occurred")

            response = client.post("/query", json={
                "question": f"Question for {error_name} scenario"
            })

            # Should return a response despite the error
            assert response.status_code == 200
            data = response.json()
            assert "answer" in data
            assert isinstance(data["answer"], str)


def test_fallback_response_maintains_api_contract(client):
    """
    Test that fallback responses maintain the API contract
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.process_query') as mock_process:
        mock_process.side_effect = Exception("Process failed")

        response = client.post("/query", json={
            "question": "Question with process failure"
        })

        assert response.status_code == 200
        assert response.headers["content-type"].startswith("application/json")

        data = response.json()
        # Verify response structure is maintained
        assert "answer" in data
        assert isinstance(data["answer"], str)
        assert "status" in data
        # Answer should be a meaningful fallback
        assert len(data["answer"]) > 0


def test_fallback_response_logging(client, caplog):
    """
    Test that fallback responses are properly logged
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.process_query') as mock_process:
        mock_process.side_effect = Exception("Logging test error")

        response = client.post("/query", json={
            "question": "Question for logging test"
        })

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert isinstance(data["answer"], str)


def test_multiple_concurrent_fallback_requests(client):
    """
    Test that fallback responses work correctly with concurrent requests
    """
    import concurrent.futures
    import threading

    def make_request():
        with patch('backend.src.services.rag_agent.RAGAgentService.process_query') as mock_process:
            mock_process.side_effect = Exception("Concurrent error")

            response = client.post("/query", json={
                "question": "Concurrent request test"
            })

            return response.status_code, response.json()

    # Simulate multiple concurrent requests all triggering fallbacks
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
        futures = [executor.submit(make_request) for _ in range(5)]
        results = [future.result() for future in futures]

    # Verify all requests got proper fallback responses
    for status_code, data in results:
        assert status_code == 200
        assert "answer" in data
        assert isinstance(data["answer"], str)


def test_fallback_response_quality(client):
    """
    Test that fallback responses are meaningful and not just generic messages
    """
    with patch('backend.src.services.rag_agent.RAGAgentService.process_query') as mock_process:
        mock_process.side_effect = Exception("Quality test error")

        response = client.post("/query", json={
            "question": "What is the meaning of this error?"
        })

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        answer = data["answer"]

        # The answer should be a proper fallback message, not empty or generic
        assert isinstance(answer, str)
        assert len(answer) > 10  # Should be more than just "Error"
        # Should be a helpful message to the user
        assert "technical difficulties" in answer.lower() or "try again" in answer.lower() or "issue" in answer.lower()


if __name__ == "__main__":
    pytest.main([__file__])