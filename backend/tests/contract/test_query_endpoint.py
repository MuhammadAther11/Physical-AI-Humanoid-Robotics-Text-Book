"""
Contract test for /query endpoint
"""
import pytest
from fastapi.testclient import TestClient
from backend.main import app
from backend.src.models.query_request import QueryRequest


@pytest.fixture
def client():
    return TestClient(app)


def test_query_endpoint_contract(client):
    """
    Test that the /query endpoint follows the expected contract
    """
    # Test basic query request
    query_data = {
        "question": "What is a humanoid robot?"
    }

    response = client.post("/query", json=query_data)

    # Assert the response structure
    assert response.status_code == 200
    assert "answer" in response.json()
    assert isinstance(response.json()["answer"], str)
    assert "status" in response.json()

    # Verify the response has the expected format
    response_data = response.json()
    assert isinstance(response_data["answer"], str)
    assert response_data["status"] in ["success", "error"]


def test_query_endpoint_json_response_format(client):
    """
    Test that the /query endpoint returns proper JSON response format
    """
    query_data = {
        "question": "What are the components of a robot?"
    }

    response = client.post("/query", json=query_data)

    # Verify it's valid JSON
    assert response.headers["content-type"].startswith("application/json")

    response_data = response.json()

    # Verify required fields exist
    assert "answer" in response_data
    assert isinstance(response_data["answer"], str)


def test_query_endpoint_with_session_id(client):
    """
    Test that the /query endpoint handles session_id properly
    """
    query_data = {
        "question": "Test question",
        "session_id": "test-session-123"
    }

    response = client.post("/query", json=query_data)

    assert response.status_code == 200
    response_data = response.json()
    assert "answer" in response_data
    assert isinstance(response_data["answer"], str)


if __name__ == "__main__":
    pytest.main([__file__])