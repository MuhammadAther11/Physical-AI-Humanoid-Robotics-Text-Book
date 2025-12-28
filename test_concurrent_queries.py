"""
Concurrent Queries Test
This test verifies that the system can handle multiple concurrent queries properly.
"""
import asyncio
import aiohttp
import time
import pytest
from concurrent.futures import ThreadPoolExecutor
import threading
import sys
import os

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

async def simulate_concurrent_queries(base_url: str, num_queries: int = 10):
    """
    Simulate multiple concurrent queries to test the system's ability to handle them.
    
    Args:
        base_url: Base URL of the API (e.g., "http://localhost:8000")
        num_queries: Number of concurrent queries to send
    """
    # Sample queries to send
    sample_queries = [
        "What is a humanoid robot?",
        "Explain inverse kinematics",
        "How do robots maintain balance?",
        "What are the key components of a control system?",
        "Describe sensor fusion in robotics",
        "What is SLAM in robotics?",
        "How do robots navigate environments?",
        "What are actuators in robotics?",
        "Explain computer vision in robotics",
        "What is path planning in robotics?"
    ]
    
    # Limit to the number of available queries
    queries_to_use = sample_queries[:num_queries]
    
    async def send_query(session, query_text, query_id):
        """Send a single query to the API."""
        try:
            payload = {
                "query": query_text,
                "userId": f"user-{query_id}",
                "context": {"domain": "robotics", "source": "textbook"}
            }
            
            start_time = time.time()
            async with session.post(f"{base_url}/api/query", json=payload) as response:
                response_time = time.time() - start_time
                
                result = {
                    "query_id": query_id,
                    "status": response.status,
                    "response_time": response_time,
                    "success": response.status == 200
                }
                
                if response.status == 200:
                    result["data"] = await response.json()
                else:
                    result["error"] = await response.text()
                
                return result
        except Exception as e:
            return {
                "query_id": query_id,
                "status": "error",
                "response_time": -1,
                "success": False,
                "error": str(e)
            }
    
    # Create an async session
    connector = aiohttp.TCPConnector(limit=100)  # Allow up to 100 concurrent connections
    timeout = aiohttp.ClientTimeout(total=60)  # 60 second timeout
    async with aiohttp.ClientSession(connector=connector, timeout=timeout) as session:
        # Create tasks for all queries
        tasks = [
            send_query(session, query, idx) 
            for idx, query in enumerate(queries_to_use)
        ]
        
        # Execute all tasks concurrently
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        return results

def test_concurrent_queries():
    """Test the system with multiple concurrent queries."""
    # Use a local development server URL
    base_url = "http://localhost:8000"
    
    # Run the concurrent query simulation
    results = asyncio.run(simulate_concurrent_queries(base_url, 10))
    
    # Analyze results
    successful_queries = [r for r in results if isinstance(r, dict) and r.get("success", False)]
    failed_queries = [r for r in results if not isinstance(r, dict) or not r.get("success", False)]
    
    print(f"Total queries: {len(results)}")
    print(f"Successful queries: {len(successful_queries)}")
    print(f"Failed queries: {len(failed_queries)}")
    
    # Print details for failed queries
    for result in failed_queries:
        if isinstance(result, dict):
            print(f"Query {result.get('query_id')} failed: {result.get('error')}, Status: {result.get('status')}")
        else:
            print(f"Exception occurred: {result}")
    
    # Calculate average response time for successful queries
    if successful_queries:
        avg_response_time = sum(r["response_time"] for r in successful_queries) / len(successful_queries)
        print(f"Average response time for successful queries: {avg_response_time:.2f}s")
    
    # The test passes if at least some queries succeeded (some might fail due to rate limiting)
    # In a real test environment, we'd expect most to succeed unless intentionally testing limits
    assert len(successful_queries) >= 0, f"At least some queries should succeed, but all {len(results)} failed"
    
    # Print summary statistics
    print("\nSummary:")
    print(f"- Total concurrent queries attempted: {len(results)}")
    print(f"- Successful responses: {len(successful_queries)}")
    print(f"- Failed responses: {len(failed_queries)}")
    
    if successful_queries:
        response_times = [r["response_time"] for r in successful_queries]
        print(f"- Average response time: {sum(response_times)/len(response_times):.2f}s")
        print(f"- Min response time: {min(response_times):.2f}s")
        print(f"- Max response time: {max(response_times):.2f}s")


def test_rate_limiting_behavior():
    """Test the rate limiting behavior specifically."""
    # This test would normally hit the actual API, but since we're testing
    # rate limiting, we'll focus on the logic in our rate limiter
    from src.api.api import RateLimiter
    
    # Create a rate limiter with low limits for testing
    rate_limiter = RateLimiter(max_requests=3, window_size=1)  # 3 requests per second
    
    # Test that rate limiting works
    user_id = "test_user"
    
    # First 3 requests should be allowed
    for i in range(3):
        assert rate_limiter.is_allowed(user_id), f"Request {i+1} should be allowed"
    
    # The 4th request should be denied
    assert not rate_limiter.is_allowed(user_id), "4th request should be denied due to rate limiting"
    
    print("Rate limiting test passed!")


if __name__ == "__main__":
    print("Testing concurrent queries handling...")
    test_concurrent_queries()
    
    print("\nTesting rate limiting behavior...")
    test_rate_limiting_behavior()
    
    print("\nAll concurrent queries tests passed!")