"""
System Validation Test
This test validates that the entire system works without errors in local development.
"""
import subprocess
import sys
import os
import time
import requests
import signal
import psutil
from typing import Dict, Any, List

def check_backend_services() -> Dict[str, Any]:
    """Check if backend services are running and accessible."""
    result = {
        "fastapi_running": False,
        "health_check_ok": False,
        "api_responding": False,
        "error": None
    }
    
    try:
        # Check if the API is responding at the expected port
        response = requests.get("http://localhost:8000/health", timeout=10)
        result["api_responding"] = response.status_code == 200
        result["health_check_ok"] = response.status_code == 200
        
        if response.status_code == 200:
            health_data = response.json()
            result["fastapi_running"] = health_data.get("status") == "healthy"
        
    except requests.exceptions.ConnectionError:
        result["error"] = "Could not connect to backend API at http://localhost:8000"
    except requests.exceptions.Timeout:
        result["error"] = "Connection to backend API timed out"
    except Exception as e:
        result["error"] = f"Error checking backend services: {str(e)}"
    
    return result

def check_frontend_services() -> Dict[str, Any]:
    """Check if frontend services are running and accessible."""
    result = {
        "docusaurus_running": False,
        "frontend_responding": False,
        "error": None
    }
    
    try:
        # Check if the frontend is responding at the expected port
        response = requests.get("http://localhost:3000", timeout=10)
        result["frontend_responding"] = response.status_code == 200
        
        # If the response is successful, assume Docusaurus is running
        if response.status_code == 200:
            result["docusaurus_running"] = True
            
    except requests.exceptions.ConnectionError:
        result["error"] = "Could not connect to frontend at http://localhost:3000"
    except requests.exceptions.Timeout:
        result["error"] = "Connection to frontend timed out"
    except Exception as e:
        result["error"] = f"Error checking frontend services: {str(e)}"
    
    return result

def test_end_to_end_flow() -> Dict[str, Any]:
    """Test the complete end-to-end flow from frontend to backend."""
    result = {
        "query_sent": False,
        "response_received": False,
        "valid_response_structure": False,
        "error": None
    }
    
    try:
        # Send a test query to the backend
        test_payload = {
            "query": "What is a humanoid robot?",
            "userId": "test-user-validation",
            "context": {"domain": "robotics", "source": "textbook"}
        }
        
        response = requests.post(
            "http://localhost:8000/api/query",
            json=test_payload,
            timeout=30  # Longer timeout for processing
        )
        
        result["query_sent"] = True
        
        if response.status_code == 200:
            result["response_received"] = True
            response_data = response.json()
            
            # Check if the response has the expected structure
            required_fields = ["id", "queryId", "content", "timestamp"]
            if all(field in response_data for field in required_fields):
                result["valid_response_structure"] = True
            else:
                result["error"] = f"Response missing required fields. Got: {list(response_data.keys())}"
        else:
            result["error"] = f"API returned status code {response.status_code}: {response.text}"
            
    except requests.exceptions.ConnectionError:
        result["error"] = "Could not connect to API for end-to-end test"
    except requests.exceptions.Timeout:
        result["error"] = "End-to-end test timed out"
    except Exception as e:
        result["error"] = f"Error in end-to-end test: {str(e)}"
    
    return result

def check_process_resources() -> Dict[str, Any]:
    """Check system resources and running processes."""
    result = {
        "cpu_percent": psutil.cpu_percent(interval=1),
        "memory_percent": psutil.virtual_memory().percent,
        "available_memory_gb": psutil.virtual_memory().available / (1024**3),
        "running_processes": len(psutil.pids()),
        "disk_usage_percent": psutil.disk_usage('/').percent if os.name != 'nt' else psutil.disk_usage('C:\\').percent
    }
    
    # Check for specific processes that should be running
    backend_found = False
    frontend_found = False
    
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
            name = proc.info['name'].lower()
            
            if 'python' in name and ('uvicorn' in cmdline or 'main' in cmdline or 'backend' in cmdline):
                backend_found = True
            elif ('node' in name or 'npm' in name) and ('start' in cmdline or 'docusaurus' in cmdline):
                frontend_found = True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    
    result["backend_process_found"] = backend_found
    result["frontend_process_found"] = frontend_found
    
    return result

def validate_system() -> Dict[str, Any]:
    """Perform comprehensive system validation."""
    print("Starting system validation...")
    
    # Check backend services
    print("Checking backend services...")
    backend_status = check_backend_services()
    
    # Check frontend services
    print("Checking frontend services...")
    frontend_status = check_frontend_services()
    
    # Test end-to-end flow
    print("Testing end-to-end flow...")
    e2e_status = test_end_to_end_flow()
    
    # Check system resources
    print("Checking system resources...")
    resource_status = check_process_resources()
    
    # Compile overall results
    overall_result = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "backend": backend_status,
        "frontend": frontend_status,
        "end_to_end": e2e_status,
        "resources": resource_status,
        "overall_success": True
    }
    
    # Determine overall success
    if not backend_status.get("api_responding"):
        overall_result["overall_success"] = False
        print("❌ Backend API is not responding")
    
    if not frontend_status.get("frontend_responding"):
        overall_result["overall_success"] = False
        print("❌ Frontend is not responding")
    
    if not e2e_status.get("valid_response_structure"):
        overall_result["overall_success"] = False
        print("❌ End-to-end flow failed")
        if e2e_status.get("error"):
            print(f"   Error: {e2e_status['error']}")
    
    if overall_result["overall_success"]:
        print("✅ All system validation checks passed!")
    else:
        print("❌ Some system validation checks failed")
    
    # Print detailed results
    print("\nDetailed Results:")
    print(f"Backend API responding: {'✅' if backend_status.get('api_responding') else '❌'}")
    print(f"Frontend responding: {'✅' if frontend_status.get('frontend_responding') else '❌'}")
    print(f"End-to-end flow working: {'✅' if e2e_status.get('valid_response_structure') else '❌'}")
    print(f"CPU usage: {resource_status['cpu_percent']}%")
    print(f"Memory usage: {resource_status['memory_percent']}%")
    print(f"Available memory: {resource_status['available_memory_gb']:.2f} GB")
    print(f"Disk usage: {resource_status['disk_usage_percent']}%")
    
    return overall_result

def main():
    """Main function to run the system validation."""
    try:
        result = validate_system()
        
        if result["overall_success"]:
            print("\n🎉 System validation completed successfully!")
            print("The entire system is working without errors in local development.")
            return 0
        else:
            print("\n❌ System validation failed!")
            print("There are issues that need to be resolved before deployment.")
            return 1
            
    except KeyboardInterrupt:
        print("\nValidation interrupted by user.")
        return 130
    except Exception as e:
        print(f"\nError during system validation: {str(e)}")
        return 1

if __name__ == "__main__":
    sys.exit(main())