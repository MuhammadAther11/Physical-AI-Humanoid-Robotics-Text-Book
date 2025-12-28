# Frontend ↔ Backend Integration - Physical AI & Humanoid Robotics

This project integrates a Docusaurus frontend with a FastAPI backend to enable communication with a RAG (Retrieval-Augmented Generation) agent for the Physical AI & Humanoid Robotics textbook.

## Prerequisites

- Python 3.11+
- Node.js 18+ LTS
- Access to Qdrant Cloud
- Access to OpenAI API (or compatible service)

## Project Completion Status

The Physical AI & Humanoid Robotics RAG system has been fully implemented and tested. All planned features are complete and the system has passed comprehensive end-to-end testing.

### Features Implemented

1. **Backend Services**:
   - FastAPI-based backend with comprehensive error handling
   - Integration with Qdrant vector database for retrieval
   - OpenAI API integration for generation
   - Proper validation and sanitization of inputs
   - Rate limiting to prevent abuse
   - Comprehensive logging and monitoring
   - Health check endpoints

2. **Frontend Components**:
   - Chatbot UI integrated into the Docusaurus frontend
   - API service for communicating with the backend
   - Loading states and error handling
   - Responsive design for different screen sizes

3. **Integration**:
   - Full integration between frontend and backend
   - End-to-end query flow from UI to RAG agent and back
   - Proper error handling throughout the stack
   - Security considerations implemented

### Testing Results

The system has undergone comprehensive testing:
- Unit tests for all major components
- Integration tests covering the complete query flow
- End-to-end tests validating the complete system
- Performance testing under various load conditions
- Security validation of API endpoints

All tests have passed and the system is ready for deployment.

### Local Development Setup

For future development or modifications, follow these steps:

#### Backend Setup

1. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

2. **Create a virtual environment** (if not already created):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**:
   Create a `.env` file in the backend root with the following:
   ```env
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   OPENAI_API_KEY=your_openai_api_key
   APP_NAME=RAG Backend Service
   DEBUG=False
   HOST=0.0.0.0
   PORT=8000
   MAX_QUERY_LENGTH=2000
   RAG_AGENT_MODEL=gpt-4o
   RAG_AGENT_TEMPERATURE=0.7
   QUERY_TIMEOUT=30
   ```

5. **Run the backend server**:
   ```bash
   cd src
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

#### Frontend Setup

1. **Navigate to the frontend directory**:
   ```bash
   cd book-frontend  # The existing Docusaurus frontend
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Configure API connection**:
   Create or update `.env` file in the frontend root:
   ```env
   REACT_APP_API_BASE_URL=http://localhost:8000
   ```

4. **Run the frontend server**:
   ```bash
   npm run start
   ```

## Running Both Together

For local development, you'll need both servers running simultaneously. In separate terminals:

1. **Terminal 1 - Backend**:
   ```bash
   cd backend
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   cd src
   uvicorn main:app --reload --port 8000
   ```

2. **Terminal 2 - Frontend**:
   ```bash
   cd book-frontend
   npm run start
   ```

## API Endpoints

- **Query Endpoint**: `POST /api/query`
  - Accepts user queries and returns responses from the RAG agent
  - Request body: `{"query": "your question", "userId": "optional user id", "context": "optional context"}`
  - Response: RAG agent's answer with sources

- **Health Check**: `GET /health`
  - Returns the health status of the API and its dependencies

- **API Documentation**: `GET /docs`
  - Interactive API documentation (Swagger UI)

- **API Schema**: `GET /redoc`
  - Alternative API documentation (ReDoc)

## Troubleshooting

- If the frontend can't reach the backend, check that CORS is properly configured in the FastAPI app
- If queries return empty responses, verify that your Qdrant Cloud instance has content indexed
- Check the console logs in both the frontend and backend for error messages
- If you encounter rate limiting errors, wait before submitting another query
- For timeout errors, try resubmitting the query

## Configuration Options

The backend can be configured using environment variables in the `.env` file:

- `APP_NAME`: Name of the service (default: "RAG Backend Service")
- `DEBUG`: Enable debug mode (default: "False")
- `HOST`: Host to bind to (default: "0.0.0.0")
- `PORT`: Port to run the server on (default: "8000")
- `MAX_QUERY_LENGTH`: Maximum length of queries in characters (default: "2000")
- `RAG_AGENT_MODEL`: OpenAI model to use (default: "gpt-4o")
- `RAG_AGENT_TEMPERATURE`: Creativity parameter for the model (default: "0.7")
- `QUERY_TIMEOUT`: Timeout for RAG agent calls in seconds (default: "30")

## Development Tips

- Use the `/docs` endpoint to explore and test the API interactively
- The system includes rate limiting (10 requests per minute per user) to prevent abuse
- All API requests and responses are logged for debugging purposes
- The system includes comprehensive error handling and will return helpful error messages

## Deployment Instructions

### Production Deployment

To deploy the application to a production environment:

#### Backend Deployment

1. **Prepare the environment**:
   - Set up a production server with Python 3.11+
   - Ensure firewall rules allow traffic on your chosen port (default 8000)

2. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd Physical-AI-Humanoid-Robotics-Text-Book/backend
   ```

3. **Set up the virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

4. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

5. **Configure environment variables**:
   Create a `.env` file with production settings:
   ```env
   QDRANT_URL=your_production_qdrant_url
   QDRANT_API_KEY=your_production_qdrant_api_key
   OPENAI_API_KEY=your_production_openai_api_key
   APP_NAME=RAG Backend Service
   DEBUG=False
   HOST=0.0.0.0
   PORT=8000
   MAX_QUERY_LENGTH=2000
   RAG_AGENT_MODEL=gpt-4o
   RAG_AGENT_TEMPERATURE=0.7
   QUERY_TIMEOUT=30
   ```

6. **Run the application with a production ASGI server**:
   ```bash
   cd src
   uvicorn main:app --host 0.0.0.0 --port 8000 --workers 4
   ```

   For even better production performance, consider using Gunicorn:
   ```bash
   gunicorn src.main:app --workers 4 --worker-class uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
   ```

7. **Set up process management**:
   Use systemd, supervisor, or pm2 to ensure the application restarts automatically:
   - Create a systemd service file at `/etc/systemd/system/rag-backend.service`
   - Or use a process manager like PM2

#### Frontend Deployment

1. **Build the static site**:
   ```bash
   cd book-frontend
   npm run build
   ```

2. **Serve the static files**:
   The build output is in the `build/` directory and can be served by any static file server like Nginx, Apache, or cloud services like AWS S3 + CloudFront.

3. **Configure the API URL**:
   Update the API URL in the frontend build to point to your production backend.

#### Docker Deployment (Alternative)

1. **Build the Docker image**:
   ```bash
   # For the backend
   docker build -t rag-backend -f Dockerfile.backend .

   # For the frontend (if using a separate container)
   docker build -t rag-frontend -f Dockerfile.frontend .
   ```

2. **Run the containers**:
   ```bash
   docker run -d -p 8000:8000 --env-file .env rag-backend
   ```

### Environment Variables for Production

Make sure to set these environment variables in your production environment:

- `DEBUG=False` - Disable debug mode in production
- `LOG_LEVEL=INFO` or `LOG_LEVEL=WARN` - Adjust logging verbosity
- Secure API keys and connection strings in environment variables, never hardcode them

### Monitoring and Logging

- The application logs to both console and files in the `logs/` directory
- Monitor the log files for errors and performance issues
- Set up alerts for critical errors or performance degradation
- Use centralized logging solutions like ELK stack or cloud logging services for production environments