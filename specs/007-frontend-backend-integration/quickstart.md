# Quickstart Guide: Frontend ↔ Backend Integration

## Prerequisites

- Python 3.11+
- Node.js 18+ LTS
- Access to Qdrant Cloud
- Access to OpenAI API (or compatible service)

## Setting Up the Backend

1. **Navigate to the backend directory**:
   ```bash
   cd backend
   ```

2. **Create a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv openai-agent qdrant-client requests
   ```

4. **Set up environment variables**:
   Create a `.env` file in the backend root with the following:
   ```env
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   OPENAI_API_KEY=your_openai_api_key
   ```

5. **Run the backend server**:
   ```bash
   uvicorn src.api.api:app --reload --port 8000
   ```

## Setting Up the Frontend

1. **Navigate to the frontend directory**:
   ```bash
   cd book-frontend  # The existing Docusaurus frontend
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Run the frontend server**:
   ```bash
   npm run start
   ```

## Running Both Together

For local development, you'll need both servers running simultaneously. In separate terminals:

1. **Terminal 1 - Backend**:
   ```bash
   cd backend
   uvicorn src.api.api:app --reload --port 8000
   ```

2. **Terminal 2 - Frontend**:
   ```bash
   cd book-frontend
   npm run start
   ```

## Testing the Integration

1. Make sure both servers are running
2. Open your browser to `http://localhost:3000` (frontend)
3. The chatbot UI should be integrated into the page
4. Submit a query to test the end-to-end functionality
5. Check the browser's developer tools for any API communication errors

## API Endpoint Details

The backend exposes a single endpoint for queries:
- **URL**: `http://localhost:8000/api/query`
- **Method**: POST
- **Content-Type**: application/json
- **Request Body**: `{"query": "your question here"}`
- **Response**: JSON object with the answer and sources

## Troubleshooting

- If the frontend can't reach the backend, check that CORS is properly configured in the FastAPI app
- If queries return empty responses, verify that your Qdrant Cloud instance has content indexed
- Check the console logs in both the frontend and backend for error messages