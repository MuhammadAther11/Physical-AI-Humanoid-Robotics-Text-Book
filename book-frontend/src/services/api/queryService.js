// API service function for query submission in book-frontend/src/services/api/queryService.js

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

class QueryService {
  /**
   * Submit a query to the backend API with retry logic
   * @param {Object} queryData - The query data to submit
   * @param {string} queryData.query - The user's query
   * @param {string} [queryData.userId] - Optional user ID
   * @param {Object} [queryData.context] - Optional context for the query
   * @param {number} [maxRetries=3] - Maximum number of retry attempts
   * @param {number} [retryDelay=1000] - Delay in milliseconds between retries
   * @returns {Promise<Object>} The response from the backend
   */
  static async submitQuery(queryData, maxRetries = 3, retryDelay = 1000) {
    let lastError;

    for (let attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        const response = await fetch(`${API_BASE_URL}/api/query`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(queryData),
        });

        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        return await response.json();
      } catch (error) {
        lastError = error;
        console.error(`Attempt ${attempt} failed:`, error.message);

        // If this was the last attempt, throw the error
        if (attempt === maxRetries) {
          console.error('All retry attempts failed. Throwing error.');
          throw lastError;
        }

        // Wait before retrying (with exponential backoff)
        const delay = retryDelay * Math.pow(2, attempt - 1); // Exponential backoff
        console.log(`Retrying in ${delay}ms...`);
        await new Promise(resolve => setTimeout(resolve, delay));
      }
    }
  }

  /**
   * Test the connection to the backend API with retry logic
   * @param {number} [maxRetries=3] - Maximum number of retry attempts
   * @param {number} [retryDelay=1000] - Delay in milliseconds between retries
   * @returns {Promise<boolean>} Whether the connection is successful
   */
  static async testConnection(maxRetries = 3, retryDelay = 1000) {
    let lastError;

    for (let attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        const response = await fetch(`${API_BASE_URL}/health`);
        if (response.ok) {
          return true;
        } else {
          throw new Error(`Health check failed with status: ${response.status}`);
        }
      } catch (error) {
        lastError = error;
        console.error(`Health check attempt ${attempt} failed:`, error.message);

        // If this was the last attempt, return false
        if (attempt === maxRetries) {
          console.error('All health check retry attempts failed.');
          return false;
        }

        // Wait before retrying
        await new Promise(resolve => setTimeout(resolve, retryDelay));
      }
    }

    return false;
  }
}

export default QueryService;