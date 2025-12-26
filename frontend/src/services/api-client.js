// frontend/src/services/api-client.js
/**
 * HTTP client for communicating with the RAG backend
 */

import axios from 'axios';

class APIClient {
  constructor() {
    this.baseURL = process.env.REACT_APP_RAG_BACKEND_URL || 'http://localhost:8000';
    this.client = axios.create({
      baseURL: this.baseURL,
      timeout: 30000, // 30 second timeout
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${process.env.REACT_APP_API_KEY}`
      }
    });

    // Add request interceptor to include session ID if available
    this.client.interceptors.request.use(
      (config) => {
        const sessionId = localStorage.getItem('sessionId');
        if (sessionId) {
          config.headers['X-Session-ID'] = sessionId;
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    // Add response interceptor to handle common response patterns
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        console.error('API request failed:', error);
        return Promise.reject(error);
      }
    );
  }

  async sendQuery(queryData) {
    try {
      const response = await this.client.post('/query', queryData);
      return response.data;
    } catch (error) {
      console.error('Error sending query:', error);
      throw error;
    }
  }

  async healthCheck() {
    try {
      const response = await this.client.get('/health');
      return response.data;
    } catch (error) {
      console.error('Health check failed:', error);
      throw error;
    }
  }
}

export default new APIClient();