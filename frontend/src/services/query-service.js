// frontend/src/services/query-service.js
/**
 * Service for handling query-specific API functions
 */

import apiClient from './api-client';

class QueryService {
  async processQuery(queryText, selectedText = null, sessionId = null) {
    const queryData = {
      query: queryText,
      selected_text: selectedText,
      session_id: sessionId,
      include_citations: true
    };

    try {
      const response = await apiClient.sendQuery(queryData);
      return response;
    } catch (error) {
      console.error('Error processing query:', error);
      throw error;
    }
  }

  async validateQuery(queryText) {
    // Basic validation of query text
    if (!queryText || typeof queryText !== 'string') {
      throw new Error('Query text is required and must be a string');
    }

    if (queryText.trim().length < 3) {
      throw new Error('Query must be at least 3 characters long');
    }

    if (queryText.length > 2000) {
      throw new Error('Query must not exceed 2000 characters');
    }

    return true;
  }

  async formatResponseForUI(responseData) {
    // Format the response from the backend for UI display
    return {
      id: responseData.id,
      answer: responseData.answer,
      citations: responseData.citations || [],
      sources: responseData.sources || [],
      confidence_score: responseData.confidence_score,
      response_time_ms: responseData.response_time_ms,
      session_id: responseData.session_id
    };
  }
}

export default new QueryService();