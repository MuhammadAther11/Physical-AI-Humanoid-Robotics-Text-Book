// frontend/src/hooks/useQuery.js
/**
 * Custom React hook for managing query state and processing
 */

import { useState, useCallback } from 'react';
import queryService from '../services/query-service';

const useQuery = () => {
  const [queryResult, setQueryResult] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const processQuery = useCallback(async (queryText, selectedText = null, sessionId = null) => {
    setIsLoading(true);
    setError(null);
    
    try {
      // Validate the query first
      await queryService.validateQuery(queryText);
      
      // Process the query through the service
      const response = await queryService.processQuery(queryText, selectedText, sessionId);
      
      // Format the response for UI
      const formattedResponse = await queryService.formatResponseForUI(response);
      
      setQueryResult(formattedResponse);
      return formattedResponse;
    } catch (err) {
      setError(err.message || 'An error occurred while processing the query');
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, []);

  const resetQuery = useCallback(() => {
    setQueryResult(null);
    setError(null);
    setIsLoading(false);
  }, []);

  return {
    queryResult,
    isLoading,
    error,
    processQuery,
    resetQuery
  };
};

export default useQuery;