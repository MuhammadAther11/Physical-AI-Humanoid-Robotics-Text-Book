// frontend/src/components/QueryInterface/QueryInterface.jsx
/**
 * Component for handling user query input and submission
 */

import React, { useState } from 'react';
import PropTypes from 'prop-types';

const QueryInterface = ({ onSubmitQuery }) => {
  const [query, setQuery] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    if (!query.trim()) {
      setError('Query cannot be empty');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      await onSubmitQuery(query);
    } catch (err) {
      setError(err.message || 'An error occurred while processing your query');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="query-interface">
      <form onSubmit={handleSubmit} className="query-form">
        <div className="query-input-container">
          <textarea
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="Enter your question about the textbook content..."
            rows={4}
            disabled={isLoading}
            className="query-textarea"
          />
        </div>
        <button 
          type="submit" 
          disabled={isLoading || !query.trim()}
          className="submit-button"
        >
          {isLoading ? 'Processing...' : 'Submit Query'}
        </button>
      </form>
      
      {error && (
        <div className="error-message">
          Error: {error}
        </div>
      )}
      
      {isLoading && (
        <div className="loading-indicator">
          Processing your query...
        </div>
      )}
    </div>
  );
};

QueryInterface.propTypes = {
  onSubmitQuery: PropTypes.func.isRequired
};

export default QueryInterface;