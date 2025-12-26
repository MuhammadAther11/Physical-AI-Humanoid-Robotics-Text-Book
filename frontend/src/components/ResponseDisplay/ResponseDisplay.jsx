// frontend/src/components/ResponseDisplay/ResponseDisplay.jsx
/**
 * Component for displaying responses from the RAG backend with proper formatting
 */

import React from 'react';
import PropTypes from 'prop-types';

const ResponseDisplay = ({ response, isLoading, error }) => {
  if (error) {
    return (
      <div className="response-display error">
        <h3>Error:</h3>
        <p>{error}</p>
      </div>
    );
  }

  if (isLoading) {
    return (
      <div className="response-display loading">
        <p>Processing your query and retrieving relevant information...</p>
      </div>
    );
  }

  if (!response) {
    return (
      <div className="response-display empty">
        <p>Submit a query to see the response here.</p>
      </div>
    );
  }

  return (
    <div className="response-display">
      <div className="answer-section">
        <h3>Response:</h3>
        <div className="answer-text">
          {response.answer}
        </div>
      </div>

      {response.citations && response.citations.length > 0 && (
        <div className="citations-section">
          <h4>Citations:</h4>
          <ul className="citations-list">
            {response.citations.map((citation, index) => (
              <li key={index} className="citation-item">
                <a 
                  href={citation.url} 
                  target="_blank" 
                  rel="noopener noreferrer"
                  className="citation-link"
                >
                  {citation.section_title}
                </a>
                <span className="confidence-score">
                  (Confidence: {(citation.confidence * 100).toFixed(1)}%)
                </span>
                {citation.text_excerpt && (
                  <div className="excerpt">
                    "{citation.text_excerpt}"
                  </div>
                )}
              </li>
            ))}
          </ul>
        </div>
      )}

      {response.sources && response.sources.length > 0 && (
        <div className="sources-section">
          <h4>Sources:</h4>
          <ul className="sources-list">
            {response.sources.map((source, index) => (
              <li key={index} className="source-item">{source}</li>
            ))}
          </ul>
        </div>
      )}

      {response.confidence_score && (
        <div className="confidence-section">
          <p>Overall Confidence: {(response.confidence_score * 100).toFixed(1)}%</p>
        </div>
      )}
    </div>
  );
};

ResponseDisplay.propTypes = {
  response: PropTypes.shape({
    answer: PropTypes.string,
    citations: PropTypes.arrayOf(
      PropTypes.shape({
        url: PropTypes.string.isRequired,
        section_title: PropTypes.string.isRequired,
        confidence: PropTypes.number.isRequired,
        text_excerpt: PropTypes.string
      })
    ),
    sources: PropTypes.arrayOf(PropTypes.string),
    confidence_score: PropTypes.number
  }),
  isLoading: PropTypes.bool,
  error: PropTypes.string
};

ResponseDisplay.defaultProps = {
  response: null,
  isLoading: false,
  error: null
};

export default ResponseDisplay;