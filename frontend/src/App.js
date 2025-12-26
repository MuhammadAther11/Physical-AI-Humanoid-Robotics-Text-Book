// frontend/src/App.js
/**
 * Main application component for the RAG Agent API
 * Orchestrates the flow: Query → retrieve → agent → response
 */

import React, { useState } from 'react';
import QueryInterface from './components/QueryInterface/QueryInterface';
import ResponseDisplay from './components/ResponseDisplay/ResponseDisplay';
import TextSelector from './components/TextSelector/TextSelector';
import useQuery from '../hooks/useQuery';

const App = () => {
  const [selectedText, setSelectedText] = useState('');
  const [sessionId, setSessionId] = useState(() => {
    // Try to get existing session ID from localStorage or create a new one
    const savedSessionId = localStorage.getItem('sessionId');
    if (savedSessionId) {
      return savedSessionId;
    }
    
    // Generate a new session ID
    const newSessionId = 'session_' + Date.now().toString(36) + Math.random().toString(36).substr(2, 5);
    localStorage.setItem('sessionId', newSessionId);
    return newSessionId;
  });

  const { queryResult, isLoading, error, processQuery } = useQuery();

  const handleSubmitQuery = async (queryText) => {
    try {
      // If there's selected text, include it in the query
      const finalQuery = selectedText ? `${queryText} (based on selected text: "${selectedText}")` : queryText;
      
      const result = await processQuery(finalQuery, selectedText, sessionId);
      setSelectedText(''); // Clear the selected text after submitting
      return result;
    } catch (err) {
      console.error('Error submitting query:', err);
      throw err;
    }
  };

  return (
    <div className="app-container">
      <header className="app-header">
        <h1>RAG Agent API - Textbook Content Assistant</h1>
        <p>Ask questions about the textbook content and get context-grounded answers</p>
      </header>

      <main className="app-main">
        <div className="query-section">
          <TextSelector onTextSelected={setSelectedText} />
          
          {selectedText && (
            <div className="selected-text-preview">
              <h4>Selected Text:</h4>
              <p>"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
            </div>
          )}
          
          <QueryInterface onSubmitQuery={handleSubmitQuery} />
        </div>

        <div className="response-section">
          <ResponseDisplay 
            response={queryResult} 
            isLoading={isLoading} 
            error={error} 
          />
        </div>
      </main>

      <footer className="app-footer">
        <p>RAG Agent API • Session ID: {sessionId}</p>
      </footer>
    </div>
  );
};

export default App;