// frontend/src/components/TextSelector/TextSelector.jsx
/**
 * Component for selecting text in the textbook content and initiating queries
 */

import React, { useEffect, useCallback } from 'react';
import PropTypes from 'prop-types';

const TextSelector = ({ onTextSelected }) => {
  const handleTextSelection = useCallback(() => {
    const selectedText = window.getSelection().toString().trim();
    
    if (selectedText) {
      // Call the callback with the selected text
      onTextSelected(selectedText);
    }
  }, [onTextSelected]);

  useEffect(() => {
    const handleMouseUp = () => {
      handleTextSelection();
    };

    document.addEventListener('mouseup', handleMouseUp);
    
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [handleTextSelection]);

  return (
    <div className="text-selector">
      <p>Select text in the textbook content to initiate a contextual query.</p>
      <div className="selection-indicator">
        {/* This could be enhanced with visual indicators for selected text */}
      </div>
    </div>
  );
};

TextSelector.propTypes = {
  onTextSelected: PropTypes.func.isRequired
};

export default TextSelector;