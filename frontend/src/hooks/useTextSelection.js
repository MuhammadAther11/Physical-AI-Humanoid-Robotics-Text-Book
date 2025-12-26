// frontend/src/hooks/useTextSelection.js
/**
 * Custom React hook for managing text selection functionality
 */

import { useState, useEffect, useCallback } from 'react';

const useTextSelection = () => {
  const [selectedText, setSelectedText] = useState('');
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });

  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text) {
      // Get the position of the selection for potential UI elements (like a query button)
      if (selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectionPosition({ x: rect.left, y: rect.top });
      }

      setSelectedText(text);
    } else {
      setSelectedText('');
    }
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [handleSelection]);

  const clearSelection = useCallback(() => {
    setSelectedText('');
    window.getSelection().removeAllRanges();
  }, []);

  return {
    selectedText,
    selectionPosition,
    clearSelection
  };
};

export default useTextSelection;