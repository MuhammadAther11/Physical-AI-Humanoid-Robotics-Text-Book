// frontend/src/models/query.js
/**
 * Query Request model based on data model specification
 */

export class QueryRequest {
  constructor(text, selectedText = null, sessionId = null) {
    this.id = this.generateId();
    this.text = text;
    this.selected_text = selectedText;
    this.session_id = sessionId;
    this.metadata = {};
    this.timestamp = new Date().toISOString();
  }

  generateId() {
    // Generate a simple UUID-like string
    return 'query_' + Math.random().toString(36).substr(2, 9);
  }

  validate() {
    if (!this.text || typeof this.text !== 'string') {
      throw new Error('Query text is required and must be a string');
    }
    if (this.text.length > 2000) {
      throw new Error('Query text must not exceed 2000 characters');
    }
    if (this.selected_text && typeof this.selected_text === 'string' && this.selected_text.length > 1000) {
      throw new Error('Selected text must not exceed 1000 characters');
    }
    return true;
  }
}