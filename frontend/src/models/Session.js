// frontend/src/models/Session.js
/**
 * Agent Session model based on data model specification
 */

export class SessionContext {
  constructor(userId = null) {
    this.id = this.generateId();
    this.user_id = userId;
    this.created_at = new Date().toISOString();
    this.last_activity = new Date().toISOString();
    this.query_history = [];
    this.preferences = {
      response_format: 'standard',
      citation_style: 'ieee'
    };
  }

  generateId() {
    // Generate a simple UUID-like string
    return 'session_' + Math.random().toString(36).substr(2, 9);
  }

  addQueryToHistory(queryId) {
    const historyEntry = {
      query_id: queryId,
      timestamp: new Date().toISOString()
    };
    this.query_history.push(historyEntry);
    this.last_activity = new Date().toISOString();
  }

  validate() {
    if (this.query_history.length > 50) {
      throw new Error('Query history should not exceed 50 items to prevent excessive memory usage');
    }
    return true;
  }
}