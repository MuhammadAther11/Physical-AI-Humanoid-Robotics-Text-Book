// frontend/src/models/request.js
/**
 * API Request model based on data model specification
 */

export class APIRequest {
  constructor(queryText, headers = {}, clientInfo = {}) {
    this.id = this.generateId();
    this.query_text = queryText;
    this.headers = headers;
    this.timestamp = new Date().toISOString();
    this.client_info = clientInfo;
  }

  generateId() {
    // Generate a simple UUID-like string
    return 'api_req_' + Math.random().toString(36).substr(2, 9);
  }
}