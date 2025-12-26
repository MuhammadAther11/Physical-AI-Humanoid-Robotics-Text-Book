// frontend/src/models/response.js
/**
 * Response Object model based on data model specification
 */

export class ResponseObject {
  constructor(queryId, text, citations = []) {
    this.id = this.generateId();
    this.query_id = queryId;
    this.text = text;
    this.citations = citations;
    this.sources = [];
    this.confidence_score = 0.0;
    this.token_usage = {
      input_tokens: 0,
      output_tokens: 0,
      total_tokens: 0
    };
    this.timestamp = new Date().toISOString();
  }

  generateId() {
    // Generate a simple UUID-like string
    return 'resp_' + Math.random().toString(36).substr(2, 9);
  }

  validate() {
    if (!this.query_id) {
      throw new Error('Query ID is required');
    }
    if (typeof this.confidence_score !== 'number' || this.confidence_score < 0.0 || this.confidence_score > 1.0) {
      throw new Error('Confidence score must be a number between 0.0 and 1.0');
    }
    return true;
  }
}