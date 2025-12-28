# Physical-AI-Humanoid-Robotics-Text-Book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-22

## Active Technologies
- Python 3.11 + Cohere API client, Qdrant client, requests, python-dotenv, uv (package manager) (001-retrieval-pipeline)
- Qdrant Cloud (vector database) (001-retrieval-pipeline)
- [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION] + [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION] (001-rag-agent-api)
- [if applicable, e.g., PostgreSQL, CoreData, files or N/A] (001-rag-agent-api)
- JavaScript/TypeScript (Node.js 18+ LTS) + Docusaurus 3.x, React, Node.js, npm (001-book-frontend-encapsulation)
- N/A (static site generator, no runtime storage needed) (001-book-frontend-encapsulation)
- JavaScript/TypeScript (Node.js 18+ LTS) + Docusaurus 3.x, React, Node.js, npm + Docusaurus 3.x, React, Node.js, npm, various Docusaurus plugins for theming and UI customization (007-docusaurus-ui-upgrade)
- Python 3.11 + FastAPI, OpenAI Agents SDK, Qdrant client, requests, python-dotenv (007-frontend-backend-integration)
- N/A (using Qdrant Cloud for vector storage, no local storage needed) (007-frontend-backend-integration)

- Python 3.11 + Cohere API client, Qdrant client, requests, beautifulsoup4, python-dotenv, uv (package manager) (003-content-ingestion)

## Project Structure

```text
src/
tests/
```

## Commands

cd src; pytest; ruff check .

## Code Style

Python 3.11: Follow standard conventions

## Recent Changes
- 007-frontend-backend-integration: Added Python 3.11 + FastAPI, OpenAI Agents SDK, Qdrant client, requests, python-dotenv
- 007-docusaurus-ui-upgrade: Added JavaScript/TypeScript (Node.js 18+ LTS) + Docusaurus 3.x, React, Node.js, npm + Docusaurus 3.x, React, Node.js, npm, various Docusaurus plugins for theming and UI customization
- 001-book-frontend-encapsulation: Added JavaScript/TypeScript (Node.js 18+ LTS) + Docusaurus 3.x, React, Node.js, npm


<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
