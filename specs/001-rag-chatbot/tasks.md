# Tasks: RAG Chatbot Fix & UI Integration

**Input**: Design documents from `/specs/001-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), data-model.md, contracts/

**Tests**: Not requested - no test tasks included

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify environment and fix foundational issues that affect all user stories

- [X] T001 Verify Qdrant configuration in backend/.env (QDRANT_URL, QDRANT_API_KEY)
- [X] T002 Verify OpenAI configuration in backend/.env (OPENAI_API_KEY)
- [X] T003 Test Qdrant connection by running backend/src/qdrant_service.py health check
- [X] T004 Test OpenAI API connectivity using backend/src/openai_service.py
- [X] T005 Review existing content_ingestion.py for required fixes in backend/content_ingestion.py
- [X] T006 [P] Run existing backend tests to identify failures in backend/tests/

**Checkpoint**: Environment verified - all services accessible ✓ COMPLETED

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure fixes that MUST be complete before user stories can be tested

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Fix content_ingestion.py to successfully extract all book content from book-frontend/docs/
- [X] T008 [P] Fix content_ingestion.py embedding generation using OpenAI text-embedding-3-small
- [X] T009 [P] Fix content_ingestion.py to store embeddings in Qdrant collection "book_content"
- [X] T010 Add metadata (source_url, title, section_path) to Qdrant points in content_ingestion.py
- [X] T011 Run content_ingestion.py successfully to populate Qdrant with all book embeddings (35 chunks stored)
- [X] T012 [P] Add error handling for Qdrant connection failures in backend/src/qdrant_service.py
- [X] T013 [P] Add error handling for OpenAI API failures in backend/src/openai_service.py

**Checkpoint**: Foundational ready - embeddings stored, error handling in place ✓ COMPLETED

---

## Phase 3: User Story 1 - Ask Questions About Book Content (Priority: P1) MVP

**Goal**: Enable users to submit questions and receive answers from the RAG system

**Independent Test**: Open the homepage, type a question about the book, and verify the answer appears with sources

### Backend Implementation for User Story 1

- [X] T014 [US1] Update backend/src/api/api.py POST /api/query to return standardized response format
- [X] T015 [US1] Update backend/src/models/response.py to include sources array with url, title, relevanceScore
- [X] T016 [US1] Update backend/src/models/response.py to include confidence score field
- [X] T017 [US1] Update backend/src/models/response.py to include processingTimeMs field
- [X] T018 [US1] Update backend/src/models/response.py to include status enum (success, partial, error)
- [X] T019 [US1] Enhance backend/src/rag_agent_service.py to include source citations in response
- [X] T020 [US1] Enhance backend/src/rag_agent_service.py to calculate confidence score from retrieval
- [X] T021 [US1] Add processing time tracking in backend/src/query_processing_service.py

### Frontend Implementation for User Story 1

- [X] T022 [US1] Update book-frontend/src/services/api/queryService.js to call POST /api/query endpoint
- [X] T023 [US1] Handle response format including queryId, answer, sources in book-frontend/src/services/api/queryService.js
- [X] T024 [US1] Display chatbot response with sources in book-frontend/src/components/Chatbot/Chatbot.jsx
- [X] T025 [US1] Show source citations as clickable links in book-frontend/src/components/Chatbot/Chatbot.jsx

**Checkpoint**: User Story 1 complete - Q&A works with source citations ✓ COMPLETED

---

## Phase 4: User Story 2 - Access Chatbot from Homepage (Priority: P2)

**Goal**: Make the chatbot UI visible and accessible on the Docusaurus homepage

**Independent Test**: Load the homepage and verify the chatbot interface is visible without any navigation

### Frontend Implementation for User Story 2

- [X] T026 [US2] Add Chatbot component to homepage in book-frontend/src/pages/index.tsx
- [X] T027 [US2] Style Chatbot as floating widget in book-frontend/src/components/Chatbot/Chatbot.css
- [X] T028 [US2] Implement expand/collapse functionality for chat window in Chatbot.jsx
- [X] T029 [US2] Ensure chatbot remains visible during scroll in book-frontend/src/components/Chatbot/Chatbot.jsx
- [X] T030 [US2] Add loading spinner during API calls in book-frontend/src/components/Chatbot/Chatbot.jsx

**Checkpoint**: User Story 2 complete - chatbot visible and usable on homepage ✓ COMPLETED

---

## Phase 5: User Story 3 - Receive Accurate Book-Based Answers (Priority: P3)

**Goal**: Ensure answers are accurate and sourced exclusively from book content

**Independent Test**: Ask questions about book content and verify answers reference actual sections, with helpful response when topic not covered

### Backend Implementation for User Story 3

- [X] T031 [US3] Update backend/src/rag_agent_service.py to verify retrieved sources before generating
- [X] T032 [US3] Add "no results" response when retrieval returns empty in rag_agent_service.py
- [X] T033 [US3] Update backend/src/rag_agent_service.py to prevent hallucination (only use retrieved content)
- [X] T034 [US3] Add confidence threshold check in query_processing_service.py (reject if < 0.5)
- [X] T035 [US3] Enhance error handling to return user-friendly messages in backend/src/api/api.py

### Frontend Implementation for User Story 3

- [X] T036 [US3] Display "no information found" message in book-frontend/src/components/Chatbot/Chatbot.jsx
- [X] T037 [US3] Show confidence indicator or source quality indicator in Chatbot.jsx
- [X] T038 [US3] Display error messages with retry option in Chatbot.jsx

**Checkpoint**: User Story 3 complete - Accurate answers with verification and user-friendly responses ✓ COMPLETED

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T039 Validate complete end-to-end flow using quickstart.md test scenarios
- [X] T040 Test health endpoint returns correct status for all dependencies
- [X] T041 Verify rate limiting works correctly (10 requests per 60 seconds)
- [X] T042 [P] Add comprehensive logging to all RAG pipeline stages
- [X] T043 [P] Verify chatbot works on different page sizes (responsive design)
- [X] T044 Test all edge cases from spec (empty DB, connection failures, etc.)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Completed ✓
- **Foundational (Phase 2)**: Completed ✓
- **User Stories (Phases 3-5)**: US1 & US2 complete, US3 in progress
- **Polish (Phase 6)**: Pending

### User Story Status

| Story | Status | Description |
|-------|--------|-------------|
| US1: Q&A with Sources | ✓ COMPLETED | Users can ask questions and get answers with source citations |
| US2: Homepage Access | ✓ COMPLETED | Chatbot is visible as floating widget on homepage |
| US3: Accurate Answers | ✓ COMPLETED | Answers verified from sources with confidence indicators and error handling |

---

## Task Summary

| Phase | Task Count | Status |
|-------|------------|--------|
| Phase 1: Setup | 6 tasks | ✓ All Complete |
| Phase 2: Foundational | 7 tasks | ✓ All Complete |
| Phase 3: US1 | 12 tasks | ✓ All Complete |
| Phase 4: US2 | 5 tasks | ✓ All Complete |
| Phase 5: US3 | 8 tasks | ✓ All Complete |
| Phase 6: Polish | 6 tasks | ✓ All Complete |
| **Total** | **44 tasks** | **44 Complete, 0 Pending** |

---

## Notes

- **[P]** tasks = different files, no dependencies
- **[Story]** label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- The MVP is User Story 1 & 2 - Q&A works with sources on homepage
- User Story 3 is for enhanced accuracy and error handling
