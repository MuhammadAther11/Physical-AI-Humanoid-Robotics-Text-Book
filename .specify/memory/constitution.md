<!--
SYNC IMPACT REPORT

- **Version Change**: `none` → `1.0.0`
- **Change Type**: Major (Initial Ratification)
- **Summary**: Establishes the foundational principles, standards, and governance for the "AI-Native Textbook & Interactive Platform" project.
- **Affected Templates**:
    - [ ] `.specify/templates/plan-template.md` (Review for alignment with new principles)
    - [ ] `.specify/templates/spec-template.md` (Review for alignment with new principles)
    - [ ] `.specify/templates/tasks-template.md` (Review for alignment with new principles)
    - [ ] `.claude/commands/sp.plan.md` (Review for alignment with new principles)
-->
# AI-Native Textbook & Interactive Platform Constitution

“Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Systems”

## Core Principles

### I. Technical Accuracy and Source-Grounded Content
All educational content, including text, diagrams, and code, MUST be technically accurate and grounded in citable, reputable sources. This is non-negotiable.
*Rationale: To establish credibility and provide a reliable learning resource for advanced students and professionals.*

### II. Reproducible and High-Quality Code
All code, examples, and simulations MUST be fully reproducible. Code MUST be runnable, clearly annotated, and version-aware to function with the specified toolchain.
*Rationale: To allow users to verify, experiment with, and build upon the concepts presented.*

### III. Clarity for the Target Audience
The material MUST be written with a clear, direct style suitable for advanced students and practitioners in robotics and AI. It should be neither oversimplified nor excessively academic.
*Rationale: To ensure the educational objectives are met effectively and the content is accessible to its intended audience.*

### IV. Zero Hallucinations and Strict Context-Bound AI
AI-generated content, particularly from the RAG system, is STRICTLY FORBIDDEN from producing "hallucinations" or fabricated information. All AI-powered answers MUST be derived solely from the indexed project content and be context-bound to the user's selection where applicable.
*Rationale: To prevent the spread of misinformation and ensure the chatbot is a reliable, trustworthy tool.*

### V. Adherence to a Standardized Toolchain
The project MUST strictly adhere to the specified technology stack and tools: ROS 2, Gazebo, Unity, NVIDIA Isaac Sim, OpenAI Agents, ChatKit, FastAPI, Qdrant Cloud, and Neon Serverless Postgres.
*Rationale: To ensure consistency, simplify the development environment, and guarantee interoperability across all modules.*

### VI. Realistic Humanoid Robotics Architectures
All system designs, simulations, and architectural patterns MUST reflect realistic and modern humanoid robotics architectures.
*Rationale: To provide practical, real-world applicable knowledge that is relevant to the current state of the field.*

## Platform and Deployment

The development environment is centered on Claude Code + Spec-Kit Plus. The final platform will be built with Docusaurus and deployed via GitHub Pages.

## Success Criteria

The project's success is measured by the following high-level outcomes:
- The entire platform builds and deploys cleanly without errors.
- All curriculum modules and interactive components render correctly.
- The RAG-powered chatbot provides consistently accurate, in-scope answers.
- The final capstone project is fully and easily reproducible by a user following the guide.

## Governance

This constitution is the single source of truth for project standards and principles. All contributions, reviews, and development activities MUST align with it. Amendments require a documented proposal, review, and an explicit update to the version number.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16