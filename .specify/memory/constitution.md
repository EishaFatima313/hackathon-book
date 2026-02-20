<!--
Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: None (new constitution created)
Added sections: All core principles and sections for AI-Native Book with RAG Chatbot
Removed sections: None
Templates requiring updates: ✅ .specify/templates/plan-template.md - updated
                        ✅ .specify/templates/spec-template.md - updated
                        ✅ .specify/templates/tasks-template.md - updated
                        ⚠️ .specify/templates/commands/*.md - pending review
                        ⚠️ README.md - pending review
Follow-up TODOs: None
-->
# AI-Native Book with Integrated RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven Development (NON-NEGOTIABLE)
All development follows Spec-Kit Plus methodology with formal specifications preceding implementation; Every feature must have a complete spec before coding begins; Changes to specs require explicit approval before implementation proceeds.
<!-- Rationale: Ensures systematic development, reduces rework, and maintains alignment between requirements and deliverables -->

### II. AI-First Implementation
Every component leverages Claude Code and AI-native approaches as the primary development methodology; Manual coding is reserved only for cases where AI assistance is inadequate; All code generation and modification is AI-assisted.
<!-- Rationale: Maximizes efficiency and leverages cutting-edge AI capabilities for rapid development -->

### III. Modular Intelligence Architecture
Intelligence is organized into modular, reusable subagents and agent skills; Clear interfaces between components with well-defined contracts; Components must be independently testable and deployable.
<!-- Rationale: Enables scalable, maintainable architecture with reusable intelligence modules -->

### IV. Production-Ready Deployment
All features are developed with production readiness in mind from the start; Includes proper error handling, monitoring, security considerations, and performance optimization; No "development-only" features.
<!-- Rationale: Ensures stable, reliable production deployments without extensive rework -->

### V. Source-Grounded RAG (NON-NEGOTIABLE)
All chatbot responses must be grounded in actual document sources; Hallucinations are strictly prohibited; Retrieved-source citations must be provided with every response.
<!-- Rationale: Maintains trust and accuracy of the AI system by ensuring all responses are factually grounded -->

### VI. Clean Architecture Separation
Strict separation between frontend and backend components; Clear API boundaries with well-defined contracts; Frontend and backend can be developed and deployed independently.
<!-- Rationale: Enables scalable development and allows for technology flexibility -->

## Technical Standards

### Book Requirements
- Built with Docusaurus for static site generation
- Deployed on GitHub Pages for public accessibility
- Minimum 8 chapters with technical clarity for developers
- Clean structure with proper navigation and markdown organization
- Original content only, no copied material
- Responsive design for multiple device sizes

### RAG Chatbot Requirements
- Embedded seamlessly inside the published book interface
- Backend implemented with FastAPI for high-performance API serving
- Database: Neon Serverless Postgres for scalability
- Vector DB: Qdrant Cloud (Free Tier) for semantic search capabilities
- AI SDK: OpenAI Agents / ChatKit for advanced AI interactions
- Must support: full-book semantic search, context-grounded answers, "selected text only" answering mode, retrieved-source citation

### Security and Environment
- Secure environment variables using proper secrets management
- No hardcoded credentials or API keys in source code
- Environment-specific configurations properly isolated
- Authentication and authorization where applicable

## Development Workflow

### Implementation Standards
- Follow Spec-Driven Development workflow using Spec-Kit Plus
- Create specifications before implementation
- Generate tasks from specifications
- Implement following the plan and task breakdown
- Maintain precise code references and testable changes

### Quality Assurance
- All changes must include appropriate testing
- Code reviews required for all substantial changes
- Automated validation of RAG accuracy and grounding
- Performance testing for search and response times
- Cross-browser compatibility for the book interface

### Documentation Requirements
- Comprehensive API documentation with OpenAPI enabled
- Clear deployment guides with step-by-step instructions
- Documented ingestion and embedding pipeline processes
- Architecture decision records for significant choices
- User guides for both book navigation and chatbot usage

## Governance

All development must comply with these constitutional principles; Deviations require explicit approval and documentation; Code reviews verify constitutional compliance; This constitution governs all technical and process decisions for the project.

**Version**: 1.1.0 | **Ratified**: 2026-02-13 | **Last Amended**: 2026-02-13
