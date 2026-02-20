---
id: 1
title: Update Constitution
stage: constitution
date: 2026-02-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: dell
command: /sp.constitution
labels: [constitution, ai-book, rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```
/sp.constitution

Project: AI-Native Book with Integrated RAG Chatbot

Core Principles:
- Spec-Driven Development using Spec-Kit Plus
- AI-First implementation with Claude Code
- Modular, reusable intelligence (Subagents + Agent Skills)
- Production-ready deployment
- Source-grounded RAG (no hallucination)

Key Standards:

Book:
- Built with Docusaurus
- Deployed on GitHub Pages
- Minimum 8 chapters
- Technical clarity for developers
- Clean structure, navigation, and markdown organization
- Original content only

RAG Chatbot:
- Embedded inside the published book
- Backend: FastAPI
- Database: Neon Serverless Postgres
- Vector DB: Qdrant Cloud (Free Tier)
- AI SDK: OpenAI Agents / ChatKit
- Must support:
  - Full-book semantic search
  - Context-grounded answers
  - "Selected text only" answering mode
  - Retrieved-source citation

Architecture:
- Clear frontend/backend separation
- Secure environment variables
- Documented ingestion + embedding pipeline
- Deployment guide included
- OpenAPI docs enabled
```

## Response snapshot

Created updated constitution file at .specify/memory/constitution.md with the following principles:

1. Spec-Driven Development (NON-NEGOTIABLE)
2. AI-First Implementation
3. Modular Intelligence Architecture
4. Production-Ready Deployment
5. Source-Grounded RAG (NON-NEGOTIABLE)
6. Clean Architecture Separation

Also included technical standards for book requirements, RAG chatbot requirements, security, development workflow, and quality assurance.

## Outcome

- ✅ Impact: Updated project constitution with comprehensive principles for AI-Native Book with RAG Chatbot project
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Continue with project implementation following the established constitution
- 🧠 Reflection: Constitution now provides clear guidance for the entire project lifecycle

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin specification work for the AI book project