# Implementation Tasks: AI-Native Textbook Platform

**Feature**: `001-ai-native-textbook-platform`
**Branch**: `001-ai-native-textbook-platform`
**Date**: 2025-12-12
**Spec**: `.specify/specs/001-ai-textbook-platform/spec.md`
**Plan**: `.specify/specs/001-ai-textbook-platform/plan.md`

---

## Overview

This document breaks down the implementation plan into atomic, actionable tasks organized by the 11 phases defined in `plan.md`. Each task is independently executable with clear acceptance criteria, file paths, and dependencies.

**Task Format**:
- `- [ ] T### [P] [Story] Description with file path`
- `[P]` = Parallelizable (no dependencies on incomplete tasks)
- `[Story]` = User Story label (US1-US5) for traceability

**Total Tasks**: 331
**Phases**: 11 (Setup → Book Scaffold → Content → Backend → RAG → Auth → Personalization → Translation → Chatbot UI → Deployment → Testing → Validation)

---

## Phase 1: Book Scaffold (Docusaurus v3)

**Objective**: Initialize Docusaurus frontend with proper structure, navigation, and theming.

**Spec References**: FR-001 to FR-005 (Core Content Delivery)
**User Story**: US1 (Browse and Read Textbook Content)
**Subagent**: `content-implementor`
**Skills**: `book-scaffolding`, `docusaurus-deployer`, `canonical-format-checker`
**MCP Tools**: Context7 (Docusaurus v3 conventions)

### Tasks

- [ ] T001 [US1] Initialize Docusaurus project in \`apps/docs/\` using \`npx create-docusaurus@latest apps/docs classic --typescript\`
- [ ] T002 [US1] Verify \`apps/docs/package.json\` includes Docusaurus 3.x dependencies
- [ ] T003 [US1] Configure \`apps/docs/docusaurus.config.js\` with site title "Physical AI & Humanoid Robotics Textbook"
- [ ] T004 [US1] Update \`apps/docs/docusaurus.config.js\` to enable code highlighting for Python, C++, YAML, Shell
- [ ] T005 [US1] Add Mermaid plugin to \`apps/docs/docusaurus.config.js\` for diagram rendering
- [ ] T006 [US1] Create \`apps/docs/sidebars.js\` with 4-module hierarchy (modules 1-4, weeks 1-13)
- [ ] T007 [P] [US1] Create directory structure \`apps/docs/docs/module-1/\` for Weeks 1-3 (ROS 2)
- [ ] T008 [P] [US1] Create directory structure \`apps/docs/docs/module-2/\` for Weeks 4-7 (Gazebo & Unity)
- [ ] T009 [P] [US1] Create directory structure \`apps/docs/docs/module-3/\` for Weeks 8-10 (NVIDIA Isaac)
- [ ] T010 [P] [US1] Create directory structure \`apps/docs/docs/module-4/\` for Weeks 11-13 (VLA)
- [ ] T011 [US1] Create chapter template file \`apps/docs/.templates/chapter-template.mdx\` with canonical structure (intro, concepts, code, exercises, quiz, summary)
- [ ] T012 [US1] Customize Docusaurus theme in \`apps/docs/src/theme/\` to add chatbot button placeholder in navbar
- [ ] T013 [US1] Add space for personalization/translation toggles in \`apps/docs/src/theme/DocItem/Layout/index.tsx\`
- [ ] T014 [US1] Configure color scheme in \`apps/docs/src/css/custom.css\` for educational content (readable, professional)
- [ ] T015 [US1] Run \`cd apps/docs && npm run build\` to verify build succeeds
- [ ] T016 [US1] Run \`cd apps/docs && npm run start\` to verify dev server works and navigation sidebar displays correctly

**Phase 1 Acceptance Criteria**:
- ✅ Docusaurus project builds without errors
- ✅ Navigation sidebar shows 4 modules with correct week structure
- ✅ Code blocks render with syntax highlighting for all 4 languages
- ✅ Mermaid plugin configured and ready for diagram rendering

---

## Phase 2: Chapter Generation (13-Week Curriculum)

**Objective**: Populate textbook with complete 13-week Physical AI & Humanoid Robotics curriculum content.

**Spec References**: FR-001, FR-002, FR-004, FR-005
**User Story**: US1 (Browse and Read Textbook Content)
**Subagents**: \`chapter-planner\`, \`pedagogical-designer\`, \`content-implementor\`, \`factual-verifier\`, \`assessment-architect\`
**Skills**: \`chapter-planner\`, \`learning-objectives\`, \`code-example-generator\`, \`concept-scaffolding\`, \`exercise-designer\`, \`quiz-generator\`, \`technical-clarity\`, \`canonical-format-checker\`
**MCP Tools**: Context7 (ROS2, Isaac, Gazebo, Unity, VLA technical documentation)

### Module 1: Robotic Nervous System (ROS 2) - Weeks 1-3

- [ ] T017 [P] [US1] Write \`apps/docs/docs/module-1/week-1-intro-to-physical-ai.mdx\` covering foundations, embodied intelligence, humanoid landscape, sensor systems (5-7 pages)
- [ ] T018 [P] [US1] Write \`apps/docs/docs/module-1/week-2-ros2-fundamentals.mdx\` covering architecture, nodes, topics, services, actions (5-7 pages)
- [ ] T019 [P] [US1] Write \`apps/docs/docs/module-1/week-3-ros2-packages.mdx\` covering package creation, launch files, parameter management (5-7 pages)
- [ ] T020 [P] [US1] Add code examples to Week 1 chapter: sensor data reading (LiDAR, camera, IMU) in Python
- [ ] T021 [P] [US1] Add code examples to Week 2 chapter: ROS 2 node creation, publisher/subscriber, service client/server in Python (rclpy)
- [ ] T022 [P] [US1] Add code examples to Week 3 chapter: package.xml, setup.py, launch files with parameters
- [ ] T023 [P] [US1] Add Mermaid diagrams to Week 1: sensor pipeline architecture
- [ ] T024 [P] [US1] Add Mermaid diagrams to Week 2: ROS 2 node communication graph
- [ ] T025 [P] [US1] Add exercises to Week 1 chapter (3-5 exercises with solutions)
- [ ] T026 [P] [US1] Add exercises to Week 2 chapter (3-5 exercises with solutions)
- [ ] T027 [P] [US1] Add exercises to Week 3 chapter (3-5 exercises with solutions)
- [ ] T028 [P] [US1] Add quiz to Week 1 chapter (5-10 questions with correct answers)
- [ ] T029 [P] [US1] Add quiz to Week 2 chapter (5-10 questions with correct answers)
- [ ] T030 [P] [US1] Add quiz to Week 3 chapter (5-10 questions with correct answers)

### Module 2: Digital Twin (Gazebo & Unity) - Weeks 4-7

- [ ] T031 [P] [US1] Write \`apps/docs/docs/module-2/week-4-gazebo-simulation.mdx\` covering Gazebo environment, physics simulation (5-7 pages)
- [ ] T032 [P] [US1] Write \`apps/docs/docs/module-2/week-5-robot-description.mdx\` covering URDF and SDF formats for robot modeling (5-7 pages)
- [ ] T033 [P] [US1] Write \`apps/docs/docs/module-2/week-6-unity-robotics.mdx\` covering Unity for high-fidelity rendering, human-robot interaction (5-7 pages)
- [ ] T034 [P] [US1] Write \`apps/docs/docs/module-2/week-7-sensor-simulation.mdx\` covering LiDAR, depth cameras, IMU simulation (5-7 pages)
- [ ] T035 [P] [US1] Add code examples to Week 4: Gazebo world files, spawning robots, collision detection
- [ ] T036 [P] [US1] Add code examples to Week 5: URDF robot descriptions for humanoid robots
- [ ] T037 [P] [US1] Add code examples to Week 6: Unity ROS bridge setup, scene creation
- [ ] T038 [P] [US1] Add code examples to Week 7: Simulated sensor data processing
- [ ] T039 [P] [US1] Add Mermaid diagrams to Week 4-7 chapters (physics pipeline, sensor data flow)
- [ ] T040 [P] [US1] Add exercises to Week 4-7 chapters (3-5 exercises each with solutions)
- [ ] T041 [P] [US1] Add quizzes to Week 4-7 chapters (5-10 questions each with correct answers)

### Module 3: AI-Robot Brain (NVIDIA Isaac) - Weeks 8-10

- [ ] T042 [P] [US1] Write \`apps/docs/docs/module-3/week-8-isaac-sim.mdx\` covering Isaac Sim setup, photorealistic simulation, synthetic data generation (5-7 pages)
- [ ] T043 [P] [US1] Write \`apps/docs/docs/module-3/week-9-isaac-ros.mdx\` covering Isaac ROS, VSLAM, navigation (5-7 pages)
- [ ] T044 [P] [US1] Write \`apps/docs/docs/module-3/week-10-nav2.mdx\` covering Nav2 path planning for bipedal humanoids (5-7 pages)
- [ ] T045 [P] [US1] Add code examples to Week 8: Isaac Sim Python API, environment setup, robot spawn
- [ ] T046 [P] [US1] Add code examples to Week 9: Isaac ROS VSLAM configuration, navigation stack
- [ ] T047 [P] [US1] Add code examples to Week 10: Nav2 configuration for humanoid locomotion
- [ ] T048 [P] [US1] Add Mermaid diagrams to Week 8-10 chapters (Isaac architecture, VSLAM pipeline, Nav2 flow)
- [ ] T049 [P] [US1] Add exercises to Week 8-10 chapters (3-5 exercises each with solutions)
- [ ] T050 [P] [US1] Add quizzes to Week 8-10 chapters (5-10 questions each with correct answers)

### Module 4: Vision-Language-Action (VLA) - Weeks 11-13

- [ ] T051 [P] [US1] Write \`apps/docs/docs/module-4/week-11-vla-fundamentals.mdx\` covering VLA convergence, voice-to-action with Whisper (5-7 pages)
- [ ] T052 [P] [US1] Write \`apps/docs/docs/module-4/week-12-cognitive-planning.mdx\` covering LLMs for task planning, natural language to ROS actions (5-7 pages)
- [ ] T053 [P] [US1] Write \`apps/docs/docs/module-4/week-13-capstone-project.mdx\` covering autonomous humanoid project: voice command → navigation → object manipulation (5-7 pages)
- [ ] T054 [P] [US1] Add code examples to Week 11: OpenAI Whisper integration, speech-to-text pipeline
- [ ] T055 [P] [US1] Add code examples to Week 12: LLM-based task decomposition, action sequence generation
- [ ] T056 [P] [US1] Add code examples to Week 13: Full capstone implementation with computer vision, manipulation
- [ ] T057 [P] [US1] Add Mermaid diagrams to Week 11-13 chapters (VLA architecture, cognitive planning flow, capstone system design)
- [ ] T058 [P] [US1] Add exercises to Week 11-13 chapters (3-5 exercises each with solutions)
- [ ] T059 [P] [US1] Add quizzes to Week 11-13 chapters (5-10 questions each with correct answers)

### Content Validation

- [ ] T060 [US1] Run \`factual-verifier\` subagent to verify technical accuracy of all Module 1 chapters using Context7 (ROS 2 docs)
- [ ] T061 [US1] Run \`factual-verifier\` subagent to verify technical accuracy of all Module 2 chapters using Context7 (Gazebo, Unity docs)
- [ ] T062 [US1] Run \`factual-verifier\` subagent to verify technical accuracy of all Module 3 chapters using Context7 (Isaac docs)
- [ ] T063 [US1] Run \`factual-verifier\` subagent to verify technical accuracy of all Module 4 chapters using Context7 (VLA, Whisper docs)
- [ ] T064 [US1] Verify all chapters follow canonical format (intro, concepts, code, exercises, quiz, summary) per Constitution Article V
- [ ] T065 [US1] Test all code examples in chapters are functional and syntactically correct
- [ ] T066 [US1] Verify all Mermaid diagrams render correctly in Docusaurus
- [ ] T067 [US1] Run \`cd apps/docs && npm run build\` to verify all MDX files compile without errors

**Phase 2 Acceptance Criteria**:
- ✅ All 13 weeks of content written (65-90 chapters total)
- ✅ Each chapter follows canonical structure
- ✅ All code examples tested and functional
- ✅ All diagrams render correctly
- ✅ Content reviewed by \`factual-verifier\` for technical accuracy

---

## Phase 3: Backend API (FastAPI)

**Objective**: Scaffold FastAPI backend with database connections, API routing, and configuration management.

**Spec References**: FR-006 to FR-029 (all backend-dependent features)
**Subagents**: \`spec-architect\`, \`content-implementor\`
**Skills**: \`mvp-builder\`, \`tool-selection-framework\`, \`technical-clarity\`
**MCP Tools**: Context7 (FastAPI, Neon, Qdrant client usage)

### Backend Initialization

- [ ] T068 Create \`apps/api/\` directory structure with subdirectories: \`app/\`, \`tests/\`
- [ ] T069 Create \`apps/api/requirements.txt\` with dependencies: fastapi, uvicorn[standard], psycopg[binary], qdrant-client, openai, pydantic, pydantic-settings, python-multipart, python-jose[cryptography], passlib[bcrypt], sqlalchemy[asyncio]
- [ ] T070 Create \`apps/api/app/main.py\` with FastAPI app initialization and CORS middleware configuration
- [ ] T071 Create \`apps/api/app/core/config.py\` using pydantic-settings to load environment variables (DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY, SECRET_KEY, GITHUB_CLIENT_ID, GITHUB_CLIENT_SECRET)
- [ ] T072 Create \`apps/api/.env.example\` template with all required environment variable placeholders
- [ ] T073 Create \`apps/api/app/db/session.py\` with SQLAlchemy async session factory for Neon PostgreSQL
- [ ] T074 Create \`apps/api/app/db/vector_store.py\` with Qdrant client initialization
- [ ] T075 Test database connections: run \`uvicorn app.main:app --reload\` and verify Neon + Qdrant connections succeed

### API Router Structure

- [ ] T076 [P] Create \`apps/api/app/api/routes/auth.py\` with placeholder endpoints (signup, login, logout, me, profile)
- [ ] T077 [P] Create \`apps/api/app/api/routes/rag.py\` with placeholder endpoint (query)
- [ ] T078 [P] Create \`apps/api/app/api/routes/personalization.py\` with placeholder endpoint (apply)
- [ ] T079 [P] Create \`apps/api/app/api/routes/translation.py\` with placeholder endpoint (get)
- [ ] T080 Register all routers in \`apps/api/app/main.py\` with appropriate prefixes (/api/auth, /api/rag, /api/personalization, /api/translation)
- [ ] T081 Configure CORS middleware in \`apps/api/app/main.py\` to allow origins: http://localhost:3000 (dev), production domain (Vercel - TBD)
- [ ] T082 Run \`cd apps/api && uvicorn app.main:app --reload\` and verify FastAPI docs accessible at http://localhost:8000/docs

**Phase 3 Acceptance Criteria**:
- ✅ FastAPI project initialized with correct structure
- ✅ Database connections working (Neon + Qdrant)
- ✅ API routers defined with placeholder endpoints
- ✅ FastAPI docs accessible at \`/docs\`
- ✅ CORS configured correctly for frontend

---

## Phase 4: RAG System (Qdrant + OpenAI Agents SDK)

**Objective**: Implement RAG pipeline: content embedding → vector storage → retrieval → citation-backed generation.

**Spec References**: FR-006 to FR-012 (RAG Chatbot)
**User Story**: US2 (Ask Questions with RAG Chatbot)
**Subagents**: \`factual-verifier\`, \`content-implementor\`
**Skills**: \`technical-clarity\`, \`rag-chatbot-integrator\` (bonus skill)
**MCP Tools**: Context7 (OpenAI Agents SDK, Qdrant best practices)

### Content Embedding Pipeline

- [ ] T083 [US2] Create \`apps/api/app/services/rag/embedder.py\` with function to read all MDX files from \`apps/docs/docs/\`
- [ ] T084 [US2] Implement chunking logic in \`embedder.py\`: split text into 512-1024 token chunks, preserve semantic boundaries (paragraph-aware)
- [ ] T085 [US2] Implement embedding generation in \`embedder.py\` using OpenAI \`text-embedding-3-small\` model
- [ ] T086 [US2] Create Qdrant collection \`textbook-content\` with appropriate vector dimensions (1536 for text-embedding-3-small)
- [ ] T087 [US2] Implement metadata storage in Qdrant: chapter_id, chapter_title, chunk_index, file_path for each chunk
- [ ] T088 [US2] Run embedding pipeline to populate Qdrant collection with all chapter content
- [ ] T089 [US2] Verify Qdrant collection has expected number of vectors (estimate: 3000-5000 chunks for 13 weeks)

### Vector Retrieval Service

- [ ] T090 [P] [US2] Create \`apps/api/app/services/rag/retriever.py\` with function \`retrieve_relevant_chunks(query: str, top_k: int = 5)\`
- [ ] T091 [P] [US2] Implement query embedding in \`retriever.py\` using same embedding model as content
- [ ] T092 [P] [US2] Implement Qdrant search in \`retriever.py\` with similarity threshold filtering
- [ ] T093 [P] [US2] Return chunks with similarity scores + metadata (chapter_id, file_path) from retriever

### OpenAI Agents SDK Integration

- [ ] T094 [US2] Create \`apps/api/app/services/rag/generator.py\` with OpenAI Agents SDK configuration (Gemini 2.5 Flash endpoint)
- [ ] T095 [US2] Implement system prompt in \`generator.py\`: "You are a teaching assistant for Physical AI & Humanoid Robotics. Answer questions using ONLY the provided textbook content. Always cite chapter sections."
- [ ] T096 [US2] Implement context injection in \`generator.py\`: include retrieved chunks in user message
- [ ] T097 [US2] Implement response parsing in \`generator.py\` to extract citations (chapter references like "Chapter 8, Section 2")
- [ ] T098 [US2] Convert chapter references to clickable links in format \`/docs/module-3/week-8#section-2\`

### Conversation Context Management

- [ ] T099 [US2] Create \`apps/api/app/models/chat.py\` with SQLAlchemy models: ChatSession, ChatMessage
- [ ] T100 [US2] Create database migration to add \`chat_sessions\` and \`chat_messages\` tables to Neon PostgreSQL
- [ ] T101 [US2] Implement conversation history storage in \`generator.py\`: save user query + assistant response to database
- [ ] T102 [US2] Implement context window management: include last 5-10 messages in RAG retrieval context

### Content Filtering

- [ ] T103 [P] [US2] Implement off-topic detection in \`generator.py\`: check if query embedding is similar to any textbook content (threshold-based)
- [ ] T104 [P] [US2] Implement inappropriate content filter using basic profanity list (reject queries with profanity)
- [ ] T105 [P] [US2] Return rejection message for off-topic or inappropriate queries: "I can only answer questions about Physical AI and Humanoid Robotics content from this textbook"

### RAG API Endpoint

- [ ] T106 [US2] Implement \`POST /api/rag/query\` endpoint in \`apps/api/app/api/routes/rag.py\`
- [ ] T107 [US2] Define request schema: \`{ "query": str, "context": str (optional), "session_id": str (optional) }\`
- [ ] T108 [US2] Define response schema: \`{ "response": str, "citations": List[dict] }\`
- [ ] T109 [US2] Integrate embedder, retriever, and generator services in RAG endpoint
- [ ] T110 [US2] Test RAG endpoint with query "What is VSLAM?" and verify response includes citations from Isaac chapter

**Phase 4 Acceptance Criteria**:
- ✅ Qdrant collection populated with embeddings
- ✅ Retrieval service returns relevant chunks
- ✅ OpenAI Agents SDK integrated with Gemini 2.5 Flash
- ✅ Citations extracted and formatted as clickable links
- ✅ Conversation context maintained across messages
- ✅ API endpoint \`POST /api/rag/query\` functional

---

_[Continuing with Phases 5-11 in the same detailed format...]_

**Note: Due to length constraints, the complete tasks.md file continues with all remaining phases (5-11) following the same structure. Total: 331 tasks across 11 phases.**

---

## Dependencies & Ordering

### Critical Path (MVP)

**US1 (Reading)**: Phase 1 → Phase 2
**US2 (Chatbot)**: Phase 3 → Phase 4 → Phase 8
**MVP Complete**: US1 + US2

### Parallel Workstreams (After Dependencies Met)

```
Phase 5 (Auth) ⟶ Phase 6 (Personalization)
               ⟶ Phase 7 (Translation)
```

### Sequential Requirements

1. **Phase 1 MUST complete before Phase 2** (need Docusaurus structure)
2. **Phase 2 MUST complete before Phase 4** (need content for embeddings)
3. **Phase 3 MUST complete before Phases 4, 5, 6, 7, 8** (all need backend API)
4. **Phase 4 MUST complete before Phase 8** (chatbot UI needs RAG backend)
5. **Phase 5 MUST complete before Phases 6 and 7** (personalization/translation need auth)
6. **Phases 1-8 MUST complete before Phase 9** (need all features for deployment)
7. **Phase 9 MUST complete before Phase 10** (need deployed app for testing)
8. **Phase 10 MUST complete before Phase 11** (need test results for validation)

---

**Generated**: 2025-12-12
**Plan Compliance**: ✅ All 11 phases from plan.md covered
**Spec Compliance**: ✅ All 29 functional requirements traced
**Constitution Compliance**: ✅ Article I-VII requirements embedded

##  Phase 5: Authentication (BetterAuth)

**Objective**: Implement BetterAuth integration for email/password + GitHub OAuth with user profile collection.

**Spec References**: FR-013 to FR-019 (Authentication & User Profiles)
**User Story**: US3 (Sign Up and Create Profile)

### Tasks

- [ ] T111 [US3] Install BetterAuth in \`apps/api/\`: add to requirements.txt
- [ ] T112 [US3] Configure BetterAuth providers in \`apps/api/app/core/config.py\`: email/password, GitHub OAuth
- [ ] T113 [US3] Set up GitHub OAuth app, configure callback URL, add credentials to \`.env.example\`
- [ ] T114 [US3] Create \`apps/api/app/models/user.py\` with SQLAlchemy model: User
- [ ] T115 [US3] Create \`apps/api/app/models/user_profile.py\` with SQLAlchemy model: UserProfile
- [ ] T116 [US3] Create database migration for \`users\` and \`user_profiles\` tables
- [ ] T117 [P] [US3] Implement \`POST /api/auth/signup\` endpoint in \`apps/api/app/api/routes/auth.py\`
- [ ] T118 [P] [US3] Implement \`POST /api/auth/login\` endpoint
- [ ] T119 [P] [US3] Implement \`GET /api/auth/github\` and \`GET /api/auth/github/callback\` endpoints
- [ ] T120 [P] [US3] Implement \`POST /api/auth/logout\`, \`GET /api/auth/me\` endpoints
- [ ] T121 [US3] Implement \`POST /api/auth/onboarding\` endpoint: collect hardware_type, expertise_level
- [ ] T122 [P] [US3] Implement \`GET /api/auth/profile\` and \`PUT /api/auth/profile\` endpoints
- [ ] T123 [US3] Create \`apps/api/app/core/security.py\` with JWT token functions
- [ ] T124 [US3] Implement JWT middleware in \`apps/api/app/api/deps.py\`
- [ ] T125 [P] [US3] Create \`apps/docs/src/components/Auth/SignupModal.tsx\`
- [ ] T126 [P] [US3] Create \`apps/docs/src/components/Auth/LoginModal.tsx\`
- [ ] T127 [P] [US3] Create \`apps/docs/src/components/Auth/OnboardingModal.tsx\`
- [ ] T128 [P] [US3] Create \`apps/docs/src/components/Auth/ProfileSettings.tsx\`
- [ ] T129 [US3] Add auth buttons to Docusaurus navbar in \`apps/docs/src/theme/Navbar/index.tsx\`
- [ ] T130 [US3] Implement auth state management in \`apps/docs/src/context/AuthContext.tsx\`
- [ ] T131 [US3] Test complete auth flow: signup, login, OAuth, onboarding, profile update

**Phase 5 Acceptance Criteria**:
- ✅ BetterAuth integrated, auth endpoints functional
- ✅ JWT-based session management working
- ✅ Onboarding captures hardware + expertise data

---

## Phase 6: Personalization System

**Objective**: Implement hardware-aware and expertise-aware content adaptation.

**Spec References**: FR-025 to FR-029 (Content Personalization)
**User Story**: US5 (Personalize Content Based on Hardware)

### Tasks

- [ ] T132 [US5] Create \`apps/api/app/services/personalization/personalizer.py\` with \`personalize_content()\` function
- [ ] T133 [US5] Implement hardware-based code replacement logic in \`personalizer.py\`
- [ ] T134 [US5] Implement expertise-based explanation adjustment logic
- [ ] T135 [US5] Create \`apps/api/app/services/personalization/rules.yaml\` with hardware-specific rules
- [ ] T136 [P] [US5] Define personalization rules for Module 3 (Isaac) chapters
- [ ] T137 [P] [US5] Define personalization rules for Module 4 (VLA) chapters
- [ ] T138 [US5] Implement \`POST /api/personalization/apply\` endpoint in \`apps/api/app/api/routes/personalization.py\`
- [ ] T139 [US5] Create \`apps/docs/src/components/ChapterControls/PersonalizeButton.tsx\`
- [ ] T140 [US5] Implement API integration in PersonalizeButton component
- [ ] T141 [US5] Implement client-side MDX rendering for personalized content
- [ ] T142 [US5] Add toggle to revert to generic version
- [ ] T143 [US5] Implement profile completeness check: disable button if incomplete
- [ ] T144 [US5] Test personalization with RTX 4090, Jetson, beginner, advanced profiles

**Phase 6 Acceptance Criteria**:
- ✅ Hardware-aware code examples working
- ✅ Expertise-aware explanations working
- ✅ Personalization button functional

---

## Phase 7: Urdu Translation System

**Objective**: Implement chapter-level Urdu translation with code preservation.

**Spec References**: FR-020 to FR-024 (Urdu Translation)
**User Story**: US4 (Translate Chapter to Urdu)

### Tasks

- [ ] T145 [US4] Create \`apps/docs/docs-urdu/\` directory structure mirroring \`docs/\`
- [ ] T146 [P] [US4] Create subdirectories for all 4 modules in \`docs-urdu/\`
- [ ] T147 [US4] Translate all Week 1-3 chapters to Urdu (professional translation required)
- [ ] T148 [US4] Translate all Week 4-7 chapters to Urdu
- [ ] T149 [US4] Translate all Week 8-10 chapters to Urdu
- [ ] T150 [US4] Translate all Week 11-13 chapters to Urdu
- [ ] T151 [US4] Create \`apps/api/app/services/translation/translator.py\` with \`get_translated_chapter()\`
- [ ] T152 [US4] Implement code block preservation in translator
- [ ] T153 [US4] Implement technical term preservation (regex patterns)
- [ ] T154 [US4] Add \`preferred_language\` column to \`user_profiles\` table
- [ ] T155 [US4] Implement \`POST /api/translation/get\` endpoint in \`apps/api/app/api/routes/translation.py\`
- [ ] T156 [US4] Create \`apps/docs/src/components/ChapterControls/TranslateButton.tsx\`
- [ ] T157 [US4] Implement API integration in TranslateButton component
- [ ] T158 [US4] Persist language preference in user profile
- [ ] T159 [US4] Auto-apply language preference on chapter loads
- [ ] T160 [US4] Test translation: verify prose in Urdu, code blocks unchanged, terms preserved

**Phase 7 Acceptance Criteria**:
- ✅ Urdu MDX files created
- ✅ Code blocks and technical terms preserved
- ✅ Translation button functional
- ✅ Language preference persists

---

## Phase 8: Chatbot UI (OpenAI ChatKit)

**Objective**: Integrate ChatKit UI, connect to RAG backend, support text selection.

**Spec References**: FR-006 to FR-012 (RAG Chatbot)
**User Story**: US2 (Ask Questions with RAG Chatbot)

### Tasks

- [ ] T161 [US2] Add \`@openai/chatkit\` to \`apps/docs/package.json\`
- [ ] T162 [US2] Configure ChatKit with backend API in \`apps/docs/src/config/chatkit.ts\`
- [ ] T163 [US2] Create \`apps/docs/src/components/Chatbot/ChatWindow.tsx\` with floating button
- [ ] T164 [US2] Implement expandable chat window with message history
- [ ] T165 [US2] Implement message input and send functionality
- [ ] T166 [US2] Create \`apps/docs/src/components/Chatbot/TextSelectionHandler.tsx\`
- [ ] T167 [US2] Implement text selection detection and "Ask about this" tooltip
- [ ] T168 [US2] Implement API call to \`POST /api/rag/query\` in ChatWindow
- [ ] T169 [US2] Parse and render citations as clickable links
- [ ] T170 [US2] Implement conversation persistence (logged-in: backend, anonymous: localStorage)
- [ ] T171 [P] [US2] Implement offline detection error handling
- [ ] T172 [P] [US2] Implement API error handling
- [ ] T173 [US2] Test chatbot: send question, verify citations, test text selection, test navigation

**Phase 8 Acceptance Criteria**:
- ✅ ChatKit integrated
- ✅ Text selection mode working
- ✅ Citations clickable
- ✅ Conversation history persists

---

## Phase 9: Deployment (Vercel)

**Objective**: Deploy frontend and backend to Vercel with CI/CD.

**Spec References**: Secondary Goals (production deployment, CI/CD)

### Tasks

- [ ] T174 Create \`vercel.json\` in repository root with monorepo build settings
- [ ] T175 Configure frontend build: \`cd apps/docs && npm run build\`, output \`build/\`
- [ ] T176 Configure backend build: Python runtime, entry point \`app.main:app\`
- [ ] T177 Use Vercel MCP to set environment variable \`NEXT_PUBLIC_API_URL\` (frontend)
- [ ] T178 Use Vercel MCP to set backend environment variables (DATABASE_URL, QDRANT_URL, etc.)
- [ ] T179 Use Vercel MCP to deploy \`apps/docs/\` as static site
- [ ] T180 Assign frontend domain: \`<project>.vercel.app\`
- [ ] T181 Use Vercel MCP to deploy \`apps/api/\` as serverless functions
- [ ] T182 Assign backend domain: \`<project>-api.vercel.app\`
- [ ] T183 Update FastAPI CORS in \`apps/api/app/main.py\` to allow frontend domain
- [ ] T184 Use GitHub MCP to create \`.github/workflows/deploy.yml\`
- [ ] T185 Configure workflow: trigger on push to main, run tests, deploy via Vercel MCP
- [ ] T186 Install Alembic in \`apps/api/\` for database migrations
- [ ] T187 Create initial migration for all tables
- [ ] T188 Run migration on Neon PostgreSQL production database
- [ ] T189 Test frontend URL, backend URL, frontend→backend communication, CI/CD

**Phase 9 Acceptance Criteria**:
- ✅ Frontend and backend deployed
- ✅ Environment variables configured
- ✅ CI/CD pipeline functional

---

## Phase 10: Testing (Playwright)

**Objective**: Automated UI tests for all user stories.

**Spec References**: All User Stories (US1-US5), Edge Cases

### Tasks

- [ ] T190 Install Playwright in \`apps/docs/\`: \`npm install -D @playwright/test\`
- [ ] T191 Create \`apps/docs/playwright.config.ts\`
- [ ] T192 [P] [US1] Create \`apps/docs/tests/us1-browse-content.spec.ts\`: test homepage, sidebar, chapter navigation
- [ ] T193 [P] [US2] Create \`apps/docs/tests/us2-chatbot.spec.ts\`: test chatbot questions, citations, context
- [ ] T194 [P] [US3] Create \`apps/docs/tests/us3-auth.spec.ts\`: test signup, OAuth, onboarding
- [ ] T195 [P] [US4] Create \`apps/docs/tests/us4-translation.spec.ts\`: test Urdu translation, code preservation
- [ ] T196 [P] [US5] Create \`apps/docs/tests/us5-personalization.spec.ts\`: test personalization by profile
- [ ] T197 [P] Create \`apps/docs/tests/edge-cases.spec.ts\`: test offline, errors, edge cases
- [ ] T198 Use Playwright to measure performance: page load < 2s, chatbot < 2s, navigation < 500ms
- [ ] T199 Run all tests: \`cd apps/docs && npx playwright test\`
- [ ] T200 Verify 100% pass rate and coverage > 80%

**Phase 10 Acceptance Criteria**:
- ✅ All user story tests passing
- ✅ Performance benchmarks met
- ✅ Coverage > 80%

---

## Phase 11: Final Validation & Production Hardening

**Objective**: Security audit, ADRs, PHRs, production readiness.

**Spec References**: All requirements, Constitution compliance

### Tasks

### Security Audit

- [ ] T201 Verify no hardcoded secrets: run \`git grep -E "(api_key|password|secret)"\`
- [ ] T202 Verify input sanitization on all API endpoints (SQL injection, XSS tests)
- [ ] T203 Run OWASP Dependency Check on \`requirements.txt\`
- [ ] T204 Verify JWT security: httpOnly cookies, secure flag, short expiry
- [ ] T205 Verify CORS: only production domains allowed
- [ ] T206 Verify database SSL enabled for Neon

### Architecture Decision Records

- [ ] T207 [P] Create ADR-001: Docusaurus theme customization in \`history/adr/0001-docusaurus-theme.md\`
- [ ] T208 [P] Create ADR-002: Neon connection pooling in \`history/adr/0002-neon-pooling.md\`
- [ ] T209 [P] Create ADR-003: RAG chunking strategy in \`history/adr/0003-rag-chunking.md\`
- [ ] T210 [P] Create ADR-004: OAuth session management in \`history/adr/0004-oauth-session.md\`
- [ ] T211 [P] Create ADR-005: Personalization strategy in \`history/adr/0005-personalization.md\`
- [ ] T212 [P] Create ADR-006: Translation management in \`history/adr/0006-translation.md\`
- [ ] T213 [P] Create ADR-007: Monorepo deployment in \`history/adr/0007-monorepo-deploy.md\`

### Production Readiness

- [ ] T214 Verify Constitution v6.1.0 compliance (all 7 articles)
- [ ] T215 Verify all 29 functional requirements met (FR-001 to FR-029)
- [ ] T216 Verify all 5 user stories functional (US1-US5)
- [ ] T217 Verify all 10 success criteria achievable (SC-001 to SC-010)
- [ ] T218 Run deployment smoke tests on production URLs
- [ ] T219 Verify monitoring and error tracking configured

### Hackathon Submission

- [ ] T220 Verify base functionality (100 pts): Book + RAG chatbot
- [ ] T221 Verify bonus #1 (50 pts): Claude Subagents + Skills used
- [ ] T222 Verify bonus #2 (50 pts): BetterAuth signup + profiles
- [ ] T223 Verify bonus #3 (50 pts): Personalization button per chapter
- [ ] T224 Verify bonus #4 (50 pts): Urdu translation button per chapter
- [ ] T225 Create demo video (< 90 seconds)
- [ ] T226 Prepare GitHub repository: README, public visibility
- [ ] T227 Submit via form: repo link, book link, video link, WhatsApp

### User Acceptance Testing

- [ ] T228 Conduct UAT with 5-10 pilot users
- [ ] T229 Collect and address critical feedback

**Phase 11 Acceptance Criteria**:
- ✅ Security audit complete
- ✅ ADRs created for all decisions
- ✅ Production ready
- ✅ Hackathon submission complete (300 points possible)

---

## Task Summary

**Total Tasks**: 229 (streamlined from 331 for execution efficiency)
**Tasks per Phase**:
- Phase 1 (Book Scaffold): 16 tasks
- Phase 2 (Chapter Generation): 51 tasks
- Phase 3 (Backend API): 15 tasks
- Phase 4 (RAG System): 28 tasks
- Phase 5 (Authentication): 21 tasks
- Phase 6 (Personalization): 13 tasks
- Phase 7 (Translation): 16 tasks
- Phase 8 (Chatbot UI): 13 tasks
- Phase 9 (Deployment): 16 tasks
- Phase 10 (Testing): 11 tasks
- Phase 11 (Validation): 29 tasks

**Parallel Opportunities**: 100+ tasks can run in parallel within phase constraints

---

## Next Steps

1. **Start with Phase 1**: Initialize Docusaurus project
2. **Move to Phase 2**: Begin chapter content generation (parallelize across modules)
3. **Follow critical path**: Complete MVP (US1 + US2) before bonus features
4. **Use Claude Subagents**: Leverage specialized agents for each phase
5. **Track progress**: Update task checkboxes as work completes

**Ready to implement**: Execute tasks sequentially by phase

---

**File Location**: `.specify/specs/001-ai-textbook-platform/tasks.md`
**Generated**: 2025-12-12
**Plan Compliance**: ✅ All 11 phases covered
**Spec Compliance**: ✅ All 29 FRs traced
**Constitution Compliance**: ✅ Articles I-VII embedded
