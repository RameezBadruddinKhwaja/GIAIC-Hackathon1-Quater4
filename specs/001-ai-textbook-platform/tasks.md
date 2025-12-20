# TASKS.md: Physical AI & Humanoid Robotics AI-Native Textbook Platform

## Feature Overview
Build a production-grade AI-native textbook platform delivering:
1. **Core Platform (100 pts)**: Docusaurus-based textbook with 13 weeks of Physical AI content + embedded RAG chatbot using OpenAI Agents, FastAPI, Neon Postgres, and Qdrant
2. **Bonus Features (200 pts)**: Authentication via BetterAuth + per-chapter personalization + Urdu translation + reusable Claude Code agents/skills

## Phase 0 — FOUNDATION (BOOK ONLY)

### Setup Tasks
- [ ] T001 Create project directory structure following monorepo approach
- [ ] T002 Initialize Git repository with proper .gitignore for Node.js and Python
- [ ] T003 Set up virtual environment for Python backend (Python 3.11+)
- [ ] T004 Initialize Node.js project in docs/ directory with package.json
- [ ] T005 [P] Install Docusaurus dependencies and initialize documentation site
- [ ] T006 [P] Install FastAPI and related dependencies in backend/
- [ ] T007 Create initial directory structure for docs/ and backend/ as specified in plan

### Foundational Tasks
- [ ] T008 Create Claude Code configuration directory .claude/
- [ ] T009 [P] Create agents directory .claude/agents/ for custom subagents
- [ ] T010 [P] Create skills directory .claude/skills/ for reusable skills
- [ ] T011 [P] Create specs directory structure with contracts/ subdirectory
- [ ] T012 [P] Set up environment variables configuration (.env.example)
- [ ] T013 Initialize docusaurus.config.ts with basic configuration
- [ ] T014 Create initial backend app structure with main.py and api/ directory

### User Story 1 - Book Content Creation (US1)
- [ ] T015 [P] [US1] Create Curriculum Architect Agent definition in .claude/agents/curriculum-architect.md
- [ ] T016 [P] [US1] Create Chapter Author Agent definition in .claude/agents/chapter-author.md
- [ ] T017 [P] [US1] Create Robotics Domain Expert Agent definition in .claude/agents/robotics-expert.md
- [ ] T018 [P] [US1] Create Pedagogy & Simplification Agent definition in .claude/agents/pedagogy-simplifier.md
- [ ] T019 [P] [US1] Create Review & Accuracy Agent definition in .claude/agents/review-accuracy.md
- [ ] T020 [P] [US1] Create Chapter Scaffolding Skill in .claude/skills/chapter-scaffolding.md
- [ ] T021 [P] [US1] Create Concept Explanation Skill in .claude/skills/concept-explanation.md
- [ ] T022 [P] [US1] Create Diagram Generation Skill in .claude/skills/diagram-generation.md
- [ ] T023 [P] [US1] Create Review/Simplification Skill in .claude/skills/review-simplification.md
- [ ] T024 [P] [US1] Create module-1-ros2 directory structure in docs/docs/
- [ ] T025 [P] [US1] Create module-2-digital-twin directory structure in docs/docs/
- [ ] T026 [P] [US1] Create module-3-nvidia-isaac directory structure in docs/docs/
- [ ] T027 [P] [US1] Create module-4-vla directory structure in docs/docs/
- [ ] T028 [P] [US1] Write Week 1-2 Introduction to Physical AI chapters
- [ ] T029 [P] [US1] Write Week 3-5 ROS 2 fundamentals chapters
- [ ] T030 [P] [US1] Write Week 6-7 Gazebo and Unity chapters
- [ ] T031 [P] [US1] Write Week 8-10 NVIDIA Isaac chapters
- [ ] T032 [P] [US1] Write Week 11-13 VLA and conversational robotics chapters
- [ ] T033 [P] [US1] Add code examples with syntax highlighting to all chapters
- [ ] T034 [P] [US1] Add learning objectives and assessments to each chapter
- [ ] T035 [US1] Test Docusaurus build to ensure all MDX files are valid
- [ ] T036 [US1] Validate all 13 weeks of content are properly structured
- [ ] T037 [US1] Run mobile/responsive design tests on book content

### Phase 0 Completion Checkpoint
- [ ] T038 Verify all 5 agents are documented and functional
- [ ] T039 Verify all 4 skills are documented and reusable
- [ ] T040 Verify all 13 weeks of content exist across 4 modules
- [ ] T041 Verify Docusaurus builds without errors
- [ ] T042 Commit Phase 0 work and push to main branch
- [ ] T043 Phase 0 STOP - verify all tasks complete before proceeding

## Phase 1 — RAG CHATBOT (CORE + BONUS)

### Setup Tasks
- [X] T044 [P] Set up Neon Serverless Postgres instance and connection
- [X] T045 [P] Set up Qdrant Cloud Free Tier account and collection
- [X] T046 [P] Obtain OpenAI API key for embeddings and chat completion
- [X] T047 Configure environment variables for backend services
- [X] T048 [P] Install Qdrant client in backend requirements.txt
- [X] T049 [P] Install OpenAI SDK in backend requirements.txt
- [ ] T050 [P] Install BetterAuth dependencies in backend

### Foundational Tasks
- [X] T051 Create database schema for Neon Postgres (users, chat_sessions, chat_messages)
- [X] T052 [P] Create Qdrant vector database connection module
- [X] T053 [P] Create Neon Postgres connection module
- [X] T054 [P] Create Pydantic models for chatbot data structures
- [X] T055 [P] Set up FastAPI middleware for request logging and error handling
- [X] T056 Create vector chunking service for book content
- [X] T057 Create embedding service using OpenAI text-embedding-3-small

### User Story 2 - RAG Chatbot Implementation (US2)
- [X] T058 [P] [US2] Create RAG service to handle retrieval and generation
- [X] T059 [P] [US2] Create chatbot API endpoints in backend/app/api/chatbot.py
- [X] T060 [P] [US2] Implement POST /api/chatbot/query endpoint
- [X] T061 [P] [US2] Implement POST /api/chatbot/query-selection endpoint
- [X] T062 [P] [US2] Implement GET /api/chatbot/history/{session_id} endpoint
- [X] T063 [P] [US2] Create off-topic detection logic for chatbot responses
- [X] T064 [P] [US2] Create citation extraction from retrieved chunks
- [X] T065 [P] [US2] Implement vector database indexing for book content
- [X] T066 [P] [US2] Create ChatWidget React component in docs/src/components/
- [X] T067 [P] [US2] Add ChatWidget to Docusaurus theme layout
- [X] T068 [P] [US2] Implement text selection listener for contextual queries
- [X] T069 [P] [US2] Add citation links that navigate to book sections
- [X] T070 [US2] Test chatbot response time (must be <5 seconds for 95% of queries)
- [X] T071 [US2] Test chatbot citation accuracy (all responses include chapter/section references)
- [X] T072 [US2] Test off-topic query rejection (chatbot rejects non-book questions)
- [X] T073 [US2] Test text selection → query functionality
- [X] T074 [US2] Index all book content into Qdrant vector database
- [X] T075 [US2] Verify 100% of book content is indexed in Qdrant

### Phase 1 Completion Checkpoint
- [ ] T076 Verify book is publicly accessible via live URL
- [ ] T077 Verify all 13 weeks of content are navigable
- [ ] T078 Verify code examples have syntax highlighting
- [ ] T079 Verify responsive design on mobile, tablet, desktop
- [X] T080 Verify chatbot answers book-specific questions with citations
- [X] T081 Verify chatbot response time < 5 seconds for 95% of queries
- [X] T082 Verify chatbot correctly handles selected text queries
- [X] T083 Verify 100% of book content is indexed in Qdrant
- [X] T084 Commit Phase 1 work and push to main branch
- [X] T085 Phase 1 STOP - verify all tasks complete before proceeding

## Phase 2 — AUTHENTICATION (BONUS)

### Setup Tasks
- [ ] T086 Configure BetterAuth in backend with email/password authentication
- [ ] T087 Extend BetterAuth user schema to include software and hardware background
- [ ] T088 Update Neon Postgres schema to store user background information
- [ ] T089 Create BetterAuth API endpoints for signup/signin/signout
- [ ] T090 [P] Install BetterAuth dependencies in backend

### User Story 3 - Authentication Implementation (US3)
- [ ] T091 [P] [US3] Create SignupModal React component
- [ ] T092 [P] [US3] Create SigninModal React component
- [ ] T093 [P] [US3] Create AuthContext React context for authentication state
- [ ] T094 [P] [US3] Implement POST /api/auth/signup endpoint
- [ ] T095 [P] [US3] Implement POST /api/auth/signin endpoint
- [ ] T096 [P] [US3] Implement POST /api/auth/signout endpoint
- [ ] T097 [P] [US3] Implement GET /api/auth/me endpoint
- [ ] T098 [P] [US3] Add software background field to signup form
- [ ] T099 [P] [US3] Add hardware background field to signup form
- [ ] T100 [P] [US3] Add session persistence across browser restarts
- [ ] T101 [P] [US3] Add authentication gates to chatbot functionality
- [ ] T102 [P] [US3] Update ChatWidget to require authentication
- [ ] T103 [US3] Test signup flow completion time (< 3 minutes)
- [ ] T104 [US3] Test background information capture during signup
- [ ] T105 [US3] Test session persistence across browser restarts
- [ ] T106 [US3] Test chatbot accessibility for authenticated users only
- [ ] T107 [US3] Test public book reading without authentication

### Phase 2 Completion Checkpoint
- [ ] T108 Verify users can complete signup in < 3 minutes
- [ ] T109 Verify signup captures software and hardware backgrounds
- [ ] T110 Verify session persistence across browser restarts
- [ ] T111 Verify chatbot requires authentication to use
- [ ] T112 Verify public book reading works without authentication
- [ ] T113 Commit Phase 2 work and push to main branch
- [ ] T114 Phase 2 STOP - verify all tasks complete before proceeding

## Phase 3 — PERSONALIZATION (BONUS)

### Setup Tasks
- [ ] T115 Create personalization service in backend/app/services/personalization_service.py
- [ ] T116 [P] Create personalization API endpoints in backend/app/api/personalization.py
- [ ] T117 Update user model to include personalization preferences
- [ ] T118 [P] Set up caching mechanism for personalized content

### User Story 4 - Content Personalization (US4)
- [ ] T119 [P] [US4] Create PersonalizeButton React component
- [ ] T120 [P] [US4] Implement POST /api/personalization/chapter endpoint
- [ ] T121 [P] [US4] Add personalization logic based on user background
- [ ] T122 [P] [US4] Add beginner/advanced content adaptation rules
- [ ] T123 [P] [US4] Implement content personalization with LLM processing
- [ ] T124 [P] [US4] Add "Show Original" functionality to revert personalization
- [ ] T125 [P] [US4] Integrate PersonalizeButton into chapter layout
- [ ] T126 [P] [US4] Preserve code examples and structure during personalization
- [ ] T127 [US4] Test personalization response time (< 3 seconds)
- [ ] T128 [US4] Test content differences for beginner vs advanced user backgrounds
- [ ] T129 [US4] Test reversibility (show original content functionality)
- [ ] T130 [US4] Verify personalization preserves core learning objectives
- [ ] T131 [US4] Test with users having different software/hardware backgrounds

### Phase 3 Completion Checkpoint
- [ ] T132 Verify personalization button appears for logged-in users
- [ ] T133 Verify content adjusts within 3 seconds of clicking button
- [ ] T134 Verify content shows measurable differences based on user background
- [ ] T135 Verify reversibility - "Show Original" restores default content
- [ ] T136 Commit Phase 3 work and push to main branch
- [ ] T137 Phase 3 STOP - verify all tasks complete before proceeding

## Phase 4 — URDU TRANSLATION (BONUS)

### Setup Tasks
- [ ] T138 Create translation service in backend/app/services/translation_service.py
- [ ] T139 [P] Create translation API endpoints in backend/app/api/translation.py
- [ ] T140 Create RTL CSS styling for Urdu content
- [ ] T141 [P] Set up caching mechanism for translated content

### User Story 5 - Urdu Translation (US5)
- [ ] T142 [P] [US5] Create TranslateButton React component
- [ ] T143 [P] [US5] Implement POST /api/translation/chapter-urdu endpoint
- [ ] T144 [P] [US5] Add translation logic with technical term preservation
- [ ] T145 [P] [US5] Implement RTL (right-to-left) formatting for Urdu text
- [ ] T146 [P] [US5] Preserve code examples in English during translation
- [ ] T147 [P] [US5] Add "Show Original English" functionality to revert translation
- [ ] T148 [P] [US5] Integrate TranslateButton into chapter layout
- [ ] T149 [P] [US5] Create technical term dictionary for robotics concepts
- [ ] T150 [US5] Test translation response time (< 3 seconds)
- [ ] T151 [US5] Test technical term preservation (ROS 2, URDF, Gazebo, etc.)
- [ ] T152 [US5] Test RTL rendering of Urdu text
- [ ] T153 [US5] Verify code examples remain in English
- [ ] T154 [US5] Test reversibility (show original English functionality)
- [ ] T155 [US5] Validate translation accuracy with bilingual reviewer

### Phase 4 Completion Checkpoint
- [ ] T156 Verify translation button appears for logged-in users
- [ ] T157 Verify translation loads within 3 seconds
- [ ] T158 Verify 100% of technical terms remain in English
- [ ] T159 Verify code examples remain in English
- [ ] T160 Verify Urdu text renders right-to-left
- [ ] T161 Verify reversibility - "Show Original English" restores English content
- [ ] T162 Commit Phase 4 work and push to main branch
- [ ] T163 Phase 4 STOP - verify all tasks complete before proceeding

## Phase 5 — FINALIZATION

### Testing Tasks
- [ ] T164 Run comprehensive E2E tests using Playwright
- [ ] T165 [P] Test book navigation across all 4 modules
- [ ] T166 [P] Test chatbot functionality with 10 book-specific questions
- [ ] T167 [P] Test chatbot rejection of 5 off-topic questions
- [ ] T168 [P] Test text selection → query functionality
- [ ] T169 [P] Test signup flow with background information capture
- [ ] T170 [P] Test signin and session persistence
- [ ] T171 [P] Test personalization with beginner vs advanced user profiles
- [ ] T172 [P] Test Urdu translation and RTL rendering
- [ ] T173 [P] Test reversibility of personalization and translation features
- [ ] T174 Run performance tests to verify response times
- [ ] T175 Run security tests to verify authentication gates work properly

### Documentation Tasks
- [ ] T176 Update README.md with setup, deployment, and usage instructions
- [ ] T177 [P] Create deployment documentation for Vercel/GitHub Pages
- [ ] T178 [P] Document environment variables and configuration requirements
- [ ] T179 [P] Create demo credentials and testing scenarios
- [ ] T180 Verify all [TODO] placeholders are removed from documentation

### Deployment Tasks
- [ ] T181 Deploy to Vercel (preferred) or GitHub Pages with backend as serverless functions
- [ ] T182 [P] Configure environment variables in deployment platform
- [ ] T183 [P] Test live deployment for all features
- [ ] T184 [P] Verify HTTPS is enabled and working
- [ ] T185 [P] Test API endpoints from deployed frontend
- [ ] T186 [P] Verify all features work in production environment

### Demo Preparation Tasks
- [ ] T187 Create demo video script covering all features in 90 seconds
- [ ] T188 Record demo video demonstrating book navigation, chatbot, personalization, and translation
- [ ] T189 [P] Upload demo video to accessible platform (YouTube, etc.)
- [ ] T190 [P] Prepare submission form with GitHub repo link and live URL
- [ ] T191 [P] Verify all bonus features are working and ready for evaluation
- [ ] T192 [P] Document total points achieved (base 100 + bonus points)

### Final Quality Gates
- [ ] T193 Verify build success with zero errors (npm run build, mypy)
- [ ] T194 Verify type safety with TypeScript strict mode and Pydantic models
- [ ] T195 Verify authentication gates block unauthorized access
- [ ] T196 Verify RAG functionality with off-topic rejection and citations
- [ ] T197 Verify deployment integrity with HTTPS and environment variables
- [ ] T198 Verify documentation completeness with no missing sections
- [ ] T199 Final test of all features on deployed URL
- [ ] T200 Submit project with GitHub repo link, live URL, and demo video

## Dependencies

### User Story Dependencies
- **User Story 1 (Book Content)**: No dependencies, can be developed in parallel
- **User Story 2 (RAG Chatbot)**: Depends on User Story 1 (book content must exist)
- **User Story 3 (Authentication)**: No direct dependency, but requires book content to be deployed
- **User Story 4 (Personalization)**: Depends on User Story 3 (requires authentication)
- **User Story 5 (Urdu Translation)**: Depends on User Story 3 (requires authentication)

### Phase Dependencies
- **Phase 1** requires **Phase 0** to be fully completed and deployed
- **Phase 2** requires **Phase 1** to be fully completed and deployed
- **Phase 3** requires **Phase 2** to be fully completed and deployed
- **Phase 4** requires **Phase 2** to be fully completed and deployed
- **Phase 5** requires all previous phases to be completed

## Parallel Execution Opportunities

### Within User Stories
- **User Story 1**: Chapters can be written in parallel by different agents
- **User Story 2**: Frontend (ChatWidget) and backend (API endpoints) can be developed in parallel
- **User Story 3**: Frontend (modals) and backend (API endpoints) can be developed in parallel
- **User Story 4**: Frontend (PersonalizeButton) and backend (API endpoints) can be developed in parallel
- **User Story 5**: Frontend (TranslateButton) and backend (API endpoints) can be developed in parallel

### Across User Stories (with proper dependencies)
- **User Story 3 (Authentication)** can be developed in parallel with User Story 2 (RAG Chatbot) once book content exists
- **User Story 4 (Personalization)** and **User Story 5 (Translation)** can be developed in parallel after authentication is complete

## Implementation Strategy

### MVP First Approach
1. Focus on base 100 points first (book + RAG chatbot)
2. Implement core functionality before adding bonus features
3. Ensure each phase is production-ready before moving to next phase
4. Prioritize features that deliver maximum value to users

### Incremental Delivery
1. Phase 0: Deliver complete book content
2. Phase 1: Deliver book + functional chatbot
3. Phase 2: Add authentication and user management
4. Phase 3: Add personalization features
5. Phase 4: Add translation features
6. Phase 5: Final polish and submission

### Risk Mitigation
- Prioritize base features over bonus features
- Implement caching early to manage API costs
- Test deployment frequently to catch issues early
- Validate all external service integrations early in each phase