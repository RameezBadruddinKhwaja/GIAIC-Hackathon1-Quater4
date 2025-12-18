# Tasks: AI-Native Textbook Platform - Physical AI & Humanoid Robotics

**Feature**: 001-ai-textbook-platform
**Input**: Hackathon execution requirements (TASK.md), spec.md (user stories), plan.md (technical approach), data-model.md (entities)
**Execution Rule**: No next phase starts until current phase is: fully working, locally tested, production-tested, and pushed to main branch

**CRITICAL RULE**: Every phase MUST be completed, tested locally AND in production, and committed to main branch before proceeding to the next phase.

---

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, etc.)
- Include exact file paths in descriptions

---

## PHASE 1: RAG CHATBOT (CORE - MUST WORK) 🎯 P1

**Goal**: Book ke har chapter par questions ka jawab dene wala chatbot, pehle local phir production

**Critical Success Criteria**:
- API starts without error locally
- Chatbot answers questions from all chapters
- No API key / env errors
- Deployed API reachable
- No CORS / timeout issues
- Clean logs in production

### Audit & Cleanup

- [ ] T001 [US2] Audit existing chatbot code across apps/docs and apps/api directories
- [ ] T002 [US2] Remove all broken/unused chatbot code from apps/docs/src/components/
- [ ] T003 [US2] Remove all broken/unused chatbot code from apps/api/app/

### FastAPI Server Clean Setup

- [ ] T004 [US2] Clean FastAPI main.py in apps/api/app/main.py (remove unused imports)
- [ ] T005 [P] [US2] Setup proper CORS configuration in apps/api/app/main.py
- [ ] T006 [P] [US2] Create clean router structure in apps/api/app/api/routes/rag.py
- [ ] T007 [US2] Configure environment variables in apps/api/app/core/config.py

### Database Connections Verification

- [ ] T008 [US2] Verify Qdrant connection (local + cloud) in apps/api/app/db/vector_store.py
- [ ] T009 [US2] Verify Neon Postgres connection in apps/api/app/db/session.py
- [ ] T010 [US2] Create database schema migrations in apps/api/migrations/001_create_chat_logs.sql
- [ ] T011 [US2] Test database connections with health check endpoint in apps/api/app/api/routes/health.py

### OpenAI / Agents SDK Integration

- [ ] T012 [US2] Integrate OpenAI Agents SDK in apps/api/app/services/rag/generator.py
- [ ] T013 [US2] Configure Gemini 2.5 Flash endpoint in apps/api/app/core/config.py
- [ ] T014 [US2] Implement embedding generation in apps/api/app/services/rag/embedder.py
- [ ] T015 [US2] Create RAG retrieval service in apps/api/app/services/rag/retriever.py

### Chapter-wise Embeddings Regeneration

- [ ] T016 [US2] Parse all MDX files from apps/docs/docs/ directory
- [ ] T017 [US2] Chunk content (512-1024 tokens) in apps/api/app/services/rag/chunker.py
- [ ] T018 [US2] Generate embeddings for all chunks using text-embedding-3-small
- [ ] T019 [US2] Upsert embeddings to Qdrant collection 'book_knowledge'
- [ ] T020 [US2] Verify embeddings count matches expected chapter count

### Selected-text Based Q&A Implementation

- [ ] T021 [US2] Implement text selection handler in apps/docs/src/components/Chatbot/TextSelectionHandler.tsx
- [ ] T022 [US2] Add "Ask about this" tooltip on text selection
- [ ] T023 [US2] Pass selected text as context to RAG query endpoint
- [ ] T024 [US2] Handle context-aware queries in apps/api/app/services/rag/generator.py

### RAG Query Endpoint Implementation

- [ ] T025 [US2] Create POST /api/rag/query endpoint in apps/api/app/api/routes/rag.py
- [ ] T026 [US2] Implement input sanitization (XSS, SQL injection) in query endpoint
- [ ] T027 [US2] Add citation extraction logic in apps/api/app/services/rag/citation_parser.py
- [ ] T028 [US2] Return response with citations array in JSON format
- [ ] T029 [US2] Store chat logs in Neon PostgreSQL chat_logs table

### Chatbot UI Integration

- [ ] T030 [US2] Create ChatWindow component in apps/docs/src/components/Chatbot/ChatWindow.tsx
- [ ] T031 [P] [US2] Add floating chat button in apps/docs/src/components/Chatbot/FloatingButton.tsx
- [ ] T032 [US2] Integrate with POST /api/rag/query backend endpoint
- [ ] T033 [US2] Render citations as clickable links to chapter sections
- [ ] T034 [US2] Implement conversation context persistence (localStorage for anonymous users)

### Local Testing

- [ ] T035 [US2] Start FastAPI server: uvicorn app.main:app --reload
- [ ] T036 [US2] Verify API starts without errors
- [ ] T037 [US2] Test chatbot answers questions from module-1 chapters
- [ ] T038 [US2] Test chatbot answers questions from module-2 chapters
- [ ] T039 [US2] Test chatbot answers questions from module-3 chapters
- [ ] T040 [US2] Test chatbot answers questions from module-4 chapters
- [ ] T041 [US2] Verify no API key / env errors in logs
- [ ] T042 [US2] Test selected-text Q&A functionality

### Production Testing

- [ ] T043 [US2] Deploy API to Vercel using vercel.json configuration
- [ ] T044 [US2] Verify deployed API is reachable at production URL
- [ ] T045 [US2] Test CORS configuration (no cross-origin errors)
- [ ] T046 [US2] Test API response times (< 2 seconds)
- [ ] T047 [US2] Check production logs for errors
- [ ] T048 [US2] Test chatbot in production environment

### Documentation

- [ ] T049 [P] [US2] Create .env.example with all required environment variables
- [ ] T050 [P] [US2] Document setup steps in apps/api/README.md
- [ ] T051 [P] [US2] Document API endpoints in apps/api/docs/API.md

### Git Commit (Phase 1)

- [ ] T052 [US2] Run all tests and verify they pass
- [ ] T053 [US2] Commit all changes with message: "feat: Complete Phase 1 - RAG Chatbot (CORE)"
- [ ] T054 [US2] Push to main branch
- [ ] T055 [US2] Verify main branch build succeeds

**Checkpoint**: Phase 1 COMPLETE - RAG chatbot fully working locally and in production

---

## PHASE 2: AUTHENTICATION (CORE - MUST WORK) 🎯 P2

**Goal**: User signup, login, and profile collection for personalization and translation features

**Critical Success Criteria**:
- Signup works (email/password + GitHub OAuth)
- Login works
- Profile onboarding works
- OAuth redirect correct
- No callback / env errors in production
- All auth flows verified

### BetterAuth Server Setup

- [ ] T056 [US3] Install BetterAuth in apps/api/
- [ ] T057 [US3] Configure BetterAuth providers (email/password + GitHub OAuth)
- [ ] T058 [US3] Set up OAuth callback URLs in apps/api/app/core/config.py
- [ ] T059 [US3] Create BetterAuth configuration in apps/api/app/core/auth_config.py

### User & Profile Database Tables

- [ ] T060 [US3] Create users table migration in apps/api/migrations/002_create_users.sql
- [ ] T061 [US3] Create user_profiles table migration in apps/api/migrations/003_create_user_profiles.sql
- [ ] T062 [US3] Create User SQLAlchemy model in apps/api/app/models/user.py
- [ ] T063 [US3] Create UserProfile SQLAlchemy model in apps/api/app/models/user_profile.py
- [ ] T064 [US3] Run migrations against Neon database

### Authentication Endpoints

- [ ] T065 [P] [US3] Implement POST /api/auth/signup in apps/api/app/api/routes/auth.py
- [ ] T066 [P] [US3] Implement POST /api/auth/login in apps/api/app/api/routes/auth.py
- [ ] T067 [P] [US3] Implement GET /api/auth/github (OAuth redirect) in apps/api/app/api/routes/auth.py
- [ ] T068 [US3] Implement GET /api/auth/github/callback in apps/api/app/api/routes/auth.py
- [ ] T069 [P] [US3] Implement POST /api/auth/logout in apps/api/app/api/routes/auth.py
- [ ] T070 [P] [US3] Implement GET /api/auth/me in apps/api/app/api/routes/auth.py
- [ ] T071 [P] [US3] Implement POST /api/auth/forgot-password in apps/api/app/api/routes/auth.py
- [ ] T072 [P] [US3] Implement POST /api/auth/reset-password in apps/api/app/api/routes/auth.py

### Onboarding Flow

- [ ] T073 [US3] Create POST /api/auth/onboarding endpoint in apps/api/app/api/routes/auth.py
- [ ] T074 [US3] Collect hardware_type (RTX 4090 / Jetson Orin Nano / Other)
- [ ] T075 [US3] Collect expertise_level (Beginner / Intermediate / Advanced)
- [ ] T076 [US3] Store profile data in user_profiles table

### Profile Management

- [ ] T077 [US3] Create PUT /api/auth/profile endpoint in apps/api/app/api/routes/auth.py
- [ ] T078 [US3] Create GET /api/auth/profile endpoint in apps/api/app/api/routes/auth.py
- [ ] T079 [US3] Validate profile update data (hardware_type, expertise_level)

### Frontend Auth Components

- [ ] T080 [P] [US3] Create SignupModal component in apps/docs/src/components/Auth/SignupModal.tsx
- [ ] T081 [P] [US3] Create LoginModal component in apps/docs/src/components/Auth/LoginModal.tsx
- [ ] T082 [P] [US3] Create OnboardingModal component in apps/docs/src/components/Auth/OnboardingModal.tsx
- [ ] T083 [P] [US3] Create ProfileSettings component in apps/docs/src/components/Auth/ProfileSettings.tsx
- [ ] T084 [US3] Add auth buttons to Docusaurus navbar in apps/docs/src/theme/Navbar/index.tsx

### Session Management

- [ ] T085 [US3] Implement JWT token generation in apps/api/app/core/security.py
- [ ] T086 [US3] Implement JWT token verification middleware in apps/api/app/api/deps.py
- [ ] T087 [US3] Configure httpOnly cookie storage for JWT tokens
- [ ] T088 [US3] Add authentication middleware to protected routes

### Local Testing

- [ ] T089 [US3] Test signup with email/password
- [ ] T090 [US3] Verify verification email is sent (or bypassed in local mode)
- [ ] T091 [US3] Test login with email/password
- [ ] T092 [US3] Test GitHub OAuth signup flow
- [ ] T093 [US3] Test onboarding form (hardware + expertise questions)
- [ ] T094 [US3] Verify profile is saved correctly
- [ ] T095 [US3] Test profile update functionality
- [ ] T096 [US3] Test JWT token refresh

### Production Testing

- [ ] T097 [US3] Deploy auth system to production
- [ ] T098 [US3] Test OAuth redirect URLs are correct
- [ ] T099 [US3] Verify no callback errors
- [ ] T100 [US3] Test GitHub OAuth in production environment
- [ ] T101 [US3] Verify environment variables are set correctly
- [ ] T102 [US3] Test session persistence across page refreshes

### Git Commit (Phase 2)

- [ ] T103 [US3] Run all auth tests and verify they pass
- [ ] T104 [US3] Commit all changes with message: "feat: Complete Phase 2 - Authentication & User Profiles"
- [ ] T105 [US3] Push to main branch
- [ ] T106 [US3] Verify main branch build succeeds

**Checkpoint**: Phase 2 COMPLETE - Auth fully working locally and in production

---

## PHASE 3: URDU TRANSLATION (FULL REWRITE) 🎯 P2

**Goal**: Stable, professional Urdu translation per chapter

**Critical Success Criteria**:
- English → Urdu switch works
- No crash / mismatch
- Performance is acceptable
- Encoding / RTL issues resolved
- Verified translations

### Remove Existing Translation Code

- [ ] T107 [US4] Audit existing Urdu translation code in apps/docs/
- [ ] T108 [US4] Remove all existing Urdu translation code from apps/docs/src/
- [ ] T109 [US4] Remove all existing Urdu translation code from apps/api/app/

### Fresh Translation Architecture Design

- [ ] T110 [US4] Design translation file structure in apps/docs/docs-urdu/
- [ ] T111 [US4] Create translation service architecture in apps/api/app/services/translation/
- [ ] T112 [US4] Design translation API contract (input/output schema)

### Translation File Structure

- [ ] T113 [US4] Create apps/docs/docs-urdu/ directory with module structure
- [ ] T114 [US4] Mirror folder structure: module-1/, module-2/, module-3/, module-4/
- [ ] T115 [US4] Set up Urdu MDX file naming convention

### Translation Service Implementation

- [ ] T116 [US4] Create translator.py in apps/api/app/services/translation/translator.py
- [ ] T117 [US4] Implement get_translated_chapter function
- [ ] T118 [US4] Implement code block extraction and preservation logic
- [ ] T119 [US4] Implement technical term preservation (ROS 2, Isaac Sim, VSLAM, etc.)
- [ ] T120 [US4] Implement prose text translation using Claude/LLM
- [ ] T121 [US4] Implement code block re-insertion logic

### Translation API Endpoint

- [ ] T122 [US4] Create POST /api/translation/get endpoint in apps/api/app/api/routes/translation.py
- [ ] T123 [US4] Validate input (chapter_slug, language)
- [ ] T124 [US4] Return chapter content in requested language

### User Language Preference

- [ ] T125 [US4] Add preferred_language column to user_profiles table
- [ ] T126 [US4] Update user profile on translation toggle
- [ ] T127 [US4] Auto-apply language preference on chapter loads

### Frontend Translation Toggle

- [ ] T128 [US4] Create TranslateButton component in apps/docs/src/components/ChapterControls/TranslateButton.tsx
- [ ] T129 [US4] Add "Translate to Urdu" / "Switch to English" button to chapter pages
- [ ] T130 [US4] Implement client-side content replacement on translation
- [ ] T131 [US4] Persist language preference in user profile

### Code Block & Term Preservation

- [ ] T132 [US4] Create regex patterns for code block detection (triple backticks)
- [ ] T133 [US4] Create list of technical terms to preserve in English
- [ ] T134 [US4] Test code block preservation with sample chapters
- [ ] T135 [US4] Test technical term preservation with sample chapters

### Generate Urdu Translations

- [ ] T136 [US4] Translate module-1 chapters to Urdu
- [ ] T137 [US4] Translate module-2 chapters to Urdu
- [ ] T138 [US4] Translate module-3 chapters to Urdu
- [ ] T139 [US4] Translate module-4 chapters to Urdu
- [ ] T140 [US4] Review all Urdu translations for quality

### Local Testing

- [ ] T141 [US4] Test translation toggle on Chapter 1
- [ ] T142 [US4] Verify prose text changes to Urdu
- [ ] T143 [US4] Verify code blocks remain unchanged
- [ ] T144 [US4] Verify technical terms remain unchanged
- [ ] T145 [US4] Test navigation to next chapter preserves Urdu preference
- [ ] T146 [US4] Test RTL text rendering in browser
- [ ] T147 [US4] Test no crashes or mismatches occur

### Production Testing

- [ ] T148 [US4] Deploy translation system to production
- [ ] T149 [US4] Test translation performance in production (< 2 seconds)
- [ ] T150 [US4] Verify encoding issues are resolved
- [ ] T151 [US4] Verify RTL issues are resolved
- [ ] T152 [US4] Test translation toggle across all chapters

### Git Commit (Phase 3)

- [ ] T153 [US4] Verify all translations are working
- [ ] T154 [US4] Commit all changes with message: "feat: Complete Phase 3 - Urdu Translation System"
- [ ] T155 [US4] Push to main branch
- [ ] T156 [US4] Verify main branch build succeeds

**Checkpoint**: Phase 3 COMPLETE - Urdu translation fully working, tested, and verified

---

## PHASE 4: PERSONALIZATION (FULL REWRITE) 🎯 P3

**Goal**: User-based personalized content without errors

**Critical Success Criteria**:
- Different users see different content based on hardware profile
- No runtime errors
- Data consistency maintained
- Safe fallback when no profile data
- Performance acceptable

### Remove Existing Personalization Code

- [ ] T157 [US5] Audit existing personalization code in apps/docs/
- [ ] T158 [US5] Remove all existing personalization code from apps/docs/src/
- [ ] T159 [US5] Remove all existing personalization code from apps/api/app/

### New Personalization Logic Design

- [ ] T160 [US5] Design personalization architecture
- [ ] T161 [US5] Define personalization rules (RTX 4090 vs Jetson Orin Nano)
- [ ] T162 [US5] Design personalization cache strategy (7-day TTL)
- [ ] T163 [US5] Create personalization API contract

### Database Schema for Personalization

- [ ] T164 [US5] Create personalized_content table migration in apps/api/migrations/004_create_personalized_content.sql
- [ ] T165 [US5] Create PersonalizedContent model in apps/api/app/models/personalized_content.py
- [ ] T166 [US5] Run migration against Neon database

### Personalization Service Implementation

- [ ] T167 [US5] Create personalizer.py in apps/api/app/services/personalization/personalizer.py
- [ ] T168 [US5] Implement personalize_content function
- [ ] T169 [US5] Implement hardware-based code example replacement logic
- [ ] T170 [US5] Implement expertise-based explanation depth adjustment
- [ ] T171 [US5] Implement cache lookup and storage logic

### Content Variation Storage

- [ ] T172 [US5] Create personalization rules configuration file (YAML/JSON)
- [ ] T173 [US5] Define RTX 4090 code variations for chapters
- [ ] T174 [US5] Define Jetson Orin Nano code variations for chapters
- [ ] T175 [US5] Implement dynamic variation generation using AI (OpenAI API)

### Personalization API Endpoint

- [ ] T176 [US5] Create POST /api/personalization/apply endpoint in apps/api/app/api/routes/personalization.py
- [ ] T177 [US5] Validate input (chapter_slug, user_id)
- [ ] T178 [US5] Return personalized chapter content (MDX)
- [ ] T179 [US5] Log personalization requests in audit_logs table

### Frontend Personalization Toggle

- [ ] T180 [US5] Create PersonalizeButton component in apps/docs/src/components/ChapterControls/PersonalizeButton.tsx
- [ ] T181 [US5] Add "Personalize for my setup" button to chapter pages
- [ ] T182 [US5] Fetch personalized content from API on button click
- [ ] T183 [US5] Replace chapter content with personalized version
- [ ] T184 [US5] Add toggle to revert to generic version

### Profile Completeness Check

- [ ] T185 [US5] Check if user profile has hardware_type set
- [ ] T186 [US5] Disable personalization button if profile incomplete
- [ ] T187 [US5] Show tooltip: "Complete your hardware profile in Settings"

### Safe Fallback Implementation

- [ ] T188 [US5] Implement fallback to generic content if personalization fails
- [ ] T189 [US5] Handle missing user profile gracefully (no crash)
- [ ] T190 [US5] Log errors without exposing to user

### Local Testing

- [ ] T191 [US5] Create test user with RTX 4090 profile
- [ ] T192 [US5] Create test user with Jetson Orin Nano profile
- [ ] T193 [US5] Test RTX 4090 user sees high-VRAM code examples
- [ ] T194 [US5] Test Jetson user sees quantized code examples
- [ ] T195 [US5] Test beginner user sees simplified explanations
- [ ] T196 [US5] Test advanced user sees concise explanations
- [ ] T197 [US5] Test personalization toggle off reverts to generic content
- [ ] T198 [US5] Test incomplete profile disables personalization button

### Production Testing

- [ ] T199 [US5] Deploy personalization system to production
- [ ] T200 [US5] Test personalization with different user profiles
- [ ] T201 [US5] Verify no runtime errors
- [ ] T202 [US5] Verify data consistency
- [ ] T203 [US5] Test cache performance (< 2 seconds)
- [ ] T204 [US5] Test fallback behavior when API fails

### Git Commit (Phase 4)

- [ ] T205 [US5] Verify all personalization features work
- [ ] T206 [US5] Commit all changes with message: "feat: Complete Phase 4 - Content Personalization System"
- [ ] T207 [US5] Push to main branch
- [ ] T208 [US5] Verify main branch build succeeds

**Checkpoint**: Phase 4 COMPLETE - Personalization fully working, tested, and verified

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (RAG Chatbot)**: No dependencies - can start immediately
- **Phase 2 (Authentication)**: No dependencies on Phase 1 - can run in parallel BUT must complete before Phase 3 and 4
- **Phase 3 (Urdu Translation)**: Depends on Phase 2 (Authentication) - requires user login and profile
- **Phase 4 (Personalization)**: Depends on Phase 2 (Authentication) - requires user profile with hardware_type

### Critical Path

```
Phase 1 (RAG Chatbot) → Production Deploy → Main Branch Push
                         ↓
Phase 2 (Authentication) → Production Deploy → Main Branch Push
                         ↓
                    ┌────┴────┐
                    ↓         ↓
Phase 3 (Translation)  Phase 4 (Personalization)
        ↓                    ↓
Production Deploy    Production Deploy
        ↓                    ↓
Main Branch Push    Main Branch Push
```

### Parallel Opportunities

- Phase 1 and Phase 2 can be developed in parallel (separate codebases)
- Phase 3 and Phase 4 can be developed in parallel AFTER Phase 2 is complete
- Within each phase:
  - Frontend components marked [P] can be built in parallel
  - Backend endpoints marked [P] can be built in parallel
  - Database migrations can be run in parallel if no FK dependencies

---

## Implementation Strategy

### MVP First (Phase 1 Only)

1. Complete Phase 1: RAG Chatbot
2. Test locally (all tasks T035-T042)
3. Test in production (all tasks T043-T048)
4. Push to main branch (tasks T052-T055)
5. **STOP and VALIDATE**: Chatbot fully functional independently

### Incremental Delivery

1. Phase 1 (RAG) → Deploy → Test → Commit → **CHECKPOINT**
2. Phase 2 (Auth) → Deploy → Test → Commit → **CHECKPOINT**
3. Phase 3 (Translation) → Deploy → Test → Commit → **CHECKPOINT**
4. Phase 4 (Personalization) → Deploy → Test → Commit → **CHECKPOINT**

Each phase adds value without breaking previous phases.

### Quality Gates

Before moving to next phase, verify:
- [ ] All tasks in current phase completed
- [ ] Local testing passed (no errors)
- [ ] Production testing passed (no errors)
- [ ] Logs are clean (no warnings/errors)
- [ ] Manual verification done
- [ ] Code committed to main branch
- [ ] Main branch build succeeds

---

## Final Rules

### No Half-Baked Code

- Every task must be fully implemented
- No TODO comments left in code
- No commented-out code blocks
- All console.log / debug statements removed

### No "It Works on My Machine"

- Every phase MUST be tested in production
- Environment variables MUST be documented
- Setup instructions MUST be verified

### No Phase Overlap

- Complete current phase fully before starting next
- No mixing tasks from different phases
- Respect phase dependencies

### After Every Phase

- [ ] Manual testing (complete user journey)
- [ ] Check logs (no errors/warnings)
- [ ] Clean commit (meaningful message)
- [ ] Only working code goes to main

### Success Definition

Project is complete when:
- All 4 phases are marked COMPLETE
- All 208 tasks are checked
- Production deployment is stable
- Main branch build is green
- User acceptance testing passes

---

**Total Tasks**: 208
**Phases**: 4
**Estimated Parallelization**: ~30% of tasks can run in parallel within phases

**Next Step**: Execute Phase 1 tasks T001-T055 sequentially, test, deploy, and push to main before starting Phase 2.
