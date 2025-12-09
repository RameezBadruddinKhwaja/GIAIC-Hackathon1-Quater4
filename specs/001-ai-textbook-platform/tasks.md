# Tasks: Physical AI & Humanoid Robotics Textbook Platform

**Input**: Design documents from `/specs/001-ai-textbook-platform/`
**Prerequisites**: plan.md (complete), spec.md v2 (complete), research.md (complete), data-model.md (complete), contracts/ (complete)

**Organization**: Tasks grouped by workflow category with explicit Claude subagent assignments (Constitution Article VIII) and skill integrations (Constitution Article IX)

**Hackathon Scoring Strategy**:
- Base MVP: 100/100 points (Categories 1-4: Planning → Specification → Content Authoring → Validation)
- Bonus Features: +200 points (Category 6: Bonus Features)
- **Total Target: 300/100 points**

---

## Format: `[ID] [P?] [Agent] [Skills] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Agent]**: Responsible Claude subagent from Constitution Article VIII
- **[Skills]**: Applicable skills from Constitution Article IX (23 canonical + 5 bonus)
- Include exact file paths in descriptions

---

## Category 1: Planning & Infrastructure Setup

**Purpose**: Establish monorepo structure, agentic infrastructure (.claude/), and development environment

**Responsible Subagent**: super-orchestrator (coordinates all setup)

### 1.1 Monorepo Initialization

- [ ] T001 [super-orchestrator] [tool-selection-framework] Create monorepo root structure with `apps/`, `packages/`, `.claude/`, `specs/`, `history/` directories
- [ ] T002 [super-orchestrator] [docusaurus-deployer] Initialize `apps/docs` with Docusaurus v3 via `npx create-docusaurus@latest apps/docs classic --typescript`
- [ ] T003 [super-orchestrator] [mvp-builder] Initialize `apps/api` with FastAPI project structure: `src/`, `tests/`, `scripts/`, `migrations/` directories
- [ ] T004 [P] [super-orchestrator] [frontend-design] Create `packages/shared-types` with TypeScript package.json for shared interfaces (User, ChatMessage, Citation)
- [ ] T005 [P] [super-orchestrator] Create root `.gitignore` with Node.js, Python, and environment variable exclusions
- [ ] T006 [P] [super-orchestrator] Create root `README.md` with project overview and quickstart link to `specs/001-ai-textbook-platform/quickstart.md`

### 1.2 Environment Configuration (Gemini Drop-in Replacement)

- [ ] T007 [super-orchestrator] Create `apps/api/.env.example` template with GEMINI_API_KEY, NEON_CONNECTION_STRING, QDRANT_URL, QDRANT_API_KEY, BETTER_AUTH_GITHUB_CLIENT_ID, BETTER_AUTH_GITHUB_CLIENT_SECRET, JWT_SECRET, ENVIRONMENT
- [ ] T008 [super-orchestrator] Create `apps/docs/.env.example` template with REACT_APP_API_URL, REACT_APP_BETTER_AUTH_GITHUB_CLIENT_ID
- [ ] T009 [super-orchestrator] Document Gemini API key procurement process in `apps/api/README.md` (obtain from https://makersuite.google.com/app/apikey)
- [ ] T010 [super-orchestrator] Create `apps/api/requirements.txt` with dependencies: fastapi, uvicorn, sqlalchemy[asyncio], qdrant-client, openai, openai-agents-sdk, better-auth-python, pydantic, python-multipart, alembic

### 1.3 Agentic Infrastructure (Matrix Protocol - Constitution Article VIII & IX)

**CRITICAL**: These agent/skill definitions enable bonus points and Constitution compliance

- [ ] T011 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/chapter-planner.md` with responsibilities: divide syllabus into 13 weeks, skills: chapter-planner, book-scaffolding, learning-objectives
- [ ] T012 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/spec-architect.md` with responsibilities: design book spec, skills: mvp-builder, tool-selection-framework, technical-clarity
- [ ] T013 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/pedagogical-designer.md` with responsibilities: learning outcomes per chapter, skills: learning-objectives, exercise-designer, assessment-builder
- [ ] T014 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/educational-validator.md` with responsibilities: validate content alignment, skills: validation-auditor, technical-clarity, canonical-format-checker
- [ ] T015 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/content-implementor.md` with responsibilities: write MDX files, skills: book-scaffolding, code-example-generator, image-generator, concept-scaffolding
- [ ] T016 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/factual-verifier.md` with responsibilities: validate technical accuracy, skills: technical-clarity, canonical-format-checker, Context7 MCP integration
- [ ] T017 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/assessment-architect.md` with responsibilities: create quizzes/challenges, skills: quiz-generator, assessment-builder, exercise-designer
- [ ] T018 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/validation-auditor.md` with responsibilities: style/UI testing, skills: validation-auditor, playwright-test-runner, canonical-format-checker
- [ ] T019 [P] [super-orchestrator] [skill-creator] Create `.claude/agents/super-orchestrator.md` with responsibilities: coordinate all agents, skills: ALL SKILLS, MCP orchestration

### 1.4 Canonical Skills (Constitution Article IX - 23 Required + 5 Bonus)

**Canonical Skills (23)**:

- [ ] T020 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/book-scaffolding.md` with Docusaurus project structure patterns, sidebars, frontmatter
- [ ] T021 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/chapter-planner.md` with curriculum division logic, module grouping, week assignments
- [ ] T022 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/concept-scaffolding.md` with conceptual frameworks for technical topics
- [ ] T023 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/summary-generator.md` with chapter summary and TL;DR patterns
- [ ] T024 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/quiz-generator.md` with interactive quiz creation (multiple-choice, code challenges)
- [ ] T025 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/technical-clarity.md` with technical accuracy guidelines and clarity standards
- [ ] T026 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/canonical-format-checker.md` with markdown validation rules
- [ ] T027 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/assessment-builder.md` with comprehensive assessment design and rubrics
- [ ] T028 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/mvp-builder.md` with MVP feature identification
- [ ] T029 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/learning-objectives.md` with clear, measurable outcome patterns
- [ ] T030 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/docusaurus-deployer.md` with Vercel/GitHub Pages deployment workflows
- [ ] T031 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/prompt-template-designer.md` with reusable agent prompts
- [ ] T032 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/code-example-generator.md` with ROS 2/Python/C++ snippet patterns
- [ ] T033 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/exercise-designer.md` with hands-on coding challenges
- [ ] T034 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/frontend-design.md` with React component and UI/UX patterns
- [ ] T035 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/validation-auditor.md` with comprehensive validation checklists
- [ ] T036 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/skill-creator.md` (meta-skill) with dynamic skill generation patterns
- [ ] T037 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/playwright-test-runner.md` with Playwright MCP UI testing workflows
- [ ] T038 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/image-generator.md` with concept diagram creation guidelines
- [ ] T039 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/ux-evaluator.md` with UX evaluation criteria
- [ ] T040 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/tool-selection-framework.md` with technology selection decision trees
- [ ] T041 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/notebooklm-slides.md` with presentation generation (optional)
- [ ] T042 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/session-intelligence-harvester.md` with user context collection for personalization

**Bonus Skills (5)**:

- [ ] T043 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/urdu-translator.md` with Urdu translation rules, code preservation patterns
- [ ] T044 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/user-profile-initializer.md` with BetterAuth signup/onboarding workflows
- [ ] T045 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/ros2-code-generator.md` with natural language to ROS2 Python code patterns
- [ ] T046 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/rag-chatbot-integrator.md` with OpenAI Agents + Qdrant + ChatKit integration
- [ ] T047 [P] [super-orchestrator] [skill-creator] Create `.claude/skills/personalization-engine.md` with hardware-aware content adaptation logic

### 1.5 Database Setup (Neon PostgreSQL)

- [ ] T048 [super-orchestrator] Create Neon account at console.neon.tech and provision "physical-ai-textbook-platform" project in US East (Ohio) region
- [ ] T049 [super-orchestrator] Create `apps/api/src/models/__init__.py` as models package initializer
- [ ] T050 [P] [super-orchestrator] Create `apps/api/src/models/user.py` with User SQLAlchemy model (id, email, auth_provider, hardware_profile, programming_language, created_at, last_login)
- [ ] T051 [P] [super-orchestrator] Create `apps/api/src/models/chat_log.py` with ChatLog SQLAlchemy model (id, user_id, query_text, response_text, cited_chapters JSONB, skills_loaded JSONB, sanitized_input, created_at)
- [ ] T052 [P] [super-orchestrator] Create `apps/api/src/models/personalized_content.py` with PersonalizedContent SQLAlchemy model (id, user_id, chapter_id, hardware_profile, personalized_mdx, created_at)
- [ ] T053 [P] [super-orchestrator] Create `apps/api/src/models/translated_content.py` with TranslatedContent SQLAlchemy model (id, user_id, chapter_id, target_language, translated_mdx, created_at)
- [ ] T054 [P] [super-orchestrator] Create `apps/api/src/models/audit_log.py` with AuditLog SQLAlchemy model (id, user_id, event_type, event_details JSONB, ip_address, created_at)
- [ ] T055 [super-orchestrator] Create `apps/api/migrations/env.py` with Alembic configuration pointing to Neon connection string
- [ ] T056 [super-orchestrator] Generate Alembic migration: `alembic revision --autogenerate -m "Create users table"`
- [ ] T057 [super-orchestrator] Generate Alembic migration: `alembic revision --autogenerate -m "Create supporting tables"`
- [ ] T058 [super-orchestrator] Create `apps/api/scripts/migrate_db.py` script to run Alembic migrations programmatically

### 1.6 Vector Store Setup (Qdrant Cloud)

- [ ] T059 [super-orchestrator] Create Qdrant Cloud account at cloud.qdrant.io and provision "book-knowledge-cluster" with Free Tier (1GB storage)
- [ ] T060 [super-orchestrator] Create `apps/api/scripts/init_qdrant.py` to initialize `book_knowledge` collection with 768-dimensional vectors (for Gemini text-embedding-004), Cosine distance, payload schema (chapter_id, section_id, part_number, week_number, content_type, hardware_context)
- [ ] T061 [super-orchestrator] Run `python apps/api/scripts/init_qdrant.py` to create Qdrant collection and verify success

### 1.7 Gemini Client Configuration (Drop-in Replacement)

**CRITICAL**: OpenAI SDK configured with Gemini backend per Constitution Article II, Item 6

- [ ] T062 [super-orchestrator] Create `apps/api/src/services/gemini_client.py` with OpenAI client initialization: `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"`, `api_key=os.getenv("GEMINI_API_KEY")`, model `gemini-2.5-flash`
- [ ] T063 [super-orchestrator] Add gemini_client.py docstring explaining drop-in replacement strategy and emphasizing NOT to use Google generativeai SDK
- [ ] T064 [super-orchestrator] Create `apps/api/src/utils/embeddings.py` with function to generate 768-dim embeddings using Gemini `text-embedding-004` model via OpenAI SDK

**Checkpoint 1 Complete**: Foundation ready - Monorepo initialized, 9 agents created, 28 skills created, Gemini configured, Neon/Qdrant provisioned

---

## Category 2: Specification & Design (User Story 7 - P0 Foundation)

**Purpose**: Generate complete book specification, chapter breakdown, and learning architecture via Claude subagent orchestration

**Responsible Subagents**: chapter-planner, spec-architect, pedagogical-designer

### 2.1 Curriculum Architecture (chapter-planner agent)

- [ ] T065 [chapter-planner] [chapter-planner, book-scaffolding, learning-objectives] Read Physical AI syllabus from `specs/001-ai-textbook-platform/syllabus.md` (create if missing)
- [ ] T066 [chapter-planner] [chapter-planner] Divide curriculum into 4 parts: Part 1 "The Nervous System" (Weeks 1-5 ROS 2), Part 2 "The Digital Twin" (Weeks 6-7 Gazebo/Unity), Part 3 "The Brain" (Weeks 8-10 NVIDIA Isaac), Part 4 "VLA & Humanoids" (Weeks 11-13)
- [ ] T067 [chapter-planner] [learning-objectives] Generate learning objectives for each of 13 weeks with measurable outcomes
- [ ] T068 [chapter-planner] [chapter-planner] Create chapter breakdown manifest in `apps/docs/chapters-manifest.json` with chapter_id, title, part_number, week_number, learning_objectives array

### 2.2 Learning Design (pedagogical-designer agent)

- [ ] T069 [pedagogical-designer] [learning-objectives, exercise-designer] For each chapter in manifest, define specific learning outcomes (understand, apply, analyze levels per Bloom's taxonomy)
- [ ] T070 [pedagogical-designer] [exercise-designer] Design hands-on exercises for each chapter (e.g., Week 1: Create ROS 2 publisher/subscriber node)
- [ ] T071 [pedagogical-designer] [assessment-builder] Design module assessments for each of 4 parts with rubrics and scoring criteria
- [ ] T072 [pedagogical-designer] [learning-objectives] Update `chapters-manifest.json` with exercises and assessment references

### 2.3 Technical Specification (spec-architect agent)

- [ ] T073 [spec-architect] [mvp-builder, tool-selection-framework] Validate spec.md v2 alignment with Constitution v5.0.0 (all 7 user stories, 44 FRs, 23 success criteria documented)
- [ ] T074 [spec-architect] [technical-clarity] Review data-model.md for completeness (5 Neon entities + 1 Qdrant collection with 768-dim vectors)
- [ ] T075 [spec-architect] [tool-selection-framework] Verify plan.md architecture (Gemini drop-in replacement, monorepo structure, MCP integrations)
- [ ] T076 [spec-architect] [mvp-builder] Confirm quickstart.md local development guide is complete

**Checkpoint 2 Complete**: Specification validated - 13-week curriculum designed, learning objectives defined, technical architecture reviewed

---

## Category 3: Content Authoring (User Story 1 - P1 MVP, User Story 7 - Content Generation)

**Purpose**: Generate all 13 weeks of Docusaurus MDX content with Mermaid diagrams, code tabs, Hardware Lab Guide

**Responsible Subagent**: content-implementor (primary), factual-verifier (validation)

**Scoring**: Completes base 100/100 hackathon points

### 3.1 Docusaurus Configuration

- [ ] T077 [content-implementor] [docusaurus-deployer, book-scaffolding] Install Docusaurus Mermaid plugin: `npm install --save @docusaurus/theme-mermaid` in `apps/docs/`
- [ ] T078 [content-implementor] [docusaurus-deployer] Configure `apps/docs/docusaurus.config.js` to enable Mermaid support with `markdown.mermaid: true` and add `@docusaurus/theme-mermaid` to themes array
- [ ] T079 [content-implementor] [book-scaffolding] Configure `apps/docs/docusaurus.config.js` with project metadata: title "Physical AI & Humanoid Robotics Textbook", tagline "AI-Native Learning Platform for ROS 2, NVIDIA Isaac, and VLA Systems"
- [ ] T080 [content-implementor] [book-scaffolding] Configure `apps/docs/docusaurus.config.js` navbar with "Content", "Hardware Lab", "Chat" links
- [ ] T081 [content-implementor] [chapter-planner, book-scaffolding] Create `apps/docs/sidebars.js` with 4 main categories: "Part 1: The Nervous System (Weeks 1-5)", "Part 2: The Digital Twin (Weeks 6-7)", "Part 3: The Brain (Weeks 8-10)", "Part 4: VLA & Humanoids (Weeks 11-13)", "Hardware Lab Guide"

### 3.2 Part 1: The Nervous System (Weeks 1-5) - ROS 2 Fundamentals

**All tasks parallelizable - independent MDX files**

- [ ] T082 [P] [content-implementor] [code-example-generator, concept-scaffolding, book-scaffolding] Create `apps/docs/docs/week-01-ros2-basics/index.md` with ROS 2 introduction, installation, "Hello Robot" example with Docusaurus Tabs for "Simulated" (Gazebo) vs "Real Robot" (Jetson) code
- [ ] T083 [P] [content-implementor] [code-example-generator, image-generator] Create `apps/docs/docs/week-02-nodes-topics/index.md` with ROS 2 Nodes and Topics explanation, Mermaid diagram for pub/sub pattern, code examples in Python/C++ tabs
- [ ] T084 [P] [content-implementor] [code-example-generator, concept-scaffolding] Create `apps/docs/docs/week-03-urdf-modeling/index.md` with URDF robot description format, visual examples, Gazebo integration code
- [ ] T085 [P] [content-implementor] [code-example-generator, image-generator] Create `apps/docs/docs/week-04-services-actions/index.md` with ROS 2 Services and Actions patterns, Mermaid sequence diagrams
- [ ] T086 [P] [content-implementor] [code-example-generator, image-generator] Create `apps/docs/docs/week-05-nav2/index.md` with Navigation2 stack overview, SLAM concepts, Mermaid architecture diagram

### 3.3 Part 2: The Digital Twin (Weeks 6-7) - Simulation

- [ ] T087 [P] [content-implementor] [code-example-generator, concept-scaffolding] Create `apps/docs/docs/week-06-gazebo-sim/index.md` with Gazebo Classic vs Gazebo Fortress comparison, world files, sensor simulation with Simulated/Real Robot tabs
- [ ] T088 [P] [content-implementor] [code-example-generator, concept-scaffolding] Create `apps/docs/docs/week-07-unity-sim/index.md` with Unity Robotics Hub, ROS-TCP-Connector setup, Unity ML-Agents integration code

### 3.4 Part 3: The Brain (Weeks 8-10) - NVIDIA Isaac

- [ ] T089 [P] [content-implementor] [code-example-generator, concept-scaffolding, image-generator] Create `apps/docs/docs/week-08-isaac-sim-basics/index.md` with NVIDIA Isaac Sim installation (Omniverse), USD file format, PhysX simulation examples
- [ ] T090 [P] [content-implementor] [code-example-generator, image-generator] Create `apps/docs/docs/week-09-isaac-ros/index.md` with Isaac ROS packages, hardware acceleration, apriltag detection, visual SLAM with Mermaid pipeline diagram
- [ ] T091 [P] [content-implementor] [code-example-generator, concept-scaffolding] Create `apps/docs/docs/week-10-isaac-orbit/index.md` with Isaac Orbit framework, reinforcement learning environments, training workflow code

### 3.5 Part 4: VLA & Humanoids (Weeks 11-13) - Advanced Topics

- [ ] T092 [P] [content-implementor] [code-example-generator, concept-scaffolding, image-generator] Create `apps/docs/docs/week-11-vla-intro/index.md` with Vision-Language-Action models explanation, RT-1/RT-2 architectures, Gemini integration examples
- [ ] T093 [P] [content-implementor] [code-example-generator, concept-scaffolding] Create `apps/docs/docs/week-12-droid-deployment/index.md` with LeRobot library, ACT (Action Chunking Transformer), teleoperation data collection code
- [ ] T094 [P] [content-implementor] [code-example-generator, concept-scaffolding] Create `apps/docs/docs/week-13-humanoid-systems/index.md` with Unitree G1/H1, Boston Dynamics Spot SDK, whole-body control examples

### 3.6 Hardware Lab Guide

- [ ] T095 [content-implementor] [concept-scaffolding, book-scaffolding] Create `apps/docs/docs/hardware-lab/index.md` with Hardware Lab overview and setup decision flowchart (Simulated vs Edge)
- [ ] T096 [content-implementor] [concept-scaffolding, technical-clarity] Create `apps/docs/docs/hardware-lab/digital-twin-rig.md` with RTX 4090 specs, Isaac Sim requirements, high-fidelity simulation setup instructions
- [ ] T097 [content-implementor] [concept-scaffolding, technical-clarity] Create `apps/docs/docs/hardware-lab/edge-kit.md` with Jetson Orin Nano specs, power optimization, ROS 2 Humble on JetPack installation, edge deployment workflow
- [ ] T098 [content-implementor] [concept-scaffolding] Create `apps/docs/docs/hardware-lab/comparison-table.md` with side-by-side comparison: compute power, memory, power consumption, use cases, cost for RTX 4090 vs Jetson Orin Nano

### 3.7 Docusaurus Customization

- [ ] T099 [content-implementor] [frontend-design, book-scaffolding] Add Docusaurus Tabs component imports to all chapters requiring "Simulated vs Real Robot" code examples (Weeks 1, 6, 8, 9)
- [ ] T100 [content-implementor] [frontend-design] Create `apps/docs/src/css/custom.css` with modern color scheme: dark background (#0d1117), accent colors, monospace code font
- [ ] T101 [content-implementor] [frontend-design] Add Mermaid theme configuration in `docusaurus.config.js` to match color scheme

**Checkpoint 3 Complete**: Content Engine MVP ready - 13-week textbook with 4 modules, Mermaid diagrams, code tabs, Hardware Lab Guide → 100/100 base points

---

## Category 4: Validation & Quality Assurance (User Story 7 - Verification)

**Purpose**: Validate technical accuracy, format compliance, and content quality across all 13 weeks

**Responsible Subagents**: factual-verifier, educational-validator, validation-auditor

### 4.1 Technical Accuracy Validation (factual-verifier agent + Context7 MCP)

**MCP Integration**: Context7 MCP for documentation verification

- [ ] T102 [factual-verifier] [technical-clarity, Context7 MCP] Verify Week 1-2 ROS 2 Basics content against official ROS 2 Humble documentation fetched via Context7 MCP (`resolve-library-id` for "ros2", then `get-library-docs` with topic "nodes topics")
- [ ] T103 [factual-verifier] [technical-clarity, Context7 MCP] Verify Week 3-5 ROS 2 Advanced content (URDF, Services, Nav2) against ROS 2 documentation via Context7 MCP (topics: "urdf", "services actions", "navigation2")
- [ ] T104 [factual-verifier] [technical-clarity, Context7 MCP] Verify Week 6-7 Simulation content against Gazebo and Unity Robotics Hub documentation via Context7 MCP (topics: "gazebo fortress", "unity robotics hub")
- [ ] T105 [factual-verifier] [technical-clarity, Context7 MCP] Verify Week 8-10 NVIDIA Isaac content against NVIDIA Isaac Sim/ROS/Orbit documentation via Context7 MCP (topics: "isaac sim", "isaac ros", "isaac orbit")
- [ ] T106 [factual-verifier] [technical-clarity] Verify Week 11-13 VLA and Humanoid content against RT-1/RT-2 papers, LeRobot, Unitree/Boston Dynamics SDK documentation (manual verification with citations)
- [ ] T107 [factual-verifier] [technical-clarity] Create `apps/docs/validation-report.md` with findings: verified claims, citations, corrections needed

### 4.2 Educational Alignment Validation (educational-validator agent)

- [ ] T108 [educational-validator] [validation-auditor, learning-objectives] Validate all 13 weeks have clear learning objectives aligned with Bloom's taxonomy
- [ ] T109 [educational-validator] [validation-auditor, exercise-designer] Verify exercises match chapter learning outcomes and provide hands-on practice
- [ ] T110 [educational-validator] [assessment-builder] Confirm module assessments (4 parts) have rubrics and scoring criteria
- [ ] T111 [educational-validator] [technical-clarity] Check content progression: concepts build on previous weeks, no knowledge gaps
- [ ] T112 [educational-validator] [validation-auditor] Update `apps/docs/validation-report.md` with educational alignment findings

### 4.3 Format & Style Validation (validation-auditor agent)

- [ ] T113 [validation-auditor] [canonical-format-checker, validation-auditor] Run markdown linter on all MDX files in `apps/docs/docs/` to verify canonical formatting
- [ ] T114 [validation-auditor] [canonical-format-checker] Verify all code blocks have language identifiers (```python, ```cpp, ```bash)
- [ ] T115 [validation-auditor] [canonical-format-checker] Verify all Mermaid diagrams use correct syntax and render properly
- [ ] T116 [validation-auditor] [canonical-format-checker] Verify Docusaurus Tabs components use correct syntax for "Simulated" and "Real Robot" variations
- [ ] T117 [validation-auditor] [validation-auditor] Generate style validation report in `apps/docs/style-validation-report.md`

### 4.4 UI Testing (validation-auditor agent + Playwright MCP)

**MCP Integration**: Playwright MCP for automated UI testing

- [ ] T118 [validation-auditor] [playwright-test-runner, Playwright MCP] Use Playwright MCP to navigate Docusaurus site (`mcp__playwright__browser_navigate` to http://localhost:3000), verify homepage loads
- [ ] T119 [validation-auditor] [playwright-test-runner, Playwright MCP] Use Playwright MCP to take snapshots (`mcp__playwright__browser_snapshot`) of all 13 week chapters, verify content renders correctly
- [ ] T120 [validation-auditor] [playwright-test-runner, Playwright MCP] Use Playwright MCP to test code tab switching (`mcp__playwright__browser_click` on "Simulated" and "Real Robot" tabs), verify correct code displays
- [ ] T121 [validation-auditor] [playwright-test-runner, Playwright MCP] Use Playwright MCP to verify Mermaid diagrams render (`mcp__playwright__browser_take_screenshot` of diagram sections)
- [ ] T122 [validation-auditor] [playwright-test-runner, Playwright MCP] Generate UI testing report with screenshots in `apps/docs/ui-test-report.md`

**Checkpoint 4 Complete**: Validation passed - Technical accuracy verified via Context7 MCP, educational alignment confirmed, format compliance checked, UI tested via Playwright MCP

---

## Category 5: Assessment & Interactive Features

**Purpose**: Add quizzes, coding challenges, and module assessments to enhance learning

**Responsible Subagent**: assessment-architect

### 5.1 Quiz Generation (assessment-architect agent)

- [ ] T123 [P] [assessment-architect] [quiz-generator, assessment-builder] Create `apps/docs/docs/week-01-ros2-basics/quiz.md` with 5 multiple-choice questions on ROS 2 basics (nodes, topics, workspace)
- [ ] T124 [P] [assessment-architect] [quiz-generator] Create quizzes for Weeks 2-5 (ROS 2 topics: nodes/topics, URDF, services/actions, Nav2)
- [ ] T125 [P] [assessment-architect] [quiz-generator] Create quizzes for Weeks 6-7 (Simulation topics: Gazebo, Unity)
- [ ] T126 [P] [assessment-architect] [quiz-generator] Create quizzes for Weeks 8-10 (NVIDIA Isaac topics: Isaac Sim, Isaac ROS, Isaac Orbit)
- [ ] T127 [P] [assessment-architect] [quiz-generator] Create quizzes for Weeks 11-13 (VLA topics: VLA intro, DROID, Humanoids)

### 5.2 Coding Challenges (assessment-architect agent)

- [ ] T128 [P] [assessment-architect] [exercise-designer, code-example-generator] Create `apps/docs/docs/week-01-ros2-basics/challenge.md` with coding challenge: "Create a ROS 2 publisher/subscriber pair for robot velocity commands"
- [ ] T129 [P] [assessment-architect] [exercise-designer, code-example-generator] Create coding challenges for Weeks 2-5 (ROS 2 challenges: topic remapping, URDF creation, service client, Nav2 configuration)
- [ ] T130 [P] [assessment-architect] [exercise-designer, code-example-generator] Create coding challenges for Weeks 6-7 (Simulation challenges: Gazebo world design, Unity-ROS integration)
- [ ] T131 [P] [assessment-architect] [exercise-designer, code-example-generator] Create coding challenges for Weeks 8-10 (Isaac challenges: Isaac Sim scene setup, Isaac ROS apriltag detection, Isaac Orbit training)
- [ ] T132 [P] [assessment-architect] [exercise-designer, code-example-generator] Create coding challenges for Weeks 11-13 (VLA challenges: VLA prompt engineering, DROID teleoperation, humanoid control)

### 5.3 Module Assessments (assessment-architect agent)

- [ ] T133 [assessment-architect] [assessment-builder, quiz-generator] Create `apps/docs/docs/assessments/part-1-assessment.md` with comprehensive Part 1 assessment (Weeks 1-5 ROS 2) including theory questions, coding problems, rubric
- [ ] T134 [assessment-architect] [assessment-builder] Create Part 2 assessment (Weeks 6-7 Simulation)
- [ ] T135 [assessment-architect] [assessment-builder] Create Part 3 assessment (Weeks 8-10 NVIDIA Isaac)
- [ ] T136 [assessment-architect] [assessment-builder] Create Part 4 assessment (Weeks 11-13 VLA & Humanoids)

**Checkpoint 5 Complete**: Assessments ready - 13 quizzes, 13 coding challenges, 4 module assessments with rubrics

---

## Category 6: Bonus Features (+200 Hackathon Points)

**Purpose**: Implement all 5 bonus features for maximum hackathon score

**Scoring Breakdown**:
- User Story 2 (RAG Chatbot): +50 points (ChatKit contextual selection)
- User Story 4 (BetterAuth): +50 points
- User Story 3 (Personalization): +50 points
- User Story 5 (Urdu Translation): +50 points
- **Total: +200 bonus points**

### 6.1 RAG Chatbot Backend (User Story 2 - P2) [+50 points]

**Responsible Subagent**: content-implementor (backend implementation)

#### 6.1.1 Content Ingestion (Qdrant Indexing)

- [ ] T137 [content-implementor] [rag-chatbot-integrator] Create `apps/api/scripts/ingest.py` to parse Docusaurus MDX files from `apps/docs/docs/` directory with frontmatter extraction (chapter_id, title, part_number, week_number)
- [ ] T138 [content-implementor] [rag-chatbot-integrator] Implement text chunking in `ingest.py` with 500-word chunks, 50-word overlap strategy to preserve context
- [ ] T139 [content-implementor] [rag-chatbot-integrator] Integrate Gemini `text-embedding-004` in `ingest.py` via `apps/api/src/utils/embeddings.py` to generate 768-dim vectors for each chunk
- [ ] T140 [content-implementor] [rag-chatbot-integrator] Implement Qdrant upsert in `ingest.py` to index chunks with payload: chapter_id, section_id, part_number, week_number, content_type, hardware_context
- [ ] T141 [content-implementor] [rag-chatbot-integrator] Run `python apps/api/scripts/ingest.py` to index all Docusaurus content and verify ~1000 chunks indexed successfully

#### 6.1.2 RAG Pipeline (FastAPI + Gemini)

- [ ] T142 [content-implementor] [rag-chatbot-integrator, mvp-builder] Create `apps/api/src/main.py` with FastAPI app initialization, CORS middleware for `http://localhost:3000` origin, and /api prefix routing
- [ ] T143 [content-implementor] [rag-chatbot-integrator] Create `apps/api/src/routers/__init__.py` as routers package initializer
- [ ] T144 [content-implementor] [rag-chatbot-integrator] Create `apps/api/src/routers/chat.py` with POST /api/chat endpoint accepting ChatRequest (query: str, hardware_context: Optional[str], selected_text: Optional[str])
- [ ] T145 [content-implementor] [rag-chatbot-integrator] Create `apps/api/src/services/__init__.py` as services package initializer
- [ ] T146 [content-implementor] [rag-chatbot-integrator, technical-clarity] Create `apps/api/src/services/rag_pipeline.py` with search_qdrant() function: embed query with `text-embedding-004`, search Qdrant with top_k=5, cosine similarity threshold 0.7
- [ ] T147 [content-implementor] [rag-chatbot-integrator] Implement chat completion in `rag_pipeline.py` using Gemini client from `gemini_client.py` with system prompt: "Answer using only provided book content. Always cite chapter sources. If user selected text, use it as context."
- [ ] T148 [content-implementor] [rag-chatbot-integrator] Implement citation extraction in `rag_pipeline.py` to parse chapter_id from Qdrant results and format as Citation objects (chapter_id, section_id, content_snippet, chapter_url)

#### 6.1.3 Input Sanitization (SOC Protocol)

- [ ] T149 [content-implementor] [technical-clarity] Create `apps/api/src/utils/sanitization.py` with sanitize_input() function: strip HTML tags, escape SQL special chars, detect XSS patterns, validate max length 500 chars
- [ ] T150 [content-implementor] [technical-clarity] Implement prompt injection detection in `sanitization.py`: regex patterns for "ignore previous", "system:", SQL keywords in chat context
- [ ] T151 [content-implementor] [technical-clarity] Integrate sanitization in `/api/chat` endpoint: sanitize query before processing, log sanitization events to AuditLog
- [ ] T152 [content-implementor] [technical-clarity] Add rate limiting middleware in `apps/api/src/main.py`: 20 requests per minute per IP address using slowapi library

### 6.2 RAG Chatbot Frontend (User Story 2 - ChatKit Integration) [+50 points]

**Responsible Subagent**: content-implementor (frontend implementation)

- [ ] T153 [content-implementor] [frontend-design, rag-chatbot-integrator] Create `apps/docs/src/components/ChatWidget/index.tsx` with React component: fixed bottom-right position, collapsible chat window, message history state
- [ ] T154 [content-implementor] [frontend-design] Implement chat input in ChatWidget with textarea, "Send" button, loading spinner during API call
- [ ] T155 [content-implementor] [frontend-design, rag-chatbot-integrator] Implement text selection handler in ChatWidget: detect when user selects text on page, show "Ask about this" button, pass selected_text to /api/chat
- [ ] T156 [content-implementor] [frontend-design, rag-chatbot-integrator] Implement API call in ChatWidget to POST /api/chat with query, hardware_context, selected_text, handle response (response_text, citations array)
- [ ] T157 [content-implementor] [frontend-design] Implement citations display in ChatWidget: render citations as clickable links to chapter anchors (e.g., `/docs/week-01-ros2-basics#nodes`)
- [ ] T158 [content-implementor] [frontend-design] Style ChatWidget with modern theme: dark background, accent colors, monospace font
- [ ] T159 [content-implementor] [frontend-design, docusaurus-deployer] Add ChatWidget to Docusaurus theme by swizzling `apps/docs/src/theme/Root.tsx` and importing ChatWidget component

**Bonus Feature 1 Complete**: RAG Chatbot with ChatKit contextual text selection → +50 points

### 6.3 Authentication & Onboarding (User Story 4 - P2) [+50 points]

**Responsible Subagent**: content-implementor (authentication implementation)

#### 6.3.1 Better-Auth Backend Setup

- [ ] T160 [content-implementor] [user-profile-initializer] Install better-auth-python in `apps/api/requirements.txt`
- [ ] T161 [content-implementor] [user-profile-initializer] Create `apps/api/src/routers/auth.py` with Better-Auth router initialization
- [ ] T162 [content-implementor] [user-profile-initializer, technical-clarity] Implement POST /api/auth/signup endpoint in `auth.py`: accept email + password, hash password with bcrypt, create User record with auth_provider='email'
- [ ] T163 [content-implementor] [user-profile-initializer] Implement POST /api/auth/signin endpoint in `auth.py`: validate email + password OR GitHub OAuth code, generate JWT token with 7-day expiration
- [ ] T164 [content-implementor] [user-profile-initializer] Configure GitHub OAuth in Better-Auth: client_id and client_secret from BETTER_AUTH_GITHUB_CLIENT_ID/SECRET env vars, callback URL `http://localhost:3000/api/auth/callback/github`
- [ ] T165 [content-implementor] [user-profile-initializer] Implement GitHub OAuth callback handler in `auth.py`: exchange code for access token, fetch GitHub user email, create/update User record with auth_provider='github'

#### 6.3.2 Onboarding Quiz

- [ ] T166 [content-implementor] [user-profile-initializer, session-intelligence-harvester] Implement POST /api/auth/onboarding endpoint in `auth.py`: accept hardware_profile ('rtx_4090' | 'jetson_orin_nano'), programming_language ('python' | 'cpp')
- [ ] T167 [content-implementor] [user-profile-initializer, technical-clarity] Add JWT authentication middleware in `apps/api/src/utils/auth_middleware.py`: verify JWT token from Authorization header, decode user_id
- [ ] T168 [content-implementor] [user-profile-initializer, session-intelligence-harvester] Update User record in /api/auth/onboarding: set hardware_profile and programming_language fields, update last_login timestamp
- [ ] T169 [content-implementor] [user-profile-initializer, technical-clarity] Add token expiration handling in auth_middleware.py: return 401 if token expired, log expiration event to AuditLog

#### 6.3.3 Authentication Frontend (Docusaurus)

- [ ] T170 [content-implementor] [frontend-design, user-profile-initializer] Install @better-auth/react in `apps/docs/package.json`
- [ ] T171 [content-implementor] [frontend-design, user-profile-initializer] Create `apps/docs/src/components/AuthProvider/index.tsx` with Better-Auth React context provider, configure API URL from REACT_APP_API_URL env var
- [ ] T172 [content-implementor] [frontend-design, docusaurus-deployer] Wrap Docusaurus app with AuthProvider in `apps/docs/src/theme/Root.tsx`
- [ ] T173 [content-implementor] [frontend-design, user-profile-initializer] Create `apps/docs/src/pages/login.tsx` with Better-Auth login UI: email/password form, "Sign in with GitHub" button
- [ ] T174 [content-implementor] [frontend-design, user-profile-initializer, session-intelligence-harvester] Create `apps/docs/src/pages/onboarding.tsx` with onboarding quiz form: "Do you own an NVIDIA RTX GPU?" radio buttons (Yes/No), "Are you a Python or C++ developer?" radio buttons (Python/C++)
- [ ] T175 [content-implementor] [frontend-design, user-profile-initializer] Implement onboarding form submission in onboarding.tsx: POST to /api/auth/onboarding, redirect to homepage on success
- [ ] T176 [content-implementor] [frontend-design, docusaurus-deployer] Add "Sign In" button to Docusaurus navbar in `docusaurus.config.js`, link to /login page
- [ ] T177 [content-implementor] [frontend-design] Add user profile display in navbar for authenticated users: show email, "Sign Out" button

**Bonus Feature 2 Complete**: BetterAuth with Email/GitHub OAuth and onboarding quiz → +50 points

### 6.4 Personalization Engine (User Story 3 - P3) [+50 points]

**Responsible Subagent**: content-implementor (personalization implementation)

#### 6.4.1 Personalization Backend

- [ ] T178 [content-implementor] [personalization-engine, session-intelligence-harvester] Create `apps/api/src/routers/personalize.py` with POST /api/personalize endpoint accepting PersonalizeRequest (chapter_id: str)
- [ ] T179 [content-implementor] [personalization-engine, user-profile-initializer] Add JWT authentication to /api/personalize endpoint using auth_middleware.py
- [ ] T180 [content-implementor] [personalization-engine, session-intelligence-harvester, technical-clarity] Create `apps/api/src/services/personalize.py` with personalize_chapter() function: fetch User hardware_profile, check PersonalizedContent cache (7-day TTL)
- [ ] T181 [content-implementor] [personalization-engine] Implement cache lookup in `personalize.py`: query PersonalizedContent table WHERE user_id = ? AND chapter_id = ? AND created_at > NOW() - INTERVAL '7 days'
- [ ] T182 [content-implementor] [personalization-engine] Implement content fetching in `personalize.py`: read original MDX from `apps/docs/docs/{chapter_id}/index.md`
- [ ] T183 [content-implementor] [personalization-engine, code-example-generator, technical-clarity] Implement Gemini personalization in `personalize.py`: use gemini_client.py with prompt template: "Rewrite code blocks ONLY for {hardware_profile}. If 'jetson_orin_nano': reduce memory, use CPU. If 'rtx_4090': enable GPU acceleration. Preserve prose and Mermaid."
- [ ] T184 [content-implementor] [personalization-engine] Implement cache storage in `personalize.py`: insert PersonalizedContent record with personalized_mdx, return cached content on subsequent requests
- [ ] T185 [content-implementor] [personalization-engine, technical-clarity] Add audit logging in `/api/personalize` endpoint: insert AuditLog record with event_type='personalization', event_details JSONB (user_id, chapter_id, hardware_profile, timestamp)

#### 6.4.2 Personalization Frontend

- [ ] T186 [content-implementor] [frontend-design, personalization-engine] Create `apps/docs/src/components/PersonalizeButton/index.tsx` with React component: "Personalize for Me" button, loading spinner, success/error toast notifications
- [ ] T187 [content-implementor] [frontend-design, user-profile-initializer] Implement authentication check in PersonalizeButton: only show button if user is authenticated (via Better-Auth context)
- [ ] T188 [content-implementor] [frontend-design, personalization-engine] Implement API call in PersonalizeButton: POST to /api/personalize with chapter_id from current route, handle response (personalized_mdx, cache_hit)
- [ ] T189 [content-implementor] [frontend-design] Implement content replacement in PersonalizeButton: replace current chapter MDX with personalized_mdx in DOM, show "Personalized for {hardware_profile}" badge
- [ ] T190 [content-implementor] [frontend-design, docusaurus-deployer] Add PersonalizeButton to all chapter pages by swizzling Docusaurus MDXComponents in `apps/docs/src/theme/MDXComponents/index.tsx`

**Bonus Feature 3 Complete**: Personalization engine with hardware-aware content adaptation → +50 points

### 6.5 Localization Engine (User Story 5 - P4) [+50 points]

**Responsible Subagent**: content-implementor (localization implementation)

#### 6.5.1 Localization Backend

- [ ] T191 [content-implementor] [urdu-translator, session-intelligence-harvester] Create `apps/api/src/routers/translate.py` with POST /api/translate endpoint accepting TranslateRequest (chapter_id: str, target_language: 'roman_urdu' | 'formal_urdu')
- [ ] T192 [content-implementor] [urdu-translator, user-profile-initializer] Add JWT authentication to /api/translate endpoint using auth_middleware.py
- [ ] T193 [content-implementor] [urdu-translator, technical-clarity] Create `apps/api/src/services/translate.py` with translate_chapter() function: check TranslatedContent cache (7-day TTL)
- [ ] T194 [content-implementor] [urdu-translator] Implement cache lookup in `translate.py`: query TranslatedContent table WHERE user_id = ? AND chapter_id = ? AND target_language = ? AND created_at > NOW() - INTERVAL '7 days'
- [ ] T195 [content-implementor] [urdu-translator, code-example-generator] Implement content fetching in `translate.py`: read original MDX from `apps/docs/docs/{chapter_id}/index.md`, parse code blocks with regex to preserve
- [ ] T196 [content-implementor] [urdu-translator, technical-clarity] Implement Gemini translation in `translate.py`: use gemini_client.py with prompt template: "Translate prose to {target_language}. DO NOT translate code blocks, Mermaid syntax, or technical identifiers (ROS 2, URDF, Isaac Sim). Preserve formatting."
- [ ] T197 [content-implementor] [urdu-translator, code-example-generator] Implement code block preservation in `translate.py`: replace code blocks with placeholders before translation, restore after translation to ensure English code
- [ ] T198 [content-implementor] [urdu-translator] Implement cache storage in `translate.py`: insert TranslatedContent record with translated_mdx
- [ ] T199 [content-implementor] [urdu-translator, technical-clarity] Implement input sanitization in `/api/translate` endpoint: use `sanitization.py` to prevent XSS in chapter_id parameter

#### 6.5.2 Localization Frontend

- [ ] T200 [content-implementor] [frontend-design, urdu-translator] Create `apps/docs/src/components/TranslateButton/index.tsx` with React component: "Translate to Urdu" dropdown (Roman Urdu | Formal Urdu), loading spinner
- [ ] T201 [content-implementor] [frontend-design, user-profile-initializer] Implement authentication check in TranslateButton: only show button if user is authenticated
- [ ] T202 [content-implementor] [frontend-design, urdu-translator] Implement API call in TranslateButton: POST to /api/translate with chapter_id and target_language, handle response (translated_mdx, cache_hit)
- [ ] T203 [content-implementor] [frontend-design, urdu-translator, code-example-generator] Implement content replacement in TranslateButton: replace chapter prose with translated_mdx, verify code blocks remain in English, show "Translated to {language}" badge
- [ ] T204 [content-implementor] [frontend-design, docusaurus-deployer] Add TranslateButton to all chapter pages alongside PersonalizeButton in MDXComponents

**Bonus Feature 4 Complete**: Localization engine with Urdu translation and code preservation → +50 points

### 6.6 ROS2 Text-to-Code Generator (User Story 6 - P5) [Additional Bonus]

**Responsible Subagent**: content-implementor (ROS2 code generation)

- [ ] T205 [content-implementor] [ros2-code-generator, code-example-generator] Create `apps/api/src/routers/ros2_codegen.py` with POST /api/ros2/generate endpoint accepting CodegenRequest (natural_language_command: str, hardware_profile: Optional[str])
- [ ] T206 [content-implementor] [ros2-code-generator, code-example-generator, technical-clarity] Create `apps/api/src/services/ros2_codegen.py` with generate_ros2_code() function using Gemini 2.5 Flash with prompt: "Generate Python ROS2 code for: {command}. Include imports (rclpy, geometry_msgs, moveit_commander), node initialization, action client, error handling."
- [ ] T207 [content-implementor] [ros2-code-generator, personalization-engine] Integrate hardware_profile in code generation: if 'jetson_orin_nano', add edge optimization comments and resource constraints
- [ ] T208 [content-implementor] [frontend-design, ros2-code-generator] Create `apps/docs/src/components/ROS2Playground/index.tsx` with React component: text input for natural language command, "Generate Code" button, code output display with syntax highlighting
- [ ] T209 [content-implementor] [frontend-design, ros2-code-generator] Implement API call in ROS2Playground: POST to /api/ros2/generate, display generated code with disclaimer: "AI-generated code. Test in simulation before real robot deployment."
- [ ] T210 [content-implementor] [frontend-design, docusaurus-deployer] Add ROS2Playground page at `apps/docs/src/pages/ros2-playground.tsx` and link from navbar

**Bonus Feature 5 Complete**: ROS2 text-to-code generator → Additional bonus points

**Checkpoint 6 Complete**: All 5 bonus features implemented → +200 points (Total: 300+/100 points)

---

## Category 7: Deployment & Production Readiness

**Purpose**: Deploy to production, configure CI/CD, implement monitoring and security

**Responsible Subagent**: super-orchestrator (deployment coordination)

**MCP Integration**: Vercel MCP (deployment), GitHub MCP (version control), Playwright MCP (verification)

### 7.1 Health & Monitoring

- [ ] T211 [super-orchestrator] [technical-clarity] Create `apps/api/src/routers/health.py` with GET /api/health endpoint: check Neon connection, Qdrant connection, Gemini API availability, Better-Auth operational status
- [ ] T212 [super-orchestrator] [technical-clarity] Implement health check responses in `health.py`: return JSON with service statuses (neon_db, qdrant_cloud, gemini_api, better_auth) and HTTP 200 if healthy, 503 if unhealthy
- [ ] T213 [super-orchestrator] [technical-clarity] Add structured error handling middleware in `apps/api/src/main.py`: catch all exceptions, log to AuditLog, return standardized error JSON (error, detail, timestamp)

### 7.2 Security & Compliance

- [ ] T214 [super-orchestrator] [technical-clarity] Implement CSRF protection in `apps/api/src/main.py`: add CSRFMiddleware from fastapi-csrf for all POST/PUT/DELETE endpoints
- [ ] T215 [super-orchestrator] [technical-clarity] Add rate limiting to all API endpoints: 100 requests per minute per IP for authenticated endpoints, 20 for anonymous
- [ ] T216 [super-orchestrator] [validation-auditor] Verify zero hardcoded secrets: audit all files in `apps/api/src/` for API keys, ensure all secrets loaded from environment variables
- [ ] T217 [super-orchestrator] [technical-clarity, validation-auditor] Add AuditLog entries for security events: failed auth attempts (401 responses), sanitized malicious inputs, rate limit violations

### 7.3 Documentation

- [ ] T218 [super-orchestrator] [technical-clarity] Create `apps/api/README.md` with API documentation: endpoint list, authentication flow, environment variables reference, local development setup linking to quickstart.md
- [ ] T219 [super-orchestrator] [frontend-design] Create `apps/docs/README.md` with frontend documentation: component structure, styling conventions, deployment instructions
- [ ] T220 [super-orchestrator] Update root `README.md` with project overview: features list, tech stack summary, hackathon scoring breakdown (Base 100 + 200 bonuses = 300 total), quickstart link

### 7.4 Deployment Configuration

- [ ] T221 [super-orchestrator] [docusaurus-deployer] Create `apps/api/Dockerfile` with Python 3.11 base image, install dependencies from requirements.txt, expose port 8000, CMD to run uvicorn
- [ ] T222 [super-orchestrator] [docusaurus-deployer] Create `docker-compose.yml` with services for `docs` (Docusaurus), `api` (FastAPI), environment variable configuration for local development

### 7.5 GitHub Integration (GitHub MCP)

**MCP Integration**: GitHub MCP for version control and deployment workflows

- [ ] T223 [super-orchestrator] [GitHub MCP] Use GitHub MCP to create GitHub repository: `mcp__github__create_repository` with name "physical-ai-textbook-platform", description "AI-Native Physical AI & Humanoid Robotics Textbook Platform", private=false
- [ ] T224 [super-orchestrator] [GitHub MCP, docusaurus-deployer] Use GitHub MCP to create `.github/workflows/deploy-docs.yml` via `mcp__github__create_or_update_file` with GitHub Actions workflow: build Docusaurus, deploy to GitHub Pages on push to main branch
- [ ] T225 [super-orchestrator] [GitHub MCP] Use GitHub MCP to create `.github/workflows/test-api.yml` via `mcp__github__create_or_update_file` with GitHub Actions workflow: run pytest on push (if tests exist), lint with flake8
- [ ] T226 [super-orchestrator] [GitHub MCP] Use GitHub MCP to push all project files: `mcp__github__push_files` with branch "main", commit message "Initial commit: Physical AI Textbook Platform with 13-week curriculum, RAG chatbot, personalization, and Urdu translation"

### 7.6 Vercel Deployment (Vercel MCP)

**MCP Integration**: Vercel MCP for frontend/backend deployment

- [ ] T227 [super-orchestrator] [Vercel MCP, docusaurus-deployer] Use Vercel MCP to deploy frontend: `mcp__vercel__deploy_to_vercel` for `apps/docs` with build command `npm run build`, output directory `build`
- [ ] T228 [super-orchestrator] [Vercel MCP] Use Vercel MCP to deploy backend (if supported): `mcp__vercel__deploy_to_vercel` for `apps/api` with Python runtime, FastAPI entry point
- [ ] T229 [super-orchestrator] [Vercel MCP] Use Vercel MCP to configure environment variables: GEMINI_API_KEY, NEON_CONNECTION_STRING, QDRANT_URL, QDRANT_API_KEY, BETTER_AUTH_GITHUB_CLIENT_ID, BETTER_AUTH_GITHUB_CLIENT_SECRET, JWT_SECRET via Vercel dashboard or CLI

### 7.7 Deployment Verification (Playwright MCP)

**MCP Integration**: Playwright MCP for deployed site verification

- [ ] T230 [validation-auditor] [Playwright MCP, playwright-test-runner] Use Playwright MCP to navigate to deployed Vercel site: `mcp__playwright__browser_navigate` to production URL
- [ ] T231 [validation-auditor] [Playwright MCP, playwright-test-runner] Use Playwright MCP to verify homepage loads: `mcp__playwright__browser_snapshot` and check for "Physical AI & Humanoid Robotics Textbook" title
- [ ] T232 [validation-auditor] [Playwright MCP, playwright-test-runner] Use Playwright MCP to test chat widget: `mcp__playwright__browser_click` on chat button, type query, verify response with citations
- [ ] T233 [validation-auditor] [Playwright MCP, playwright-test-runner] Use Playwright MCP to test authentication: `mcp__playwright__browser_navigate` to /login, `mcp__playwright__browser_fill_form` with credentials, verify redirect
- [ ] T234 [validation-auditor] [Playwright MCP, playwright-test-runner] Use Playwright MCP to test personalization: `mcp__playwright__browser_click` on "Personalize for Me" button, verify content updates
- [ ] T235 [validation-auditor] [Playwright MCP, playwright-test-runner] Use Playwright MCP to test translation: `mcp__playwright__browser_click` on "Translate to Urdu", verify Urdu text with English code blocks
- [ ] T236 [validation-auditor] [Playwright MCP, playwright-test-runner] Use Playwright MCP to generate deployment verification report: `mcp__playwright__browser_take_screenshot` of all major features

### 7.8 Final Verification & Constitutional Compliance

- [ ] T237 [super-orchestrator] [validation-auditor] Verify all Constitution checks pass: run constitution validation script if available, manually verify Article I-XI compliance per `specs/001-ai-textbook-platform/plan.md` Constitution Check section
- [ ] T238 [super-orchestrator] [validation-auditor] Verify all functional requirements implemented: audit FR-001 through FR-044 against codebase, check off each requirement in spec.md
- [ ] T239 [super-orchestrator] [validation-auditor] Verify all 7 user stories complete: test independent acceptance criteria for Stories 1-7
- [ ] T240 [super-orchestrator] [validation-auditor] Verify all 9 subagents created: check `.claude/agents/` directory for 9 agent definition files
- [ ] T241 [super-orchestrator] [validation-auditor] Verify all 28 skills created: check `.claude/skills/` directory for 23 canonical + 5 bonus skill files
- [ ] T242 [super-orchestrator] [validation-auditor] Run end-to-end manual test: Navigate textbook → Ask chatbot question → Select text and ask → Personalize chapter → Translate chapter → Generate ROS2 code → Verify all features work together
- [ ] T243 [super-orchestrator] [validation-auditor] Generate final hackathon submission report: document all implemented features, bonus points earned, deployed URLs, GitHub repository link

**Checkpoint 7 Complete**: Deployment successful - Frontend/backend deployed to Vercel, GitHub CI/CD configured, all features verified via Playwright MCP → Production ready for hackathon submission

---

## Dependencies & Execution Strategy

### Critical Path (For MVP - 100/100 Points)

1. **Category 1**: Planning & Infrastructure Setup (T001-T064) - **BLOCKING ALL**
2. **Category 2**: Specification & Design (T065-T076) - **BLOCKING Category 3**
3. **Category 3**: Content Authoring (T077-T101) - **MVP COMPLETE** → 100/100 points
4. **Category 4**: Validation & Quality Assurance (T102-T122) - **ENHANCES MVP**
5. **Category 5**: Assessment & Interactive Features (T123-T136) - **ENHANCES MVP**

### Bonus Features Path (+200 Points)

**Requirements**: Category 1 complete, Category 3 content exists

6. **Category 6.2**: Authentication (T160-T177) - **+50 points** - **BLOCKS 6.4 and 6.5**
7. **Category 6.1**: RAG Chatbot (T137-T159) - **+50 points** - Can run parallel with 6.2
8. **Category 6.4**: Personalization (T178-T190) - **+50 points** - Requires 6.2
9. **Category 6.5**: Localization (T191-T204) - **+50 points** - Requires 6.2
10. **Category 6.6**: ROS2 Text-to-Code (T205-T210) - **Additional bonus** - Can run parallel with others

### Deployment Path

11. **Category 7**: Deployment (T211-T243) - **FINAL** - Requires all categories complete

### Parallel Execution Opportunities

**Category 1 (Planning & Infrastructure)**:
- T004-T006 (shared-types, .gitignore, README) - 3 tasks
- T011-T019 (9 agent definitions) - 9 tasks
- T020-T047 (28 skill definitions) - 28 tasks
- T050-T054 (5 SQLAlchemy models) - 5 tasks
- **Total: 45 parallelizable tasks**

**Category 3 (Content Authoring)**:
- T082-T094 (13 week MDX files) - 13 tasks
- **Total: 13 parallelizable tasks**

**Category 5 (Assessment)**:
- T123-T127 (5 quiz sets) - 5 tasks
- T128-T132 (5 coding challenge sets) - 5 tasks
- **Total: 10 parallelizable tasks**

**Overall**: ~68 tasks parallelizable (28% of 243 total tasks)

### Incremental Delivery Strategy

**Phase 1: MVP Foundation (100/100 points)**
- Complete Categories 1-3: Planning → Specification → Content Authoring
- Deliverable: Working Docusaurus textbook with 13 weeks, Mermaid diagrams, code tabs
- **Score: 100/100 base points**

**Phase 2: Quality & Assessments**
- Complete Categories 4-5: Validation → Assessments
- Deliverable: Validated, tested textbook with quizzes and coding challenges
- **Score: 100/100 (enhanced quality)**

**Phase 3: Bonus Features Tier 1 (+100 points)**
- Complete Category 6.1-6.2: RAG Chatbot + Authentication
- Deliverable: Interactive chatbot with contextual selection, user authentication
- **Score: 200/100 (+100 bonus)**

**Phase 4: Bonus Features Tier 2 (+100 points)**
- Complete Category 6.4-6.5: Personalization + Localization
- Deliverable: Hardware-aware content, Urdu translation
- **Score: 300/100 (+200 bonus total)**

**Phase 5: Additional Bonus & Deployment**
- Complete Category 6.6 + Category 7: ROS2 Codegen + Deployment
- Deliverable: Full production platform with all features
- **Score: 300+/100 (maximum score)**

---

## Notes

**Gemini Configuration Reminders**:
- Always use `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"` in OpenAI client
- Use `GEMINI_API_KEY` environment variable (NOT `OPENAI_API_KEY`)
- Model string: `gemini-2.5-flash` for chat completions
- Embedding model: `text-embedding-004` (768-dimensional vectors, NOT 1536-dim)
- Do NOT use Google generativeai SDK - OpenAI SDK only for compatibility with openai-agents-sdk

**Constitution Compliance Checklist**:
- Article I (SDD): spec.md and plan.md in place ✅
- Article II (Tech Stack): All technologies authorized ✅
- Article III (MCP Mandate): Context7 (T102-T106), GitHub MCP (T223-T226), Playwright MCP (T118-T122, T230-T236), Vercel MCP (T227-T229) ✅
- Article IV (Agent Behavior): Language protocol (English code/docs, Roman Urdu chat) ✅
- Article V (Publishing): Docusaurus conventions, Mermaid diagrams ✅
- Article VI (Engineering): Zero hardcoded secrets (T216), input sanitization (T149-T151, T199), RAG citations (T148) ✅
- Article VII (Intelligence): PHR creation, ADR planning, folder integrity ✅
- Article VIII (Agentic Orchestration): All 9 subagents created (T011-T019) ✅
- Article IX (Skill System): All 28 skills created (T020-T047) ✅
- Article X (Bonus Features): All 5 bonus features implemented (Category 6) ✅
- Article XI (Deployment): Vercel/GitHub/Playwright MCP documented (Category 7) ✅

**Testing Strategy**:
- Manual testing for all user stories (no automated tests per spec.md)
- Each user story has independent test criteria in category headers
- End-to-end verification in Category 7 (T242)
- Playwright MCP for UI testing (T118-T122, T230-T236)

**Success Criteria**:
- MVP (Categories 1-3): Working textbook → 100/100 points
- All Categories (1-7): Full platform with 5 bonuses → 300/100 points
- Category 1.3-1.4 critical for Constitutional compliance (9 agents + 28 skills)
- Category 6 critical for bonus points (+200 total)

**Estimated Task Counts**:
- **Category 1 (Planning)**: 64 tasks
- **Category 2 (Specification)**: 12 tasks
- **Category 3 (Content Authoring)**: 25 tasks
- **Category 4 (Validation)**: 21 tasks
- **Category 5 (Assessment)**: 14 tasks
- **Category 6 (Bonus Features)**: 74 tasks
- **Category 7 (Deployment)**: 33 tasks

**Total Tasks**: 243 tasks

**Parallel Opportunities**: ~68 tasks can run in parallel (28% of total)

---

**END OF TASKS**
