# Feature Specification: Physical AI & Humanoid Robotics AI-Native Textbook Platform

**Feature Branch**: `001-ai-textbook-platform`
**Created**: 2025-12-04
**Updated**: 2025-12-09 (Constitution v5.0.0 Alignment)
**Status**: Active Development
**Input**: User description: "Generate a feature specification for the complete AI-native technical book titled 'Physical AI & Humanoid Robotics' that aligns with Panaversity hackathon goals, covering AI-native textbook creation with Docusaurus, 13-week syllabus breakdown, Claude subagents/skills for auto-generation and validation, personalized content flow via BetterAuth, chapter-level toggles (personalization/Urdu), embedded RAG chatbot (Qdrant/ChatKit/FastAPI), Vercel MCP deployment, GitHub MCP version control, Playwright MCP UI testing, Context7 MCP for documentation assistance, and ROS2-based code examples with text-to-code simulation for robotics."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Browse Structured AI-Native Course Content (Priority: P1)

A learner visits the platform to study Physical AI & Humanoid Robotics in a well-organized, AI-native format with visual aids, interactive code examples, and comprehensive 13-week curriculum.

**Why this priority**: Core educational content is the foundation of the entire platform. Without this, there's no textbook to build upon. This delivers immediate value as a standalone educational resource and enables all subsequent features. Represents base 100/100 hackathon points.

**Independent Test**: Can be fully tested by navigating through all 4 course parts (13 weeks), viewing Mermaid diagrams for ROS graphs, switching between "Simulated" and "Real Robot" code tabs, and verifying chapter-level learning objectives. Delivers complete educational content without requiring any interactive features.

**Acceptance Scenarios**:

1. **Given** learner opens the textbook homepage, **When** they view the table of contents, **Then** they see 4 clearly organized parts covering 13 weeks: Part 1 "The Nervous System" (Weeks 1-5 ROS 2), Part 2 "The Digital Twin" (Weeks 6-7 Gazebo/Unity), Part 3 "The Brain" (Weeks 8-10 NVIDIA Isaac), Part 4 "VLA & Humanoids" (Weeks 11-13)
2. **Given** learner navigates to any ROS 2 chapter, **When** they scroll through content, **Then** they see Mermaid.js diagrams illustrating ROS graphs, node communications, and system architecture
3. **Given** learner reads a code example, **When** they interact with the code block, **Then** they can switch between "Simulated" (Gazebo/Isaac) and "Real Robot" (Jetson deployment) implementations using Docusaurus tabs
4. **Given** learner views any chapter, **When** they check learning objectives, **Then** they see clear, measurable outcomes aligned with pedagogical design principles (Constitution Article VIII, pedagogical-designer agent)
5. **Given** learner accesses Hardware Context section, **When** they read Lab Setup, **Then** they see detailed comparison between Digital Twin Rig (RTX 4090) and Edge Kit (Jetson Orin Nano) with specs, use cases, and code optimization strategies

---

### User Story 2 - Query Course Content via RAG Chatbot with Contextual Selection (Priority: P2)

A learner has specific questions about Physical AI topics and wants instant, contextual answers from the textbook, including the ability to select text and ask questions with that context.

**Why this priority**: Enhances learning experience significantly but requires Story 1 content to exist first. Implements bonus feature (+50 points) for ChatKit integration with contextual text selection. Can be added after core content is ready.

**Independent Test**: Can be tested by selecting text from any chapter, clicking ChatKit interface, asking questions, and verifying answers cite specific book chapters with selected text as context. Works independently if content from Story 1 exists.

**Acceptance Scenarios**:

1. **Given** learner has a question about ROS 2 topics, **When** they type query in the ChatKit interface, **Then** the RAG system returns relevant answers with citations from indexed Docusaurus chapters using Qdrant vector search
2. **Given** learner selects text from a chapter about "Jetson Nano optimization", **When** they click ChatKit button and ask "How does this apply to my RTX 4090 setup?", **Then** system uses selected text as context and provides hardware-specific comparison
3. **Given** learner asks complex multi-part question, **When** RAG processes query using OpenAI Agents SDK with Google Gemini 2.5 Flash, **Then** answer synthesizes information from multiple chapters with proper source attribution
4. **Given** learner submits malicious input with SQL injection or XSS patterns, **When** RAG system processes it, **Then** input is sanitized and logged, preventing any security breach (Constitution Article VI, SOC Protocol)
5. **Given** RAG chatbot responds, **When** learner reviews answer, **Then** system displays "Cited Sources" section with clickable chapter links for verification

---

### User Story 3 - Personalize Content for Hardware Setup via AI Generation (Priority: P3)

A learner wants content optimized for their specific hardware (RTX 4090 for simulation vs Jetson Orin Nano for edge deployment) with AI-generated code adaptations.

**Why this priority**: Significant value-add (+50 bonus points) but not essential for learning. Requires authentication system (Story 4) and base content (Story 1). Leverages session-intelligence-harvester skill (Constitution Article IX).

**Independent Test**: Can be tested by signing in, setting hardware preference to "Jetson Orin Nano", clicking "Personalize for Me" button, and verifying code examples are rewritten for edge optimization. Works independently after Stories 1 and 4 are complete.

**Acceptance Scenarios**:

1. **Given** authenticated learner with "Jetson Orin Nano" hardware preference, **When** they click "Personalize for Me" on any chapter, **Then** code blocks are rewritten using OpenAI Agents SDK to optimize for edge constraints (lower memory, CPU-focused processing, power efficiency)
2. **Given** authenticated learner with "RTX 4090" hardware preference, **When** they personalize a simulation chapter, **Then** code shows high-fidelity rendering settings, GPU-accelerated physics, and ray-tracing optimizations
3. **Given** learner personalizes content, **When** personalization completes, **Then** request is logged in audit trail with user_id, timestamp, chapter_id, hardware_profile, and generation_time
4. **Given** learner has previously personalized a chapter, **When** they revisit it within cache expiry period (7 days), **Then** cached personalized version loads instantly from Neon database
5. **Given** personalization uses AI generation, **When** code is modified, **Then** system displays disclaimer: "AI-adapted for your hardware. Verify before production use."

---

### User Story 4 - Authenticate and Configure Learning Profile with Onboarding (Priority: P2)

A learner creates an account via BetterAuth (Email or GitHub OAuth) to unlock personalized features, complete onboarding quiz, and configure learning preferences.

**Why this priority**: Required for personalization (Story 3), localization (Story 5), and earns +50 bonus points for BetterAuth integration. Must come before personalization features but not needed for basic content browsing.

**Independent Test**: Can be tested by signing up with email or GitHub OAuth, completing onboarding quiz capturing hardware preferences and programming language, and verifying profile is saved to Neon database. Works independently and enables future personalization.

**Acceptance Scenarios**:

1. **Given** new visitor on the platform, **When** they click "Sign In" button, **Then** they see BetterAuth authentication options for Email (with verification) and GitHub OAuth
2. **Given** user completes email/GitHub authentication, **When** they proceed to onboarding, **Then** they are asked: (1) "What hardware do you have?" [RTX 4090 / Jetson Orin Nano / Other], (2) "What is your programming background?" [Python / C++ / Both / Beginner]
3. **Given** user answers onboarding questions, **When** they submit responses, **Then** profile (user_id, email, auth_provider, hardware_type, programming_language, experience_level, created_at) is stored in Neon PostgreSQL
4. **Given** authenticated user, **When** they access personalization or translation features, **Then** system loads their profile and applies preferences automatically
5. **Given** authentication token, **When** configured timeout period expires, **Then** token becomes invalid and user must re-authenticate for security

---

### User Story 5 - Access Content in Urdu Language with Code Preservation (Priority: P4)

A learner prefers to read technical content in Urdu (Roman or Formal) while keeping code examples, technical terms, and Mermaid diagrams in English.

**Why this priority**: Valuable for accessibility (+50 bonus points) but serves smaller audience. Can be added last as enhancement using urdu-translator skill (Constitution Article IX). Requires authentication (Story 4) and base content (Story 1).

**Independent Test**: Can be tested by clicking "Translate to Urdu" button, selecting Roman/Formal Urdu preference, and verifying technical prose is translated while code blocks, Mermaid syntax, and API references remain in English. Works independently after Stories 1 and 4 exist.

**Acceptance Scenarios**:

1. **Given** authenticated learner viewing any chapter, **When** they click "Translate to Urdu" button and select preference, **Then** technical prose is translated to Roman Urdu or Formal Urdu using OpenAI Agents SDK while code blocks remain untouched
2. **Given** chapter is being translated, **When** translation service processes content, **Then** Mermaid diagram labels, code blocks, technical identifiers (ROS 2, NVIDIA, Isaac), and API function names remain in English
3. **Given** translation is generated, **When** it completes, **Then** translated version is cached in Neon database with translation_id, target_language, cache_expiry for future requests to avoid redundant API calls
4. **Given** translated chapter displays, **When** learner views code examples, **Then** inline code comments are translated but syntax, function names, and imports remain English
5. **Given** malicious user submits XSS payload via translation input, **When** system processes it, **Then** input is sanitized per SOC Protocol and XSS attempt is logged to audit_logs table

---

### User Story 6 - Generate ROS2 Robot Commands from Natural Language (Priority: P5 - Bonus)

A learner wants to experiment with robot commands by describing actions in natural language and receiving generated ROS2 Python code snippets.

**Why this priority**: Bonus feature (+bonus points) that enhances learning through interactive code generation. Uses ros2-code-generator skill (Constitution Article IX). Not required for base or standard bonus points.

**Independent Test**: Can be tested by entering natural language command like "Pick up the red cube using MoveIt2" and verifying system generates valid ROS2 Python code with proper imports, node structure, and action clients. Works independently with ROS2 knowledge base.

**Acceptance Scenarios**:

1. **Given** learner accesses ROS2 code playground, **When** they enter natural language command "Pick up the red cube", **Then** system generates Python ROS2 code using OpenAI Agents SDK with ros2-code-generator skill
2. **Given** generated code, **When** learner reviews output, **Then** code includes proper imports (rclpy, geometry_msgs, moveit_commander), node initialization, action client setup, and error handling
3. **Given** learner wants to modify hardware, **When** they specify "using Jetson Orin Nano", **Then** generated code includes edge-optimized settings and resource constraints
4. **Given** generated code snippet, **When** displayed to learner, **Then** system includes disclaimer: "AI-generated code. Test in simulation before real robot deployment."

---

### User Story 7 - Automated Content Generation and Validation via Claude Subagents (Priority: P0 - Foundation)

Content creation team (or AI orchestrator) generates and validates all 13 weeks of textbook content using 9 specialized Claude subagents defined in Constitution Article VIII.

**Why this priority**: Foundation priority (P0) - must happen before any user-facing features. This is the authoring/production workflow that creates the content consumed in Story 1. Not user-facing but critical for content quality and hackathon delivery.

**Independent Test**: Can be tested by invoking super-orchestrator agent with syllabus input, verifying chapter-planner divides into 13 weeks, content-implementor generates MDX files, factual-verifier validates using Context7 MCP, and validation-auditor confirms using Playwright MCP. Produces complete textbook content.

**Acceptance Scenarios**:

1. **Given** Physical AI syllabus input, **When** chapter-planner agent processes it, **Then** curriculum is divided into 4 parts covering 13 weeks with learning objectives per chapter
2. **Given** chapter breakdown, **When** pedagogical-designer agent processes chapters, **Then** each chapter includes clear learning outcomes, exercises, and assessment criteria aligned with educational standards
3. **Given** learning objectives, **When** content-implementor agent generates content, **Then** produces rich Docusaurus MDX files with Mermaid diagrams, code examples (tabs for Simulated/Real), and canonical markdown formatting
4. **Given** generated content, **When** factual-verifier agent validates, **Then** uses Context7 MCP to fetch official ROS 2, NVIDIA Isaac, Gazebo, Unity, Whisper documentation and verifies technical accuracy with source citations
5. **Given** complete chapters, **When** assessment-architect agent processes them, **Then** generates interactive quizzes, coding challenges, and module assessments using quiz-generator skill
6. **Given** all content ready, **When** validation-auditor agent runs checks, **Then** performs style/format validation via canonical-format-checker skill and UI testing via Playwright MCP, producing audit report
7. **Given** validation passes, **When** super-orchestrator agent completes, **Then** triggers deployment pipeline using Vercel MCP for frontend (Docusaurus) and backend (FastAPI), and GitHub MCP for version control commit
8. **Given** any agent execution, **When** work completes, **Then** creates Prompt History Record (PHR) in history/prompts/001-ai-textbook-platform/ per Constitution Article VII

---

### Edge Cases

- What happens when a user asks RAG chatbot about content not yet published in the textbook? (System should respond with "This topic is not yet covered in the textbook. Check back for future updates.")
- How does system handle concurrent personalization requests from same user across multiple chapters? (Queue requests, process sequentially, show "Personalization in progress" indicator)
- What happens if OpenAI API rate limits are exceeded during high-traffic periods for personalization/translation? (Fall back to cached generic content, display rate limit notice, queue request for later)
- How does platform handle users with both RTX 4090 AND Jetson Orin Nano (hybrid setups)? (Allow multi-hardware profile, default to RTX 4090, provide hardware selector per chapter)
- What happens when Qdrant vector database is temporarily unavailable? (RAG chatbot shows "Search temporarily unavailable, please try again later", logs outage)
- How does system handle chapter content updates after users have cached personalized versions? (Invalidate cache on content update, notify users of new version, regenerate personalization on next visit)
- What happens if BetterAuth GitHub OAuth credentials are invalid or GitHub API is down? (Fall back to email authentication only, display "GitHub sign-in temporarily unavailable")
- How does urdu-translator handle technical terms with no direct Urdu equivalent? (Keep original English term in parentheses after Urdu approximation, e.g., "رابوٹکس (Robotics)")
- What happens when Context7 MCP fails to fetch documentation during factual-verifier validation? (Skip external verification for that source, log warning, mark chapter with "Pending external validation" flag)
- How does Playwright MCP handle UI testing failures for deployed pages? (Generate screenshot of failure, log detailed error, notify validation-auditor agent, block deployment if critical failure)

## Requirements *(mandatory)*

### Functional Requirements

**Content Engine & Curriculum Structure (Story 1, 7):**

- **FR-001**: Platform MUST display textbook content organized into 4 parts covering 13 weeks of Physical AI & Humanoid Robotics curriculum aligned with provided syllabus
- **FR-002**: Platform MUST render Mermaid.js diagrams for all ROS 2 communication graphs, architecture visualizations, and system designs
- **FR-003**: Platform MUST provide Docusaurus tab components allowing users to switch between "Simulated" (Gazebo/Isaac) and "Real Robot" (Jetson deployment) code implementations
- **FR-004**: Each chapter MUST include learning objectives, code examples, exercises, and assessments generated by pedagogical-designer and assessment-architect agents (Constitution Article VIII)
- **FR-005**: Content MUST align with ROS 2, NVIDIA Isaac Sim, Gazebo, Unity, and Whisper technical standards validated using Context7 MCP (Constitution Article III)

**Claude Subagent Orchestration (Story 7):**

- **FR-006**: System MUST implement 9 specialized Claude subagents as defined in Constitution Article VIII: chapter-planner, spec-architect, pedagogical-designer, educational-validator, content-implementor, factual-verifier, assessment-architect, validation-auditor, super-orchestrator
- **FR-007**: chapter-planner agent MUST divide syllabus into 4 parts (13 weeks) with learning objectives using chapter-planner, book-scaffolding, learning-objectives skills
- **FR-008**: content-implementor agent MUST generate Docusaurus MDX files with canonical formatting using book-scaffolding, code-example-generator, image-generator, concept-scaffolding skills
- **FR-009**: factual-verifier agent MUST validate chapter accuracy using Context7 MCP to fetch official ROS 2, Isaac, Gazebo, Unity, Whisper documentation
- **FR-010**: validation-auditor agent MUST run style/format validation using canonical-format-checker skill and UI testing using Playwright MCP (Constitution Article III)
- **FR-011**: super-orchestrator agent MUST coordinate all subagents and trigger deployment via Vercel MCP (frontend/backend) and GitHub MCP (version control)

**RAG Chatbot with Contextual Selection (Story 2):**

- **FR-012**: System MUST index all Docusaurus chapter content into Qdrant Cloud vector database for semantic search
- **FR-013**: Chatbot MUST answer user queries using OpenAI Agents SDK configured with Google Gemini 2.5 Flash (Constitution Article II, model: gemini-2.5-flash)
- **FR-014**: ChatKit integration MUST allow users to select text from any chapter and ask questions with selected text as context
- **FR-015**: Chatbot responses MUST cite specific chapter sources with clickable links for verification
- **FR-016**: RAG backend MUST sanitize all user inputs to prevent SQL injection, XSS, and prompt injection attacks per SOC Protocol (Constitution Article VI)

**Authentication & Onboarding (Story 4):**

- **FR-017**: System MUST integrate BetterAuth supporting Email (with verification) and GitHub OAuth authentication methods (Constitution Article II)
- **FR-018**: System MUST present onboarding quiz asking: (1) "What hardware do you have?" [RTX 4090 / Jetson Orin Nano / Other], (2) "What is your programming background?" [Python / C++ / Both / Beginner]
- **FR-019**: System MUST persist user profile (user_id, email, auth_provider, hardware_type, programming_language, experience_level, created_at, last_login) in Neon PostgreSQL database
- **FR-020**: Authentication tokens MUST expire after configured timeout period and require re-authentication for security

**Personalization Engine (Story 3):**

- **FR-021**: Platform MUST display "Personalize for Me" button at the top of every chapter for authenticated users
- **FR-022**: System MUST rewrite code blocks using OpenAI Agents SDK based on user's hardware profile: optimize for edge constraints (Jetson Nano low memory/power) or high-fidelity simulation (RTX 4090 GPU acceleration)
- **FR-023**: Personalized content MUST be generated using session-intelligence-harvester skill (Constitution Article IX) and cached in Neon database with 7-day expiry
- **FR-024**: System MUST log all personalization requests with user_id, timestamp, chapter_id, hardware_profile, generation_time for audit trail
- **FR-025**: Personalized code MUST display disclaimer: "AI-adapted for your hardware. Verify before production use."

**Localization Engine (Story 5):**

- **FR-026**: Platform MUST display "Translate to Urdu" button on every chapter for authenticated users with language preference selector (Roman Urdu / Formal Urdu)
- **FR-027**: System MUST translate technical prose using urdu-translator skill (Constitution Article IX) while preserving code blocks, Mermaid syntax, technical identifiers, and API function names in English
- **FR-028**: Translated content MUST be cached in Neon database (translation_id, target_language, translated_mdx, cache_expiry) to avoid redundant API calls
- **FR-029**: Translation service MUST sanitize inputs to prevent XSS and injection attacks per SOC Protocol

**ROS2 Text-to-Code Generator (Story 6 - Bonus):**

- **FR-030**: Platform MUST provide ROS2 code playground where learners can enter natural language commands
- **FR-031**: System MUST generate Python ROS2 code using OpenAI Agents SDK with ros2-code-generator skill (Constitution Article IX)
- **FR-032**: Generated code MUST include proper imports (rclpy, geometry_msgs, moveit_commander), node initialization, action clients, and error handling
- **FR-033**: Generated code MUST display disclaimer: "AI-generated code. Test in simulation before real robot deployment."

**Deployment & Infrastructure (Story 7):**

- **FR-034**: Frontend (Docusaurus) MUST be deployed to Vercel using Vercel MCP with build command `cd apps/docs && npm run build` and output directory `apps/docs/build` (Constitution Article XI)
- **FR-035**: Backend (FastAPI RAG API) MUST be deployed to Vercel using Vercel MCP with Python runtime configuration and environment variables via Vercel MCP
- **FR-036**: All major commits MUST be pushed to GitHub using GitHub MCP for version control (Constitution Article III)
- **FR-037**: Playwright MCP MUST perform UI testing on deployed Docusaurus pages and chatbot interface, generating screenshots and audit reports (Constitution Article III)
- **FR-038**: Context7 MCP MUST be used to fetch Docusaurus, FastAPI, Neon, ROS 2, NVIDIA Isaac, Gazebo, Unity, Whisper documentation during setup and validation (Constitution Article III)

**Security & Compliance (SOC Protocol - Constitution Article VI):**

- **FR-039**: System MUST implement input sanitization for all 5 interactive features: Chat, Auth, Personalize, Translate, ROS2-Code-Gen
- **FR-040**: API keys and secrets MUST be loaded from environment variables (.env), never hardcoded in source code
- **FR-041**: System MUST log all security events (failed auth attempts, sanitized malicious inputs, rate limit violations) to audit_logs table in Neon
- **FR-042**: All user-facing forms MUST implement CSRF protection and rate limiting

**Intelligence Preservation (Constitution Article VII):**

- **FR-043**: Every subagent execution MUST create Prompt History Record (PHR) in history/prompts/001-ai-textbook-platform/ using .specify/scripts/bash/create-phr.sh
- **FR-044**: All architectural decisions during content generation MUST be recorded in ADRs in history/adr/ per Constitution Article I

### Key Entities

- **User**: Authenticated learner with profile attributes (user_id [UUID PK], email [VARCHAR UNIQUE], auth_provider [ENUM: email/github], hardware_type [ENUM: rtx4090/jetson/other], programming_language [ENUM: python/cpp/both/beginner], experience_level [ENUM: beginner/intermediate/advanced], created_at [TIMESTAMP], last_login [TIMESTAMP])

- **Chapter**: Textbook content unit (chapter_id [UUID PK], part_number [INT 1-4], week_number [INT 1-13], title [VARCHAR], content_mdx [TEXT], mermaid_diagrams [JSONB], code_blocks [JSONB with tabs], learning_objectives [TEXT[]], published_at [TIMESTAMP], updated_at [TIMESTAMP], validation_status [ENUM: draft/validated/published])

- **PersonalizedContent**: Cached personalized chapter version (personalization_id [UUID PK], user_id [UUID FK→User], chapter_id [UUID FK→Chapter], hardware_profile [ENUM], personalized_mdx [TEXT], generation_prompt [TEXT], generated_at [TIMESTAMP], cache_expiry [TIMESTAMP], generation_time_ms [INT])

- **TranslatedContent**: Cached translated chapter version (translation_id [UUID PK], user_id [UUID FK→User], chapter_id [UUID FK→Chapter], target_language [ENUM: roman_urdu/formal_urdu], translated_mdx [TEXT], generated_at [TIMESTAMP], cache_expiry [TIMESTAMP])

- **ChatQuery**: RAG interaction record (query_id [UUID PK], user_id [UUID FK→User NULL], query_text [TEXT], selected_context [TEXT NULL], response_text [TEXT], cited_chapters [UUID[] FK→Chapter], vector_search_time_ms [INT], llm_generation_time_ms [INT], created_at [TIMESTAMP])

- **AuditLog**: Security and compliance event log (log_id [UUID PK], event_type [ENUM: auth_failed/input_sanitized/rate_limit/xss_blocked], user_id [UUID FK→User NULL], ip_address [INET], action_details [JSONB], severity [ENUM: info/warning/critical], timestamp [TIMESTAMP])

- **ROS2CodeSnippet**: Generated ROS2 code from text-to-code feature (snippet_id [UUID PK], user_id [UUID FK→User NULL], natural_language_input [TEXT], generated_code [TEXT], hardware_context [ENUM: rtx4090/jetson/generic], generation_prompt [TEXT], generated_at [TIMESTAMP])

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Content Quality & Accessibility:**

- **SC-001**: Learners can navigate through all 13 weeks of course content (4 parts) with less than 3 clicks per chapter transition
- **SC-002**: 100% of ROS 2 chapters include at least one Mermaid.js diagram illustrating system architecture or node communication
- **SC-003**: Each of 13 weeks includes learning objectives, code examples with tabs (Simulated/Real), and exercises validated by educational-validator agent
- **SC-004**: All technical content accuracy is verified using Context7 MCP against official ROS 2, Isaac, Gazebo, Unity, Whisper documentation

**RAG Chatbot Performance:**

- **SC-005**: Chatbot responds to queries within 3 seconds (95th percentile) and cites at least 2 relevant chapter sources per response
- **SC-006**: ChatKit contextual selection feature allows users to select text and ask questions, with selected text properly included in query context
- **SC-007**: RAG system successfully sanitizes 95% of malicious inputs (SQL injection, XSS, prompt injection) before processing

**Personalization & Localization:**

- **SC-008**: Personalized content generation completes within 10 seconds for a typical chapter (2000-3000 words)
- **SC-009**: Personalized code shows measurable optimization: Jetson Nano code uses 40% less memory than RTX 4090 baseline, RTX 4090 code enables GPU acceleration
- **SC-010**: Translation service converts chapter content to Urdu within 8 seconds while preserving 100% of code blocks intact (no syntax corruption)
- **SC-011**: Cached personalized/translated content loads instantly (under 500ms) on subsequent visits within 7-day expiry window

**Authentication & User Experience:**

- **SC-012**: New users complete BetterAuth signup (email or GitHub OAuth) and onboarding quiz within 2 minutes
- **SC-013**: User profile data is persisted to Neon database with 100% success rate for all authentication methods

**ROS2 Text-to-Code Quality:**

- **SC-014**: Generated ROS2 code snippets are syntactically valid Python with proper imports, node initialization, and error handling in 90% of cases
- **SC-015**: Generated code adapts to hardware context, showing edge-optimized patterns for Jetson and GPU-accelerated patterns for RTX 4090

**Subagent Orchestration & Deployment:**

- **SC-016**: super-orchestrator agent successfully coordinates all 9 subagents to generate, validate, and deploy complete 13-week curriculum
- **SC-017**: validation-auditor agent identifies and reports style/format issues with 100% coverage using canonical-format-checker skill
- **SC-018**: Playwright MCP successfully tests deployed UI (Docusaurus pages, chatbot interface) and generates screenshots for all critical user paths
- **SC-019**: Vercel MCP deploys frontend (Docusaurus) and backend (FastAPI) with zero manual configuration using automated environment variable setup
- **SC-020**: GitHub MCP commits and pushes all generated content, achieving full version control history

**System Performance & Scalability:**

- **SC-021**: Platform supports 500 concurrent users without performance degradation (page load under 2 seconds, chat response under 3 seconds)
- **SC-022**: Qdrant vector search returns relevant chapters within 500ms for 95th percentile queries

**Hackathon Scoring Achievement:**

- **SC-023**: Platform achieves 300/100 points score breakdown:
  - Base (100 points): Story 1 (Content Engine) + Story 2 (RAG Chatbot)
  - Bonus 1 (+50 points): ChatKit contextual selection (Story 2 enhancement)
  - Bonus 2 (+50 points): BetterAuth integration (Story 4)
  - Bonus 3 (+50 points): Personalization Engine (Story 3)
  - Bonus 4 (+50 points): Localization Engine (Story 5)
  - Additional Bonus: ROS2 text-to-code (Story 6)

### Assumptions

1. **Content Source**: Physical AI & Humanoid Robotics course syllabus is available and covers all 13 weeks with topics: ROS 2 (Weeks 1-5), Gazebo/Unity Digital Twins (Weeks 6-7), NVIDIA Isaac (Weeks 8-10), VLA & Humanoids (Weeks 11-13)
2. **Hardware Profiles**: Only two primary hardware configurations (RTX 4090 for simulation, Jetson Orin Nano for edge deployment) - hybrid setups handled as edge case with multi-profile support
3. **Language Preference**: Urdu translation targets primarily Roman Urdu (Latin script) with Formal Urdu (Nastaliq) as secondary option - no other languages required in MVP
4. **Subagent Infrastructure**: Claude Code CLI environment supports creating 9 custom subagent definitions in .claude/agents/ and 28+ skill definitions in .claude/skills/
5. **MCP Server Availability**: Context7, GitHub, Playwright, Vercel MCP servers are installed, configured, and accessible per Constitution Article III
6. **Authentication**: Email verification for email-based signups is handled by BetterAuth's built-in workflow - no custom SMTP configuration needed
7. **AI Model Configuration**: OpenAI Agents SDK is configured with base_url pointing to Google Gemini 2.5 Flash API endpoint using model string "gemini-2.5-flash" (Constitution Article II)
8. **API Rate Limits**: OpenAI API (or Gemini proxy) has sufficient rate limits and quota for expected user base during hackathon evaluation period (500 concurrent users)
9. **Caching Strategy**: Personalized/translated content cached for 7 days - users rarely request re-personalization/re-translation within this window
10. **Deployment Platform**: Vercel supports both Node.js (Docusaurus frontend) and Python (FastAPI backend) deployments, with mono-repo or separate service configurations
11. **Security Baseline**: Platform operates in educational/evaluation context - production-grade DDoS protection and advanced threat detection deferred to post-hackathon phase
12. **Content Updates**: Chapter content updates are infrequent during hackathon evaluation - cache invalidation on update is acceptable trade-off

## Out of Scope *(optional)*

- **User Progress Tracking**: Chapter completion percentage, quiz scores, learning analytics, progress dashboards
- **Collaborative Features**: Comments, annotations, social learning features, discussion forums
- **Offline Mode**: Downloaded chapters, offline chatbot functionality, PWA (Progressive Web App) features
- **Mobile Native Apps**: iOS/Android native apps - MVP is web-only via responsive Docusaurus
- **Video Content**: Embedded tutorial videos, interactive 3D simulations beyond static Mermaid diagrams
- **Advanced Personalization**: Learning style adaptation, difficulty level adjustment, adaptive content sequencing, spaced repetition
- **Multi-Language Support**: Languages beyond Urdu (e.g., Arabic, Spanish, Chinese, French)
- **Real-Time Collaboration**: Live co-browsing, shared personalization sessions, collaborative note-taking
- **Payment/Monetization**: Subscription tiers, premium content, payment processing, course certificates
- **Advanced Analytics**: User behavior tracking, A/B testing, conversion funnels, engagement metrics
- **Integration with LMS**: Moodle, Canvas, Blackboard integration for grade sync or assignment submission
- **Gamification**: Badges, leaderboards, achievement systems, experience points
- **Advanced Security**: Two-factor authentication (2FA), biometric authentication, hardware security keys
- **Content Versioning**: Multiple versions of same chapter for different skill levels or learning paths
- **Export Features**: PDF export, EPUB generation, print-optimized layouts

## Dependencies *(optional)*

### External Services (MCP Servers per Constitution Article III)

- **Context7 MCP (The Librarian)**: Fetches latest Docusaurus v3, FastAPI, Neon, ROS 2, NVIDIA Isaac, Gazebo, Unity, Whisper documentation - requires Context7 API key
- **GitHub MCP (The Operator)**: Creates repositories, manages Pull Requests, triggers GitHub Pages workflows - requires GitHub personal access token with repo permissions
- **Playwright MCP (The Navigator)**: Verifies deployed pages, takes screenshots, runs UI tests - requires Playwright installation and browser binaries
- **Vercel MCP (The Deployer)**: Deploys frontend (Docusaurus) and backend (FastAPI) - requires Vercel account, team ID, deployment token

### Cloud Services

- **Qdrant Cloud**: Vector database for RAG semantic search - requires active subscription (Free Tier acceptable) and API credentials (URL, API key)
- **Neon**: Serverless PostgreSQL for user profiles, cached content, audit logs - requires database provisioned with connection string
- **OpenAI API** (or Gemini proxy): GPT-4/Gemini 2.5 Flash for personalization, translation, ROS2-code-gen - requires API key with sufficient quota
- **BetterAuth**: Authentication library for email and OAuth flows - requires GitHub OAuth app credentials (client ID, client secret)

### Technical Prerequisites

- **Docusaurus v3**: Static site generator for textbook content - requires Node.js 18+ environment
- **FastAPI**: Backend API framework for RAG chatbot - requires Python 3.12+ runtime
- **Claude Code CLI**: Execution environment for subagent orchestration - requires claude CLI installed with MCP server configurations
- **Turbo Repo** (or Nx): Mono-repo management for apps/docs (Docusaurus) and apps/api (FastAPI) - requires turbo or nx CLI

### Content Prerequisites

- Physical AI & Humanoid Robotics 13-week syllabus with detailed topic breakdown
- ROS 2 architecture diagrams and code examples for Weeks 1-5 (nodes, topics, services, actions)
- Gazebo and Unity simulation guides for Weeks 6-7 (Digital Twin concepts, URDF models)
- NVIDIA Isaac Sim documentation and examples for Weeks 8-10 (GPU-accelerated simulation, synthetic data generation)
- VLA (Vision-Language-Action) models and humanoid robotics capstone project guidelines for Weeks 11-13
- Hardware comparison documentation: RTX 4090 vs Jetson Orin Nano specs, use cases, optimization strategies

## Constraints *(optional)*

### Constitutional Constraints (Constitution v5.0.0)

- **Tech Stack Immutability (Article II)**: MUST use Docusaurus v3, Qdrant Cloud, Neon PostgreSQL, FastAPI, BetterAuth, OpenAI Agents SDK (with Gemini 2.5 Flash) - no alternative technologies permitted
- **MCP Tool Mandate (Article III)**: MUST use Context7, GitHub, Playwright, Vercel MCP servers for their respective domains - direct API calls discouraged
- **Agentic Orchestration (Article VIII)**: MUST implement 9 specialized subagents (chapter-planner, spec-architect, pedagogical-designer, educational-validator, content-implementor, factual-verifier, assessment-architect, validation-auditor, super-orchestrator)
- **Skill System (Article IX)**: MUST connect agents with 23+ canonical skills and 5+ bonus skills as defined in agent-skill mapping matrix
- **Language Protocol (Article IV)**: All code and documentation MUST be in Professional US English; chat/conversation in Roman Urdu
- **Security Non-Negotiables (Article VI)**: Zero hardcoded secrets, mandatory input sanitization, .env-based configuration only
- **Intelligence Preservation (Article VII)**: All subagent executions MUST create PHRs in history/prompts/001-ai-textbook-platform/

### Performance Constraints

- **Page Load Time**: Docusaurus pages MUST load in under 2 seconds on standard broadband connection (3 Mbps+)
- **Chat Response Latency**: RAG chatbot MUST respond within 3 seconds for 95th percentile queries (includes vector search + LLM generation)
- **Personalization Timeout**: Content personalization MUST complete within 10 seconds or fail gracefully with cached fallback
- **Translation Timeout**: Urdu translation MUST complete within 8 seconds for typical chapter (2000-3000 words)
- **Concurrent Users**: Platform MUST support 500 concurrent users without degradation (no increased latency, no failures)
- **Vector Search Performance**: Qdrant queries MUST return results within 500ms for 95th percentile

### Scope Constraints

- **Hackathon Timeline**: MVP must achieve 300/100 points within hackathon submission deadline (all 5 user stories + subagent orchestration)
- **Content Scope**: Platform covers Physical AI & Humanoid Robotics curriculum only - no expansion to other robotics topics (industrial robotics, drones, autonomous vehicles)
- **Hardware Profiles**: Limited to 2 primary configurations (RTX 4090, Jetson Orin Nano) - no support for other edge devices (Raspberry Pi, Coral TPU) or GPUs (Tesla V100, A100)
- **Language Support**: Only English (base content) and Urdu (translation) - no other languages in MVP

### Regulatory Constraints

- **Data Privacy**: User profile data (email, auth provider, hardware preferences) stored in Neon must comply with standard data protection practices (secure storage, no unnecessary retention)
- **AI-Generated Content Disclaimers**: All personalized code, translated content, and ROS2-generated snippets MUST display disclaimers about AI adaptation

## Risks *(optional)*

### High-Impact Risks

1. **OpenAI API Rate Limits**: High personalization/translation demand could exceed quota during hackathon evaluation
   - **Impact**: Feature degradation, user frustration, lost bonus points
   - **Mitigation**: Implement aggressive caching (7-day TTL), queue system for non-urgent requests, fallback to cached generic content, monitor usage dashboard

2. **Subagent Orchestration Complexity**: Coordinating 9 subagents with 28+ skills may introduce failures or bottlenecks
   - **Impact**: Incomplete content generation, validation failures, missed deadline
   - **Mitigation**: Implement super-orchestrator error handling, create fallback to manual content generation for critical chapters, thorough testing of agent coordination

3. **Content Quality for AI Personalization**: AI-generated personalized code may contain errors, security vulnerabilities, or suboptimal patterns
   - **Impact**: User learns incorrect practices, production deployment failures, safety issues in robotics context
   - **Mitigation**: Implement validation layer for generated code (syntax check, import verification), human review of sample outputs, clear disclaimers for AI-modified content

### Medium-Impact Risks

4. **Qdrant Vector DB Performance**: Slow semantic search could bottleneck chatbot responsiveness, exceeding 3-second target
   - **Impact**: Poor user experience, reduced engagement with RAG chatbot
   - **Mitigation**: Pre-warm vector indexes, implement query result caching (5-minute TTL), optimize embedding dimensions (reduce from 1536 to 768), use Qdrant's HNSW index tuning

5. **BetterAuth Integration Complexity**: OAuth flow issues, GitHub API downtime, or email verification failures could block authentication
   - **Impact**: Users unable to access personalization/translation features, lost bonus points
   - **Mitigation**: Allocate extra time for auth testing, implement email-only auth as fallback if GitHub OAuth fails, use BetterAuth's built-in error recovery

6. **Translation Accuracy**: Technical concepts may lose precision or introduce confusion when translated to Urdu
   - **Impact**: Users misunderstand core robotics concepts, reduced learning effectiveness
   - **Mitigation**: Use specialized prompts for technical translation (preserve technical terms in English), provide inline glossary, clear disclaimer about AI translation limitations, human review of sample translations

7. **MCP Server Configuration**: Context7, GitHub, Playwright, Vercel MCP servers may have installation or authentication issues
   - **Impact**: Blocked workflows, manual workarounds needed, reduced automation benefits
   - **Mitigation**: Document MCP server setup procedures, create fallback scripts for manual operations, test MCP integrations early in development cycle

### Low-Impact Risks

8. **Cache Invalidation Strategy**: Stale personalized/translated content after chapter updates could confuse users
   - **Impact**: Users see outdated content, reduced trust in platform
   - **Mitigation**: Implement automatic cache invalidation on chapter update (set cache_expiry to chapter.updated_at), notify users of new version, regenerate personalization on next visit

9. **Playwright UI Testing Flakiness**: Intermittent failures in automated UI tests could block deployment
   - **Impact**: False positives, delayed deployments, manual verification overhead
   - **Mitigation**: Implement retry logic for Playwright tests (3 attempts), use explicit waits instead of implicit timeouts, capture screenshots on failure for debugging

10. **GitHub Pages Deployment Lag**: Vercel deployment successful but GitHub Pages mirror delayed
    - **Impact**: Users accessing GitHub Pages see outdated content
    - **Mitigation**: Prioritize Vercel as primary deployment target, treat GitHub Pages as secondary mirror, document deployment lag in user communications

## Notes *(optional)*

### Hackathon Scoring Strategy

- **Base Score (100 points)**: Achieved by completing Story 1 (Content Engine with 13 weeks) and Story 2 (RAG Chatbot with Qdrant)
- **Bonus 1 (+50 points)**: ChatKit contextual text selection (Story 2 enhancement)
- **Bonus 2 (+50 points)**: BetterAuth with Onboarding Quiz (Story 4)
- **Bonus 3 (+50 points)**: Personalization Engine (Story 3)
- **Bonus 4 (+50 points)**: Localization Engine (Story 5)
- **Additional Bonus**: ROS2 text-to-code generator (Story 6) - bonus points beyond standard categories

**Total Target**: 300+/100 points if all stories implemented

### Implementation Sequence Recommendation

1. **Phase 0 (Foundation - Story 7)**: Set up subagent orchestration, create 9 agent definitions, connect with skills
2. **Phase 1 (MVP for Base Score)**: Story 1 → Story 2 (delivers functional textbook with RAG, earns 100 points)
3. **Phase 2 (First Bonus)**: Enhance Story 2 with ChatKit contextual selection for +50 points
4. **Phase 3 (Auth Foundation)**: Story 4 for +50 points (unlocks personalization/localization features)
5. **Phase 4 (Personalization)**: Story 3 for +50 points
6. **Phase 5 (Localization)**: Story 5 for +50 points
7. **Phase 6 (Bonus Enhancement)**: Story 6 (ROS2 text-to-code) for additional bonus

### Constitution v5.0.0 Compliance Checklist

- ✅ **Article I (SDD)**: Spec created before any code, plan.md and tasks.md will follow
- ✅ **Article II (Tech Stack)**: Docusaurus v3, Qdrant Cloud, Neon, FastAPI, BetterAuth, OpenAI Agents SDK (Gemini 2.5 Flash), ChatKit specified
- ✅ **Article III (MCP Mandate)**: Context7, GitHub, Playwright, Vercel MCP usage documented
- ✅ **Article IV (Agent Behavior)**: Language protocol documented (English for code/docs, Roman Urdu for chat)
- ✅ **Article V (Publishing Standards)**: Docusaurus conventions (tabs, Mermaid), 13-week structure, canonical style specified
- ✅ **Article VI (Engineering Standards)**: SOC Protocol mandated (sanitization, no hardcoded secrets), RAG integrity (source citations), Playwright testing, Vercel deployment
- ✅ **Article VII (Intelligence Preservation)**: PHR creation planned for all subagent executions in history/prompts/001-ai-textbook-platform/
- ✅ **Article VIII (Agentic Orchestration)**: All 9 subagents documented in Story 7 with acceptance scenarios
- ✅ **Article IX (Skill System)**: 23 canonical + 5 bonus skills referenced throughout requirements (session-intelligence-harvester, urdu-translator, ros2-code-generator, etc.)
- ✅ **Article X (Bonus Features)**: All 5 bonus features documented (personalization, Urdu translation, BetterAuth, RAG chatbot with ChatKit, ROS2 text-to-code)
- ✅ **Article XI (Deployment)**: Vercel MCP frontend/backend deployment, GitHub MCP version control, Playwright MCP testing documented

### Technical Architecture Notes

- **Mono-Repo Structure**: Turbo repo with apps/docs (Docusaurus), apps/api (FastAPI), packages/shared-types (TypeScript interfaces), .claude/agents (9 subagents), .claude/skills (28+ skills)
- **Docusaurus Custom Plugins**: Need plugins for "Personalize for Me" button, "Translate to Urdu" button, ChatKit integration, hardware profile selector
- **FastAPI Endpoints**:
  - POST /chat (RAG query with optional selected_context)
  - POST /personalize (chapter_id, hardware_profile → personalized_mdx)
  - POST /translate (chapter_id, target_language → translated_mdx)
  - POST /generate-ros2-code (natural_language_input, hardware_context → code_snippet)
  - POST /auth/signin, POST /auth/signup, POST /auth/onboarding
- **Qdrant Collections**:
  - textbook_chapters (vector embeddings for semantic search)
  - chapter_metadata (titles, part numbers, week numbers, learning objectives)
- **Neon Schema**: users, chapters, personalized_content, translated_content, chat_queries, ros2_code_snippets, audit_logs
- **Environment Variables Required**:
  - OPENAI_API_KEY (or GEMINI_API_KEY if using direct Gemini endpoint)
  - QDRANT_API_KEY, QDRANT_URL
  - NEON_CONNECTION_STRING (PostgreSQL connection string)
  - BETTER_AUTH_GITHUB_CLIENT_ID, BETTER_AUTH_GITHUB_CLIENT_SECRET
  - JWT_SECRET (for token signing)
  - VERCEL_TOKEN (for Vercel MCP deployment)
  - GITHUB_TOKEN (for GitHub MCP operations)
- **Claude Subagent Files**: .claude/agents/{chapter-planner,spec-architect,pedagogical-designer,educational-validator,content-implementor,factual-verifier,assessment-architect,validation-auditor,super-orchestrator}.md
- **Skill Files**: .claude/skills/{book-scaffolding,chapter-planner,concept-scaffolding,summary-generator,quiz-generator,technical-clarity,canonical-format-checker,assessment-builder,mvp-builder,learning-objectives,docusaurus-deployer,prompt-template-designer,code-example-generator,exercise-designer,frontend-design,validation-auditor,skill-creator,playwright-test-runner,image-generator,ux-evaluator,tool-selection-framework,notebooklm-slides,session-intelligence-harvester,urdu-translator,user-profile-initializer,ros2-code-generator,rag-chatbot-integrator,personalization-engine}.md
