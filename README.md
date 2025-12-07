# Physical AI & Humanoid Robotics Textbook Platform

AI-Native educational platform for learning Physical AI, ROS 2, NVIDIA Isaac, and Humanoid Robotics.

## Features

- **Interactive Textbook**: Docusaurus-powered content with 4 modules (13 weeks), Mermaid diagrams, code tabs
- **RAG Chatbot**: Ask questions and get answers with cited sources from the textbook
- **Personalization Engine**: Hardware-aware content adaptation (RTX 4090 vs Jetson Orin Nano)
- **Localization**: Urdu translation (Roman/Formal) with code preservation
- **Authentication**: Email/GitHub OAuth with onboarding quiz

## Tech Stack

- **Frontend**: Docusaurus v3 (React 18 + TypeScript)
- **Backend**: FastAPI (Python 3.12+)
- **Databases**: Neon (PostgreSQL), Qdrant Cloud (Vector Store)
- **AI**: Google Gemini 2.5 Flash via OpenAI SDK
- **Auth**: Better-Auth

## Quickstart

See detailed setup instructions in: [specs/001-ai-textbook-platform/quickstart.md](specs/001-ai-textbook-platform/quickstart.md)

## Project Structure

```
├── apps/
│   ├── docs/          # Docusaurus frontend
│   └── api/           # FastAPI backend
├── packages/
│   └── shared-types/  # TypeScript interfaces
├── .claude/           # Agentic infrastructure
├── specs/             # Feature specifications
└── history/           # PHRs and ADRs
```

## Hackathon Scoring

- **Base MVP** (Textbook + Content): 100/100 points
- **Bonus Features**: +200 points (Matrix Protocol, Better-Auth, Personalization, Localization)
- **Target Score**: 300/100

## License

MIT
