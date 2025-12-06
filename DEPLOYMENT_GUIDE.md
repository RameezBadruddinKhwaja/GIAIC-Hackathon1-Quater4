# 🚀 Manual Handover Report: Physical AI Textbook Platform

**Project**: Physical AI & Humanoid Robotics Textbook Platform
**Status**: Development Complete - Ready for Testing & Deployment
**Date**: December 6, 2025
**Branch**: `001-ai-textbook-platform`

---

## ✅ Completion Status

### Completed Phases
- [x] **Phase 1**: Foundation (Monorepo, Gemini, Neon PostgreSQL, Qdrant Cloud)
- [x] **Phase 2**: Content Engine (Docusaurus with 4 modules, deep technical content)
- [x] **Phase 3**: RAG Intelligence (ChatWidget, Qdrant vector search, Matrix Protocol)
- [x] **Phase 4**: Authentication & Onboarding (JWT auth, hardware profile quiz)
- [x] **Phase 5**: Personalization Engine (Hardware-aware content rewriting)
- [x] **Phase 6**: Localization Engine (Roman Urdu & Formal Urdu translation)
- [x] **Phase 7**: Content Expansion & Polish (8,000+ words of deep technical content)

### What's Built
- ✅ Docusaurus frontend with 4 course modules
- ✅ FastAPI backend with 4 routers (chat, auth, personalize, translate)
- ✅ Neon PostgreSQL database (5 tables: users, chat_logs, personalized_content, translated_content, audit_logs)
- ✅ Qdrant vector store (book_knowledge collection with 768-dim embeddings)
- ✅ RAG chatbot with Matrix Protocol skill loading
- ✅ JWT authentication with onboarding quiz
- ✅ Hardware-aware personalization (RTX 4090 vs Jetson Orin Nano)
- ✅ Urdu localization (Roman Urdu + Formal Urdu with code preservation)

---

## 📋 SECTION 1: Local Verification Steps

### Prerequisites Check

Before starting, verify you have these installed:

```bash
# Check Node.js (need 18+)
node --version

# Check Python (need 3.11+)
python3 --version

# Check npm (need 9+)
npm --version

# Check Git
git --version
```

**If missing**: Install from [nodejs.org](https://nodejs.org/), [python.org](https://www.python.org/), and [git-scm.com](https://git-scm.com/)

---

### Step 1: Environment Setup

#### 1.1 Update Backend Environment Variables

You already have `apps/api/.env`, but need to generate a JWT secret:

```bash
# Navigate to project root
cd /mnt/d/Rameez-Bader/Quater\ 4\ Hackathon/Ebook

# Generate JWT secret
python3 -c "import secrets; print(secrets.token_urlsafe(32))"
```

Copy the output and update `apps/api/.env`:

```bash
# apps/api/.env
JWT_SECRET=<paste-the-generated-secret-here>
```

#### 1.2 Create Frontend Environment File

```bash
# Create .env for frontend
cat > apps/docs/.env << 'EOF'
REACT_APP_API_URL=http://localhost:8000
EOF
```

---

### Step 2: Install Dependencies

#### 2.1 Backend Dependencies

```bash
# Navigate to backend
cd apps/api

# Create virtual environment (if not exists)
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Verify installation
pip list | grep -E "fastapi|uvicorn|sqlalchemy|qdrant|openai"

# Return to project root
cd ../..
```

**Expected packages**: fastapi, uvicorn, sqlalchemy, alembic, qdrant-client, openai, passlib, python-jose

#### 2.2 Frontend Dependencies

```bash
# Navigate to frontend
cd apps/docs

# Install dependencies
npm install

# Return to project root
cd ../..
```

**Expected duration**: 2-3 minutes

---

### Step 3: Database Setup

#### 3.1 Run Migrations (Neon PostgreSQL)

```bash
cd apps/api
source venv/bin/activate

# Run migrations
alembic upgrade head

# Expected output:
# INFO  [alembic.runtime.migration] Running upgrade  -> 001_create_users
# INFO  [alembic.runtime.migration] Running upgrade 001_create_users -> 002_create_chat_logs
# ...

cd ../..
```

**If error "alembic: command not found"**:
```bash
pip install alembic
```

#### 3.2 Verify Database Tables

```bash
# Install psql if needed (Ubuntu/WSL)
sudo apt-get install postgresql-client

# Connect and list tables
psql "postgresql://neondb_owner:npg_ayv1fB5TmUJA@ep-gentle-shape-a1eup7fl-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require" -c "\dt"
```

**Expected tables**:
- users
- chat_logs
- personalized_content
- translated_content
- audit_logs

---

### Step 4: Initialize Qdrant Collection

```bash
cd apps/api
source venv/bin/activate

# Initialize Qdrant collection
python scripts/init_qdrant.py

# Expected output:
# [INFO] Creating collection: book_knowledge
# [INFO] Collection created with 768-dimensional vectors

cd ../..
```

---

### Step 5: Index Content into Qdrant

```bash
cd apps/api
source venv/bin/activate

# Run ingestion script
python scripts/ingest.py

# Expected output:
# Processing: /path/to/docs/intro.md
# Processing: /path/to/docs/module-1/ros2-fundamentals.md
# ...
# Total chunks indexed: 50+

cd ../..
```

**Note**: This indexes all MDX files from `apps/docs/docs/` into Qdrant for RAG.

---

### Step 6: Start the Application

Open **TWO terminal windows**:

#### Terminal 1: Backend (FastAPI)

```bash
cd /mnt/d/Rameez-Bader/Quater\ 4\ Hackathon/Ebook/apps/api
source venv/bin/activate
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345]
INFO:     Application startup complete.
```

**Keep this terminal running**

#### Terminal 2: Frontend (Docusaurus)

```bash
cd /mnt/d/Rameez-Bader/Quater\ 4\ Hackathon/Ebook/apps/docs
npm start
```

**Expected output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/

✔ Client
  Compiled successfully in 3.21s
```

**Keep this terminal running**

---

### Step 7: Verify Services

Open your browser and verify these URLs:

#### 7.1 Backend Health Check
- **URL**: http://localhost:8000/api/health
- **Expected**: JSON response showing "healthy" status

#### 7.2 API Documentation
- **URL**: http://localhost:8000/docs
- **Expected**: FastAPI Swagger UI with 4 router sections (chat, auth, personalize, translate)

#### 7.3 Frontend Homepage
- **URL**: http://localhost:3000
- **Expected**: Docusaurus homepage with 4 course modules

---

## 🧪 SECTION 2: Manual Testing Checklist

### Test 1: Homepage Navigation

- [ ] **Step 1.1**: Open http://localhost:3000
- [ ] **Step 1.2**: Verify you see the homepage with title "Physical AI & Humanoid Robotics"
- [ ] **Step 1.3**: Click "Get Started" button
- [ ] **Step 1.4**: Verify you're redirected to `/docs/intro`

**Expected Result**: Intro page loads with course overview

---

### Test 2: RAG Chatbot (Unauthenticated)

- [ ] **Step 2.1**: On any docs page, look for floating chat button (bottom-right corner)
- [ ] **Step 2.2**: Click the chat button
- [ ] **Step 2.3**: Type: "What is ROS 2?"
- [ ] **Step 2.4**: Press Send
- [ ] **Step 2.5**: Verify you receive an answer with citations

**Expected Result**: Answer appears with source citations (e.g., "Source: module-1/ros2-fundamentals")

---

### Test 3: Authentication Flow

#### 3.1 Sign Up

- [ ] **Step 3.1.1**: Navigate to http://localhost:3000/login
- [ ] **Step 3.1.2**: Click "Sign Up" tab
- [ ] **Step 3.1.3**: Enter email: `test@example.com`
- [ ] **Step 3.1.4**: Enter password: `TestPass123!`
- [ ] **Step 3.1.5**: Click "Sign Up"
- [ ] **Step 3.1.6**: Verify redirect to http://localhost:3000/onboarding

**Expected Result**: Account created, redirected to onboarding

#### 3.2 Onboarding Quiz

- [ ] **Step 3.2.1**: On onboarding page, select hardware: **"RTX 4090 (Sim Rig)"**
- [ ] **Step 3.2.2**: Select language: **"Python"**
- [ ] **Step 3.2.3**: Click "Complete Onboarding"
- [ ] **Step 3.2.4**: Verify redirect to homepage with "Logout" button visible

**Expected Result**: Profile saved, user logged in

#### 3.3 Sign Out & Sign In

- [ ] **Step 3.3.1**: Click "Logout" button (top-right navbar)
- [ ] **Step 3.3.2**: Verify redirect to `/login`
- [ ] **Step 3.3.3**: Enter same email/password from Step 3.1
- [ ] **Step 3.3.4**: Click "Sign In"
- [ ] **Step 3.3.5**: Verify you're logged in (homepage shows "Logout" button)

**Expected Result**: Can sign out and sign back in successfully

---

### Test 4: Personalization Engine

- [ ] **Step 4.1**: Ensure you're logged in with RTX 4090 profile (from Test 3)
- [ ] **Step 4.2**: Navigate to any docs page (e.g., `/docs/module-2/gazebo-simulation`)
- [ ] **Step 4.3**: Locate "Personalize for RTX 4090" button (top of content)
- [ ] **Step 4.4**: Click the button
- [ ] **Step 4.5**: Wait for personalization (loading spinner)
- [ ] **Step 4.6**: Verify content updates with GPU-specific examples
- [ ] **Step 4.7**: Look for checkmark icon indicating personalized state

**Expected Result**: Content rewrites to include GPU acceleration, Isaac Sim references, and high-fidelity simulation tips

**Bonus Test**:
- [ ] Create another account with "Jetson Orin Nano" profile
- [ ] Personalize same page
- [ ] Verify content emphasizes power optimization and edge deployment

---

### Test 5: Urdu Translation

- [ ] **Step 5.1**: On any docs page, locate "Translate to Roman Urdu" button
- [ ] **Step 5.2**: Click the button
- [ ] **Step 5.3**: Wait for translation (loading spinner)
- [ ] **Step 5.4**: Verify content translates to Roman Urdu
- [ ] **Step 5.5**: **CRITICAL CHECK**: Scroll through content and verify code blocks remain in English
- [ ] **Step 5.6**: Verify Mermaid diagrams remain unchanged
- [ ] **Step 5.7**: Click "Back to English" button
- [ ] **Step 5.8**: Verify content reverts to original English

**Expected Result**: Prose translated, code/diagrams preserved

**Bonus Test**:
- [ ] Click "Translate to Formal Urdu" button
- [ ] Verify content uses Urdu script (اردو)
- [ ] Verify code blocks still in English

---

### Test 6: RAG Chatbot (Authenticated)

- [ ] **Step 6.1**: Ensure you're logged in
- [ ] **Step 6.2**: Open chat widget
- [ ] **Step 6.3**: Type: "Explain URDF friction parameters"
- [ ] **Step 6.4**: Verify answer cites newly expanded content from Module 2

**Expected Result**: Answer references deep technical content from `gazebo-simulation.md` and `urdf-guides.md`

---

### Test 7: Matrix Protocol Skill Loading

- [ ] **Step 7.1**: Open browser DevTools (F12)
- [ ] **Step 7.2**: Go to Console tab
- [ ] **Step 7.3**: In chat widget, type: "What is a ROS 2 Node?"
- [ ] **Step 7.4**: Check console for log: `Matrix Skill Loaded: ros2-mastery`

**Expected Result**: Console shows dynamic skill loading based on keyword detection

---

### Test 8: Audit Logging

- [ ] **Step 8.1**: Use Personalize and Translate features
- [ ] **Step 8.2**: Connect to Neon database:
  ```bash
  psql "postgresql://neondb_owner:npg_ayv1fB5TmUJA@ep-gentle-shape-a1eup7fl-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require"
  ```
- [ ] **Step 8.3**: Query audit logs:
  ```sql
  SELECT * FROM audit_logs ORDER BY created_at DESC LIMIT 10;
  ```
- [ ] **Step 8.4**: Verify logs show personalization and translation events

**Expected Result**: Audit logs capture all user actions with timestamps

---

## 📦 SECTION 3: Deployment Preparation

### Pre-Deployment Checklist

- [ ] All tests from Section 2 passed
- [ ] No errors in browser console (F12 → Console)
- [ ] No errors in backend terminal
- [ ] Frontend builds successfully: `cd apps/docs && npm run build`

---

### Step 1: Create GitHub Repository

```bash
# Initialize git (if not already initialized)
cd /mnt/d/Rameez-Bader/Quater\ 4\ Hackathon/Ebook
git init

# Add all files
git add .

# Create initial commit
git commit -m "feat: Complete Physical AI Textbook Platform

- Docusaurus frontend with 4 modules
- FastAPI backend with RAG chatbot
- JWT authentication with hardware onboarding
- Personalization engine (RTX 4090 vs Jetson)
- Urdu localization (Roman + Formal)
- Deep technical content (8000+ words)

🤖 Generated with Claude Code"

# Create GitHub repo (manually via GitHub UI):
# 1. Go to https://github.com/new
# 2. Repository name: physical-ai-textbook-platform
# 3. Description: AI-Native Textbook Platform for Physical AI & Humanoid Robotics
# 4. Visibility: Public
# 5. Click "Create repository"

# Add remote (replace YOUR_USERNAME)
git remote add origin https://github.com/YOUR_USERNAME/physical-ai-textbook-platform.git

# Push to GitHub
git branch -M main
git push -u origin main
```

---

### Step 2: Prepare for Deployment

#### 2.1 Create `.gitignore` (if not exists)

```bash
cat > .gitignore << 'EOF'
# Python
venv/
__pycache__/
*.pyc
.env
*.db

# Node
node_modules/
.docusaurus/
build/
.cache/

# IDE
.vscode/
.idea/

# OS
.DS_Store
EOF

git add .gitignore
git commit -m "chore: Add .gitignore"
git push
```

#### 2.2 Create Production Requirements

```bash
# Create production requirements (without dev dependencies)
cat > apps/api/requirements-prod.txt << 'EOF'
fastapi==0.104.1
uvicorn[standard]==0.24.0
sqlalchemy==2.0.23
alembic==1.12.1
psycopg2-binary==2.9.9
qdrant-client==1.7.0
openai==1.3.5
pydantic==2.5.0
python-dotenv==1.0.0
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
google-generativeai==0.3.1
EOF

git add apps/api/requirements-prod.txt
git commit -m "chore: Add production requirements"
git push
```

---

### Step 3: Deploy Backend (Render)

#### 3.1 Create Render Account

1. Go to [render.com](https://render.com/)
2. Sign up with GitHub
3. Click "New +" → "Web Service"
4. Connect your GitHub repository
5. Select the repository: `physical-ai-textbook-platform`

#### 3.2 Configure Render Service

**Service Settings**:
- **Name**: `physical-ai-textbook-api`
- **Region**: `Oregon (US West)`
- **Branch**: `main`
- **Root Directory**: `apps/api`
- **Runtime**: `Python 3.11`
- **Build Command**: `pip install -r requirements-prod.txt && alembic upgrade head`
- **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- **Instance Type**: `Free` (for testing) or `Starter` (for production)

#### 3.3 Add Environment Variables (Render Dashboard)

Navigate to "Environment" tab and add:

| Key | Value | Source |
|-----|-------|--------|
| `NEON_CONNECTION_STRING` | `postgresql://neondb_owner:npg_ayv1fB5TmUJA@ep-gentle-shape-a1eup7fl-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require` | From `apps/api/.env` |
| `QDRANT_URL` | `https://6e308fb1-9061-452f-a079-3198ffec0182.eu-central-1-0.aws.cloud.qdrant.io` | From `apps/api/.env` |
| `QDRANT_API_KEY` | `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.ELKVAFJi7eJgtc1ikvTGMBtejqxAo0NacxvpMX1D92s` | From `apps/api/.env` |
| `GEMINI_API_KEY` | `AIzaSyDOLgz0PsYitnEfTcpskiIubgD3KGzYKs8` | From `apps/api/.env` |
| `JWT_SECRET` | (generated in Section 1) | From `apps/api/.env` |
| `ENVIRONMENT` | `production` | New |

**Click "Save Changes"**

#### 3.4 Deploy

- Click "Create Web Service"
- Wait for deployment (5-10 minutes)
- Copy the deployment URL (e.g., `https://physical-ai-textbook-api.onrender.com`)

---

### Step 4: Deploy Frontend (Vercel)

#### 4.1 Create Vercel Account

1. Go to [vercel.com](https://vercel.com/)
2. Sign up with GitHub
3. Click "Add New..." → "Project"
4. Import your GitHub repository

#### 4.2 Configure Vercel Project

**Project Settings**:
- **Framework Preset**: `Docusaurus`
- **Root Directory**: `apps/docs`
- **Build Command**: `npm run build`
- **Output Directory**: `build`
- **Install Command**: `npm install`

#### 4.3 Add Environment Variables (Vercel Dashboard)

Navigate to "Settings" → "Environment Variables":

| Key | Value |
|-----|-------|
| `REACT_APP_API_URL` | `https://physical-ai-textbook-api.onrender.com` (from Step 3.4) |

**Click "Save"**

#### 4.4 Deploy

- Click "Deploy"
- Wait for deployment (2-3 minutes)
- Copy the deployment URL (e.g., `https://physical-ai-textbook-platform.vercel.app`)

---

### Step 5: Post-Deployment Verification

#### 5.1 Update CORS in Backend

After deploying frontend, update CORS in `apps/api/src/main.py`:

```python
# apps/api/src/main.py (line 19-25)
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "https://physical-ai-textbook-platform.vercel.app"  # Production
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Commit and push**:
```bash
git add apps/api/src/main.py
git commit -m "chore: Add production CORS origin"
git push
```

Render will automatically redeploy.

#### 5.2 Test Production Deployment

- [ ] Visit production frontend URL
- [ ] Test chat widget with a question
- [ ] Sign up for an account
- [ ] Test personalization
- [ ] Test translation

---

## 🔐 SECTION 4: Secrets Management

### Environment Variables Summary

#### Backend (Render)

| Variable | Purpose | How to Get | Security Level |
|----------|---------|------------|----------------|
| `NEON_CONNECTION_STRING` | Database connection | Neon Console → Connection Details | **CRITICAL** - Never expose |
| `QDRANT_URL` | Vector store endpoint | Qdrant Cloud Console → Clusters | Public (but use with API key) |
| `QDRANT_API_KEY` | Vector store auth | Qdrant Cloud Console → API Keys | **CRITICAL** - Never expose |
| `GEMINI_API_KEY` | AI model access | Google AI Studio → API Keys | **CRITICAL** - Never expose |
| `JWT_SECRET` | Token signing | `python3 -c "import secrets; print(secrets.token_urlsafe(32))"` | **CRITICAL** - Never expose |
| `ENVIRONMENT` | Runtime mode | Set manually (`development` or `production`) | Public |

#### Frontend (Vercel)

| Variable | Purpose | Security Level |
|----------|---------|----------------|
| `REACT_APP_API_URL` | Backend URL | Public (visible in browser) |

---

### Security Best Practices

#### ✅ DO

- [ ] Use separate API keys for development and production
- [ ] Rotate JWT secret regularly (every 90 days)
- [ ] Monitor Gemini API usage in Google Cloud Console
- [ ] Enable Neon PostgreSQL connection pooling
- [ ] Use Qdrant API key rotation feature
- [ ] Set up monitoring/alerts for database access
- [ ] Review audit logs weekly

#### ❌ DON'T

- [ ] Commit `.env` files to Git
- [ ] Share API keys in Slack/Discord
- [ ] Use the same JWT secret across environments
- [ ] Expose database credentials in client-side code
- [ ] Log sensitive data (passwords, tokens) in backend

---

### Emergency Procedures

#### If API Key Leaked

1. **Gemini API Key**:
   - Go to [Google AI Studio](https://makersuite.google.com/app/apikey)
   - Delete compromised key
   - Create new key
   - Update in Render environment variables

2. **Qdrant API Key**:
   - Go to Qdrant Cloud Console → API Keys
   - Revoke compromised key
   - Generate new key
   - Update in Render environment variables

3. **Neon Connection String**:
   - Go to Neon Console → Settings → Reset Password
   - Update connection string in Render

4. **JWT Secret**:
   - Generate new secret: `python3 -c "import secrets; print(secrets.token_urlsafe(32))"`
   - Update in Render
   - **WARNING**: This will invalidate all existing user sessions

---

## 📊 SECTION 5: Monitoring & Maintenance

### Daily Checks

- [ ] Check Render dashboard for backend uptime
- [ ] Check Vercel dashboard for frontend uptime
- [ ] Monitor Gemini API usage (free tier: 60 requests/minute)
- [ ] Check Neon database storage (free tier: 3 GB)
- [ ] Check Qdrant vector count (free tier: 1M vectors)

### Weekly Tasks

- [ ] Review audit logs for suspicious activity
- [ ] Check error logs in Render dashboard
- [ ] Update content if needed (commit → push → auto-deploy)
- [ ] Test all features in production

### Monthly Tasks

- [ ] Update dependencies: `npm update` and `pip list --outdated`
- [ ] Review and optimize Qdrant collection (delete old vectors)
- [ ] Backup Neon database: `pg_dump $NEON_CONNECTION_STRING > backup.sql`

---

## 🎯 Success Criteria

Your deployment is successful if:

- [x] Frontend accessible at `https://your-app.vercel.app`
- [x] Backend accessible at `https://your-api.onrender.com/api/health`
- [x] Chat widget responds to questions with citations
- [x] Users can sign up, personalize content, and translate to Urdu
- [x] No errors in browser console or backend logs
- [x] All environment variables configured correctly

---

## 📞 Support & Troubleshooting

### Common Issues

#### Issue: "502 Bad Gateway" on Render

**Cause**: Backend startup timeout or crash

**Solution**:
1. Check Render logs for error messages
2. Verify environment variables are set correctly
3. Test migrations: `alembic upgrade head`
4. Check that `uvicorn` command is correct in Start Command

#### Issue: Chat widget not responding

**Cause**: CORS error or backend unreachable

**Solution**:
1. Open browser DevTools (F12) → Console
2. Look for CORS errors (red text)
3. Verify backend URL in `REACT_APP_API_URL`
4. Check CORS origins in `apps/api/src/main.py`

#### Issue: Personalize/Translate buttons not working

**Cause**: User not authenticated or API error

**Solution**:
1. Ensure user is logged in (check for "Logout" button)
2. Check browser console for error messages
3. Verify Gemini API key has quota remaining
4. Check backend logs for 500 errors

---

## 🎉 You're Done!

Your Physical AI Textbook Platform is now:
- ✅ Fully tested locally
- ✅ Deployed to production (Render + Vercel)
- ✅ Secured with proper secrets management
- ✅ Ready for users

**Next Steps**:
1. Share the production URL with users
2. Monitor usage and performance
3. Collect feedback and iterate on features
4. Consider adding more modules or expanding content

**Questions or Issues?**
- Check `specs/001-ai-textbook-platform/research.md` for architecture details
- Review API documentation at `https://your-api.onrender.com/docs`
- Open a GitHub issue for bugs or feature requests

---

**Deployment Guide Version**: 1.0
**Last Updated**: December 6, 2025
**Author**: Physical AI Team

🤖 Generated with [Claude Code](https://claude.com/claude-code)
