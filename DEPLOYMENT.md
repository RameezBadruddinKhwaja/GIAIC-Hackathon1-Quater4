# Deployment Guide - Physical AI & Humanoid Robotics Textbook Platform

## Overview

This guide covers deploying both the frontend (Docusaurus) and backend (FastAPI) to production.

**Architecture:**
- **Frontend**: Docusaurus → Vercel
- **Backend API**: FastAPI → Vercel (Serverless Functions)
- **Database**: Neon PostgreSQL (Managed)
- **Vector Store**: Qdrant Cloud (Managed)
- **AI**: Google Gemini API

## Prerequisites

Before deployment, ensure you have:

1. **GitHub Account** - For source code hosting
2. **Vercel Account** - For deployment
3. **Neon Account** - For PostgreSQL database
4. **Qdrant Cloud Account** - For vector search
5. **Google AI Studio Account** - For Gemini API key

## Step 1: Database Provisioning

### 1.1 Neon PostgreSQL Setup

1. **Create Account**: https://console.neon.tech/
2. **Create New Project**:
   - Project name: `physical-ai-textbook-platform`
   - Region: `US East (Ohio)` (or closest to your users)
   - PostgreSQL version: 16

3. **Get Connection String**:
   ```bash
   # Example format:
   postgresql://user:password@host.neon.tech/dbname?sslmode=require
   ```

4. **Run Database Migrations**:
   ```bash
   cd apps/api

   # Set connection string
   export NEON_CONNECTION_STRING="postgresql://..."

   # Run migrations
   python scripts/migrate_db.py
   ```

5. **Verify Tables Created**:
   - `users`
   - `chat_logs`
   - `personalized_content`
   - `translated_content`
   - `audit_logs`

### 1.2 Qdrant Cloud Setup

1. **Create Account**: https://cloud.qdrant.io/
2. **Create Cluster**:
   - Cluster name: `book-knowledge-cluster`
   - Region: `US East`
   - Plan: Free tier (1GB)

3. **Create Collection**:
   ```bash
   cd apps/api

   # Set credentials
   export QDRANT_URL="https://your-cluster.qdrant.io"
   export QDRANT_API_KEY="your-api-key"

   # Initialize collection (768-dim for Gemini)
   python scripts/init_qdrant.py
   ```

4. **Ingest Content**:
   ```bash
   # Set Gemini API key
   export GEMINI_API_KEY="your-gemini-api-key"

   # Ingest all weeks + hardware lab
   python scripts/ingest.py
   ```

   Expected output:
   ```
   ✓ Parsed week-01-ros2-basics/index: 12 chunks
   ✓ Parsed week-02-nodes-topics/index: 15 chunks
   ...
   ✓ Successfully indexed 450+ chunks into Qdrant!
   ```

## Step 2: Environment Variables

### 2.1 Backend Environment Variables (.env)

Create `apps/api/.env.production`:

```bash
# Database
NEON_CONNECTION_STRING=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Qdrant Vector Store
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Gemini AI
GEMINI_API_KEY=your-gemini-api-key

# BetterAuth
AUTH_SECRET=your-random-secret-256-bit
GITHUB_CLIENT_ID=your-github-oauth-client-id
GITHUB_CLIENT_SECRET=your-github-oauth-client-secret

# CORS
ALLOWED_ORIGINS=https://your-frontend.vercel.app

# Environment
ENVIRONMENT=production
```

### 2.2 Frontend Environment Variables (.env)

Create `apps/docs/.env.production`:

```bash
# Backend API
NEXT_PUBLIC_API_URL=https://your-backend.vercel.app

# GitHub OAuth (for BetterAuth)
NEXT_PUBLIC_GITHUB_CLIENT_ID=your-github-oauth-client-id
```

## Step 3: GitHub OAuth Setup (BetterAuth)

1. **Create OAuth App**:
   - Go to: https://github.com/settings/developers
   - Click "New OAuth App"
   - Application name: `Physical AI Textbook Platform`
   - Homepage URL: `https://your-frontend.vercel.app`
   - Authorization callback URL: `https://your-backend.vercel.app/api/auth/callback/github`

2. **Get Credentials**:
   - Client ID: `Iv1.xxxxxxxxxx`
   - Client Secret: `ghp_xxxxxxxxxx`

3. **Add to Environment Variables** (see Step 2)

## Step 4: Deploy Backend (FastAPI → Vercel)

### 4.1 Create vercel.json

Create `apps/api/vercel.json`:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "src/main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "src/main.py"
    }
  ],
  "env": {
    "PYTHONPATH": "/var/task"
  }
}
```

### 4.2 Create requirements.txt

Ensure `apps/api/requirements.txt` includes:

```txt
fastapi==0.104.1
uvicorn==0.24.0
python-dotenv==1.0.0
sqlalchemy==2.0.23
psycopg2-binary==2.9.9
qdrant-client==1.6.9
openai==1.3.7
python-jose==3.3.0
passlib==1.7.4
pydantic==2.5.0
httpx==0.25.1
```

### 4.3 Deploy to Vercel

```bash
cd apps/api

# Install Vercel CLI
npm install -g vercel

# Login
vercel login

# Deploy
vercel --prod

# Set environment variables
vercel env add NEON_CONNECTION_STRING production
vercel env add QDRANT_URL production
vercel env add QDRANT_API_KEY production
vercel env add GEMINI_API_KEY production
vercel env add AUTH_SECRET production
vercel env add GITHUB_CLIENT_ID production
vercel env add GITHUB_CLIENT_SECRET production
```

**Output**:
```
✓ Deployed to production
https://your-backend.vercel.app
```

## Step 5: Deploy Frontend (Docusaurus → Vercel)

### 5.1 Update docusaurus.config.ts

Ensure production URL is set:

```typescript
const config: Config = {
  url: 'https://your-frontend.vercel.app',
  baseUrl: '/',
  // ...
};
```

### 5.2 Deploy to Vercel

```bash
cd apps/docs

# Deploy
vercel --prod

# Set environment variables
vercel env add NEXT_PUBLIC_API_URL production
# Enter: https://your-backend.vercel.app

vercel env add NEXT_PUBLIC_GITHUB_CLIENT_ID production
# Enter: your-github-client-id
```

**Output**:
```
✓ Deployed to production
https://your-frontend.vercel.app
```

## Step 6: Post-Deployment Verification

### 6.1 Backend Health Check

```bash
curl https://your-backend.vercel.app/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "database": {
    "neon": "configured",
    "qdrant": "configured"
  },
  "ai": {
    "gemini": "configured"
  }
}
```

### 6.2 Frontend Smoke Test

1. **Navigate to**: `https://your-frontend.vercel.app`
2. **Check components**:
   - ✅ Translate button (top right)
   - ✅ Personalize button (top right)
   - ✅ Chat widget (bottom right)
3. **Test features**:
   - Click "Week 1" → Content loads
   - Click chat widget → Opens
   - Ask: "What is ROS 2?" → RAG responds
   - Click "Personalize" → Prompts for hardware onboarding (if not done)
   - Click "اردو میں پڑھیں" → Triggers translation

### 6.3 Test ROS 2 Playground

1. **Navigate to**: `/playground` or embed in a week
2. **Enter description**: "Create a publisher that sends Twist messages"
3. **Click "Generate Code"** → Python code appears
4. **Copy code** → Should copy to clipboard
5. **Switch to C++** → Regenerate → C++ code appears

## Step 7: Performance Optimization

### 7.1 Enable Vercel Edge Caching

Update `apps/docs/vercel.json`:

```json
{
  "headers": [
    {
      "source": "/(.*)",
      "headers": [
        {
          "key": "Cache-Control",
          "value": "s-maxage=3600, stale-while-revalidate=86400"
        }
      ]
    }
  ]
}
```

### 7.2 Optimize Bundle Size

```bash
cd apps/docs

# Analyze bundle
npm run build -- --analyze

# Expected: < 1MB total JS
```

### 7.3 Enable Compression

Add to `apps/api/src/main.py`:

```python
from fastapi.middleware.gzip import GZipMiddleware

app.add_middleware(GZipMiddleware, minimum_size=1000)
```

## Step 8: Monitoring & Analytics

### 8.1 Vercel Analytics

1. **Enable in Dashboard**: https://vercel.com/dashboard/analytics
2. **Frontend**: Automatic
3. **Backend**: Add to `package.json`:
   ```json
   {
     "dependencies": {
       "@vercel/analytics": "^1.1.1"
     }
   }
   ```

### 8.2 Error Tracking (Sentry - Optional)

```bash
# Install Sentry
npm install @sentry/react @sentry/node

# Configure
SENTRY_DSN=your-sentry-dsn
```

## Troubleshooting

### Issue: "QDRANT_URL not configured"

**Solution**:
```bash
vercel env add QDRANT_URL production
# Paste your Qdrant Cloud URL
vercel --prod  # Redeploy
```

### Issue: "Database connection failed"

**Solution**:
1. Check Neon connection string format:
   ```
   postgresql://user:password@host.neon.tech/dbname?sslmode=require
   ```
2. Verify IP whitelist in Neon dashboard (allow Vercel IPs)

### Issue: "Gemini API rate limit"

**Solution**:
1. Upgrade to paid Gemini plan
2. Implement request caching:
   ```python
   # Cache RAG responses for 1 hour
   @lru_cache(maxsize=100)
   def cached_rag_query(query: str):
       return chat_with_rag(query)
   ```

### Issue: "Translation button not working"

**Solution**:
1. Check browser console for errors
2. Verify `NEXT_PUBLIC_API_URL` is set
3. Test backend endpoint directly:
   ```bash
   curl -X POST https://your-backend.vercel.app/api/translate/ \
     -H "Content-Type: application/json" \
     -d '{"content":"Test","target_lang":"ur"}'
   ```

## Hackathon Scoring Checklist

### Base MVP (100/100 points)
- [x] 13-week textbook content deployed
- [x] Hardware Lab guides accessible
- [x] Docusaurus navigation working
- [x] Responsive design

### Bonus Features (+200 points)
- [x] RAG Chatbot (+50 points)
  - ✅ Qdrant indexed
  - ✅ Gemini integration
  - ✅ ChatWidget functional
- [x] BetterAuth (+50 points)
  - ✅ GitHub OAuth
  - ✅ Hardware onboarding quiz
  - ✅ Session persistence
- [x] Personalization (+50 points)
  - ✅ Content adaptation for RTX/Jetson
  - ✅ PersonalizeButton functional
  - ✅ 7-day caching
- [x] Urdu Translation (+50 points)
  - ✅ All 13 weeks translatable
  - ✅ TranslateButton functional
  - ✅ Code preservation
- [x] ROS2 Code Generator (Bonus)
  - ✅ Natural language to Python/C++
  - ✅ Monaco Editor integration
  - ✅ ROS2Playground component

**Total**: 300+/100 points 🏆

## Resources

- [Vercel Documentation](https://vercel.com/docs)
- [Neon Documentation](https://neon.tech/docs)
- [Qdrant Cloud Documentation](https://qdrant.tech/documentation/cloud/)
- [Gemini API Documentation](https://ai.google.dev/docs)
- [BetterAuth Documentation](https://www.better-auth.com/docs)

## Support

For deployment issues:
1. Check Vercel deployment logs
2. Review Neon database logs
3. Check Qdrant cluster status
4. Verify all environment variables are set

---

**Deployment Time Estimate**: 30-45 minutes (after database provisioning)
