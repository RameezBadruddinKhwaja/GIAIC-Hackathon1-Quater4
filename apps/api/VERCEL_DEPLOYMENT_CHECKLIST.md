# Vercel Deployment Checklist - RAG Chatbot API

**Status**: 🔴 PRODUCTION FAILING - `FUNCTION_INVOCATION_FAILED`

**Last Updated**: 2025-12-18

---

## 🚨 Critical Issues Fixed in Code

✅ **Fixed in commit `1df02e6`**:
- Removed async lifespan event (causes Vercel serverless failures)
- Updated API key handling to support `GEMINI_API_KEY`
- Created `.env.example` documentation

---

## 📋 Vercel Dashboard Configuration Checklist

### Step 1: Navigate to Vercel Project Settings

1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Select project: `physical-ai-textbook-api` or your project name
3. Go to **Settings** → **Environment Variables**

### Step 2: Verify ALL Environment Variables Are Set

**Required Variables** (check each one exists):

#### AI API Configuration
- [ ] `GEMINI_API_KEY` = `AIzaSyAPYaTBhW4jbXEptYPOzs9OgBmcWprAL8M`
  - **OR**
- [ ] `OPENAI_API_KEY` = (if using OpenAI instead)

#### Database Configuration
- [ ] `NEON_CONNECTION_STRING` = `postgresql://neondb_owner:npg_ayv1fB5TmUJA@ep-gentle-shape-a1eup7fl-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require`

#### Vector Store Configuration
- [ ] `QDRANT_URL` = `https://6e308fb1-9061-452f-a079-3198ffec0182.eu-central-1-0.aws.cloud.qdrant.io`
- [ ] `QDRANT_API_KEY` = `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.ELKVAFJi7eJgtc1ikvTGMBtejqxAo0NacxvpMX1D92s`

#### Authentication Configuration
- [ ] `BETTER_AUTH_GITHUB_CLIENT_ID` = `Ov23lirsi7tAO1ETW2MQ`
- [ ] `BETTER_AUTH_GITHUB_CLIENT_SECRET` = `a7d704369fc6d651db455da624dcfa33482abe7a`
- [ ] `JWT_SECRET` = `6SQjorOC7oYZtIAoFZAsoNUE8wPKjW8B9YiBhSNdeDA`

#### Deployment Configuration
- [ ] `ENVIRONMENT` = `production`
- [ ] `API_URL` = `https://giaic-hackathon1-quater4.vercel.app`
- [ ] `FRONTEND_URL` = `https://giaic-hackathon1-quater4-frontend.vercel.app`

### Step 3: Check Environment Scope

For each variable above, verify:
- [ ] **Environment**: Set for `Production`, `Preview`, and `Development`
- [ ] **Git Branch**: Available to all branches (or at least `main`)

### Step 4: Verify Build Configuration

In **Settings** → **General**:

- [ ] **Framework Preset**: `Other`
- [ ] **Build Command**: (leave empty or default)
- [ ] **Output Directory**: (leave empty)
- [ ] **Install Command**: `pip install -r requirements.txt`
- [ ] **Root Directory**: `apps/api` (if monorepo)

### Step 5: Check Python Runtime

In **Settings** → **General** → **Node.js Version**:
- [ ] Python version: `3.12` (should be set via `vercel.json`)

Verify `vercel.json` contains:
```json
{
  "env": {
    "PYTHON_VERSION": "3.12"
  }
}
```

### Step 6: Review Deployment Logs

1. Go to **Deployments** tab
2. Click on the latest deployment
3. Check **Build Logs** for errors:
   - [ ] No missing dependencies errors
   - [ ] No module import errors
   - [ ] Build completed successfully

4. Check **Function Logs** (Runtime Logs):
   - [ ] No `ModuleNotFoundError`
   - [ ] No `KeyError` for environment variables
   - [ ] No database connection errors

---

## 🔍 Common Vercel Deployment Issues & Solutions

### Issue 1: `FUNCTION_INVOCATION_FAILED`

**Possible Causes**:
1. Missing environment variables → Check Step 2 above
2. Import errors in `main.py` → Check Build Logs
3. Database connection failing at import time → Should be lazy-loaded
4. Async lifespan events (FIXED in latest code)

**Solution**:
- Verify all env vars are set
- Check that latest commit `1df02e6` is deployed
- Review Function Logs for specific error

### Issue 2: Missing Environment Variables

**Symptoms**:
- Runtime errors mentioning "not set"
- `ValueError: OPENAI_API_KEY must be set`

**Solution**:
- Go to Vercel Dashboard → Settings → Environment Variables
- Add missing variables from Step 2
- Redeploy (Deployments → latest → click "..." → Redeploy)

### Issue 3: Import Errors

**Symptoms**:
- `ModuleNotFoundError: No module named 'fastapi'`
- Build fails during dependency installation

**Solution**:
- Check `requirements.txt` is in correct location (`apps/api/requirements.txt`)
- Verify all dependencies are compatible with Python 3.12
- Check Build Logs for installation errors

### Issue 4: Database Connection Errors

**Symptoms**:
- Connection timeout
- SSL errors
- `asyncpg` connection failures

**Solution**:
- Verify `NEON_CONNECTION_STRING` includes `sslmode=require`
- Check Neon dashboard that database is active
- Ensure connection string format is correct for `asyncpg`

---

## ✅ Verification Steps After Configuration

### 1. Redeploy After Setting Environment Variables

```bash
# Option A: Push a new commit (triggers auto-deploy)
git commit --allow-empty -m "chore: trigger redeploy"
git push origin main

# Option B: Manual redeploy from Vercel Dashboard
# Go to Deployments → Latest → "..." → Redeploy
```

### 2. Test Health Endpoint

```bash
# Should return JSON with status "healthy"
curl https://giaic-hackathon1-quater4.vercel.app/api/health

# Expected response:
# {
#   "status": "healthy",
#   "database": {
#     "neon": "configured",
#     "qdrant": "configured"
#   },
#   "ai": {
#     "gemini": "configured"
#   }
# }
```

### 3. Test Chat Health Endpoint

```bash
curl https://giaic-hackathon1-quater4.vercel.app/api/chat/health

# Expected response:
# {
#   "status": "healthy",
#   "services": {
#     "ai_api": "configured",
#     "qdrant": "configured"
#   }
# }
```

### 4. Test RAG Query Endpoint

```bash
curl -X POST https://giaic-hackathon1-quater4.vercel.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "chapter_id": null,
    "conversation_history": null
  }'

# Should return JSON with answer and citations
```

---

## 📊 Expected Production Behavior

### Healthy Deployment Indicators:

✅ **Build Logs**:
- `Building...` → `Success`
- All dependencies installed
- No import errors

✅ **Function Logs**:
- No errors on first invocation
- Database connections succeed (lazy-loaded)
- API requests return 200 OK

✅ **Health Endpoints**:
- `/api/health` returns `status: "healthy"`
- `/api/chat/health` returns `status: "healthy"`
- All services show `"configured"`

✅ **Chat Endpoint**:
- `/api/chat/query` returns valid JSON
- Response includes `answer` and `citations`
- No 500 errors

---

## 🐛 Debugging Production Issues

### View Real-Time Logs

1. Go to Vercel Dashboard → Project → **Deployments**
2. Click on latest deployment → **Function Logs** tab
3. Watch for errors in real-time as you test endpoints

### Check Specific Error Messages

Look for these patterns in logs:

**Missing Env Var**:
```
ValueError: GEMINI_API_KEY must be set in environment variables
```
→ Add env var in Vercel Dashboard

**Import Error**:
```
ModuleNotFoundError: No module named 'qdrant_client'
```
→ Check `requirements.txt`, rebuild

**Database Error**:
```
asyncpg.exceptions.InvalidPasswordError
```
→ Verify `NEON_CONNECTION_STRING` is correct

**Qdrant Error**:
```
qdrant_client.http.exceptions.UnexpectedResponse: 403 Forbidden
```
→ Verify `QDRANT_API_KEY` is correct

---

## 📝 Post-Deployment Verification Checklist

After fixing Vercel configuration:

- [ ] Health endpoint returns HTTP 200
- [ ] Chat health endpoint returns `status: "healthy"`
- [ ] Can query chatbot successfully
- [ ] Chatbot returns citations
- [ ] No errors in Function Logs
- [ ] Frontend can connect to API (CORS working)
- [ ] ChatWidget loads without errors

---

## 🔐 Security Notes

⚠️ **CRITICAL**: All API keys and secrets shown in this document are:
- Already exposed in the repository's `.env` file
- Should be rotated immediately after hackathon
- Only for temporary hackathon use

**Post-Hackathon**:
1. Generate new API keys for all services
2. Update Vercel environment variables
3. Never commit `.env` files to git
4. Use Vercel's secret management

---

## 📞 Support Resources

- **Vercel Docs**: https://vercel.com/docs/functions/serverless-functions/runtimes/python
- **FastAPI on Vercel**: https://vercel.com/guides/deploying-fastapi-with-vercel
- **Neon Docs**: https://neon.tech/docs
- **Qdrant Cloud**: https://qdrant.tech/documentation/cloud/

---

## ✨ Summary

**What Was Fixed in Code**:
1. Removed async lifespan (Vercel incompatible)
2. Added GEMINI_API_KEY support
3. Updated health checks

**What Needs Manual Verification in Vercel Dashboard**:
1. All environment variables are set correctly
2. Environment scope includes Production
3. Latest deployment includes the fix commit
4. Function logs show no errors

**Next Steps**:
1. Follow Step 2 checklist above to verify all env vars
2. Redeploy if needed
3. Test health endpoints
4. Verify chatbot works end-to-end

---

**Generated**: 2025-12-18 by Claude Code
**Commit with fixes**: `1df02e6`
