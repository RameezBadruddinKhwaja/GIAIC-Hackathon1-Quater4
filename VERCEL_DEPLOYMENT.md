# 🚀 Complete Vercel Deployment Guide

## Part 1: Backend API Deployment (FastAPI on Vercel)

### Step 1: Create Vercel Account
1. Go to https://vercel.com/
2. Sign up with GitHub
3. Connect your repository: `GIAIC-Hackathon1-Quater4`

### Step 2: Configure Environment Variables

In Vercel Dashboard → Project Settings → Environment Variables, add:


### Step 3: Deploy Backend

**Option A: Deploy via Vercel Dashboard**
1. In Vercel Dashboard → Add New Project
2. Import your GitHub repository
3. Framework Preset: Other
4. Root Directory: Leave empty (we handle it in vercel.json)
5. Click Deploy

**Option B: Deploy via Vercel CLI**
```bash
# Install Vercel CLI
npm i -g vercel

# Login
vercel login

# Deploy
cd /mnt/d/Rameez-Bader/Quater\ 4\ Hackathon/Ebook
vercel --prod
```

### Step 4: Get Your API URL

After deployment, you'll get a URL like:
```
https://giaic-hackathon1-quater4-api.vercel.app
```

**IMPORTANT:** Update frontend to use this URL!

---

## Part 2: Frontend Deployment (Already Done!)

Your frontend is already deployed at:
```
https://giaic-hackathon1-quater4.vercel.app/
```

### Update Frontend to Use Deployed API

Update `apps/docs/.env`:
```
REACT_APP_API_URL=https://giaic-hackathon1-quater4-api.vercel.app

```

Then rebuild and redeploy frontend:
```bash
cd apps/docs
npm run build
git add .
git commit -m "feat: Connect frontend to deployed API"
git push
```

---

## Part 3: Post-Deployment Configuration

### 1. Update GitHub OAuth Callback URLs

Go to: https://github.com/settings/developers

Update your OAuth App:
- **Homepage URL:** `https://giaic-hackathon1-quater4.vercel.app`
- **Authorization callback URL:**
  - `https://giaic-hackathon1-quater4.vercel.app/auth/callback`
  - `https://giaic-hackathon1-quater4-api.vercel.app/api/auth/callback`

### 2. Test Deployed API

```bash
# Health check
curl https://giaic-hackathon1-quater4-api.vercel.app/api/health

# Root endpoint
curl https://giaic-hackathon1-quater4-api.vercel.app/
```

### 3. Initialize Qdrant Collection

**IMPORTANT:** Run this ONCE after deploying API:

```bash
# Locally run the initialization script
cd apps/api
source .venv/bin/activate
python scripts/init_qdrant.py
python scripts/ingest.py  # Index all textbook content
```

---

## Part 4: Troubleshooting

### Backend Issues

**Problem:** API not responding
**Solution:** Check Vercel Functions logs in dashboard

**Problem:** Import errors
**Solution:** Ensure all dependencies in requirements.txt

**Problem:** Database connection fails
**Solution:** Check NEON_CONNECTION_STRING in environment variables

### Frontend Issues

**Problem:** API calls failing with CORS error
**Solution:** Verify CORS origins in `apps/api/src/main.py` include your Vercel domain

**Problem:** GitHub login not working
**Solution:** Update OAuth callback URLs in GitHub settings

---

## Part 5: Verification Checklist

- [ ] Backend deployed and responding at `/api/health`
- [ ] Frontend deployed and loading correctly
- [ ] GitHub OAuth configured with correct callback URLs
- [ ] Qdrant collection initialized with content
- [ ] Neon database tables created
- [ ] All environment variables set in Vercel
- [ ] CORS configured to allow frontend domain
- [ ] Test RAG chatbot functionality
- [ ] Test personalization feature
- [ ] Test translation feature

---

## Part 6: GitHub Post-Deployment Changes

After deploying to Vercel, commit these changes:

```bash
git add vercel.json apps/api/.vercelignore apps/docs/.env
git commit -m "feat: Add Vercel deployment configuration"
git push
```

---

## Expected Final URLs

- **Frontend:** https://giaic-hackathon1-quater4.vercel.app/
- **Backend API:** https://giaic-hackathon1-quater4-api.vercel.app/
- **API Health:** https://giaic-hackathon1-quater4-api.vercel.app/api/health
- **API Docs:** https://giaic-hackathon1-quater4-api.vercel.app/docs

---

## Scoring Impact

With backend deployed:
- ✅ **Base MVP (Textbook):** 100/100
- ✅ **RAG Chatbot:** +50 points
- ✅ **Better-Auth:** +50 points
- ✅ **Personalization:** +50 points
- ✅ **Localization:** +50 points

**TOTAL: 300/300** 🎯
