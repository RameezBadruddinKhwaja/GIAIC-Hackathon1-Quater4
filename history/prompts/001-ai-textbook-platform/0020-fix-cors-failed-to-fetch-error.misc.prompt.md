---
id: 0020
title: Fix CORS Failed to Fetch Error
stage: misc
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-textbook-platform
branch: main
user: rameez_badruddin
command: /sp.implement
labels: ["cors", "authentication", "bug-fix", "production"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - apps/api/src/main.py
  - CORS_FIX.md
tests:
  - Manual authentication testing required
---

## Prompt

sign in aur create account dono me yeh error araha hai Failed to fetch

## Response snapshot

Diagnosed and fixed critical CORS misconfiguration causing "Failed to fetch" errors on authentication endpoints.

**Root Cause:**
- Backend CORS configuration in `apps/api/src/main.py` was blocking frontend requests
- Frontend URL `https://giaic-hackathon1-quater4-frontend.vercel.app` was not in allowed origins
- CORS only allowed backend URL and localhost, blocking cross-origin requests from frontend

**Fix Applied:**
- Updated `allow_origins` in CORS middleware to include correct frontend URL
- Removed invalid wildcard pattern `https://*.vercel.app`
- Added explicit frontend URL: `https://giaic-hackathon1-quater4-frontend.vercel.app`

**Files Modified:**
- `apps/api/src/main.py` (line 24-28): Added frontend URL to CORS allowed origins
- Created `CORS_FIX.md`: Comprehensive deployment and testing guide

**Deployment Required:**
```bash
git add apps/api/src/main.py CORS_FIX.md
git commit -m "fix: Add frontend URL to CORS allowed origins"
git push
```

After deployment (2-3 minutes), all authentication endpoints will work:
- Email/Password Sign Up
- Email/Password Sign In
- GitHub OAuth
- Translate and Personalize buttons (post-authentication)

## Outcome

- ✅ Impact: Critical bug fix - enables all authentication flows (email/password and GitHub OAuth)
- 🧪 Tests: Manual testing required after deployment - test sign up, sign in, and GitHub OAuth
- 📁 Files: Modified 1 file (main.py), created 1 documentation file
- 🔁 Next prompts: Test authentication after deployment, configure GitHub OAuth callback URL if not done
- 🧠 Reflection: CORS errors are common in production when frontend/backend are on different domains. Always verify CORS configuration includes all legitimate origins.

## Evaluation notes (flywheel)

- Failure modes observed: CORS misconfiguration blocking legitimate cross-origin requests from frontend
- Graders run and results (PASS/FAIL): Pending deployment - manual verification required
- Prompt variant (if applicable): N/A - direct bug fix request
- Next experiment (smallest change to try): Deploy and verify with browser DevTools Network tab showing successful 200 responses instead of CORS errors
