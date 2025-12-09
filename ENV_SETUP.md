# Environment Variables Setup Guide

## Overview

This guide lists all required environment variables for the Physical AI & Humanoid Robotics Textbook Platform.

## Backend Environment Variables

### Required Variables

Create `apps/api/.env`:

```bash
# ========================================
# DATABASE
# ========================================

# Neon PostgreSQL Connection String
# Format: postgresql://user:password@host.neon.tech/dbname?sslmode=require
# Get from: https://console.neon.tech/
NEON_CONNECTION_STRING=postgresql://your-user:your-password@your-host.neon.tech/your-db?sslmode=require

# ========================================
# VECTOR STORE
# ========================================

# Qdrant Cloud URL
# Format: https://your-cluster-id.qdrant.io
# Get from: https://cloud.qdrant.io/
QDRANT_URL=https://your-cluster-id.qdrant.io

# Qdrant API Key
# Get from: Qdrant Cloud Dashboard → API Keys
QDRANT_API_KEY=your-qdrant-api-key

# ========================================
# AI / LLM
# ========================================

# Google Gemini API Key
# Get from: https://ai.google.dev/
# CRITICAL: Use with OpenAI SDK, NOT Google generativeai SDK
GEMINI_API_KEY=your-gemini-api-key

# ========================================
# AUTHENTICATION
# ========================================

# BetterAuth Secret (256-bit random string)
# Generate with: openssl rand -hex 32
AUTH_SECRET=your-256-bit-random-secret-here

# GitHub OAuth Client ID
# Get from: https://github.com/settings/developers
GITHUB_CLIENT_ID=Iv1.your-github-client-id

# GitHub OAuth Client Secret
# Get from: https://github.com/settings/developers
GITHUB_CLIENT_SECRET=ghp_your-github-client-secret

# ========================================
# CORS & SECURITY
# ========================================

# Allowed Origins (comma-separated for multiple)
# Development: http://localhost:3000
# Production: https://your-frontend.vercel.app
ALLOWED_ORIGINS=http://localhost:3000,https://your-frontend.vercel.app

# ========================================
# OPTIONAL
# ========================================

# Environment
# Options: development, production
ENVIRONMENT=development

# Log Level
# Options: DEBUG, INFO, WARNING, ERROR
LOG_LEVEL=INFO
```

## Frontend Environment Variables

Create `apps/docs/.env`:

```bash
# ========================================
# BACKEND API
# ========================================

# Backend API URL
# Development: http://localhost:8000
# Production: https://your-backend.vercel.app
NEXT_PUBLIC_API_URL=http://localhost:8000

# ========================================
# AUTHENTICATION
# ========================================

# GitHub OAuth Client ID (same as backend)
# Get from: https://github.com/settings/developers
NEXT_PUBLIC_GITHUB_CLIENT_ID=Iv1.your-github-client-id

# ========================================
# OPTIONAL
# ========================================

# Analytics (Vercel Analytics)
# Auto-detected by Vercel, no env var needed

# Google Analytics (if you want additional tracking)
NEXT_PUBLIC_GA_ID=G-XXXXXXXXXX
```

## How to Get API Keys

### 1. Neon PostgreSQL

1. **Sign up**: https://console.neon.tech/signup
2. **Create project**: Click "New Project"
3. **Get connection string**:
   - Go to project dashboard
   - Click "Connection String"
   - Copy the `postgresql://...` string
   - Format includes SSL: `?sslmode=require`

### 2. Qdrant Cloud

1. **Sign up**: https://cloud.qdrant.io/signup
2. **Create cluster**:
   - Click "Create Cluster"
   - Select region (US East recommended)
   - Free tier: 1GB storage
3. **Get credentials**:
   - **URL**: `https://your-cluster-id.qdrant.io`
   - **API Key**: Dashboard → Settings → API Keys → Create

### 3. Google Gemini API

1. **Sign up**: https://ai.google.dev/
2. **Get API key**:
   - Go to: https://makersuite.google.com/app/apikey
   - Click "Create API Key"
   - Copy the key (starts with `AI...`)

**CRITICAL**: Use with OpenAI SDK:
```python
from openai import OpenAI

client = OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)
```

### 4. GitHub OAuth

1. **Go to**: https://github.com/settings/developers
2. **Click**: "New OAuth App"
3. **Fill in**:
   - **Application name**: `Physical AI Textbook Platform`
   - **Homepage URL**: `http://localhost:3000` (dev) or `https://your-frontend.vercel.app` (prod)
   - **Authorization callback URL**: `http://localhost:8000/api/auth/callback/github` (dev)
4. **Get credentials**:
   - **Client ID**: `Iv1.xxxxxxxxxx`
   - **Client Secret**: Click "Generate new client secret"

## Environment-Specific Configurations

### Development (.env.development)

```bash
# Backend
NEON_CONNECTION_STRING=postgresql://dev-user:dev-pass@dev-host.neon.tech/dev-db?sslmode=require
QDRANT_URL=https://dev-cluster.qdrant.io
QDRANT_API_KEY=dev-qdrant-key
GEMINI_API_KEY=your-gemini-api-key
AUTH_SECRET=dev-secret-256-bit
GITHUB_CLIENT_ID=dev-github-client-id
GITHUB_CLIENT_SECRET=dev-github-client-secret
ALLOWED_ORIGINS=http://localhost:3000
ENVIRONMENT=development

# Frontend
NEXT_PUBLIC_API_URL=http://localhost:8000
NEXT_PUBLIC_GITHUB_CLIENT_ID=dev-github-client-id
```

### Production (.env.production)

```bash
# Backend
NEON_CONNECTION_STRING=postgresql://prod-user:prod-pass@prod-host.neon.tech/prod-db?sslmode=require
QDRANT_URL=https://prod-cluster.qdrant.io
QDRANT_API_KEY=prod-qdrant-key
GEMINI_API_KEY=your-gemini-api-key
AUTH_SECRET=prod-secret-256-bit
GITHUB_CLIENT_ID=prod-github-client-id
GITHUB_CLIENT_SECRET=prod-github-client-secret
ALLOWED_ORIGINS=https://your-frontend.vercel.app
ENVIRONMENT=production

# Frontend
NEXT_PUBLIC_API_URL=https://your-backend.vercel.app
NEXT_PUBLIC_GITHUB_CLIENT_ID=prod-github-client-id
```

## Vercel Deployment

### Backend Environment Variables (via Vercel CLI)

```bash
cd apps/api

# Set all variables for production
vercel env add NEON_CONNECTION_STRING production
vercel env add QDRANT_URL production
vercel env add QDRANT_API_KEY production
vercel env add GEMINI_API_KEY production
vercel env add AUTH_SECRET production
vercel env add GITHUB_CLIENT_ID production
vercel env add GITHUB_CLIENT_SECRET production
vercel env add ALLOWED_ORIGINS production
vercel env add ENVIRONMENT production

# Verify
vercel env ls
```

### Frontend Environment Variables (via Vercel CLI)

```bash
cd apps/docs

# Set all variables for production
vercel env add NEXT_PUBLIC_API_URL production
vercel env add NEXT_PUBLIC_GITHUB_CLIENT_ID production

# Verify
vercel env ls
```

## Security Best Practices

### 1. Never Commit .env Files

Ensure `.gitignore` includes:
```
.env
.env.local
.env.development
.env.production
.env*.local
```

### 2. Use Different Credentials for Dev/Prod

- **Development**: Use separate Neon database, Qdrant cluster, GitHub OAuth app
- **Production**: Use dedicated production credentials

### 3. Rotate Secrets Regularly

```bash
# Generate new AUTH_SECRET
openssl rand -hex 32

# Update in Vercel
vercel env rm AUTH_SECRET production
vercel env add AUTH_SECRET production
```

### 4. Limit CORS Origins

```bash
# Development
ALLOWED_ORIGINS=http://localhost:3000

# Production
ALLOWED_ORIGINS=https://your-frontend.vercel.app

# Multiple (comma-separated)
ALLOWED_ORIGINS=https://domain1.com,https://domain2.com
```

## Validation Script

Create `apps/api/scripts/validate_env.py`:

```python
import os
from dotenv import load_dotenv

load_dotenv()

required_vars = [
    "NEON_CONNECTION_STRING",
    "QDRANT_URL",
    "QDRANT_API_KEY",
    "GEMINI_API_KEY",
    "AUTH_SECRET",
    "GITHUB_CLIENT_ID",
    "GITHUB_CLIENT_SECRET",
]

missing = []
for var in required_vars:
    if not os.getenv(var):
        missing.append(var)

if missing:
    print("❌ Missing environment variables:")
    for var in missing:
        print(f"   - {var}")
    exit(1)
else:
    print("✅ All required environment variables are set!")

# Validate formats
neon_conn = os.getenv("NEON_CONNECTION_STRING")
if not neon_conn.startswith("postgresql://"):
    print("⚠️  NEON_CONNECTION_STRING should start with 'postgresql://'")

qdrant_url = os.getenv("QDRANT_URL")
if not qdrant_url.startswith("https://"):
    print("⚠️  QDRANT_URL should start with 'https://'")

print("✅ Environment validation complete!")
```

Run validation:
```bash
python scripts/validate_env.py
```

## Troubleshooting

### Issue: "NEON_CONNECTION_STRING not found"

**Solution**:
```bash
# Check if .env exists
ls -la apps/api/.env

# Load manually
export $(cat apps/api/.env | xargs)

# Verify
echo $NEON_CONNECTION_STRING
```

### Issue: "Qdrant connection failed"

**Solution**:
1. Check URL format: `https://your-cluster-id.qdrant.io` (no trailing slash)
2. Verify API key is active in Qdrant dashboard
3. Test connection:
   ```python
   from qdrant_client import QdrantClient
   client = QdrantClient(url="...", api_key="...")
   client.get_collections()  # Should not error
   ```

### Issue: "Gemini API key invalid"

**Solution**:
1. Verify key starts with `AI...` (not `AIza...`)
2. Check API is enabled: https://console.cloud.google.com/apis/library
3. Test:
   ```python
   from openai import OpenAI
   client = OpenAI(
       api_key="your-key",
       base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
   )
   response = client.chat.completions.create(
       model="gemini-2.5-flash",
       messages=[{"role": "user", "content": "Hello"}]
   )
   print(response.choices[0].message.content)
   ```

## Reference Links

- [Neon Documentation](https://neon.tech/docs/introduction)
- [Qdrant Cloud Documentation](https://qdrant.tech/documentation/cloud/)
- [Gemini API Documentation](https://ai.google.dev/docs)
- [GitHub OAuth Documentation](https://docs.github.com/en/developers/apps/building-oauth-apps)
- [Vercel Environment Variables](https://vercel.com/docs/concepts/projects/environment-variables)

---

**Last Updated**: 2025-12-10
