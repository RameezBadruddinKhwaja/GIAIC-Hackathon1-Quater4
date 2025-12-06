# Better-Auth Implementation Skill

**Description**: Implementation guide for Better-Auth with Email/GitHub OAuth and security best practices

**Core Knowledge**:

## Better-Auth Overview
Better-Auth is a modern authentication framework with built-in OAuth support and session management.

**Key Features**:
- Email/password authentication
- GitHub OAuth integration
- JWT token generation
- Session management
- Password hashing (bcrypt)

## Backend Setup (FastAPI)

### 1. Installation
```bash
pip install better-auth-python bcrypt python-jose[cryptography]
```

### 2. Email Authentication
```python
# apps/api/src/routers/auth.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, EmailStr
import bcrypt
from jose import JWTError, jwt
from datetime import datetime, timedelta

router = APIRouter(prefix="/api/auth")

class SignupRequest(BaseModel):
    email: EmailStr
    password: str

class SigninRequest(BaseModel):
    email: EmailStr
    password: str

@router.post("/signup")
async def signup(request: SignupRequest):
    # Hash password
    hashed_password = bcrypt.hashpw(
        request.password.encode('utf-8'),
        bcrypt.gensalt()
    )

    # Create user in database
    user = User(
        email=request.email,
        password_hash=hashed_password,
        auth_provider='email'
    )
    db.add(user)
    await db.commit()

    return {"message": "User created successfully"}

@router.post("/signin")
async def signin(request: SigninRequest):
    # Fetch user from database
    user = await db.query(User).filter(User.email == request.email).first()

    if not user or not bcrypt.checkpw(
        request.password.encode('utf-8'),
        user.password_hash
    ):
        raise HTTPException(status_code=401, detail="Invalid credentials")

    # Generate JWT token
    token = create_jwt_token(user.id)

    return {"access_token": token, "token_type": "bearer"}

def create_jwt_token(user_id: str) -> str:
    payload = {
        "sub": user_id,
        "exp": datetime.utcnow() + timedelta(days=7)
    }
    return jwt.encode(payload, os.getenv("JWT_SECRET"), algorithm="HS256")
```

### 3. GitHub OAuth
```python
# apps/api/src/routers/auth.py
from fastapi import APIRouter
import httpx

@router.get("/callback/github")
async def github_callback(code: str):
    # Exchange code for access token
    token_response = await httpx.post(
        "https://github.com/login/oauth/access_token",
        data={
            "client_id": os.getenv("BETTER_AUTH_GITHUB_CLIENT_ID"),
            "client_secret": os.getenv("BETTER_AUTH_GITHUB_CLIENT_SECRET"),
            "code": code
        },
        headers={"Accept": "application/json"}
    )
    access_token = token_response.json()["access_token"]

    # Fetch GitHub user email
    user_response = await httpx.get(
        "https://api.github.com/user",
        headers={"Authorization": f"Bearer {access_token}"}
    )
    github_user = user_response.json()

    # Create or update user
    user = await db.query(User).filter(User.email == github_user["email"]).first()
    if not user:
        user = User(email=github_user["email"], auth_provider='github')
        db.add(user)
        await db.commit()

    # Generate JWT token
    token = create_jwt_token(user.id)

    return {"access_token": token, "token_type": "bearer"}
```

### 4. JWT Middleware
```python
# apps/api/src/utils/auth_middleware.py
from fastapi import Depends, HTTPException, Header
from jose import JWTError, jwt
import os

def verify_jwt_token(authorization: str = Header(None)):
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing or invalid token")

    token = authorization.replace("Bearer ", "")

    try:
        payload = jwt.decode(token, os.getenv("JWT_SECRET"), algorithms=["HS256"])
        user_id = payload.get("sub")
        if not user_id:
            raise HTTPException(status_code=401, detail="Invalid token")
        return user_id
    except JWTError:
        raise HTTPException(status_code=401, detail="Token expired or invalid")
```

## Frontend Setup (React)

### 1. Installation
```bash
npm install @better-auth/react axios
```

### 2. Auth Provider
```tsx
// apps/docs/src/components/AuthProvider/index.tsx
import React, { createContext, useState, useEffect } from 'react';
import axios from 'axios';

const AuthContext = createContext(null);

export function AuthProvider({ children }) {
  const [user, setUser] = useState(null);
  const [token, setToken] = useState(localStorage.getItem('token'));

  const signin = async (email: string, password: string) => {
    const response = await axios.post('/api/auth/signin', { email, password });
    const { access_token } = response.data;
    localStorage.setItem('token', access_token);
    setToken(access_token);
  };

  const signout = () => {
    localStorage.removeItem('token');
    setToken(null);
    setUser(null);
  };

  return (
    <AuthContext.Provider value={{ user, token, signin, signout }}>
      {children}
    </AuthContext.Provider>
  );
}
```

## Security Best Practices

1. **Password Hashing**: Always use bcrypt with salt
2. **JWT Expiration**: Set 7-day expiration for tokens
3. **Environment Variables**: Store secrets in `.env` (never hardcode)
4. **HTTPS Only**: Enforce HTTPS in production
5. **CORS Configuration**: Whitelist only frontend origin
6. **Rate Limiting**: Implement rate limits on auth endpoints

## GitHub OAuth Setup

1. Go to https://github.com/settings/developers
2. Create New OAuth App
3. Set callback URL: `http://localhost:3000/api/auth/callback/github`
4. Copy Client ID and Client Secret to `.env`

**Keyword Triggers**:
- **Load when query contains**: Better-Auth, OAuth, GitHub, authentication, JWT, signin, signup

**Invocation**:
Automatically loaded when implementing authentication features or debugging auth issues.
