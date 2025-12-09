# user-profile-initializer Skill (Bonus)

## Purpose
Handle BetterAuth signup/onboarding workflows with user profile initialization.

## Key Patterns

### BetterAuth Configuration
```typescript
// apps/api/src/auth/better-auth.config.ts
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "pg",
    url: process.env.NEON_CONNECTION_STRING,
  },
  emailAndPassword: {
    enabled: true,
  },
  socialProviders: {
    github: {
      clientId: process.env.BETTER_AUTH_GITHUB_CLIENT_ID,
      clientSecret: process.env.BETTER_AUTH_GITHUB_CLIENT_SECRET,
    },
  },
  callbacks: {
    async signUp({ user }) {
      // Initialize user profile
      await initializeUserProfile(user.id);
    },
  },
});
```

### Onboarding Quiz
```typescript
// apps/docs/src/components/OnboardingQuiz.tsx
interface OnboardingData {
  hardwareProfile: 'rtx4090' | 'jetson' | 'cloud' | 'none';
  programmingLanguage: 'python' | 'cpp' | 'both';
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  learningGoals: string[];
}

export const OnboardingQuiz: React.FC = () => {
  const [step, setStep] = React.useState(1);
  const [data, setData] = React.useState<Partial<OnboardingData>>({});

  const handleSubmit = async () => {
    await fetch('/api/user/profile', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data),
    });
  };

  return (
    <div className="onboarding-quiz">
      {step === 1 && (
        <div>
          <h2>What hardware will you use?</h2>
          <button onClick={() => setData({...data, hardwareProfile: 'rtx4090'})}>
            RTX 4090 (Simulation)
          </button>
          <button onClick={() => setData({...data, hardwareProfile: 'jetson'})}>
            Jetson Orin Nano (Edge)
          </button>
        </div>
      )}
      {step === 2 && (
        <div>
          <h2>Preferred programming language?</h2>
          <button onClick={() => setData({...data, programmingLanguage: 'python'})}>
            Python
          </button>
          <button onClick={() => setData({...data, programmingLanguage: 'cpp'})}>
            C++
          </button>
          <button onClick={() => setData({...data, programmingLanguage: 'both'})}>
            Both
          </button>
        </div>
      )}
      {/* More steps... */}
    </div>
  );
};
```

### Profile Initialization Endpoint
```python
# apps/api/src/routes/user.py
from fastapi import APIRouter, Depends
from ..models.user import User
from ..auth import get_current_user

router = APIRouter()

@router.post("/user/profile")
async def initialize_profile(
    profile_data: dict,
    current_user: User = Depends(get_current_user)
):
    """Initialize user profile with onboarding data"""
    current_user.hardware_profile = profile_data['hardwareProfile']
    current_user.programming_language = profile_data['programmingLanguage']
    current_user.experience_level = profile_data.get('experienceLevel', 'beginner')

    await current_user.save()
    return {"status": "success", "profile": current_user.dict()}
```

## Usage Context
- User signup flow
- Profile initialization
- Personalization setup
- Learning path customization
