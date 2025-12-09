# session-intelligence-harvester Skill

## Purpose
Collect user context and session data for personalization features.

## Key Patterns

### User Context Collection
```typescript
interface UserContext {
  // Hardware Profile
  hardwareProfile: 'rtx4090' | 'jetson' | 'cloud' | 'none';

  // Programming Preference
  programmingLanguage: 'python' | 'cpp' | 'both';

  // Learning Style
  learningPace: 'fast' | 'moderate' | 'slow';
  preferredFormat: 'code-first' | 'theory-first' | 'mixed';

  // Progress Tracking
  completedWeeks: number[];
  currentWeek: number;
  timeSpentMinutes: number;

  // Interaction Patterns
  usesChat: boolean;
  prefersTranslation: boolean;
  activeHours: string[]; // ['09:00-12:00', '18:00-21:00']
}
```

### Session Data Collection
```python
# apps/api/src/services/session_intelligence.py
from datetime import datetime
import json

class SessionIntelligenceHarvester:
    def collect_context(self, user_id: str) -> dict:
        """Collect user context from various sources"""
        return {
            "hardware_profile": self._detect_hardware(),
            "programming_language": self._infer_language_preference(),
            "learning_pace": self._calculate_pace(),
            "interaction_patterns": self._analyze_interactions(),
            "timestamp": datetime.utcnow().isoformat()
        }

    def _detect_hardware(self) -> str:
        """Infer hardware from user selections and chat queries"""
        # Check localStorage, user profile, chat history
        pass

    def _infer_language_preference(self) -> str:
        """Detect Python vs C++ preference from code interactions"""
        # Analyze which code tabs user clicks more
        pass

    def _calculate_pace(self) -> str:
        """Calculate learning pace from time spent per week"""
        # Compare time_spent vs expected_time
        pass
```

### Privacy Considerations
- Store only anonymized data
- Provide opt-out mechanism
- Clear data retention policy
- No PII collection without consent

### Storage Schema
```sql
CREATE TABLE user_context (
    id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    context_data JSONB NOT NULL,
    collected_at TIMESTAMP DEFAULT NOW(),
    expires_at TIMESTAMP  -- Auto-expire after 30 days
);
```

## Usage Context
- Personalization features
- Adaptive learning paths
- Content recommendations
- UX optimization
