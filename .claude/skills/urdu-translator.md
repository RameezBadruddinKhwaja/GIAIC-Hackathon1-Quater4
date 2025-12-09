# Urdu Translator - Multilingual Content Adaptation

## Purpose
Translate technical robotics content from English to Urdu while preserving:
- Code blocks (untranslated)
- Technical terms (romanized or explained)
- Markdown structure and formatting
- Learning objectives and success criteria

## Key Patterns

### 1. Translation API Endpoint

```python
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from openai import OpenAI
import os

router = APIRouter(prefix="/api/translate", tags=["translation"])

# Gemini via OpenAI SDK
client = OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

class TranslationRequest(BaseModel):
    content: str
    source_lang: str = "en"
    target_lang: str = "ur"
    preserve_code: bool = True

class TranslationResponse(BaseModel):
    translated_content: str
    original_content: str
    language: str

@router.post("/", response_model=TranslationResponse)
async def translate_content(request: TranslationRequest):
    """
    Translate technical content to Urdu using Gemini.

    Preserves:
    - Code blocks (```...```)
    - Technical terms (ROS 2, CUDA, etc.)
    - Markdown formatting
    - URLs and links
    """

    system_prompt = """You are an expert technical translator specializing in robotics and AI content.

Translation Rules:
1. **Preserve Code**: NEVER translate code blocks (```...```). Keep them exactly as-is.
2. **Technical Terms**: Keep English terms for: ROS 2, CUDA, GPU, API, Docker, Gazebo, Isaac Sim, etc.
3. **Romanize When Needed**: For terms without Urdu equivalents, use romanization with explanation.
4. **Markdown Structure**: Preserve all markdown formatting (headers, lists, tables, links).
5. **Learning Objectives**: Translate objectives but keep measurable criteria clear.
6. **Natural Flow**: Ensure Urdu text reads naturally, not word-for-word translation.

Examples:
- "ROS 2 Node" → "ROS 2 نوڈ"
- "Publisher-Subscriber pattern" → "Publisher-Subscriber پیٹرن (اشاعت کنندہ-رکنیت کا نمونہ)"
- "Install CUDA" → "CUDA انسٹال کریں"

Preserve:
- Code: `ros2 run my_package my_node` (unchanged)
- URLs: [Link](https://example.com) (unchanged)
- Mermaid diagrams: ```mermaid ... ``` (unchanged)
"""

    try:
        response = client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Translate to Urdu:\n\n{request.content}"}
            ],
            temperature=0.3,
            max_tokens=4000
        )

        translated = response.choices[0].message.content

        return TranslationResponse(
            translated_content=translated,
            original_content=request.content,
            language=request.target_lang
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")
```

### 2. Batch Translation Script

```python
import os
import glob
import asyncio
from pathlib import Path

async def translate_all_weeks():
    """Translate all 13 weeks + Hardware Lab to Urdu"""

    # Week content files
    week_files = glob.glob("apps/docs/docs/week-*/index.md")

    # Hardware Lab files
    lab_files = glob.glob("apps/docs/docs/hardware-lab/*.md")

    all_files = week_files + lab_files

    for file_path in all_files:
        print(f"Translating: {file_path}")

        # Read original content
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Translate using API
        translated = await translate_content_api(content)

        # Create Urdu version
        urdu_path = file_path.replace('.md', '.ur.md')
        with open(urdu_path, 'w', encoding='utf-8') as f:
            f.write(translated)

        print(f"✅ Created: {urdu_path}")

async def translate_content_api(content: str) -> str:
    """Call translation API"""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "http://localhost:8000/api/translate/",
            json={
                "content": content,
                "target_lang": "ur",
                "preserve_code": True
            }
        )
        return response.json()["translated_content"]

if __name__ == "__main__":
    asyncio.run(translate_all_weeks())
```

### 3. Frontend Translation Toggle

```tsx
// apps/docs/src/components/TranslateButton.tsx
import React, { useState } from 'react';
import { useLocation } from '@docusaurus/router';

export function TranslateButton() {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');
  const location = useLocation();

  const toggleLanguage = () => {
    const currentPath = location.pathname;

    if (language === 'en') {
      // Switch to Urdu
      const urduPath = currentPath.replace('.md', '.ur.md');
      window.location.href = urduPath;
      setLanguage('ur');
    } else {
      // Switch to English
      const enPath = currentPath.replace('.ur.md', '.md');
      window.location.href = enPath;
      setLanguage('en');
    }
  };

  return (
    <button
      onClick={toggleLanguage}
      className="translate-button"
      style={{
        position: 'fixed',
        top: '80px',
        right: '20px',
        padding: '10px 20px',
        backgroundColor: '#2e8555',
        color: 'white',
        border: 'none',
        borderRadius: '8px',
        cursor: 'pointer',
        zIndex: 1000,
        fontWeight: 'bold',
      }}
    >
      {language === 'en' ? 'اردو میں پڑھیں' : 'Read in English'}
    </button>
  );
}
```

### 4. Docusaurus Integration

```tsx
// apps/docs/src/theme/Root.tsx
import React from 'react';
import { TranslateButton } from '@site/src/components/TranslateButton';

export default function Root({ children }) {
  return (
    <>
      {children}
      <TranslateButton />
    </>
  );
}
```

### 5. Database Model for Cached Translations

```python
from sqlalchemy import Column, Integer, String, Text, DateTime, Index
from datetime import datetime

class TranslatedContent(Base):
    __tablename__ = "translated_content"

    id = Column(Integer, primary_key=True, index=True)
    original_path = Column(String(500), nullable=False)
    original_content_hash = Column(String(64), nullable=False)  # SHA256
    translated_content = Column(Text, nullable=False)
    target_language = Column(String(10), default="ur")
    created_at = Column(DateTime, default=datetime.utcnow)

    __table_args__ = (
        Index('idx_translation_lookup', 'original_path', 'target_language', 'original_content_hash'),
    )
```

### 6. Translation with Caching

```python
import hashlib

async def get_or_create_translation(content: str, file_path: str, target_lang: str = "ur"):
    """
    Check cache first, translate if not found.
    Reduces API calls and costs.
    """

    # Compute hash
    content_hash = hashlib.sha256(content.encode()).hexdigest()

    # Check cache
    cached = db.query(TranslatedContent).filter(
        TranslatedContent.original_path == file_path,
        TranslatedContent.target_language == target_lang,
        TranslatedContent.original_content_hash == content_hash
    ).first()

    if cached:
        print(f"✅ Cache hit for {file_path}")
        return cached.translated_content

    # Translate
    print(f"🔄 Translating {file_path}...")
    translated = await translate_with_gemini(content, target_lang)

    # Save to cache
    new_translation = TranslatedContent(
        original_path=file_path,
        original_content_hash=content_hash,
        translated_content=translated,
        target_language=target_lang
    )
    db.add(new_translation)
    db.commit()

    return translated
```

## Usage Context

### When to Use:
1. **Initial Translation** - Translate all 13 weeks + Hardware Lab content to Urdu
2. **On-Demand Translation** - User clicks "اردو میں پڑھیں" button
3. **Cached Retrieval** - Serve from database for previously translated content
4. **Update Detection** - Re-translate if source content changes (hash mismatch)

### Where to Use:
- **Backend**: Translation API endpoint in FastAPI
- **Frontend**: TranslateButton component in Docusaurus
- **Batch Script**: Translate all content offline before deployment
- **Database**: Cache translations to reduce API costs

### Best Practices:
1. **Preserve Code**: Never translate code blocks, commands, or technical syntax
2. **Technical Terms**: Keep English for standard robotics/AI terminology
3. **Natural Flow**: Ensure Urdu reads naturally, not literal translation
4. **Caching**: Always check cache before calling Gemini API
5. **Bidirectional**: Support switching back to English from Urdu

## Technical Requirements

### Gemini Configuration:
```python
client = OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Use gemini-2.5-flash for translation (fast + accurate)
model = "gemini-2.5-flash"
temperature = 0.3  # Lower for consistent technical translation
```

### Environment Variables:
```bash
GEMINI_API_KEY=your-gemini-api-key
DATABASE_URL=postgresql://user:pass@host/db
```

### Database Migration:
```sql
-- Alembic migration for TranslatedContent table
CREATE TABLE translated_content (
    id SERIAL PRIMARY KEY,
    original_path VARCHAR(500) NOT NULL,
    original_content_hash VARCHAR(64) NOT NULL,
    translated_content TEXT NOT NULL,
    target_language VARCHAR(10) DEFAULT 'ur',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_translation_lookup
ON translated_content (original_path, target_language, original_content_hash);
```

## Example Translations

### English (Original):
```markdown
# Week 1: ROS 2 Basics

## Learning Objectives
- ✅ Understand ROS 2 architecture
- ✅ Create your first ROS 2 node
- ✅ Publish and subscribe to topics

## Installation
sudo apt install ros-humble-desktop
```

### Urdu (Translated):
```markdown
# ہفتہ 1: ROS 2 بنیادی باتیں

## سیکھنے کے مقاصد
- ✅ ROS 2 آرکیٹیکچر کو سمجھنا
- ✅ اپنا پہلا ROS 2 نوڈ بنانا
- ✅ Topics پر publish اور subscribe کرنا

## انسٹالیشن
sudo apt install ros-humble-desktop
```

**Note**: Code blocks remain unchanged, technical terms (ROS 2, Topics, publish, subscribe) preserved with Urdu explanation where needed.

## Success Criteria

### For +50 Hackathon Points:
- ✅ All 13 weeks translated to Urdu
- ✅ All 4 Hardware Lab guides translated
- ✅ Translation toggle button functional in UI
- ✅ Code blocks preserved exactly
- ✅ Technical terms handled correctly
- ✅ Caching reduces API costs
- ✅ Bidirectional language switching works

## Key Takeaways

- **Gemini** handles Urdu translation better than most models (multilingual strength)
- **Caching** is critical (13 weeks × 500 lines = expensive without cache)
- **Code preservation** requires explicit prompt instructions
- **Technical terms** should remain in English with Urdu explanation
- **Natural flow** matters more than literal translation for learning content
