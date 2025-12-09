# frontend-design Skill

## Purpose
Design React components and UI/UX patterns for the Docusaurus frontend.

## Key Patterns

### React Component Pattern
```typescript
// apps/docs/src/components/PersonalizeButton.tsx
import React from 'react';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalize: (hardwareProfile: string) => Promise<void>;
}

export const PersonalizeButton: React.FC<PersonalizeButtonProps> = ({
  chapterId,
  onPersonalize,
}) => {
  const [loading, setLoading] = React.useState(false);
  const [hardware, setHardware] = React.useState<string>('rtx4090');

  const handleClick = async () => {
    setLoading(true);
    try {
      await onPersonalize(hardware);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="personalize-button">
      <select value={hardware} onChange={(e) => setHardware(e.target.value)}>
        <option value="rtx4090">RTX 4090 (Simulation)</option>
        <option value="jetson">Jetson Orin Nano (Edge)</option>
      </select>
      <button onClick={handleClick} disabled={loading}>
        {loading ? 'Personalizing...' : 'Personalize Content'}
      </button>
    </div>
  );
};
```

### Custom CSS (apps/docs/src/css/custom.css)
```css
:root {
  --primary-color: #2e8555;
  --secondary-color: #f39c12;
  --background: #ffffff;
  --text-color: #333333;
}

.personalize-button {
  display: flex;
  gap: 10px;
  margin: 20px 0;
  align-items: center;
}

.personalize-button select {
  padding: 8px 12px;
  border: 1px solid var(--primary-color);
  border-radius: 4px;
}

.personalize-button button {
  background: var(--primary-color);
  color: white;
  padding: 8px 16px;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

.personalize-button button:hover {
  background: var(--secondary-color);
}
```

### Docusaurus Integration
```tsx
// apps/docs/src/theme/DocItem/index.tsx
import React from 'react';
import DocItem from '@theme-original/DocItem';
import { PersonalizeButton } from '@site/src/components/PersonalizeButton';

export default function DocItemWrapper(props) {
  return (
    <>
      <DocItem {...props} />
      <PersonalizeButton chapterId={props.content.metadata.id} />
    </>
  );
}
```

## Usage Context
- Custom React components
- UI/UX enhancements
- Interactive features
- Theme customization
