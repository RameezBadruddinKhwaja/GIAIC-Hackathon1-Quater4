# React Error #310 Fix - Critical Bug Resolution

**Date**: 2025-12-13
**Error**: Minified React error #310
**Message**: "Rendered more hooks than during the previous render"
**Severity**: Critical - Page crashes
**Status**: ✅ FIXED

---

## Problem Description

The application was crashing with React error #310:
```
Minified React error #310; visit https://react.dev/errors/310
for the full message or use the non-minified dev environment
```

This error means: **"Rendered more hooks than during the previous render"**

---

## Root Cause

Both `ChatWidget` and `TranslateButton` components violated **React's Rules of Hooks** by calling hooks BEFORE conditional returns:

### Violation Pattern (Before Fix)

```typescript
export default function ChatWidget(): JSX.Element | null {
  // ❌ WRONG: Hooks called here
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  // ... more hooks

  // ❌ WRONG: Early return AFTER hooks
  if (!isAuthenticated) {
    return null;
  }

  // ... rest of component
}
```

**Why This Breaks**:

1. **First Render (not authenticated)**:
   - Hooks are called
   - Component returns `null`
   - React records: "This component uses X hooks"

2. **Second Render (user logs in)**:
   - Hooks are called again
   - Component continues past the `if` statement
   - More code runs (including `useEffect`)
   - React sees: "This component now uses X+1 hooks!"
   - **ERROR #310**: Hook count mismatch

---

## The Fix

Move the conditional return to **AFTER all hooks are called**:

### Correct Pattern (After Fix)

```typescript
export default function ChatWidget(): JSX.Element | null {
  // ✅ CORRECT: ALL hooks called first
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Define all functions that use hooks
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // ... all other functions

  // ✅ CORRECT: Conditional return AFTER all hooks
  if (!isAuthenticated) {
    return null;
  }

  return (
    // ... JSX
  );
}
```

---

## Files Fixed

### 1. `apps/docs/src/components/ChatWidget/index.tsx`

**Lines Changed**: 26-47

**Before**:
```typescript
export default function ChatWidget(): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  // ... hooks

  // ❌ Early return at line 37
  if (!isAuthenticated) {
    return null;
  }

  const scrollToBottom = () => { ... };
  useEffect(() => { ... }, [messages]);
  // ...
}
```

**After**:
```typescript
export default function ChatWidget(): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  // ... all hooks

  const scrollToBottom = () => { ... };
  useEffect(() => { ... }, [messages]);

  const sendMessage = async () => { ... };
  const handleKeyPress = (e) => { ... };

  // ✅ Auth check moved to line 100 (after all hooks)
  if (!isAuthenticated) {
    return null;
  }

  return (/* ... */);
}
```

### 2. `apps/docs/src/components/TranslateButton/index.tsx`

**Lines Changed**: 11-23

**Before**:
```typescript
export default function TranslateButton({ onContentUpdate }): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated, token } = useAuth();
  // ... hooks

  // ❌ Early return at line 21
  if (!isAuthenticated) {
    return null;
  }

  const getChapterId = () => { ... };
  // ...
}
```

**After**:
```typescript
export default function TranslateButton({ onContentUpdate }): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated, token } = useAuth();
  // ... all hooks

  const getChapterId = () => { ... };
  const handleTranslate = async () => { ... };
  const handleRevertToEnglish = () => { ... };

  // ✅ Auth check moved to line 100 (after all hooks and functions)
  if (!isAuthenticated) {
    return null;
  }

  return (/* ... */);
}
```

---

## React Rules of Hooks (Refresher)

### Rule #1: Only Call Hooks at the Top Level

**❌ DON'T**:
- Call hooks inside conditions
- Call hooks inside loops
- Call hooks after early returns

**✅ DO**:
- Call hooks at the top of your component
- Call hooks in the same order every time
- Put conditional logic AFTER all hooks

### Rule #2: Only Call Hooks from React Functions

- Call from React function components ✅
- Call from custom hooks ✅
- Call from regular JavaScript functions ❌

---

## Why This Matters

React relies on the **order of hook calls** to maintain state between renders. When you conditionally call hooks or call them after an early return, React loses track of which state corresponds to which hook, leading to:

1. **Error #310** (hook count mismatch)
2. **Incorrect state values**
3. **App crashes**
4. **Unpredictable behavior**

---

## Testing Verification

After applying the fix:

1. ✅ **Guest users**: No crash, components return `null` cleanly
2. ✅ **Authenticated users**: Components render correctly
3. ✅ **Login transition**: No crash when auth state changes
4. ✅ **Logout transition**: No crash when auth state changes

---

## Prevention

To prevent this in the future:

1. **ESLint Rule**: Enable `eslint-plugin-react-hooks`
   ```json
   {
     "plugins": ["react-hooks"],
     "rules": {
       "react-hooks/rules-of-hooks": "error",
       "react-hooks/exhaustive-deps": "warn"
     }
   }
   ```

2. **Code Review Checklist**:
   - [ ] All hooks called at component top
   - [ ] No hooks after conditional returns
   - [ ] No hooks inside if/loops
   - [ ] Hook call order consistent

3. **TypeScript Helper**:
   ```typescript
   // Use this pattern for conditional rendering:
   function MyComponent() {
     // 1. ALL hooks first
     const data = useHook();

     // 2. ALL logic second
     const result = processData(data);

     // 3. Conditional returns last
     if (!result) return null;

     // 4. Main render
     return <div>{result}</div>;
   }
   ```

---

## References

- [React Error #310](https://react.dev/errors/310)
- [Rules of Hooks](https://react.dev/reference/rules/rules-of-hooks)
- [GitHub Issue #78396](https://github.com/vercel/next.js/issues/78396)

---

**Fix Committed**: 2025-12-13
**Impact**: Critical bug preventing app usage
**Resolution Time**: Immediate
**Status**: ✅ Deployed and Verified
