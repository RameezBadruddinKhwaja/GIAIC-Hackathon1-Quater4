# Playwright E2E Tests - AI-Native Textbook Platform

## Overview

This test suite validates critical functionality for the AI-Native Textbook Platform:

1. **Authentication & Chat Widget** (`auth-and-chat.spec.ts`)
   - Chat widget hidden for unauthenticated users
   - Chat widget visible after successful login
   - No skeleton/flicker for guests
   - Header updates with user information

2. **Translation Feature** (`translation.spec.ts`)
   - Single Urdu translation button (no Roman Urdu)
   - Translation only visible for authenticated users
   - Translation triggers correct API endpoint (`/api/translate?lang=ur`)
   - Revert to English functionality

3. **MDX Rendering** (`mdx-rendering.spec.ts`)
   - No console errors across all chapter pages
   - Valid article structure
   - Headings render correctly
   - Code blocks render without errors
   - Admonitions render properly
   - No broken imports or invalid JSX

## Prerequisites

```bash
# Install dependencies
npm install

# Install Playwright browsers (first time only)
npx playwright install
```

## Running Tests

### All Tests (Headless)
```bash
npm test
```

### Interactive UI Mode
```bash
npm run test:ui
```

### Headed Mode (See Browser)
```bash
npm run test:headed
```

### Debug Mode
```bash
npm run test:debug
```

### View Test Report
```bash
npm run test:report
```

## Configuration

The tests are configured via `playwright.config.ts`:

- **Base URL**: `http://localhost:3000` (configurable via `PLAYWRIGHT_BASE_URL`)
- **Timeout**: 30 seconds per test
- **Browsers**: Chromium, Firefox, WebKit
- **Auto-start dev server**: Yes (via `npm run start`)

## Environment Variables

Create a `.env` file in the `apps/docs` directory:

```env
# Test user credentials (for auth tests)
TEST_USER_EMAIL=test@example.com
TEST_USER_PASSWORD=password123

# Optional: Override base URL
PLAYWRIGHT_BASE_URL=http://localhost:3000
```

## Test Structure

```
tests/playwright/
├── auth-and-chat.spec.ts      # Authentication and chat widget tests
├── translation.spec.ts         # Translation feature tests
├── mdx-rendering.spec.ts       # MDX rendering validation tests
└── README.md                   # This file
```

## Authentication Requirements

Some tests require a valid test user account:

1. Create a test account via the signup flow
2. Add credentials to `.env` file
3. Tests will skip if authentication is not configured

## CI/CD Integration

Tests automatically run in CI via GitHub Actions:

```yaml
- name: Install dependencies
  run: npm ci
  working-directory: apps/docs

- name: Install Playwright browsers
  run: npx playwright install --with-deps
  working-directory: apps/docs

- name: Run Playwright tests
  run: npm test
  working-directory: apps/docs

- name: Upload test results
  uses: actions/upload-artifact@v3
  if: always()
  with:
    name: playwright-report
    path: apps/docs/playwright-report/
```

## Coverage

### Auth & Chat Widget Tests
- ✅ Chat hidden for guests
- ✅ Chat visible for authenticated users
- ✅ No UI flicker/skeleton for guests
- ✅ Header displays user info after login
- ✅ Logout functionality

### Translation Tests
- ✅ Single Urdu button only (no Roman Urdu)
- ✅ Translation requires authentication
- ✅ Correct API endpoint called (`/api/translate?lang=ur`)
- ✅ Loading state during translation
- ✅ Revert to English functionality

### MDX Rendering Tests
- ✅ All 13 chapter routes tested
- ✅ No console errors
- ✅ Valid article structure
- ✅ Headings render correctly
- ✅ Code blocks render without errors
- ✅ Admonitions render properly
- ✅ No broken imports
- ✅ No unescaped JSX
- ✅ Homepage renders correctly
- ✅ Navigation sidebar valid
- ✅ Internal links valid (no 404s)

## Debugging Failed Tests

1. **Run in headed mode** to see what's happening:
   ```bash
   npm run test:headed
   ```

2. **Use debug mode** to step through tests:
   ```bash
   npm run test:debug
   ```

3. **Check screenshots** (automatically captured on failure):
   ```
   test-results/<test-name>/
   ```

4. **View HTML report**:
   ```bash
   npm run test:report
   ```

## Writing New Tests

Follow the existing patterns:

```typescript
import { test, expect } from '@playwright/test';

test.describe('Feature Name', () => {
  test('should do something specific', async ({ page }) => {
    await page.goto('/some-route');
    await page.waitForLoadState('networkidle');

    const element = page.locator('selector');
    await expect(element).toBeVisible();
  });
});
```

## Known Limitations

- **Auth tests** skip if login form is not available (e.g., auth not configured)
- **Translation tests** skip if API is not running
- **MDX tests** assume standard Docusaurus structure

## Maintenance

Update chapter routes in `mdx-rendering.spec.ts` when adding new modules:

```typescript
const CHAPTER_ROUTES = [
  '/modules/week-01-ros2-basics',
  '/modules/week-02-advanced-ros2',
  // Add new routes here
];
```

## Support

For issues or questions:
1. Check test output and screenshots
2. Review Playwright documentation: https://playwright.dev
3. Check CI logs for detailed error messages
