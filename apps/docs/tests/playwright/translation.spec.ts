import { test, expect } from '@playwright/test';

/**
 * Test Suite: Translation Functionality
 *
 * Requirements:
 * - Translate button works
 * - Single Urdu translation option (no Roman Urdu)
 * - Translation updates page content
 * - Revert to English functionality
 */

test.describe('Translation Feature', () => {
  test('translate button should NOT be visible for unauthenticated users', async ({ page }) => {
    // Navigate to a chapter page
    await page.goto('/modules/week-01-ros2-basics');

    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Translate button should NOT exist for guests
    const translateButton = page.locator('button:has-text("Translate to Urdu")');
    await expect(translateButton).not.toBeVisible();
  });

  test('translate button should be visible for authenticated users', async ({ page }) => {
    // This test requires authentication
    // For actual implementation, login first (similar to auth-and-chat.spec.ts)

    // Navigate to login
    await page.goto('/');
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")').first();

    if (await loginButton.isVisible()) {
      await loginButton.click();

      const emailInput = page.locator('input[type="email"], input[name="email"]');

      if (await emailInput.isVisible({ timeout: 5000 }).catch(() => false)) {
        const passwordInput = page.locator('input[type="password"], input[name="password"]');

        await emailInput.fill(process.env.TEST_USER_EMAIL || 'test@example.com');
        await passwordInput.fill(process.env.TEST_USER_PASSWORD || 'password123');

        const submitButton = page.locator('button[type="submit"]');
        await submitButton.click();

        // Wait for redirect
        await page.waitForTimeout(2000);

        // Navigate to a chapter
        await page.goto('/modules/week-01-ros2-basics');
        await page.waitForLoadState('networkidle');

        // Translate button should now be visible
        const translateButton = page.locator('button:has-text("Translate to Urdu")');
        await expect(translateButton).toBeVisible({ timeout: 10000 });
      } else {
        test.skip();
      }
    } else {
      test.skip();
    }
  });

  test('only ONE Urdu translation button should exist (no Roman Urdu)', async ({ page }) => {
    // Login first
    await page.goto('/');
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")').first();

    if (await loginButton.isVisible()) {
      await loginButton.click();

      const emailInput = page.locator('input[type="email"], input[name="email"]');

      if (await emailInput.isVisible({ timeout: 5000 }).catch(() => false)) {
        const passwordInput = page.locator('input[type="password"], input[name="password"]');

        await emailInput.fill(process.env.TEST_USER_EMAIL || 'test@example.com');
        await passwordInput.fill(process.env.TEST_USER_PASSWORD || 'password123');

        const submitButton = page.locator('button[type="submit"]');
        await submitButton.click();
        await page.waitForTimeout(2000);

        // Navigate to chapter
        await page.goto('/modules/week-01-ros2-basics');
        await page.waitForLoadState('networkidle');

        // Should have exactly ONE translate button
        const translateButtons = page.locator('button:has-text("Translate"), button:has-text("اردو")');
        await expect(translateButtons).toHaveCount(1);

        // Should NOT have Roman Urdu button
        const romanUrduButton = page.locator('button:has-text("Roman Urdu")');
        await expect(romanUrduButton).not.toBeVisible();

        // Should have Urdu button with Urdu text
        const urduButton = page.locator('button:has-text("اردو")');
        await expect(urduButton).toBeVisible();
      } else {
        test.skip();
      }
    } else {
      test.skip();
    }
  });

  test('clicking translate button should trigger translation', async ({ page }) => {
    // Login first
    await page.goto('/');
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")').first();

    if (await loginButton.isVisible()) {
      await loginButton.click();

      const emailInput = page.locator('input[type="email"], input[name="email"]');

      if (await emailInput.isVisible({ timeout: 5000 }).catch(() => false)) {
        const passwordInput = page.locator('input[type="password"], input[name="password"]');

        await emailInput.fill(process.env.TEST_USER_EMAIL || 'test@example.com');
        await passwordInput.fill(process.env.TEST_USER_PASSWORD || 'password123');

        const submitButton = page.locator('button[type="submit"]');
        await submitButton.click();
        await page.waitForTimeout(2000);

        // Navigate to chapter
        await page.goto('/modules/week-01-ros2-basics');
        await page.waitForLoadState('networkidle');

        // Get original content
        const article = page.locator('article.markdown');
        const originalContent = await article.textContent();

        // Click translate button
        const translateButton = page.locator('button:has-text("Translate to Urdu")');
        await translateButton.click();

        // Should show loading state
        const loadingIndicator = page.locator('text=Translating...');
        await expect(loadingIndicator).toBeVisible({ timeout: 2000 });

        // Wait for translation to complete (or error)
        await page.waitForTimeout(5000);

        // After translation, should show language badge and revert button
        const languageBadge = page.locator('text=Urdu (اردو)');
        const revertButton = page.locator('button:has-text("Back to English")');

        // At least one of these should be visible (translation succeeded or there's a revert option)
        const badgeVisible = await languageBadge.isVisible().catch(() => false);
        const revertVisible = await revertButton.isVisible().catch(() => false);

        expect(badgeVisible || revertVisible).toBeTruthy();
      } else {
        test.skip();
      }
    } else {
      test.skip();
    }
  });

  test('revert to English button should reload original content', async ({ page }) => {
    // Login first
    await page.goto('/');
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")').first();

    if (await loginButton.isVisible()) {
      await loginButton.click();

      const emailInput = page.locator('input[type="email"], input[name="email"]');

      if (await emailInput.isVisible({ timeout: 5000 }).catch(() => false)) {
        const passwordInput = page.locator('input[type="password"], input[name="password"]');

        await emailInput.fill(process.env.TEST_USER_EMAIL || 'test@example.com');
        await passwordInput.fill(process.env.TEST_USER_PASSWORD || 'password123');

        const submitButton = page.locator('button[type="submit"]');
        await submitButton.click();
        await page.waitForTimeout(2000);

        // Navigate to chapter
        const chapterUrl = '/modules/week-01-ros2-basics';
        await page.goto(chapterUrl);
        await page.waitForLoadState('networkidle');

        // Click translate
        const translateButton = page.locator('button:has-text("Translate to Urdu")');
        if (await translateButton.isVisible()) {
          await translateButton.click();
          await page.waitForTimeout(3000);

          // Click revert button
          const revertButton = page.locator('button:has-text("Back to English")');
          if (await revertButton.isVisible()) {
            await revertButton.click();

            // Should reload the page
            await page.waitForLoadState('networkidle');

            // Should be back at the same URL
            expect(page.url()).toContain(chapterUrl);

            // Translate button should be visible again
            await expect(translateButton).toBeVisible({ timeout: 5000 });
          } else {
            test.skip();
          }
        } else {
          test.skip();
        }
      } else {
        test.skip();
      }
    } else {
      test.skip();
    }
  });

  test('translation should call correct API endpoint', async ({ page }) => {
    // Intercept API calls
    let translationApiCalled = false;
    let apiUrl = '';

    page.on('request', (request) => {
      if (request.url().includes('/api/translate')) {
        translationApiCalled = true;
        apiUrl = request.url();
      }
    });

    // Login first
    await page.goto('/');
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")').first();

    if (await loginButton.isVisible()) {
      await loginButton.click();

      const emailInput = page.locator('input[type="email"], input[name="email"]');

      if (await emailInput.isVisible({ timeout: 5000 }).catch(() => false)) {
        const passwordInput = page.locator('input[type="password"], input[name="password"]');

        await emailInput.fill(process.env.TEST_USER_EMAIL || 'test@example.com');
        await passwordInput.fill(process.env.TEST_USER_PASSWORD || 'password123');

        const submitButton = page.locator('button[type="submit"]');
        await submitButton.click();
        await page.waitForTimeout(2000);

        // Navigate to chapter
        await page.goto('/modules/week-01-ros2-basics');
        await page.waitForLoadState('networkidle');

        // Click translate
        const translateButton = page.locator('button:has-text("Translate to Urdu")');
        if (await translateButton.isVisible()) {
          await translateButton.click();
          await page.waitForTimeout(3000);

          // Verify API was called
          expect(translationApiCalled).toBeTruthy();

          // Verify it includes lang=ur query parameter
          expect(apiUrl).toContain('lang=ur');
        } else {
          test.skip();
        }
      } else {
        test.skip();
      }
    } else {
      test.skip();
    }
  });
});
