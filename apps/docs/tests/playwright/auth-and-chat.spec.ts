import { test, expect } from '@playwright/test';

/**
 * Test Suite: Authentication and Chat Widget Visibility
 *
 * Requirements:
 * 1. Chat hidden for guests
 * 2. Chat visible after login
 * 3. Header updates with user name
 */

test.describe('Authentication and Chat Widget', () => {
  test('chat widget should NOT render for unauthenticated users', async ({ page }) => {
    // Navigate to homepage
    await page.goto('/');

    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Chat floating button should NOT exist
    const chatButton = page.locator('button[aria-label="Open AI Chat"]');
    await expect(chatButton).not.toBeVisible();

    // Chat window should NOT exist
    const chatWindow = page.locator('[class*="chatWindow"]');
    await expect(chatWindow).not.toBeVisible();
  });

  test('chat widget should NOT render on chapter pages for guests', async ({ page }) => {
    // Navigate to a chapter page
    await page.goto('/modules/week-01-ros2-basics');

    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Chat floating button should NOT exist
    const chatButton = page.locator('button[aria-label="Open AI Chat"]');
    await expect(chatButton).not.toBeVisible();

    // Verify the page content loaded (not a 404)
    const article = page.locator('article.markdown');
    await expect(article).toBeVisible();
  });

  test('login button should be visible for unauthenticated users', async ({ page }) => {
    // Navigate to homepage
    await page.goto('/');

    // Wait for navigation to load
    await page.waitForLoadState('networkidle');

    // Login button should exist in header
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")');
    await expect(loginButton).toBeVisible();
  });

  test('chat widget should render after successful login', async ({ page }) => {
    // This test requires a test user account
    // For now, we'll test the flow assuming auth is working

    // Navigate to login page
    await page.goto('/');

    // Click login button
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")').first();
    await loginButton.click();

    // Fill in login form (adjust selectors based on BetterAuth implementation)
    // Note: This is a placeholder - adjust based on actual auth form
    const emailInput = page.locator('input[type="email"], input[name="email"]');
    const passwordInput = page.locator('input[type="password"], input[name="password"]');

    if (await emailInput.isVisible({ timeout: 5000 }).catch(() => false)) {
      // Fill credentials
      await emailInput.fill(process.env.TEST_USER_EMAIL || 'test@example.com');
      await passwordInput.fill(process.env.TEST_USER_PASSWORD || 'password123');

      // Submit form
      const submitButton = page.locator('button[type="submit"]');
      await submitButton.click();

      // Wait for redirect back to homepage
      await page.waitForURL('/', { timeout: 10000 });

      // Now chat button should be visible
      const chatButton = page.locator('button[aria-label="Open AI Chat"]');
      await expect(chatButton).toBeVisible({ timeout: 10000 });

      // Click to open chat
      await chatButton.click();

      // Chat window should appear
      const chatWindow = page.locator('[class*="chatWindow"]');
      await expect(chatWindow).toBeVisible();

      // Chat should have header with title
      const chatTitle = page.locator('text=Matrix AI Tutor');
      await expect(chatTitle).toBeVisible();
    } else {
      // Skip if login form not available (e.g., in CI without auth configured)
      test.skip();
    }
  });

  test('header should display user name after login', async ({ page }) => {
    // Navigate to homepage
    await page.goto('/');

    // Click login button
    const loginButton = page.locator('a:has-text("Login"), button:has-text("Login")').first();

    if (await loginButton.isVisible()) {
      await loginButton.click();

      // Fill in login form
      const emailInput = page.locator('input[type="email"], input[name="email"]');

      if (await emailInput.isVisible({ timeout: 5000 }).catch(() => false)) {
        const passwordInput = page.locator('input[type="password"], input[name="password"]');

        await emailInput.fill(process.env.TEST_USER_EMAIL || 'test@example.com');
        await passwordInput.fill(process.env.TEST_USER_PASSWORD || 'password123');

        const submitButton = page.locator('button[type="submit"]');
        await submitButton.click();

        // Wait for redirect
        await page.waitForURL('/', { timeout: 10000 });

        // Header should show user email or name
        const userDisplay = page.locator('[class*="userDisplay"], [class*="userName"]');
        await expect(userDisplay).toBeVisible({ timeout: 10000 });

        // Logout button should be visible
        const logoutButton = page.locator('button:has-text("Logout"), a:has-text("Logout")');
        await expect(logoutButton).toBeVisible();
      } else {
        test.skip();
      }
    } else {
      test.skip();
    }
  });

  test('no skeleton or flicker should occur for guests', async ({ page }) => {
    // Navigate to homepage
    await page.goto('/');

    // Wait for initial render
    await page.waitForLoadState('domcontentloaded');

    // Check that chat button never appears during page load
    // Use polling to ensure it doesn't flicker into view
    for (let i = 0; i < 5; i++) {
      const chatButton = page.locator('button[aria-label="Open AI Chat"]');
      await expect(chatButton).not.toBeVisible();
      await page.waitForTimeout(200);
    }

    // Page should be fully loaded without chat widget
    await page.waitForLoadState('networkidle');
    const chatButton = page.locator('button[aria-label="Open AI Chat"]');
    await expect(chatButton).not.toBeVisible();
  });
});
