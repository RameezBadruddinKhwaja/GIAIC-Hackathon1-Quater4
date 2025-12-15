import { test, expect } from '@playwright/test';

/**
 * Test Suite: MDX Rendering Validation
 *
 * Requirements:
 * - No MDX errors across all pages
 * - No red codeblock errors
 * - No invalid JSX rendering
 * - All headings render correctly
 * - All admonitions render correctly
 */

// List of all chapter routes to test
const CHAPTER_ROUTES = [
  '/modules/week-01-ros2-basics',
  '/modules/week-02-advanced-ros2',
  '/modules/week-03-gazebo-simulation',
  '/modules/week-04-unity-integration',
  '/modules/week-05-nvidia-isaac',
  '/modules/week-06-isaac-sim',
  '/modules/week-07-isaac-ros',
  '/modules/week-08-nav2',
  '/modules/week-09-vla',
  '/modules/week-10-humanoid-kinematics',
  '/modules/week-11-humanoid-dynamics',
  '/modules/week-12-natural-interaction',
  '/modules/week-13-conversational-robotics',
];

test.describe('MDX Rendering - Error Detection', () => {
  for (const route of CHAPTER_ROUTES) {
    test(`${route} should render without console errors`, async ({ page }) => {
      const consoleErrors: string[] = [];
      const consoleWarnings: string[] = [];

      // Listen for console errors
      page.on('console', (msg) => {
        if (msg.type() === 'error') {
          consoleErrors.push(msg.text());
        } else if (msg.type() === 'warning') {
          consoleWarnings.push(msg.text());
        }
      });

      // Listen for page errors
      page.on('pageerror', (error) => {
        consoleErrors.push(`Page Error: ${error.message}`);
      });

      // Navigate to chapter
      const response = await page.goto(route, { waitUntil: 'networkidle' });

      // Verify page loaded (not 404)
      expect(response?.status()).toBeLessThan(400);

      // Verify no React/MDX rendering errors
      const reactErrors = consoleErrors.filter(
        (err) =>
          err.includes('React') ||
          err.includes('MDX') ||
          err.includes('Uncaught') ||
          err.includes('TypeError')
      );

      if (reactErrors.length > 0) {
        console.error(`Errors found on ${route}:`, reactErrors);
      }

      expect(reactErrors).toHaveLength(0);
    });

    test(`${route} should have valid article structure`, async ({ page }) => {
      await page.goto(route, { waitUntil: 'networkidle' });

      // Main article should exist
      const article = page.locator('article.markdown, article[role="main"], main article');
      await expect(article).toBeVisible();

      // Article should have content (not empty)
      const articleText = await article.textContent();
      expect(articleText?.length).toBeGreaterThan(100);
    });

    test(`${route} should render all headings correctly`, async ({ page }) => {
      await page.goto(route, { waitUntil: 'networkidle' });

      // Check for headings
      const headings = page.locator('article h1, article h2, article h3, article h4');
      const headingCount = await headings.count();

      // Should have at least one heading (usually the title)
      expect(headingCount).toBeGreaterThan(0);

      // All headings should be visible and have text
      for (let i = 0; i < Math.min(headingCount, 10); i++) {
        const heading = headings.nth(i);
        await expect(heading).toBeVisible();

        const headingText = await heading.textContent();
        expect(headingText?.trim().length).toBeGreaterThan(0);
      }
    });

    test(`${route} should render code blocks without errors`, async ({ page }) => {
      await page.goto(route, { waitUntil: 'networkidle' });

      // Find all code blocks
      const codeBlocks = page.locator('pre code, .prism-code');
      const codeBlockCount = await codeBlocks.count();

      if (codeBlockCount > 0) {
        // Check first few code blocks
        for (let i = 0; i < Math.min(codeBlockCount, 5); i++) {
          const codeBlock = codeBlocks.nth(i);

          // Code block should be visible
          await expect(codeBlock).toBeVisible();

          // Should not have error styling
          const hasErrorClass = await codeBlock.evaluate((el) => {
            return el.classList.contains('error') || el.classList.contains('invalid');
          });
          expect(hasErrorClass).toBe(false);
        }
      }
    });

    test(`${route} should render admonitions correctly`, async ({ page }) => {
      await page.goto(route, { waitUntil: 'networkidle' });

      // Check for Docusaurus admonitions
      const admonitions = page.locator('.admonition, [class*="admonition"]');
      const admonitionCount = await admonitions.count();

      if (admonitionCount > 0) {
        // All admonitions should render properly
        for (let i = 0; i < Math.min(admonitionCount, 3); i++) {
          const admonition = admonitions.nth(i);
          await expect(admonition).toBeVisible();

          // Should have content
          const admonitionText = await admonition.textContent();
          expect(admonitionText?.trim().length).toBeGreaterThan(0);
        }
      }
    });
  }
});

test.describe('MDX Rendering - Homepage and Navigation', () => {
  test('homepage should render without errors', async ({ page }) => {
    const consoleErrors: string[] = [];

    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        consoleErrors.push(msg.text());
      }
    });

    page.on('pageerror', (error) => {
      consoleErrors.push(`Page Error: ${error.message}`);
    });

    await page.goto('/', { waitUntil: 'networkidle' });

    // Filter for critical errors only
    const criticalErrors = consoleErrors.filter(
      (err) =>
        err.includes('React') ||
        err.includes('MDX') ||
        err.includes('Uncaught') ||
        err.includes('Failed to fetch')
    );

    expect(criticalErrors).toHaveLength(0);
  });

  test('navigation sidebar should render all module links', async ({ page }) => {
    await page.goto('/', { waitUntil: 'networkidle' });

    // Find sidebar navigation
    const sidebar = page.locator('[class*="sidebar"], nav[class*="menu"]');

    if (await sidebar.isVisible()) {
      // Should have module links
      const moduleLinks = page.locator('a[href*="/modules/"]');
      const linkCount = await moduleLinks.count();

      expect(linkCount).toBeGreaterThan(0);
    }
  });

  test('all internal links should be valid (no 404s)', async ({ page }) => {
    await page.goto('/', { waitUntil: 'networkidle' });

    // Get all internal links
    const internalLinks = await page.locator('a[href^="/"]').evaluateAll((links) =>
      links
        .map((link) => (link as HTMLAnchorElement).href)
        .filter((href) => !href.includes('#') && !href.includes('mailto:'))
        .slice(0, 10) // Test first 10 links only
    );

    // Check each link
    for (const link of internalLinks) {
      const response = await page.goto(link, { waitUntil: 'domcontentloaded' });
      expect(response?.status()).toBeLessThan(400);
    }
  });
});

test.describe('MDX Rendering - Specific Error Patterns', () => {
  test('should not have unescaped curly braces in text', async ({ page }) => {
    // Test a sample chapter
    await page.goto('/modules/week-01-ros2-basics', { waitUntil: 'networkidle' });

    // Get article HTML
    const article = page.locator('article');
    const html = await article.innerHTML();

    // Should not have raw {{ or }} in rendered text (indicates unescaped JSX)
    // Note: This might be in code blocks, which is fine
    const hasUnescapedBraces = html.match(/(?<!<code[^>]*>){{|}}(?![^<]*<\/code>)/);

    // If found, it's likely an MDX error
    expect(hasUnescapedBraces).toBeNull();
  });

  test('should not have broken JSX component imports', async ({ page }) => {
    const consoleErrors: string[] = [];

    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        consoleErrors.push(msg.text());
      }
    });

    await page.goto('/modules/week-01-ros2-basics', { waitUntil: 'networkidle' });

    // Check for import errors
    const importErrors = consoleErrors.filter(
      (err) => err.includes('import') || err.includes('module not found')
    );

    expect(importErrors).toHaveLength(0);
  });

  test('should not have invalid Markdown syntax errors', async ({ page }) => {
    const consoleErrors: string[] = [];

    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        consoleErrors.push(msg.text());
      }
    });

    await page.goto('/modules/week-01-ros2-basics', { waitUntil: 'networkidle' });

    // Check for parsing errors
    const parsingErrors = consoleErrors.filter(
      (err) =>
        err.toLowerCase().includes('parse') ||
        err.toLowerCase().includes('syntax') ||
        err.toLowerCase().includes('unexpected token')
    );

    expect(parsingErrors).toHaveLength(0);
  });
});
