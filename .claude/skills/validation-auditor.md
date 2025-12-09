# validation-auditor Skill

## Purpose
Comprehensive validation of content, format, UI, and functionality.

## Key Patterns

### Content Validation Checklist
```markdown
## Content Quality
- [ ] All 13 weeks have complete content (400+ lines each)
- [ ] Learning objectives are measurable and clear
- [ ] Code examples are tested and runnable
- [ ] Mermaid diagrams render correctly
- [ ] Images have alt text
- [ ] Links are not broken
- [ ] Prerequisites are stated
- [ ] Key takeaways summarize concepts

## Format Validation
- [ ] Frontmatter complete on all pages
- [ ] Title hierarchy correct (H1 → H2 → H3)
- [ ] Code blocks specify language
- [ ] Docusaurus Tabs properly imported
- [ ] Custom components render correctly

## UI/UX Validation
- [ ] Sidebar navigation works
- [ ] Search functionality works
- [ ] Mobile responsive design
- [ ] Dark mode works correctly
- [ ] Code syntax highlighting correct
```

### Playwright MCP Testing Pattern
```typescript
// Use Playwright MCP for UI testing
test('Week 1 content renders', async ({ page }) => {
  await page.goto('/week-01-ros2-basics');
  await expect(page.locator('h1')).toContainText('Week 1');
  await expect(page.locator('.tabs')).toBeVisible();
  await expect(page.locator('pre code')).toHaveCount(5); // 5 code blocks
});

test('Personalize button works', async ({ page }) => {
  await page.goto('/week-01-ros2-basics');
  await page.click('button:has-text("Personalize")');
  await expect(page.locator('.loading')).toBeVisible();
});
```

## Usage Context
- Pre-deployment validation
- CI/CD quality gates
- Content review
- UI testing
