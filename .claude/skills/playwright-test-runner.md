# playwright-test-runner Skill

## Purpose
Execute UI tests using Playwright MCP for validation and regression testing.

## Key Patterns

### Playwright MCP Integration
Use Playwright MCP tools from Claude Code for automated UI testing:

```typescript
// Navigate and snapshot
await mcp__playwright__browser_navigate({ url: 'http://localhost:3000' });
await mcp__playwright__browser_snapshot({});

// Click element
await mcp__playwright__browser_click({
  element: 'Personalize button',
  ref: 'button[data-testid="personalize-btn"]'
});

// Type and submit
await mcp__playwright__browser_type({
  element: 'Search box',
  ref: 'input[type="search"]',
  text: 'ROS 2 basics'
});

// Verify network request
const requests = await mcp__playwright__browser_network_requests({});
console.log(requests.filter(r => r.url.includes('/api/personalize')));

// Take screenshot
await mcp__playwright__browser_take_screenshot({
  filename: 'homepage.png',
  fullPage: true
});
```

### Test Scenarios
```markdown
## Critical User Flows

1. **Content Navigation**
   - Navigate to Week 1
   - Verify content loads
   - Check sidebar navigation
   - Test internal links

2. **Personalization**
   - Click Personalize button
   - Select hardware profile
   - Verify API call
   - Check content updates

3. **Translation**
   - Click Translate button
   - Select Urdu
   - Verify code blocks preserved
   - Check UI updates

4. **RAG Chatbot**
   - Open chat widget
   - Send query
   - Verify response
   - Check citations
```

### Validation Checks
- ✅ All pages return 200 status
- ✅ No console errors
- ✅ Responsive design on mobile
- ✅ Dark mode works
- ✅ Code blocks have proper syntax highlighting
- ✅ Mermaid diagrams render
- ✅ Search returns results

## Usage Context
- Pre-deployment validation
- CI/CD pipeline
- Regression testing
- User acceptance testing
