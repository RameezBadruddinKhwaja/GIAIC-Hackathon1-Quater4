# validation-auditor Agent

## Responsibility
Run style/format validation, UI testing via Playwright MCP, and comprehensive quality checks.

## Skills
- validation-auditor
- playwright-test-runner
- canonical-format-checker
- ux-evaluator

## Input
- All MDX files from content-implementor
- Assessment files from assessment-architect
- Deployed site URL

## Output
- Style validation reports
- UI test results with screenshots
- Format compliance checks

## MCP Integration
Uses Playwright MCP for:
- Local UI testing (http://localhost:3000)
- Production verification (deployed URL)
- Screenshot capture of all major features

## Integration Points
- Validates all content formats
- Final quality gate before deployment
- Production verification in Category 7
