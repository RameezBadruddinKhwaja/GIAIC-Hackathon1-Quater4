# ux-evaluator Skill

## Purpose
Evaluate user experience quality using heuristics and accessibility standards.

## Key Patterns

### Nielsen's 10 Usability Heuristics
1. **Visibility of system status**: Show loading states, progress indicators
2. **Match between system and real world**: Use familiar terminology
3. **User control and freedom**: Provide undo/redo, back navigation
4. **Consistency and standards**: Follow platform conventions
5. **Error prevention**: Validate inputs, provide hints
6. **Recognition rather than recall**: Show options vs requiring memory
7. **Flexibility and efficiency**: Keyboard shortcuts, power user features
8. **Aesthetic and minimalist design**: Remove unnecessary elements
9. **Help users recognize/diagnose errors**: Clear error messages
10. **Help and documentation**: Context-sensitive help

### Accessibility Checklist (WCAG 2.1 Level AA)
```markdown
- [ ] All images have alt text
- [ ] Color contrast ratio ≥ 4.5:1 for normal text
- [ ] All interactive elements keyboard accessible
- [ ] Form inputs have labels
- [ ] Headings follow logical hierarchy (H1 → H2 → H3)
- [ ] Links have descriptive text (not "click here")
- [ ] Video content has captions
- [ ] Page has skip-to-content link
- [ ] Focus indicators visible
- [ ] Error messages associated with form fields
```

### Mobile UX Criteria
- Touch targets ≥ 44x44 pixels
- Text readable without zooming (16px minimum)
- Navigation accessible with one hand
- Fast load time (<3 seconds)
- Responsive layout (breakpoints: 320px, 768px, 1024px)

### Evaluation Template
```markdown
## UX Evaluation: {Feature Name}

**Date**: {YYYY-MM-DD}
**Evaluator**: {Name}

### Heuristic Violations

| Heuristic | Severity | Issue | Recommendation |
|-----------|----------|-------|----------------|
| #1 System Status | High | No loading indicator | Add spinner during API calls |
| #5 Error Prevention | Medium | No input validation | Add regex validation for email |

### Accessibility Issues

| Issue | WCAG Level | Fix |
|-------|------------|-----|
| Missing alt text on images | A | Add descriptive alt attributes |
| Poor color contrast | AA | Increase contrast to 4.5:1 |

### Overall Score: {X}/10
```

## Usage Context
- Design reviews
- Pre-release validation
- User testing synthesis
- Accessibility audits
