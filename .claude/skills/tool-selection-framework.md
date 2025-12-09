# tool-selection-framework Skill

## Purpose
Systematic framework for evaluating and selecting technologies, libraries, and tools.

## Key Patterns

### Decision Matrix Template
```markdown
## Tool Selection: {Category}

**Options Considered**: {Tool A}, {Tool B}, {Tool C}

| Criteria | Weight | Tool A | Tool B | Tool C |
|----------|--------|--------|--------|--------|
| Performance | 25% | 8/10 | 6/10 | 9/10 |
| Documentation | 20% | 9/10 | 7/10 | 8/10 |
| Community Support | 15% | 10/10 | 5/10 | 7/10 |
| Learning Curve | 15% | 6/10 | 9/10 | 7/10 |
| License | 10% | 10/10 | 10/10 | 8/10 |
| Cost | 15% | 10/10 | 8/10 | 6/10 |
| **Total** | 100% | **8.55** | **7.45** | **7.85** |

**Decision**: Choose Tool A (highest score)

**Rationale**: Superior performance and documentation outweigh moderate learning curve.
```

### Key Evaluation Criteria

**Technical Fit**:
- Meets functional requirements?
- Integrates with existing stack?
- Performance acceptable?
- Scalability requirements?

**Non-Technical Factors**:
- License compatible? (MIT, Apache 2.0 preferred)
- Active maintenance? (commits in last 3 months)
- Community size? (GitHub stars, Stack Overflow questions)
- Documentation quality? (API docs, tutorials, examples)

**Risk Assessment**:
- Vendor lock-in risk?
- Breaking change frequency?
- Security track record?
- Bus factor? (key maintainers)

### Example: Gemini vs OpenAI Selection

```markdown
## LLM Selection for RAG Chatbot

**Context**: Need LLM for chat completions and embeddings

| Criteria | Weight | Gemini | OpenAI |
|----------|--------|--------|---------|
| Cost | 30% | 10/10 (free tier) | 6/10 ($) |
| API Compatibility | 25% | 9/10 (OpenAI SDK) | 10/10 (native) |
| Embedding Dimensions | 20% | 8/10 (768-dim) | 10/10 (1536-dim) |
| Rate Limits | 15% | 7/10 (60 RPM) | 9/10 (500 RPM) |
| Latency | 10% | 8/10 (~1s) | 9/10 (~0.5s) |
| **Total** | 100% | **8.75** | **8.65** |

**Decision**: Gemini (marginally better due to cost)

**Key Constraint**: OpenAI SDK compatibility required for openai-agents-sdk

**Solution**: Use Gemini via OpenAI SDK drop-in replacement
```

## Usage Context
- Architecture decisions
- Technology evaluation
- Library selection
- Build vs buy decisions
