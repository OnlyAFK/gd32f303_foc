---
name: precommit
description: Pre-commit hygiene gate for this GD32 FOC firmware. Run ONCE before committing/pushing to review the software framework for degradation and update CHANGELOG.md (and ARCHITECTURE.md only if layering changed). Use when the user is about to commit/push or invokes /precommit. Do not run this every conversation turn.
---

# Pre-commit check

Run this as a single deliberate gate **before a commit** — not after every edit.
The goal: catch framework drift early and keep the docs clean without spamming
the changelog or wasting tokens on per-turn reviews. **Do not commit** at the
end; report and let the user commit.

**Output language: 用中文向用户汇报。** All prose reported to the user (summaries,
findings, verdict) must be in Chinese. Keep code/CHANGELOG comments in English per
STYLE.md, and keep the suggested git commit message in English imperative.

## 1. Scope the pending change
- `git status` and `git --no-pager diff HEAD` (include staged + unstaged).
- Summarize what actually changed (files, modules, intent) in 2-4 bullets.

## 2. Architecture exam (objective, from ARCHITECTURE.md)
Run these; any non-empty result in 2b/2c/2d is a layering leak to flag:
- 2a sizes: `wc -l app/*.c bsp/*.c driver/*/*.c foc/*.c | sort -n | tail` — flag any file > ~600 lines as a split candidate.
- 2b chip drivers above the HAL: `grep -rl "drv8323_\|tle5012b_" foc app bsp | grep -v "motor_hw.c\|interrupt.c\|gd32f30x_it.c"` (should be empty).
- 2c bsp leak: `grep -rn "read_vbus" foc app | grep -v motor_hw.c` (should be empty).
- 2d header globals: `grep -rn "extern" foc/*.h app/*.h bsp/*.h | grep -v 'extern \"C\"'` — mutable externs violate STYLE (note known: `vofa_data`, `adc_buff`/offsets).

## 3. Review the diff against the rules
Check the changed code only (not the whole tree) for:
- STYLE.md: English comments, 2-space K&R, snake_case, no magic numbers, no VLAs/malloc, ISR rules.
- ARCHITECTURE.md dependency red lines (only `motor_hw` includes chip/bsp headers; cross-cutting services consume abstractions).
- Dead code / stale comments / leftover debug or test scaffolding.
Report concrete findings with `file:line`. Offer fixes; apply only trivial,
clearly-correct ones (and re-run the relevant exam command).

## 4. Update CHANGELOG.md
Add ONE entry at the top (below the template) covering the whole pending change
set, using the template format (What / Why / How to use / Notes). Keep it tight.
Do not create multiple entries for one commit.

## 5. Update ARCHITECTURE.md (conditional)
Only if this change altered layering, module responsibilities, dependency rules,
or known debts. If the structure is unchanged, leave it alone.

## 6. Report (中文)
用中文给出简短结论：框架 OK / 出现漂移（列出具体的分层泄漏）、文档更新了什么、以及
建议的简洁 git commit message（英文，祈使句）。然后停下 —— 由用户自己执行 commit。
