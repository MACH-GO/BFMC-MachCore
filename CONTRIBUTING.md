# MACH::GO Team Workflow (Required)

These rules exist to keep `dev` and `main` stable and prevent integration disasters.

## 1 - Branching Rules

### Protected branches

- `main`: always stable / release-ready.
- `dev`: integration branch (latest working team version).

### You MUST NOT

- Push directly to `main` or `dev`.
- Force-push to shared branches.
- Merge your own PR without required approvals.

### You MUST

- Create a new branch for every task, branched from `dev`.
- Remember to pull new changes from upstream to avoid merge conflicts.

**Branch naming:**

- `feat/<module>-<short-desc>` (new features)
- `fix/<module>-<short-desc>` (bug fixes)
- `refactor/<module>-<short-desc>` (refactor code)

**Examples:**

- `feat/vision-stop-sign`
- `fix/control-steering-clamp`
- `refactor/planning-algorithm`

## 2 - Commit Message Rules

We use **Conventional Commits**:

**Format:**

```
type(scope): short summary
```

**Allowed types:**  
`feat`, `fix`, `refactor`, `test`, `docs`, `build`

**Scopes (use what fits):**  
`control`, `planning`, `vision`, `dashboard`, `hardware`, `sim`

**Examples:**

- `feat(vision): add stop sign detection node`
- `fix(control): clamp steering command range`

## 3 - Pull Request Rules

All changes MUST come via Pull Request targeting:

- `dev` (normal work)
- `main` (only releases/hotfixes)

PR title MUST follow the same format as commits:

```
type(scope): short summary
```

PR description MUST:

- Use the provided PR template
- Include testing performed
- Link the issue if one exists (e.g., `Closes #12`)

Keep PRs small and focused:

- Prefer < ~300 changed lines where possible
- Split big changes into multiple PRs

## 4 - Review Rules

A PR may be merged only if:

- Required approvals are met
- All review conversations are resolved

**Minimum approvals:**

- PR → `dev`: 1 approval
- PR → `main`: 2 approvals (or team lead)

**You MUST NOT** "approve and merge" without testing.

## 5 - Merge Rules

We use **Squash merge**.

That means:

- 1 PR becomes 1 commit in `dev`/`main`
- The final squash commit message should match the PR title

**After merge:**

- Delete the branch (auto-delete is enabled)

## Hotfix Policy

If a critical bug is found on a stable build:

1. Create branch from `main`: `hotfix/<module>-<desc>`
2. Fix + test
3. PR into `main`
4. Back-merge the hotfix into `dev` (so branches don't diverge)
