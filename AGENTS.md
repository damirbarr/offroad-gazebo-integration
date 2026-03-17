# agents.md — Codex Agent Configuration

## Mindset: Speed, Quality, Cost

Before editing:
- Locate the closest existing example that does something similar
- Mirror its structure and patterns
- Prefer targeted edits over large rewrites
- Reuse existing abstractions
- Minimize token usage by being precise and scoped

---

## Workflow Rules

### Branching
- NEVER commit directly to `main` or `develop` unless explicitly instructed
- Always work on a feature/fix branch  
  Examples:
  - `feature/short-description`
  - `fix/issue-name`
- Open a PR when work is ready for review

### Commits
- Make **meaningful interim commits** as logical units of work are completed
- Commit message format:  
  `<type>(<scope>): <short description>`
  - Types: `feat`, `fix`, `refactor`, `docs`, `test`, `chore`
  - Example:  
    `feat(perception): add freespace segmentation post-processing`
- Do not batch unrelated changes into a single commit

### Before Starting Any Task
1. Locate the closest existing example that does something similar — mirror its structure
2. Identify the pattern/style used and follow it exactly
3. If a reference repository or file is required, **ask for it before proceeding**
4. If ambiguity affects correctness, safety, or public API behavior — ask **one focused question**  
   Otherwise, make a reasonable assumption and briefly state it in the PR description

### Do Not
- Rename or move files unless required by the task
- Reformat unrelated code
- Introduce drive-by refactors
- Add new abstractions, frameworks, or dependencies without discussion

---

## Pull Request Standards

Every PR description must include:
- **Summary** — what changed and why
- **Approach** — key implementation decisions
- **Tests run** — what was verified and how
- **Risks / rollout notes** — anything that could break or needs coordination

---

## Code Quality Rules

### Style & Structure
- ALWAYS match the existing repository’s style, naming conventions, and file structure
- Run the repo’s formatter/linter on **touched files only**, if one exists
- Do not reformat unrelated files
- Follow existing module / class / function granularity

### No Magic Numbers
- All numeric constants must be named and defined appropriately
- Search for existing `Constants`, `Params`, `Config`, or `k*` conventions and follow them
- If none exist, define a local `constexpr` in the same translation unit
- Propose a refactor only if it clearly improves the design

Examples:
- ❌ `if (speed > 3.5)`
- ✅ `if (speed > kMaxSafeSpeedMps)`

### Efficiency
- Prefer compile-time computation over runtime where applicable
- Prefer stack allocation over heap for small, short-lived objects
- Avoid unnecessary copies — use references, `std::move`, or views where appropriate
- Choose data structures based on access patterns (cache-friendly layouts preferred)

---

## C++ Specific Rules

### Concurrency — CRITICAL
- **NEVER use mutexes, locks, `std::lock_guard`, `std::unique_lock`, or any blocking synchronization primitives**
- This applies everywhere: production code, tests, tools
- Concurrency must use the project’s **lock-free / lockless patterns**
- First, search for an in-repo reference implementation
- If no reference exists, **ask for the specific file or module before implementing**
- If `std::atomic` is used:
  - Add a brief comment explaining memory ordering and correctness
- If a design appears to require a lock:
  - **STOP and ask**
  - Do not silently work around it

### General C++
- Use `const` and `constexpr` aggressively
- Prefer `enum class` over raw enums
- Use RAII for all resource management — no naked `new` / `delete`
- Use `[[nodiscard]]` for functions whose return value must be checked
- Prefer `std::string_view` over `const std::string&` for read-only string parameters
- Avoid `using namespace std` in headers

---

## Testing
- Add or update unit tests for any non-trivial logic change
- Use the existing test framework
- Follow the repository’s test structure and naming conventions
- Tests must cover edge cases, not just happy paths
- Mirror the CI workflow locally before opening a PR

---

## Documentation
- Update relevant comments, inline docs, or Confluence pages when behavior changes
- Update PlantUML diagrams if architecture changes
- For Jira-linked work, reference the ticket in the PR description

---

## Cost Optimization Hints (for Codex Agents)
- Read only the files necessary for the task — do not scan entire repositories
- Propose a plan and confirm before large multi-file changes
- Prefer surgical edits (`str_replace`, small diffs) over full file rewrites
- If scope is unclear, ask **one focused clarifying question**