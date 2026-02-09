# Agent development guide

This guide provides comprehensive instructions for AI agents working on the desperado project.

## Project structure

The project is organized as a Rust workspace with three specialized crates:

```
desperado/
├── crates/
│   ├── desperado/            # Core I/Q streaming and DSP primitives
│   ├── fmradio/              # FM radio receiver and RDS decoder
│   └── rs-spy/               # Airspy R2/Mini/HF+ hardware driver (libairspy in Rust)
│
├── docs/                     # MkDocs documentation (deployed to mode-s.org/desperado)
├── plan.md                   # Development roadmap and milestone tracking
├── agents.md                 # This file - agent development guidelines
├── lessons_learned.md        # Project history, mistakes, and best practices
└── README.md
```

### Crate Responsibilities

**desperado** - Core I/Q streaming
- Generic I/Q sample source abstraction
- Core DSP building blocks for signal processing
- Hardware-agnostic processing pipeline
- Example: `examples/file_iter.rs` (read raw I/Q from file)

**fmradio** - FM radio processing stack
- FM demodulation (stereo, de-emphasis)
- RDS data decoding (station name, radiotext)
- Adaptive audio resampling with PI controller
- Binary application for streaming FM radio to audio
- Depends on: desperado (for I/Q sources)

**rs-spy** - Airspy R2/Mini/HF+ driver
- Pure-Rust alternative to `libairspy` (no C dependencies)
- USB control and streaming via `rusb`
- Gain and frequency control
- Streaming at 6-10 MSPS
- Example apps: `airspy_info`, `airspy_tune`, `airspy_rx`
- Standalone (can be used without desperado/fmradio)


## Setup and build

### Initial build

```sh
# Development build (thin LTO, faster: ~47s incremental, keeps symbols)
cargo build --release --all-features

# Distribution build (full LTO, optimal binary: ~94s incremental, stripped)
cargo build --profile dist --all-features
```

**Build profiles:**
- `--release`: Thin LTO for fast development iteration (~15-16 MB with symbols, 47s incremental)
- `--profile dist`: Full LTO for production releases (12 MB stripped, 94s incremental)
  - Used automatically by `cargo dist` for releases

### Building specific components

```sh
# Core crates
cargo build -p desperado --release       # I/Q streaming and DSP primitives
cargo build -p fmradio --release         # FM radio receiver
cargo build -p rs-spy --release          # Airspy hardware driver

# Applications/binaries
cargo build -p fmradio --release --bin fmradio    # FM radio CLI app
cargo run --example airspy_info                    # Airspy device info
cargo run --example airspy_tune                    # Airspy tuning/configuration
cargo run --example airspy_rx                      # Airspy streaming to file

# All at once
cargo build --release --workspace
```

## Testing

### Rust tests

```sh
# Run all tests (workspace-wide)
cargo test --lib --workspace

# Run tests for specific crate
cargo test -p desperado --lib
cargo test -p fmradio --lib
cargo test -p rs-spy --lib

# Run specific test
cargo test test_name -- --nocapture

# Test count
cargo test --lib --workspace 2>&1 | tail -5  # Shows summary
```

## Code quality and style

### Rust

**Linting:**

```sh
cargo clippy --workspace --all-targets --all-features -- -D warnings
```

**Formatting:**

```sh
cargo fmt --all              # Format all code
cargo fmt --all --check      # Check without modifying
```

**Documentation:**

```sh
cargo doc --all-features --no-deps        # Build docs
cargo doc --all-features --no-deps --open # Build and open in browser

# Check for documentation issues
RUSTDOCFLAGS="-D rustdoc::all -A rustdoc::private-doc-tests" cargo doc --all-features --no-deps
```

### Markdown

- Use `prettier` for formatting documentation and markdown files
- Follow CommonMark specification

### Code conventions

**Rust:**

- Use descriptive variable names 
- Document public APIs with `///` doc comments
- Use `tracing` for logging, not `println!`
- Handle errors with `Result<T, E>`, avoid unwrap in library code
- Use `#[must_use]` for important return values

## Documentation

### Building documentation

**MkDocs site (jet1090 user docs):**

```sh
uvx --with "mkdocs-material[imaging]" mkdocs serve  # Local preview
uvx --with "mkdocs-material[imaging]" mkdocs build -d site  # Build static site
```

Site deploys automatically to https://mode-s.org/desperado on push to master.

**Rust API docs:**

```sh
cargo doc --all-features --no-deps --open
```

Published automatically to https://docs.rs/desperado

### Documentation structure

- `plan.md`: Development roadmap, milestones, architecture overview
- `agents.md`: This file - guidelines for AI agent development
- `lessons_learned.md`: Project history, challenges, and best practices
- `docs/`: MkDocs markdown files (user guides, installation, configuration)
- `crates/*/src/`: Inline Rust documentation (extracted by rustdoc)

## Code analysis

- Put any markdown file with summaries and explanations in the analysis/ folder

## Release

- Ensure latest commmit on master has no failing CI actions
- `cargo release [patch,minor]`

## Git workflow and commits

### Branching strategy

- `master`: Main development branch (protected)
- Feature branches: `feature/description` or `fix/issue-number`
- Always create PRs for review, never push directly to master

### Commit guidelines

**IMPORTANT:**

- **Never commit without explicit user approval**
- If the user gives you approval for one commit, do not commit again later without explicit user approval.
- Always ask for confirmation before creating commits
- **Never commit .md files** - Markdown files are documentation and planning artifacts. They should be updated separately as needed, not bundled with code commits.
- If fixing a GitHub issue, create a dedicated branch and PR

**Commit message format:**

```
type: brief description (imperative mood)

Optional longer explanation of what changed and why.

Fixes #123
```

### GitHub issues and PRs

**Opening issues:**

```sh
# Never open issues without user acknowledgement
gh issue create --title "Title" --body "Description"
```

**Analyzing issues:**

```sh
# Always read ALL comments before planning
gh issue view 123
gh issue view 123 --comments
```

**Creating pull requests:**

```sh
# Never open PR without user acknowledgement
gh pr create --title "Title" --body "Description"

# Link to issue
gh pr create --title "Fix altitude bug" --body "Fixes #123"
```

Update changelog.md after fixing issues

## Task planning

### Using plan.md

- **Always** use `plan.md` to track complex tasks
- Update frequently as you work through tasks
- **CRITICAL:** Always include a final task item reminding yourself to get user approval before committing
- Structure:

  ```markdown
  ## Current task: [Brief description]

  - [ ] Step 1
  - [ ] Step 2
  - [x] Completed step
  - [ ] ⚠️ STOP: Get explicit user approval before committing

  ## Next:

  - Future tasks
  ```

- Prune completed tasks after commits are merged

### Lessons Learned Framework

**CRITICAL:** The repository maintains [`lessons_learned.md`](lessons_learned.md) to track mistakes, challenges, and best practices. This is essential for continuous improvement and avoiding repeated errors.

**Every todo list MUST include a task to maintain `lessons_learned.md`:**

```markdown
- [ ] Update lessons_learned.md with any new insights or mistakes encountered
```

**When creating a todo list:**
1. Always add "Update lessons_learned.md" as the final item (after "Get explicit user approval before committing")
2. Before starting work, review `lessons_learned.md` sections relevant to the current task
3. After completing each major task, add new lessons to the document

**Lessons Learned Structure:**
- Categorized by domain (Task Management, USB/Hardware, Testing, etc.)
- Each lesson includes: Mistake, Impact, Fix, Action Items
- Quick reference checklist at the end for common operations

**Examples from past work:**
- Lesson 1.1: Always use TodoWrite for complex tasks
- Lesson 3.1: USB interface lifecycle management (why device cleanup matters)
- Lesson 4.1: Test repeatability (single run vs multiple runs)
- Lesson 6.1: Understand reference implementation first (before porting)

### Task breakdown approach

1. **Understand the requirement** - Read issue, analyze code context, review relevant `lessons_learned.md` sections
2. **Plan steps** - Break into discrete, testable units:
    - Always include "Update lessons_learned.md" as second-to-last step
    - Always include "Get user approval before committing" as final step
3. **Execute incrementally** - Small commits, test frequently
4. **Verify** - Run tests, check lints, update docs
5. **Update documentation** - Maintain `lessons_learned.md` with insights from this work
6. **Review** - Self-review changes before proposing to user
7. **⚠️ CRITICAL RULE:** After the user provides compacted/summarized instructions, **IMMEDIATELY re-read the entire agents.md, plan.md, and lessons_learned.md files** to ensure you're operating under the current guidelines. This prevents inconsistent behavior after session context is compacted.
8. **⚠️ Get explicit user approval** - NEVER commit or create anything on GitHub without asking first

## Support and contributions

- Test thoroughly before proposing changes
- Document breaking changes clearly in PRs
