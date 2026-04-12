# Agent development guide

This guide provides comprehensive instructions for AI agents working on the desperado project and its dependent ecosystems.

## Project structure

The project is organized as a Rust workspace with specialized crates for DSP, hardware drivers, and domain-specific decoders:

```
desperado/
├── crates/
│   ├── dabradio/             # DAB/DAB+ decoder (OFDM, FIC/FIB, AAC audio)
│   ├── desperado/            # Core I/Q streaming and DSP primitives
│   ├── fmradio/              # FM radio receiver and RDS decoder
│   ├── rs-rtl/               # RTL-SDR (RTL2832U) hardware driver (librtlsdr in Rust)
│   ├── rs-spy/               # Airspy R2/Mini/HF+ hardware driver (libairspy in Rust)
│   └── voracious/            # VOR/ILS/DME aviation navigation decoder
├── analysis/                 # Analysis documents and refactoring recommendations
├── docs/                     # MkDocs documentation (deployed to mode-s.org/desperado)
├── plan.md                   # Development roadmap and milestone tracking
├── agents.md                 # This file - agent development guidelines
├── lessons_learned.md        # Project history, mistakes, and best practices
└── README.md
```

### Dependent Projects (Ecosystem)

Desperado serves as a core I/Q streaming and DSP library for the following projects in the same workspace:

| Project      | Location       | Dependency                                       | Purpose                | Risk Level |
| ------------ | -------------- | ------------------------------------------------ | ---------------------- | ---------- |
| **jet1090**  | `../jet1090/`  | `desperado` I/Q source abstraction               | ADS-B Mode-S decoder   | LOW        |
| **ship162**  | `../ship162/`  | `desperado` I/Q source abstraction               | AIS Maritime receiver  | LOW        |
| **datalink** | `../datalink/` | `desperado::dsp::*` (Chebyshev2Lpf, Nco, EveryN) | VDL2/ARINC 629 decoder | HIGH       |

**Critical Note for Refactoring:** Any changes to desperado DSP modules (especially Chebyshev2Lpf, Nco, EveryN) must be tested with datalink samples before committing. Changes to IqAsyncSource/IqSource traits may affect jet1090 and ship162.

### Crate Responsibilities

**dabradio** - DAB/DAB+ decoder

- OFDM demodulation for DAB/DAB+ broadcast reception
- FIC (Fast Information Channel) and FIB (Fast Information Block) parsing
- AAC audio decoding and playback
- Depends on: desperado (for I/Q sources)

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

**rs-rtl** - RTL-SDR (RTL2832U) hardware driver

- Pure-Rust alternative to `librtlsdr` (no C dependencies)
- USB control and streaming via `rusb`
- Frequency and gain control
- Sample rate configuration and streaming
- Standalone (can be used without desperado/fmradio)

**rs-spy** - Airspy R2/Mini/HF+ driver

- Pure-Rust alternative to `libairspy` (no C dependencies)
- USB control and streaming via `rusb`
- Gain and frequency control
- Streaming at 6-10 MSPS
- Example apps: `airspy_info`, `airspy_tune`, `airspy_rx`
- Standalone (can be used without desperado/fmradio)

**voracious** - VOR/ILS/DME aviation navigation decoder

- VOR (VHF Omnidirectional Range) receiver and bearing calculation
- ILS (Instrument Landing System) localizer and glideslope decoder
- DME (Distance Measuring Equipment) range calculation
- Aviation navigation signal processing and demodulation
- Depends on: desperado (for I/Q sources)

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

## I/Q File Validation Testing

**CRITICAL:** Before committing any changes that affect DSP modules, I/Q sources, or signal processing, validate that all decoders can successfully process real-world I/Q files. This is the ultimate integration test.

**Test Files Location:** `~/Documents/data/samples/`

| File                                                                       | Project   | Purpose                                            | Command                                                    |
| -------------------------------------------------------------------------- | --------- | -------------------------------------------------- | ---------------------------------------------------------- |
| `decode_adsb/gqrx_20260127_141546_1090000000_6000000_fc.raw`               | jet1090   | ADS-B Mode-S decoding (1090 MHz)                   | `jet1090 file`                                             |
| `decode_ais/ais_paris_162m_288k.bin`                                       | ship162   | AIS Maritime receiver (162 MHz)                    | `ship162 file`                                             |
| `decode_dab/dab_12A_2min.iq`                                               | dabradio  | DAB+ OFDM and audio decode (Band III, Channel 12A) | `dabradiod --input file`                                   |
| `decode_fm/rtlsdr_20210215_103300000_1102500_cu8.bin`                      | fmradio   | FM stereo + RDS decoding (110.25 MHz)              | `fmradio --center-freq 110.25M --sample-rate 1102500 file` |
| `decode_datalink/rtlsdr_136850000_1050000_dumpvdl_6min.rtl`                | datalink  | VDL2 aviation datalink decoder (136.85 MHz)        | `dumpvdl2 -r file`                                         |
| `decode_datalink/sdruno_129535000_2000000_acars_83s_20200908_152020Z.cs16` | datalink  | Vanilla ACARS decoder (129.535 MHz)                | `acarsdec -r file`                                         |
| `decode_vor/gqrx_20250925_144051_114647000_1800000_fc.raw`                 | voracious | VOR navigation receiver (114.85 MHz, KLO ident)    | `voracious <file> --vor-freq 114.85`                       |
| `decode_vor/gqrx_20251107_181747_116000000_1800000_fc.raw`                 | voracious | VOR navigation receiver (116.00 MHz, ARL ident)    | `voracious <file> --vor-freq 116`                          |
| `decode_ils/cubic_20241024_103228_111755000.wav`                           | voracious | ILS localizer/glideslope receiver (108.8 MHz)      | `voracious <file> --ils-freq 108.8 --center-freq 114.647`  |

**Validation Checklist (before committing):**

- [ ] jet1090 can decode ADS-B messages from `decode_adsb/gqrx_*.raw`
- [ ] ship162 can decode AIS packets from `decode_ais/ais_paris_*.bin`
- [ ] dabradio produces valid audio from `decode_dab/dab_12A_2min.iq`
- [ ] fmradio decodes FM + RDS correctly from `decode_fm/rtlsdr_*.bin`
  - **Audio Quality:** Clear, intelligible audio output (no artifacts, timing glitches, or stuttering)
  - **RDS Decoding:** Valid RDS blocks detected and groups decoded (use `-vv` or `--tui` to see output)
  - **Symbol Sync:** RDS symbol clock recovers after ~2-3 seconds (debug output shows increasing signal levels)
  - **File-based timing:** Timing is controlled by adding artificial delays matching sample rate (not wall-clock time)
- [ ] datalink VDL2 mode decodes frames from `decode_datalink/rtlsdr_136*.rtl`
- [ ] datalink ACARS mode decodes messages from `decode_datalink/sdruno_129*.cs16`
- [ ] voracious VOR decoder (KLO): Returns ident **KLO** with stable radial around **119°** from `decode_vor/gqrx_20250925_*.raw`
- [ ] voracious VOR decoder (ARL): Returns ident **ARL** with stable radial around **115°** from `decode_vor/gqrx_20251107_*.raw`
- [ ] voracious ILS decoder: Trajectory shows movement from **left to right** of localizer from `decode_ils/cubic_20241024_*.wav`

**Why this matters:**

- Unit tests can pass while real-world decoding fails due to edge cases in DSP processing
- USB throughput, buffer management, and signal processing changes can silently break decoders
- Audio quality and bit error rates are only measurable with real I/Q data
- This is the definitive proof that the ecosystem still works end-to-end

## Support and contributions

- Test thoroughly before proposing changes
- Document breaking changes clearly in PRs
