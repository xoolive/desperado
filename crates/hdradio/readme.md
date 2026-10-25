# hdradio — Pure-Rust HD Radio Decoder

A pure-Rust NRSC-5 / HD Radio receiver for FM-hybrid captures. It decodes the
OFDM physical layer, station and service metadata, AAS/SIG/ID3 now-playing data,
and HDC audio without C bindings, FFI, or a runtime `nrsc5` dependency.

## Features

- **NRSC-5 PHY** — acquisition, OFDM synchronization, carrier equalization,
  Costas tracking, fine timing, and reacquisition
- **Primary data decoding** — PIDS, P1, CRC validation, and tail-biting Viterbi
- **Station metadata** — SIS identity, HD program labels, SIG labels, and ID3
  now-playing information
- **HDC audio** — pure-Rust HDC AAC core, HDC TNS, DRM-derived SBR handling,
  44.1 kHz stereo PCM, and soundcard playback
- **WAV export** — save decoded audio with `--wav`
- **Stage diagnostics** — hidden dumps for FFT, primary-main soft bits, PIDS,
  P1, and HDC transport comparison against offline reference traces
- **GQRX input** — automatic center frequency, sample rate, and I/Q format
  detection from standard GQRX filenames

## Building

From the workspace root:

```bash
cargo build --release -p hdradio
```

The binary is `target/release/hdradio`.

## Usage

### Listen to HD1

```bash
cargo run --release -p hdradio -- \
  capture_106800000_2400000_fc.raw --frequency 106.7M
```

Audio is sent to the default soundcard. Use `--no-audio` for headless or
metadata-only decoding.

### Select another HD program

Program numbers are zero-based:

```bash
# HD2
cargo run --release -p hdradio -- capture.raw \
  --frequency 106.7M --program 1
```

Use the JSON `programs` field to discover labels. A station may advertise a
program in SIS metadata without carrying recoverable audio in a particular
capture.

### Export WAV audio

```bash
cargo run --release -p hdradio -- capture.raw \
  --frequency 106.7M --max-seconds 30 \
  --no-audio --wav /tmp/hd1.wav
```

Output is 16-bit stereo PCM at 44.1 kHz. Frames with unsupported HDC SBR
enhancement fall back to their valid AAC core while preserving the output
timeline.

### Inspect station and program metadata

```bash
cargo run --release -q -p hdradio -- capture.raw \
  --frequency 106.7M --max-seconds 10 --no-audio | jq
```

The report includes station identity, program labels, now-playing metadata,
lock counters, HDC PDU counts, and decoded/rejected HDC frame counts.

## Architecture

```text
I/Q capture
   │
   ▼
front_end/       FM filtering, translation, and resampling
   │
   ▼
ofdm/           coarse/fine synchronization, FFT, carrier tracking
   │
   ▼
protocol/        primary-main extraction, PIDS, P1, FEC
   │
   ├────────────► metadata/   SIS, AAS/SIG, labels, ID3, JSON model
   │
   └────────────► audio/      HDC PDU transport, AAC/SBR core, PCM/WAV
```

The implementation is organized into domain submodules matching the DAB and
FM receiver crates:

- `front_end` — input normalization and FM-hybrid channel selection
- `ofdm` — carriers, frame synchronization, and PHY tracking
- `protocol` — configuration and primary data transport
- `fec` — convolutional decoding
- `metadata` — SIS, AAS, labels, now-playing, and report models
- `audio` — P1 audio transport and HDC PCM output

## Pure-Rust codec dependency

`hdradio` uses the adjacent `hdradio-syom` workspace crate. It is derived from
the MIT-licensed `syom` project and contains the NRSC-5 HDC syntax adapter.
There is no linked C codec or runtime dependency on the GPL `nrsc5` program.

NRSC-5 receiver algorithms are GPL-3.0-or-later derived ports. See
[`LICENSE`](LICENSE) and [`NOTICE`](NOTICE) for scope and provenance.

## Testing

```bash
cargo fmt --all --check
cargo test -p hdradio --release
cargo test -p hdradio-syom --lib --release
cargo clippy -p hdradio --all-targets -- -D warnings
```

Validation captures remain outside Git. The development process uses shared
reference traces to compare FFT, soft bits, PIDS, metadata, and HDC transport
before evaluating PCM output.

## Current limitations

- The receiver currently accepts file-based I/Q input; live SDR URI support is
  not yet exposed by this binary.
- HDC SBR is adapted from the DRM syntax and unsupported enhancement tables may
  fall back to AAC-core audio.
- HD program availability depends on the audio PDUs present in the capture.
- WAV and soundcard playback are selected-program outputs, not a simultaneous
  multi-program mixer.
