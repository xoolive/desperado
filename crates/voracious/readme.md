# Voracious - VOR Signal Decoder

A Rust library and CLI tool for decoding VHF Omnidirectional Range (VOR) navigation signals used in aviation.

## Features

- Decode VOR signals from I/Q sample files or live SDR devices
- Extract VOR radial (bearing) using the VORtrack phase-tracking algorithm
- Decode the Morse station identifier (e.g. `KLO`, `ARL`)
- Signal quality metrics per window (SNR, clipping, lock quality, radial variance)
- JSON output for easy integration with `jq` or other tools

## VOR Signal Overview

VOR stations transmit on 108–117.95 MHz. The signal encodes the receiver's bearing to the station using two 30 Hz tones:

- A **reference** signal: FM-modulated on a 9960 Hz subcarrier, phase is fixed
- A **variable** signal: AM-modulated directly on the carrier, phase rotates with the antenna pattern

The bearing (radial) is the phase difference between the two 30 Hz components. Additionally, a Morse ident (three letters, ~1020 Hz tone) is transmitted every ~30 seconds.

## Installation

```bash
cargo build --release -p voracious
```

Without RTL-SDR or Airspy support:

```bash
cargo build --release -p voracious --no-default-features
```

## Usage

### Decode from a gqrx recording

gqrx filenames encode center frequency and sample rate. `voracious` infers them automatically:

```bash
voracious gqrx_20251107_182558_116000000_1800000_fc.raw --vor-freq 116
```

### Decode from any I/Q file

```bash
voracious samples.cf32 \
    --vor-freq 114.85 \
    --center-freq 114.647 \
    --sample-rate 1800000 \
    --format cf32 \
    --window 3.0
```

### Decode from a live SDR

```bash
# RTL-SDR (center frequency defaults to VOR frequency)
voracious rtlsdr:// --vor-freq 114.85

# Airspy device 0
voracious airspy:// --vor-freq 116 --sample-rate 2500000

# SoapySDR with embedded tuning
voracious "soapy://driver=rtlsdr?freq=114.85M&rate=1.8M" --vor-freq 114.85
```

### Parameters

| Flag | Default | Description |
|------|---------|-------------|
| `--vor-freq` | required | VOR station frequency in MHz |
| `--center-freq` | auto | SDR center frequency in MHz (inferred from gqrx filenames) |
| `--sample-rate` | 1800000 | Sample rate in Hz |
| `--format` | cf32 | I/Q format: `cu8`, `cs8`, `cs16`, `cf32` |
| `--window` | 3.0 | Radial calculation window in seconds |
| `--morse-window` | 15.0 | Audio buffer size for Morse decoding in seconds |
| `--debug-morse` | off | Include Morse decode attempt details in JSON output |

Feature flags (all enabled by default):

- `rtlsdr` — RTL-SDR device support
- `airspy` — Airspy HF+/Mini support
- `soapy` — SoapySDR support (requires system `libsoapysdr`)

### Output

One JSON line per radial window (default every 3 seconds):

```json
{
  "timestamp": 1762539961.05835,
  "radial_deg": 115.31,
  "frequency_mhz": 116.0,
  "signal_quality": {
    "clipping_ratio": "<1.82e-7",
    "snr_30hz_db": 36.89,
    "snr_9960hz_db": 15.7,
    "lock_quality": "9.72e-2",
    "radial_variance": "1.00e0"
  },
  "ident": "ARL"
}
```

`timestamp` is a Unix timestamp in seconds (UTC, floating-point):
- For file inputs: `t0 + elapsed`, where `t0` comes from the gqrx filename, file creation time, or modification time (in that order).
- For live SDR: wall-clock time at output.

Pretty-print with human-readable timestamps:

```bash
voracious gqrx_20251107_182558_116000000_1800000_fc.raw --vor-freq 116 \
  | jq -c '.timestamp = (.timestamp | floor | strftime("%Y-%m-%d %H:%M:%S"))'
```

### Signal Quality Metrics

`signal_quality` contains raw per-window indicators. These are internal relative measures — compare across frames from the same run rather than to absolute thresholds.

| Field | Format | Meaning | Good | Bad |
|-------|--------|---------|------|-----|
| `clipping_ratio` | scientific string | Fraction of I/Q samples hitting the ADC ceiling | near `0` | `> 0.01` suggests gain overload |
| `snr_30hz_db` | float (dB) | SNR of the 30 Hz bearing tone | `10–40+ dB` | near `0` or negative |
| `snr_9960hz_db` | float (dB) | SNR of the 9960 Hz VOR subcarrier | `6–20+ dB` | near `0` or negative |
| `lock_quality` | scientific string, `0–1` | Phase coherence between variable and reference 30 Hz tones | near `1` | near `0` |
| `radial_variance` | scientific string, `0–1` | Short-term spread of recent radial outputs | near `0` (stable) | large (jittery) |

## Library Usage

```rust
use voracious::{IqSource, IqFormat};

let source = IqSource::new(
    "samples.cf32",
    1_800_000,     // sample rate (Hz)
    IqFormat::Cf32,
    114.85,        // VOR frequency (MHz)
    114.647,       // SDR center frequency (MHz)
    3.0,           // radial window (seconds)
    15.0,          // Morse window (seconds)
    false,         // debug_morse
)?;

for result in source {
    match result {
        Ok(radial) => println!("{:.1}° ident={:?}", radial.radial_deg, radial.ident),
        Err(e) => eprintln!("Error: {}", e),
    }
}
```

## Architecture

```
src/
├── lib.rs              — public re-exports
├── main.rs             — CLI (clap, gqrx filename inference, SDR URI parsing)
├── source.rs           — IqSource iterator: chunked I/Q → VorRadial
├── metrics.rs          — signal quality computation (SNR, lock, variance)
└── decoders/
    ├── mod.rs          — module coordinator and re-exports
    ├── vor.rs          — VorDemodulator, VORtrack radial algorithm, DSP pipeline
    └── morse.rs        — generic Morse ident parser (reusable for NDB, ILS, DME)
```

DSP filters are provided by the `desperado` crate (`desperado::dsp::filters`, `desperado::dsp::iir`).

## Signal Processing Pipeline

```
IQ samples (cf32, 1.8 MSps)
  │
  ├─ Frequency shift to VOR baseband
  ├─ 200 kHz Butterworth lowpass
  ├─ AM envelope detection
  ├─ 20 kHz lowpass
  └─ Decimate 38× → audio at ~47368 Hz
       │
       ├─── VORtrack radial algorithm
       │      Tracks 30 Hz variable (9–11 kHz BPF + envelope)
       │      and reference (9.5–10.5 kHz BPF + FM demod)
       │      → radial_deg
       │
       └─── Morse ident decoder (sliding 15-second windows)
              900–1100 Hz BPF + Hilbert envelope
              → threshold → dot/dash detection → 3-letter ident
```

## Testing

The test suite covers both unit tests (within modules) and integration tests against real VOR captures.

### Unit tests

Run with:

```bash
cargo test -p voracious
```

### Integration tests

Integration tests in `tests/vor_decoding.rs` exercise the full decoding pipeline using two real gqrx recordings, pre-demodulated to audio fixtures stored in `tests/data/`:

| Fixture stem | Station | Freq | Duration | Source recording |
|---|---|---|---|---|
| `gqrx_20250925_144051_114647000_1800000_fc` | KLO | 114.85 MHz | ~18 s | `gqrx_…114647000…_fc.raw` |
| `gqrx_20251107_182558_116000000_1800000_fc` | ARL | 116.00 MHz | ~26 s | `gqrx_…116000000…_fc.raw` |

Each fixture set contains three f32 binary files (all < 5 MB):

- `*_audio.f32` — mono audio at ~47368 Hz; input to the Morse decoder and VORtrack algorithm
- `*_var30.f32` — 30 Hz variable signal extracted from the 9960 Hz subcarrier envelope
- `*_ref30.f32` — 30 Hz reference signal from FM-demodulation of the 9960 Hz subcarrier

The fixtures were generated from the raw `.raw` captures using the actual Rust `VorDemodulator` (not a Python approximation), so they match the exact filter chain used in production. To regenerate them:

```bash
cargo run --release --example extract_audio -- \
  samples/gqrx_20250925_144051_114647000_1800000_fc.raw \
  114.85 114.647 tests/data/gqrx_20250925_144051_114647000_1800000_fc

cargo run --release --example extract_audio -- \
  samples/gqrx_20251107_182558_116000000_1800000_fc.raw \
  116.0 116.0 tests/data/gqrx_20251107_182558_116000000_1800000_fc 26
```

The tests cover:

| Test | What it checks |
|------|----------------|
| `test_klo_vortrack_radial_in_range` | Full-signal radial in 115–125° for KLO |
| `test_klo_vortrack_windowed_consistent` | Three 3-second windows each agree with full-signal within 5° |
| `test_klo_morse_ident` | Sliding 15-second Morse windows decode to `"KLO"` |
| `test_arl_vortrack_radial_in_range` | Full-signal radial in 110–120° for ARL |
| `test_arl_vortrack_windowed_consistent` | Five 3-second windows each agree with full-signal within 5° |
| `test_arl_morse_ident` | Sliding 15-second Morse windows decode to `"ARL"` |
| `test_vortrack_returns_none_for_short_input` | <0.5 s of audio returns `None` gracefully |
| `test_calculate_radial_returns_none_for_short_input` | FFT radial method also returns `None` for <0.5 s |
| `test_morse_returns_none_for_silence` | Pure silence produces no tokens and no ident |

**Why sliding windows for Morse?** The VOR Morse ident is a 3-letter code transmitted for ~3 seconds out of every ~12-second cycle. Passing a 26-second buffer directly to `decode_morse_ident` drops the duty cycle well below the decoder's 25–55% on-ratio threshold. The `IqSource` iterator in production uses a 15-second sliding buffer with 50% overlap — the tests mirror this with `decode_morse_sliding`.

Run the integration tests with:

```bash
cargo test -p voracious --test vor_decoding
```

## License

MIT License
