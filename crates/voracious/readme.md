# Voracious - VOR Signal Decoder

A Rust library and CLI tool for decoding VHF Omnidirectional Range (VOR) navigation signals used in aviation.

## Features

- Decode VOR signals from I/Q sample files or RTL-SDR
- Extract VOR radial (bearing) from 30 Hz variable and reference signals
- JSON output for easy integration
- Configurable window size for radial calculation

## VOR Signal Overview

VOR stations transmit on frequencies between 108-117.95 MHz. The signal consists of:

- A carrier wave with voice/identification
- A 30 Hz reference signal (FM modulated on a 9960 Hz subcarrier)
- A 30 Hz variable signal (AM modulated, rotates to create directional information)

The bearing (radial) to the station is determined by the phase difference between the 30 Hz variable and reference signals.

## Installation

```bash
cargo build --release
```

To build without RTL-SDR support:

```bash
cargo build --release --no-default-features
```

## Usage

### Decode from I/Q file

```bash
voracious samples.cf32 \
    --sample-rate 1800000 \
    --format cf32 \
    --vor-freq 114.85 \
    --center-freq 114.647 \
    --window 3.0
```

### Decode from SDR URI

The positional input also accepts SDR URIs:

- `rtlsdr://`
- `airspy://`
- `soapy://`

Examples:

```bash
# RTL-SDR (uses --vor-freq as center frequency if --center-freq is omitted)
voracious rtlsdr:// --vor-freq 114.85 --sample-rate 1800000

# SoapySDR with driver args and embedded tuning params
voracious "soapy://driver=rtlsdr?freq=114.85M&rate=1.8M&gain=auto" --vor-freq 114.85

# Airspy device index 1
voracious airspy://1 --vor-freq 114.85 --sample-rate 2500000
```

### Parameters

- `--sample-rate`: Sample rate in Hz (default: 1800000)
- `--format`: I/Q format - cu8, cs8, cs16, or cf32 (default: cf32)
- `--vor-freq`: VOR frequency in MHz
- `--center-freq`: SDR center frequency in MHz (optional for URI inputs)
- `--window`: Window size in seconds for radial calculation (default: 3.0)
- `<input>`: I/Q file path or SDR URI (`rtlsdr://`, `airspy://`, `soapy://`)

Feature flags:

- `rtlsdr://` requires `--features rtlsdr` (enabled by default)
- `soapy://` requires `--features soapy`
- `airspy://` requires `--features airspy`

### Output

JSON format on stdout, one line per radial calculation:

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
  "ident": null
}
```

Convert Unix timestamp to readable UTC with `jq`:

```bash
./target/release/voracious crates/voracious/samples/gqrx_20251107_182558_116000000_1800000_fc.raw --vor-freq 116 \
  | jq -c '.timestamp = (.timestamp | floor | strftime("%Y-%m-%d %H:%M:%S"))'
```

`timestamp` is a Unix timestamp in seconds (UTC, floating-point).

- For file inputs, it uses `t0 + sample_count / sample_rate`, where `t0` is taken from:
  1. gqrx filename timestamp when available, otherwise
  2. file creation time, otherwise
  3. file modification time.
- For live SDR URIs, it uses wall-clock Unix time at emission.

### Signal Quality Metrics (Beginner-Friendly)

`signal_quality` is a dictionary of raw indicators. It is not a pass/fail flag.

- `clipping_ratio`
  - Format: scientific-notation string (for example `"2.30e-3"`).
  - What it means: fraction of I/Q samples that hit the maximum measurable value.
  - Good sign: close to `0.0`.
  - Bad sign: higher values (for example `> 0.01`) suggest overload/distortion.

- `snr_30hz_db`
  - What it means: estimated signal-to-noise ratio of the 30 Hz bearing component in the current window.
  - Good sign: higher values.
  - Typical values on usable signals: about `10` to `25` dB.
  - Strong/clean captures can be `25` to `40+` dB.
  - Bad sign: near `0` dB or negative values mean weak 30 Hz content relative to nearby noise.

- `snr_9960hz_db`
  - What it means: estimated signal-to-noise ratio of the 9960 Hz VOR subcarrier in the current window.
  - Good sign: higher values.
  - Typical values on usable signals: about `6` to `18` dB.
  - Strong captures can reach `20+` dB.
  - Bad sign: around `0` dB or lower can make reference extraction less reliable.

- `lock_quality`
  - Format: scientific-notation string (for example `"8.70e-1"`).
  - What it means: how consistently the extracted 30 Hz variable and reference waveforms track each other.
  - Range: `0.0` to `1.0`.
  - Good sign: closer to `1.0`.

- `radial_variance`
  - Format: scientific-notation string (for example `"1.20e-4"`).
  - What it means: short-term spread of recent radial outputs.
  - Range: `0.0` to `1.0`.
  - Good sign: closer to `0.0` (stable bearing).
  - Bad sign: larger values indicate jitter/instability.

Notes:

- dB values are relative internal measures for this decoder, best used to compare frames from the same run.
- For reliable interpretation, watch trends over time instead of a single frame.

## Library Usage

```rust
use voracious::sources::{IqSource, IqFormat};

let source = IqSource::new(
    "samples.cf32",
    1_800_000,
    IqFormat::Cf32,
    114.85,  // VOR freq
    114.647, // Center freq
    3.0,     // Radial window seconds
    15.0,    // Morse window seconds
    false,   // debug_morse
)?;

for result in source {
    match result {
        Ok(radial) => println!("VOR radial: {:.1}°", radial.radial_deg),
        Err(e) => eprintln!("Error: {}", e),
    }
}
```

## Architecture

The project follows a modular architecture inspired by the ship162 AIS decoder:

- `src/dsp/` - Digital signal processing
  - `vor.rs` - VOR demodulation logic
  - `filter.rs` - Butterworth filters and DSP utilities
- `src/decode/` - VOR radial decoding and output formatting
- `src/sources/` - Data sources
  - `iq.rs` - I/Q file reader with iterator interface
  - `rtlsdr.rs` - RTL-SDR real-time source
- `src/main.rs` - CLI application

## Signal Processing Pipeline

1. **Frequency shift** - Mix to baseband (VOR frequency)
2. **Lowpass filter** - 200 kHz cutoff
3. **FM demodulation** - Phase differentiation
4. **Audio filter** - 20 kHz lowpass
5. **Decimation** - Downsample to 48 kHz audio rate
6. **Subcarrier extraction**:
   - Variable: 9-11 kHz bandpass
   - Reference: 9.5-10.5 kHz bandpass
7. **30 Hz component extraction**:
   - Variable: AM demodulation (envelope) + 100 Hz lowpass
   - Reference: FM demodulation (phase diff) + 100 Hz lowpass
8. **Phase estimation** - DFT at 30 Hz
9. **Radial calculation** - Phase difference in degrees

## License

MIT License
