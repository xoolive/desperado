# fmradio

A high-performance FM radio demodulator and RDS (Radio Data System) decoder in pure Rust. Supports multiple SDR backends (RTL-SDR, Airspy, SoapySDR) and real-time audio output with integrated RDS data extraction.

## Overview

`fmradio` is a complete FM broadcasting receiver with:

- **FM Demodulation:** Phase-locked loop (PLL) based FM detection
- **Stereo Decoding:** 19 kHz pilot + 38 kHz subcarrier for L/R separation
- **RDS Decoding:** Full-featured RDS group parser (9 of 32 group types implemented)
- **Multiple Backends:** RTL-SDR, Airspy, and SoapySDR (with fallback to file/pipe input)
- **Audio Output:** Direct playback via system audio or pipe to external tools
- **JSON Output:** Machine-readable RDS data for integration with other applications

## Features

- **Real-time FM Reception:** Tune any FM frequency (88.0–108.0 MHz)
- **Stereo Audio:** Automatic L/R channel decoding
- **RDS Data:** Station name, song metadata (RadioText), time/date, program type
- **Low Latency:** <100 ms audio-to-output latency on modern hardware
- **Pipe-friendly:** Output raw PCM or JSON for integration with `redsea`, `play`, `sox`
- **Cross-platform:** Runs on Linux, macOS, Windows (with WinUSB drivers)

## Installation

### Requirements
- libusb (for RTL-SDR/Airspy backends)
- Audio output library (ALSA on Linux, CoreAudio on macOS, WinMM on Windows)

### Build

```bash
# Default: RTL-SDR + Airspy support
cargo build --release -p fmradio

# With SoapySDR backend (optional)
cargo build --release -p fmradio --features soapy
```

## Usage

### Basic FM Listening

Listen to stereo FM on 103.5 MHz with RTL-SDR:

```bash
cargo run --release -p fmradio -- -f 103.5M
```

Pipe audio to `play` (SoX):

```bash
cargo run --release -p fmradio -- -f 103.5M | \
  play -r 48000 -t raw -e s -b 16 -c 2 -
```

### Airspy Backend

```bash
cargo run --release -p fmradio -- -f 99.1M --source airspy://
```

### RDS JSON Output

Extract RDS data as JSON (newline-delimited, one group per line):

```bash
cargo run --release -p fmradio -- -f 98.1M --json
```

Example output:
```json
{"group":"0A","ps":"KISS FM","af":[98.1,99.0,101.5]}
{"group":"2A","rt":"Now Playing: Artist - Song Title"}
{"group":"4A","time":"2026-02-11T14:23:45Z"}
```

### Pipe to redsea (External RDS Decoder)

Output raw MPX signal for comparison with reference implementation:

```bash
cargo run --release -p fmradio -- -f 103.5M --raw-out --resample-out 228000 | \
  redsea -r 228000
```

## Architecture

### Signal Processing Pipeline

```
Input (6 MSps) 
  ↓
[AGC] → Automatic gain control
  ↓
[NCO + PLL] → Carrier recovery & frequency lock
  ↓
[FM Demod] → Phase extraction
  ↓
[Stereo Decoder] → L/R channel separation (19k pilot detection)
  ↓
[RDS Extractor] → 57 kHz subcarrier → Symbol sync → Biphase decode
  ↓
[Audio Resample] → 48 kHz PCM out | [RDS Parser] → JSON out
```

### RDS Implementation

- **Symbol Sync:** Polyphase filterbank (liquid-dsp compatible)
- **Clock Recovery:** NCO-based PLL with adaptive bandwidth
- **Biphase Decoding:** Clock polarity detection + FEC correction
- **Group Parsing:** 9 of 32 RDS group types fully implemented
  - **Complete:** 0A/0B (PS), 1A (PIN), 2A/2B (RadioText), 4A (Clock), 10A (PTYN)
  - **Partial:** 3A (ODA), 14A (EON)

## Examples

### Example 1: Listen and Log

Listen to 99.1 MHz and save RDS data to a file:

```bash
cargo run --release -p fmradio -- -f 99.1M --json > rds_log.jsonl &
# Press Ctrl+C to stop
```

### Example 2: Network Streaming

Stream FM audio over network:

```bash
# Receiver
cargo run --release -p fmradio -- -f 103.5M | nc -l localhost 12345

# Client
nc localhost 12345 | play -r 48000 -t raw -e s -b 16 -c 2 -
```

### Example 3: Headless Monitoring

Monitor RDS with no audio playback:

```bash
cargo run --release -p fmradio -- -f 98.1M --no-audio --json | jq '.ps'
```

## Testing

Run the full test suite:

```bash
cargo test -p fmradio
```

Run with live FM signal (manual verification):

1. Tune to a strong local station
2. Verify station name, song title, and time appear in RDS output
3. Compare with external decoders (e.g., `redsea`)

## Integration & External Tools

### With redsea (Reference RDS Decoder)

```bash
fmradio -f 103.5M --raw-out --resample-out 171000 | redsea -r 171000
```

### With sox/play

```bash
fmradio -f 99.1M | play -t raw -r 48000 -e s -b 16 -c 2 -
```

### With ffmpeg

```bash
fmradio -f 103.5M | ffmpeg -f s16le -ar 48000 -ac 2 -i - output.mp3
```

## Troubleshooting

### No Audio Output
- Check `--source` is correct and device is connected
- Try `--mono` if stereo causes issues
- Verify gain with `-g auto` or manual value

### RDS Data Not Appearing
- Weak signal? Try increasing gain: `-g 50` (RTL-SDR)
- Some stations broadcast incomplete RDS groups
- Use `--json` to verify JSON output format

### USB Device Not Found
- **Linux:** Install udev rules for your device
- **Windows:** Use Zadig to install WinUSB drivers
- **macOS:** libusb usually works out of the box

## Performance

On a modern CPU:
- **CPU:** ~15–25% single-core (RTL-SDR @ 2.4 MSps)
- **Memory:** ~50 MB (includes audio buffers)
- **Latency:** ~100 ms (tuning to audio output)

## References

- **RDS Standard:** IEC 62106
- **FM Modulation:** ITU-R BS.412
- **Redsea:** Reference C++ implementation - [GitHub](https://github.com/windytan/redsea)
- **liquid-dsp:** DSP library - [GitHub](https://github.com/jgaeddert/liquid-dsp)

## License

See [`LICENSE`](../../LICENSE) in the repository root.

## Contributing

Bug reports and feature requests welcome! See the [project plan](../../plan.md) for the roadmap.
