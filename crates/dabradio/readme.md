# dabradio — DAB/DAB+ Digital Radio Decoder

A high-performance, production-ready decoder for DAB (Digital Audio Broadcasting) and DAB+ signals. Reads IQ samples from files or SDRs and decodes ensemble information, service listings, and audio streams in real-time.

## Features

- **Full DAB/DAB+ decoding pipeline** — OFDM sync, FIC/MSC extraction, Viterbi FEC, AAC audio
- **Multi-service support** — List all available services or decode a specific one
- **Robust synchronization** — Handles frame alignment drift due to dropped samples or clock errors
- **DAB+ audio output** — AAC-LC and HE-AAC v2 decoding via fdk-aac, 48 kHz stereo output
- **Programme Associated Data (PAD)** — Extract DLS text metadata and MOT slideshow images
  - **DLS (Dynamic Label Segment)**: Song titles, artist names, and text metadata
  - **MOT (Multimedia Object Transfer)**: Album art and slideshow images (JPEG/PNG)
- **JSON output** — Service listings as JSON for programmatic access
- **Multiple input formats** — cu8, cs8, cs16, cf32 IQ samples from files or network streams
- **Real-time audio playback** — Streams decoded audio directly to soundcard via tinyaudio
- **Cross-platform** — Tested on Linux x86_64 and macOS Apple Silicon

## Building

```bash
# From the desperado workspace root:
cargo build --release -p dabradio

# Enable live SDR backends as needed:
cargo build --release -p dabradio --features rtlsdr
cargo build --release -p dabradio --features soapy
cargo build --release -p dabradio --features airspy
```

The binary will be at `target/release/dabradio`.

## Usage

### List all services in a DAB ensemble

```bash
# cu8 format (RTL-SDR raw samples)
./target/release/dabradio recording.cu8 --channel 12A

# cf32 format (gqrx raw complex float)
./target/release/dabradio recording.cf32 --channel 12A --format cf32

# cs16 format (signed 16-bit I/Q)
./target/release/dabradio recording.cs16 --channel 12A --format cs16
```

Output:
```
Ensemble: Métropolitain 2 (EId: 0xF044)

Services:
SId      Label                SubCh  Bitrate    Protection
------------------------------------------------------------
0xF201   FRANCE INTER         8      88 kbps    EEP 3-A
0xF202   FRANCE CULTURE       10     88 kbps    EEP 3-A
...
```

### Decode audio from a specific service

```bash
# Play audio from "FRANCE CULTURE" to soundcard
./target/release/dabradio recording.cu8 --channel 12A --service "FRANCE CULTURE"

# Decode by hex SId instead of label
./target/release/dabradio recording.cu8 --channel 12A --service 0xF202

# Save raw DAB+ frames to file (no audio output)
./target/release/dabradio recording.cu8 --channel 12A --service "FRANCE CULTURE" \
  --output frames.bin --no-audio
```

### Live SDR sources (Phase 6)

```bash
# RTL-SDR (build with --features rtlsdr)
./target/release/dabradio rtlsdr:// --channel 12A --service "FIP"

# SoapySDR (build with --features soapy)
./target/release/dabradio soapy://driver=rtlsdr --channel 12A --service "FIP"

# Airspy (build with --features airspy)
./target/release/dabradio airspy://0 --channel 12A --service "FIP"
```

If `freq`/`rate` are not provided in the URI query, `dabradio` injects them from `--channel`/`--freq` and the DAB sample rate (2.048 MHz).

### Output formats

```bash
# JSON service listing
./target/release/dabradio recording.cu8 --channel 12A --json

# Limit frame processing (useful for testing)
./target/release/dabradio recording.cu8 --channel 12A --max-frames 100
```

### Extract DLS Metadata and MOT Slideshow Images

```bash
# Display DLS text metadata while decoding audio
./target/release/dabradio recording.cu8 --channel 12A --service "FIP" --metadata

# Extract MOT slideshow images to directory (album art, cover art, etc.)
./target/release/dabradio recording.cu8 --channel 12A --service "FIP" \
  --metadata --slideshow /tmp/fip_slides

# Extract images without audio playback
./target/release/dabradio recording.cu8 --channel 12A --service "FIP" \
  --metadata --slideshow /tmp/fip_slides --no-audio
```

**Output example:**
```
$ ls -lah /tmp/fip_slides/
-rw-r--r-- 1 user user 14279 slide_001.jpg  (JPEG 320×240)
-rw-r--r-- 1 user user 14279 slide_002.jpg  (JPEG 320×240)
-rw-r--r-- 1 user user 17639 slide_003.jpg  (JPEG 320×240)
-rw-r--r-- 1 user user  5416 slide_004.png  (PNG 320×240)
```

## Command-line Reference

```
USAGE:
    dabradio <SOURCE> --channel <CHANNEL> [OPTIONS]

ARGUMENTS:
    <SOURCE>    File path or SDR URI (rtlsdr://, soapy://, airspy://)

OPTIONS:
    --channel <CHANNEL>
            DAB channel (e.g., "12A", "12C")

    -f, --freq <FREQ>
            Center frequency in Hz (alternative to --channel)

    --format <FORMAT>
            IQ format for file sources: cu8, cs8, cs16, cf32 [default: cu8]

    --list
            List services and exit (no audio decoding)

    --json
            Output as JSON

    --service <SERVICE>
            Service to decode (label or hex SId like "0xF201")

    -o, --output <OUTPUT>
            Output file for raw DAB+ logical frames

    --no-audio
            Disable audio output to soundcard

    --metadata
            Display DLS text metadata (song titles, artist names)

    --slideshow <DIR>
            Extract MOT slideshow images to directory

    --max-frames <MAX_FRAMES>
            Maximum number of OFDM frames to process [default: 0 = unlimited]

    --bypass-deinterleave
            Debug: skip time de-interleaving in MSC (testing only)

    -h, --help
            Print help information
```

## Architecture

### OFDM Processing (`ofdm/processor.rs`)

The OFDM processor implements DAB's frame synchronization pipeline:

1. **Null symbol detection** — finds power drops in the IQ stream
2. **PRS correlation** — IFFT-based timing via Phase Reference Symbol
3. **Frequency estimation** — cyclic prefix and coarse frequency correction
4. **Symbol extraction** — FFT of 76 OFDM symbols (1 PRS + 75 data)

**Robustness feature**: When PRS correlation SNR drops below 5.0 (indicating alignment drift), the processor automatically re-detects the null symbol and re-synchronizes. This handles files with dropped samples (e.g., gqrx cf32 recordings with capture interrupts).

### FIC Handler (`fic/handler.rs`, `fic/fib.rs`)

Decodes the Fast Information Channel (FIC):

- **FIC symbols 0-2** → depuncture, Viterbi decode, energy dispersal → FIBs (Fast Information Blocks)
- **FIB parsing** → FIG (Fast Information Group) processing
- **Ensemble & service discovery** → service labels, subchannel config, bitrates, protection levels

Waits for complete FIC data (all services with labels, all subchannels) before declaring sync complete.

### MSC Handler (`msc/mod.rs`)

Decodes the Main Service Channel (MSC):

- **CIF assembly** — buffers OFDM symbols into Common Interleaved Frames
- **Time de-interleaving** — reverses the time-domain interleave applied by transmitter
- **Subchannel extraction** — isolates the target service's data
- **FEC decoding** — depuncturing (UEP or EEP), Viterbi, energy dispersal

Supports both UEP (Unequal Error Protection) and EEP (Equal Error Protection) schemes.

### DAB+ Audio Decoder (`audio/mod.rs`)

Decodes DAB+ superframes to PCM audio:

1. **Superframe sync** — finds fire code in logical frames
2. **Reed-Solomon correction** — RS(120,110) error correction
3. **AU extraction** — parses Access Units with CRC validation
4. **AAC decoding** — fdk-aac library handles AudioSpecificConfig and decoding
5. **Audio output** — sends 48 kHz stereo PCM to tinyaudio for playback

## Performance

- **Real-time decode** — typical 5-10x realtime on modern CPUs
- **Memory-efficient** — circular buffers and lazy allocation
- **Low-latency audio** — 4-second crossbeam buffer for smooth playback

## Testing

```bash
# Run all tests
cargo test -p dabradio

# Run tests with output
cargo test -p dabradio -- --nocapture

# Check for clippy warnings
cargo clippy -p dabradio --tests

# Build with optimizations
cargo build --release -p dabradio
```

All 45 unit tests pass (2 require external test fixtures and are ignored). 0 clippy warnings.

## IQ Format Support

### cu8 (Unsigned 8-bit I/Q) — Default
Used by RTL-SDR and most software radios.
```
Bytes per sample: 2 (I byte, Q byte)
Value range: 0-255 → normalized to [-1, +1]
Conversion: (byte - 127.5) / 128.0
```

### cf32 (Complex 32-bit float)
Used by gqrx, USRP, and advanced SDRs.
```
Bytes per sample: 8 (I float, Q float)
Value range: as-is (typically [-1, +1])
Conversion: direct from IEEE 754 f32 bytes
```

### cs8, cs16
Signed 8-bit and 16-bit I/Q formats. Specify with `--format cs8` or `--format cs16`.

## Supported DAB Channels

Use `--channel` with standard designations (150-240 MHz band):

```
5A, 5B, 5C, 5D (174.928-181.936 MHz)
6A, 6B, 6C, 6D (181.936-188.944 MHz)
7A, 7B, 7C, 7D (188.944-195.952 MHz)
8A, 8B, 8C, 8D (195.952-202.960 MHz)
9A, 9B, 9C, 9D (202.960-209.968 MHz)
10A, 10B, 10C, 10D (209.968-216.976 MHz)
11A, 11B, 11C, 11D (216.976-223.984 MHz)
12A, 12B, 12C, 12D (223.984-230.992 MHz)
13A, 13B, 13C, 13D (230.992-238.000 MHz)
```

Or specify frequency directly with `--freq 223936000` (Hz).

## Sample Rate

DAB uses 2.048 MHz sample rate. Ensure your recordings are at exactly 2048000 samples/sec.

## Known Issues & Limitations

1. **MSC decoding requires complete FIC** — services can only be decoded after ensemble info is available (typically 1-2 seconds)
2. **Single service at a time** — use `--service` to select one service; multiplexing not yet supported
3. **Soundcard output on Mac** — requires audio device permissions via system settings

## Debugging

Enable trace-level logging to see frequency estimation and synchronization details:

```bash
RUST_LOG=trace ./target/release/dabradio recording.cu8 --channel 12A --max-frames 5
```

Key debug output:
- `PRS correlation failed, re-acquiring sync` — frame alignment recovery in progress
- `DAB+ audio configured` — audio pipeline ready
- `Fire code sync acquired` — superframe boundary found

## Dependencies

Core:
- **rustfft** — FFT for OFDM processing
- **fdk-aac** — AAC audio decoding
- **reed-solomon** — Forward error correction
- **tinyaudio** — Cross-platform audio output

Utilities:
- **clap** — command-line parsing
- **serde/serde_json** — JSON output
- **tracing** — structured logging
- **crossbeam-channel** — lock-free audio buffering

## License

See `LICENSE` in the workspace root.

## References

- [ETSI EN 300 401 v2.2.1](https://www.etsi.org/deliver/etsi_en/300400_300499/300401/02.02.01_60/en_300401v020201p.pdf) — DAB specification
- [welle.io](https://github.com/alanthird/DABstar) — reference decoder architecture
- [FDK-AAC](https://github.com/mstorsjo/fdk-aac) — AAC decoder library

## Contributing

Contributions welcome! Areas for enhancement:

- [ ] Multiplex multiple services in a single run
- [ ] Real-time recording from SDRs (SoapySDR/Airspy)
- [ ] TII (Transmitter Identification Information) decoder for SFN detection
- [ ] WebAssembly/browser decoder
- [ ] Ensemble metadata export (RSID/PI-code)
- [ ] JSON output for metadata and MOT images

---

**Last updated:** March 2026  
**Status:** Production-ready, actively maintained
