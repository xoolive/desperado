# rs-spy

A pure-Rust library for real-time I/Q sample conversion from real (baseband) samples, with support for Airspy devices. Inspired by [`rtl-sdr-rs`](https://github.com/ccostes/rtl-sdr-rs) with a focus on high-performance I/Q conversion and Airspy backend integration.

## Overview

`rs-spy` provides high-performance I/Q (in-phase/quadrature) sample conversion from real ADC output, enabling SDR (Software Defined Radio) applications. It includes:

- **Real-to-IQ Conversion:** Converts raw real samples to complex I/Q pairs using Fs/4 translation
- **DC Removal:** Removes DC offset for cleaner signal processing
- **FIR Filtering:** Polyphase filterbank-based I/Q conversion with tunable parameters
- **Batch Processing:** Efficient buffer-based operations for high-throughput use cases

## Features

- Zero-copy compatible design (uses `Complex<f32>` from `num-complex`)
- Configurable FIR kernel size and tap count for quality vs. performance tradeoffs
- Integration with Airspy hardware via `rusb`

## Usage

### Basic I/Q Conversion

```rust
use rs_spy::IqConverter;

let mut converter = IqConverter::new(
    sample_rate,       // Hz
    fir_kernel_size,   // e.g., 64
    fir_tap_count,     // e.g., 31
);

let mut real_samples = vec![/* ADC data */];
converter.process(&mut real_samples);

// real_samples is now interleaved I/Q: [I0, Q0, I1, Q1, ...]
let iq_pairs: Vec<Complex<f32>> = 
    converter.process_to_complex(&mut real_samples);
```

### With Airspy Device

```rust
use rs_spy::Airspy;

let mut airspy = Airspy::open()?;
airspy.set_sample_rate(6_000_000)?;

let mut buffer = vec![0u8; airspy.recommended_buffer_size()];
let n_bytes = airspy.read_sync(&mut buffer)?;

// Process buffer...
```

## Architecture

### IqConverter

The core converter uses a **polyphase filterbank** approach:

1. **DC Removal:** High-pass filter to eliminate baseband offset
2. **Fs/4 Translation:** Frequency shift via complex rotation
3. **FIR Filtering:** Decimation and sample rate alignment
4. **Complex Output:** Pairs of consecutive samples become I/Q pairs

### Transport Layer

- **Airspy Support:** Full USB control and bulk data transfer via `rusb`
- **Buffer Pooling:** Efficient memory management for real-time streaming
- **Async Ready:** Compatible with tokio-based pipelines (via `desperado`)

## Performance

On modern CPUs (~3 GHz):
- **Throughput:** ~100 MSps (6 MHz sample rate × 16× overhead)
- **Latency:** <1 ms per chunk (typical chunk = 262,144 samples)
- **Memory:** ~1 MB per IqConverter instance + USB buffers

## Testing

Run unit tests:
```bash
cargo test -p rs-spy
```

Tests cover:
- DC removal correctness
- FIR filter impulse response
- I/Q pair generation and alignment
- Edge cases (empty buffers, misaligned data)
