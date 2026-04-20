# rs-spy

A pure-Rust library for Airspy R2, Mini, and HF+ Software Defined Radio devices. Zero C dependencies — built entirely on [`nusb`](https://docs.rs/nusb) for native USB communication.

## Overview

`rs-spy` provides a complete Airspy driver with device discovery, configuration, real-to-IQ conversion, and high-throughput streaming. It includes:

- **Device Discovery:** Enumerate and open Airspy devices by index
- **Full Configuration:** Center frequency, sample rate, LNA/mixer/VGA gain, AGC, gain presets (linearity/sensitivity), RF bias (bias-T), sample packing
- **Real-to-IQ Conversion:** Airspy outputs real ADC samples — the built-in `IqConverter` converts to complex I/Q using Fs/4 translation with a 47-tap half-band FIR filter
- **Multi-Transfer Streaming:** Keeps multiple USB bulk transfers in-flight simultaneously, eliminating inter-transfer gaps that cause FIFO overflow
- **Backpressure-Safe Delivery:** Blocking channel ensures no samples are silently dropped

## Features

- No libusb / C dependency — pure Rust USB via `nusb`
- Supports Airspy R2, Mini, and HF+ devices
- 22-level linearity and sensitivity gain presets (matching libairspy)
- Independent LNA (0–14), mixer (0–15), and VGA (0–15) gain control with optional AGC
- Runtime control (retune, gain changes) during active streaming sessions
- IQ converter with DC removal, Fs/4 frequency translation, and symmetric half-band FIR filtering

## Usage

### Device Info

```rust
use rs_spy::Airspy;

let device = Airspy::open_first()?;
println!("Firmware: {}", device.version()?);
println!("Board ID: {}", device.board_id()?);
println!("Sample rates: {:?}", device.supported_sample_rates()?);

let (part1, part2, serial) = device.board_partid_serialno()?;
println!("Serial: {:016x}", serial);
```

### Configuration & Synchronous Read

```rust
use rs_spy::Airspy;

let device = Airspy::open_first()?;
device.set_freq(100_000_000)?;        // 100 MHz
device.set_sample_rate(0)?;           // First supported rate (e.g. 6 MS/s)
device.set_linearity_gain(15)?;       // Linearity preset 0-21

device.start_rx()?;

let mut buf = vec![0u8; rs_spy::RECOMMENDED_BUFFER_SIZE];
let n = device.read_sync(&mut buf)?;
// buf contains raw 16-bit real ADC samples (little-endian)

device.stop_rx()?;
```

### Multi-Transfer Streaming

```rust
use rs_spy::Airspy;

let device = Airspy::open_first()?;
device.set_freq(100_000_000)?;
device.set_sample_rate(0)?;
device.set_sensitivity_gain(18)?;
device.start_rx()?;

// Consumes device, starts background streaming thread
let reader = device.into_multi_transfer_reader(0, 0)?; // 0 = use defaults

// Runtime control from any thread
let ctrl = reader.control_handle();
ctrl.tune(200_000_000)?;

while let Some(Ok(data)) = reader.recv() {
    // data: Vec<u8> of raw 16-bit real ADC samples
    println!("received {} bytes", data.len());
}
```

### Real-to-IQ Conversion

```rust
use rs_spy::IqConverter;

let mut converter = IqConverter::new();

// Convert 16-bit real samples to f32 first, then process
let mut samples: Vec<f32> = raw_u16_samples.iter()
    .map(|&s| s as f32 / 32768.0)
    .collect();

// In-place conversion: even indices become I, odd become Q
converter.process(&mut samples);

// Or get Complex<f32> directly (output length = input length / 2)
let iq_pairs = converter.process_to_complex(&mut samples);
```

## Architecture

### Airspy Device

The main device handle wraps a `nusb::Interface` and provides:

1. **Device Discovery:** USB enumeration filtering by VID/PID (0x1d50:0x60a1)
2. **Vendor Commands:** All control transfers use the Airspy vendor command protocol (matching libairspy)
3. **Synchronous Read:** Single-transfer bulk reads via `start_rx()` / `read_sync()` / `stop_rx()`
4. **Multi-Transfer Streaming:** `into_multi_transfer_reader()` moves the device into a dedicated thread with multiple in-flight USB transfers

### Streaming Thread

```text
streaming thread (nusb endpoint queue)  ──sync_channel──▶  consumer
  └── control commands via mpsc channel   (tune / gain)
```

- Keeps N transfers in-flight simultaneously (default: 16 × 256 KB)
- Processes control commands (retune, gain stages) between transfer completions
- Automatic disconnect detection after consecutive transfer errors
- Clean shutdown on drop

### IqConverter

Converts real ADC samples to complex I/Q pairs using a polyphase approach:

1. **DC Removal:** Single-pole high-pass filter (α = 0.01) removes baseband offset
2. **Fs/4 Translation:** Rotation sequence `[-1, -hbc, +1, +hbc]` shifts spectrum by sample_rate/4
3. **Half-Band FIR Filter:** 47-tap symmetric filter on even samples (I channel), with specialized unrolled path for the standard kernel
4. **Delay Line:** Aligns Q channel group delay with I channel FIR delay

Output: N real samples → N/2 complex I/Q pairs (2:1 decimation).

## Sample Format

Airspy hardware outputs **real** samples from a single ADC (not I/Q). Raw USB data contains 16-bit unsigned samples (little-endian). Use `IqConverter` to produce complex I/Q pairs for SDR processing.

After IQ conversion, the interleaved buffer contains:
```text
[I0, Q0, I1, Q1, I2, Q2, ...]
```

## Gain Control

Three gain control modes are available:

| Mode | Method | Range | Description |
|------|--------|-------|-------------|
| Linearity | `set_linearity_gain(n)` | 0–21 | Optimized for linear response |
| Sensitivity | `set_sensitivity_gain(n)` | 0–21 | Optimized for weak signals |
| Manual | `set_lna_gain()` / `set_mixer_gain()` / `set_vga_gain()` | 0–14 / 0–15 / 0–15 | Independent stage control |
| AGC | `set_lna_agc()` / `set_mixer_agc()` | on/off | Automatic per-stage |

## Testing

Run unit tests:
```bash
cargo test -p rs-spy
```

Tests cover:
- IQ converter creation and kernel symmetry
- DC removal convergence
- FIR filter impulse response and circular buffer wrap-around
- Delay line cycling
- I/Q pair phase relationship
- Extended processing stability
