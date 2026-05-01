# rs-rtl

A pure-Rust library for RTL-SDR devices (RTL2832U + R820T/R828D) using [`nusb`](https://docs.rs/nusb) for native USB communication. Supports standard RTL-SDR dongles and RTL-SDR Blog V4.

## Overview

`rs-rtl` provides a complete RTL-SDR driver with device initialization, tuner control, and high-throughput IQ streaming. It includes:

- **Full R82xx Tuner Support:** R820T and R828D tuner initialization, PLL programming, gain control, bandwidth selection
- **RTL2832U Baseband:** Demodulator configuration, FIR filtering, sample rate control, I2C bridge for tuner communication
- **Multi-Transfer Streaming:** Keeps 15 USB bulk transfers in-flight simultaneously, eliminating the inter-transfer gap that causes RTL2832U FIFO overflow
- **Backpressure-Safe Delivery:** Blocking channel ensures no IQ samples are silently dropped
- **Blog V4 Support:** HF upconversion and three-input switching for RTL-SDR Blog V4 devices

## Features

- No libusb / C dependency — pure Rust USB via `nusb`
- Automatic tuner detection via I2C probing (R820T at 0x34, R828D at 0x74)
- 29 discrete gain steps from 0.0 to 49.6 dB
- Sample rates from 225 kS/s to 3.2 MS/s
- Bias-T (phantom power) control with EEPROM override support
- Runtime frequency and gain changes during active streaming
- EEPROM read for device-specific flags

## Usage

### Quick Start

```rust
use rs_rtl::{DeviceId, RtlSdr};

let mut sdr = RtlSdr::open(DeviceId::Index(0))?;
sdr.set_center_freq(100_000_000)?;  // 100 MHz
sdr.set_sample_rate(2_048_000)?;    // 2.048 MS/s
sdr.set_gain_manual(496)?;          // 49.6 dB

let reader = sdr.start_streaming()?;
while let Some(data) = reader.recv() {
    // data contains interleaved u8 I/Q samples: [I0, Q0, I1, Q1, ...]
    println!("received {} bytes", data.len());
}
```

### Device Discovery

```rust
use rs_rtl::DeviceDescriptors;

for dev in DeviceDescriptors::new()?.iter() {
  println!("#{}: {} {} (serial: {:?})",
    dev.index,
        dev.manufacturer.as_deref().unwrap_or("?"),
        dev.product.as_deref().unwrap_or("?"),
        dev.serial,
    );
}
```

### Configuration

```rust
use rs_rtl::{DeviceId, RtlSdr};

let mut sdr = RtlSdr::open(DeviceId::Index(0))?;

// Frequency: 24 MHz – 1766 MHz (R820T), HF with Blog V4
sdr.set_center_freq(433_920_000)?;

// Sample rate: 225,001–300,000 or 900,001–3,200,000 Hz
sdr.set_sample_rate(2_400_000)?;

// Gain: auto or manual (tenths of dB)
sdr.set_gain_auto()?;
sdr.set_gain_manual(297)?;  // 29.7 dB

// Bandwidth: 0 = automatic (matches sample rate)
sdr.set_bandwidth(0)?;

// Bias-T (antenna phantom power)
sdr.set_bias_t(true)?;

// Available gain steps
println!("gains: {:?}", sdr.gains());
```

### Runtime Control During Streaming

```rust
use rs_rtl::{DeviceId, RtlSdr};

let mut sdr = RtlSdr::open(DeviceId::Index(0))?;
sdr.set_center_freq(100_000_000)?;
sdr.set_sample_rate(2_048_000)?;
sdr.set_gain_manual(296)?;

let reader = sdr.start_streaming()?;

// Get a clonable control handle for use from any thread
let ctrl = reader.control_handle();
ctrl.tune(200_000_000)?;
ctrl.set_gain(400)?;
ctrl.set_gain_auto()?;
```

## Architecture

```text
RtlSdr (public API)
  ├── Device (USB register access, I2C bridge)
  ├── R82xx  (tuner driver: freq, gain, bandwidth)
  └── streaming (multi-transfer bulk endpoint queue)
```

### Device Layer

Low-level RTL2832U register access via USB vendor control transfers:
- **USB/SYS registers:** Direct read/write with block addressing
- **Demodulator registers:** Page-addressed with mandatory sync read after writes
- **I2C bridge:** Block 6 access for tuner communication (with repeater gating)
- **GPIO:** Output control for bias-T and device-specific features
- **EEPROM:** 256-byte read via I2C at address 0xA0

### Tuner Driver (R82xx)

Complete R820T/R828D driver with shadow register cache:
- **PLL Synthesizer:** Integer + sigma-delta fractional divider with VCO auto-ranging
- **Gain Control:** Separate LNA (16 steps) and mixer (16 steps) with alternating allocation
- **Bandwidth:** 350 kHz to 8 MHz with automatic IF frequency adjustment
- **Filter Calibration:** Automatic on init with retry logic
- **Blog V4:** HF upconversion via crystal offset, three-input antenna switching with notch filter

### Streaming Thread

```text
streaming thread (nusb endpoint queue)  ──sync_channel──▶  consumer
  └── control commands via mpsc channel   (tune / gain)
```

- Keeps 15 transfers in-flight simultaneously (15 × 16 KB by default)
- Processes control commands (retune, gain) between transfer completions
- Automatic disconnect detection after 5 consecutive transfer errors
- Clean shutdown on drop (tuner placed in standby)

## Sample Format

RTL-SDR outputs interleaved 8-bit **unsigned** I/Q samples:

```text
[I0, Q0, I1, Q1, I2, Q2, ...]
```

Each I and Q value is a `u8` (0–255), with 128 representing zero. To convert to floating point: `sample_f32 = (sample_u8 as f32 - 127.5) / 127.5`.

## Testing

Run unit tests:
```bash
cargo test -p rs-rtl
```
