# rs-hackrf

A pure-Rust library for HackRF One Software Defined Radio devices. Zero C dependencies — built entirely on [`nusb`](https://docs.rs/nusb) for native USB communication.

## Overview

`rs-hackrf` provides a complete HackRF One driver with device discovery, configuration, and high-throughput RX streaming. It includes:

- **Device Discovery:** Enumerate and open HackRF devices by index or serial number
- **Full Configuration:** Center frequency, sample rate, LNA/VGA gain, RF amplifier, baseband filter bandwidth, bias tee
- **Multi-Transfer Streaming:** Keeps multiple USB bulk transfers in-flight simultaneously, eliminating inter-transfer gaps that cause FIFO overflow
- **Backpressure-Safe Delivery:** Blocking channel ensures no IQ samples are silently dropped

## Features

- No libusb / C dependency — pure Rust USB via `nusb`
- Supports HackRF One, Jawbreaker, and rad1o devices
- 8-bit signed interleaved I/Q samples directly over USB (no DSP conversion needed)
- Runtime control (retune, gain changes) during active streaming sessions
- Automatic baseband filter bandwidth selection (75% of sample rate)

## Usage

### Device Info

```rust
use rs_hackrf::HackRf;

let device = HackRf::open_first()?;
println!("Firmware: {}", device.version()?);
println!("Board: {}", rs_hackrf::board_id_name(device.board_id()?));

let (part0, part1, serial) = device.board_partid_serialno()?;
println!("Serial: {}", serial);
```

### Configuration & Synchronous Read

```rust
use rs_hackrf::HackRf;

let mut device = HackRf::open_first()?;
device.set_freq(100_000_000)?;       // 100 MHz
device.set_sample_rate(8_000_000)?;   // 8 MS/s
device.set_lna_gain(24)?;            // 24 dB (0-40, 8 dB steps)
device.set_vga_gain(20)?;            // 20 dB (0-62, 2 dB steps)

device.start_rx()?;

let mut buf = vec![0u8; rs_hackrf::RECOMMENDED_BUFFER_SIZE];
let n = device.read_sync(&mut buf)?;
// buf contains interleaved i8 I/Q: [I0, Q0, I1, Q1, ...]

device.stop_rx()?;
```

### Multi-Transfer Streaming

```rust
use rs_hackrf::HackRf;

let mut device = HackRf::open_first()?;
device.set_freq(100_000_000)?;
device.set_sample_rate(8_000_000)?;
device.set_lna_gain(24)?;
device.set_vga_gain(20)?;

// Consumes device, starts background streaming thread
let reader = device.into_streaming_reader(0, 0)?; // 0 = use defaults

// Runtime control from any thread
let ctrl = reader.control_handle();
ctrl.tune(200_000_000)?;
ctrl.set_lna_gain(32)?;

while let Some(Ok(data)) = reader.recv() {
    // data: Vec<u8> of interleaved i8 I/Q samples
    println!("received {} bytes", data.len());
}
```

## Architecture

### HackRf

The main device handle wraps a `nusb::Interface` and provides:

1. **Device Discovery:** USB enumeration filtering by VID/PID (HackRF One, Jawbreaker, rad1o)
2. **Vendor Requests:** All control transfers use the HackRF USB vendor request protocol
3. **Synchronous Read:** Single-transfer bulk reads via `start_rx()` / `read_sync()` / `stop_rx()`
4. **Multi-Transfer Streaming:** `into_streaming_reader()` moves the device into a dedicated thread with multiple in-flight USB transfers

### Streaming Thread

```text
streaming thread (nusb endpoint queue)  ──sync_channel──▶  consumer
  └── control commands via mpsc channel   (tune / gain / amp)
```

- Keeps N transfers in-flight simultaneously (default: 4 × 256 KB)
- Processes control commands (retune, gain) between transfer completions
- Automatic disconnect detection after consecutive transfer errors
- Clean shutdown on drop

## Sample Format

HackRF outputs interleaved 8-bit **signed** I/Q samples directly over USB:

```text
[I0, Q0, I1, Q1, I2, Q2, ...]
```

Each I and Q value is an `i8` (-128 to 127). No additional DSP conversion is needed (unlike Airspy which outputs real samples requiring Fs/4 translation).

## Testing

Run unit tests:
```bash
cargo test -p rs-hackrf
```
