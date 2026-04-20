<div align="center">
  <img src="logo.png" alt="Desperado Logo" width="200"/>
</div>

# Desperado

_A unified Rust library for reading I/Q samples from files, SDR devices, and streams_

[![Crates.io](https://img.shields.io/crates/v/desperado.svg)](https://crates.io/crates/desperado)
[![Documentation](https://docs.rs/desperado/badge.svg)](https://docs.rs/desperado)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/xoolive/desperado/workflows/Rust/badge.svg)](https://github.com/xoolive/desperado/actions)

Desperado is a library designed to factorize and reuse code for reading I/Q samples from files, SDR devices, and other sources.

Desperado provides a unified interface for iterating (synchronously) and streaming (asynchronously) complex I/Q samples in `Complex<f32>` format.

The library intentionally does not include demodulation, focusing instead on providing a consistent interface over various sources.

The name "Desperado" is a playful nod to DSP (Digital Signal Processing), and tips its hat to <https://www.youtube.com/watch?v=-q93wc3-deU>.

## What are I/Q Samples?

I/Q (In-phase/Quadrature) samples are the fundamental representation of radio signals in software-defined radio (SDR). They represent complex numbers where:

- **I (In-phase)**: The real component, representing the signal along the cosine axis
- **Q (Quadrature)**: The imaginary component, representing the signal along the sine axis

Together, I and Q samples capture both the amplitude and phase information of a radio signal, allowing software to process, demodulate, and analyze RF signals that have been digitized by SDR hardware.

Desperado abstracts away the complexity of reading these samples from various sources (files, devices, network streams), providing a consistent interface regardless of the source or format (8-bit, 16-bit, float, etc.).

## Installation

Add Desperado to your `Cargo.toml`:

```toml
[dependencies]
desperado = "0.1"
```

### With SDR device support

To use hardware SDR devices, enable the appropriate feature flags:

```toml
[dependencies]
desperado = { version = "0.1", features = ["rtlsdr"] }  # For RTL-SDR devices
# or
desperado = { version = "0.1", features = ["airspy"] }  # For Airspy devices (R2, Mini, HF+)
# or
desperado = { version = "0.1", features = ["hackrf"] }  # For HackRF devices
# or
desperado = { version = "0.1", features = ["soapy"] }   # For SoapySDR-compatible devices
# or
desperado = { version = "0.1", features = ["pluto"] }   # For Adalm-Pluto devices
```

### Available features

- **`rtlsdr`**: RTL-SDR device support (DVB-T dongles)
- **`airspy`**: Airspy device support (R2, Mini, HF+) via pure-Rust `rs_spy` crate
- **`hackrf`**: HackRF device support via pure-Rust `rs_hack` crate (nusb backend)
- **`soapy`**: SoapySDR device support (LimeSDR, BladeRF, etc.)
- **`pluto`**: Adalm-Pluto SDR support

The following features are only needed for examples:

- **`clap`**: Command-line argument parsing for examples
- **`waterfall`**: Waterfall plot visualization example
- **`audio`**: FM demodulation examples with audio output

## Dependent projects

This Desperado workspace includes several specialized crates built on top of the core I/Q streaming library:

- **[fmradio](crates/fmradio)** - FM radio receiver with RDS data decoding and adaptive audio resampling
- **[dabradio](crates/dabradio)** - DAB/DAB+ digital radio decoder with OFDM and AAC audio
- **[voracious](crates/voracious)** - VOR/ILS/DME aviation navigation signal decoder

External projects that depend on Desperado for I/Q streaming:

- **[jet1090](https://github.com/xoolive/jet1090)** - Real-time ADS-B decoder for tracking aircraft
- **[ship162](https://github.com/xoolive/ship162)** - Real-time AIS decoder for tracking maritime vessels
- **[datalink](https://github.com/xoolive/datalink)** - VDL2 and ARINC 629 aviation datalink decoder

If you're using Desperado in your project, feel free to open a PR to add it here!

## Usage

### Basic example (synchronous version)

```rust ignore
use desperado::{IqFormat, IqSource};

fn main() -> desperado::Result<()> {
    // Create an IQ source from a binary file
    let path = "sample.iq";
    let sample_rate = 96_000;
    let center_freq = 162_000_000;
    let chunk_size = 8136;
    let iq_format = IqFormat::Cu8;

    for samples in IqSource::from_file(path, center_freq, sample_rate, chunk_size, iq_format)? {
        for s in samples? {  // samples is a Result<Vec<Complex<f32>>, _>
            println!("  I: {}, Q: {}", s.re, s.im);
        }
    }
    Ok(())
}
```

### RTL-SDR (async version)

Access to RTL-SDR devices is provided with the `rtlsdr` feature enabled.

```rust ignore
use desperado::IqAsyncSource;
use futures::StreamExt;

#[tokio::main]
async fn main() -> desperado::Result<()> {
    let device_index = 0;
    let sample_rate = 2_400_000;
    let center_freq = 1_090_000_000;
    let gain = Some(496);

    let reader = IqAsyncSource::from_rtlsdr(device_index, center_freq, sample_rate, gain).await?;

    while let Some(samples) = reader.next().await {
        // Process samples...
    }

    Ok(())
}
```

### More data sources

Desperado supports various data sources: the following table summarizes the available sources.

Methods are available in both synchronous (`IqSource`) and asynchronous (`AsyncIqSource`) versions.

| **Frontend**   | Method name                    | Optional feature | Identifier       |
| -------------- | ------------------------------ | ---------------- | ---------------- |
| I/Q File       | `[Async]IqSource::from_file`   |                  | file name        |
| Standard Input | `[Async]IqSource::from_stdin`  |                  |
| TCP socket     | `[Async]IqSource::from_tcp`    |                  | address and port |
| RTL-SDR        | `[Async]IqSource::from_rtlsdr` | `rtlsdr`         | device index     |
| Airspy         | `[Async]IqSource::from_airspy` | `airspy`         | device index     |
| HackRF         | `[Async]IqSource::from_hackrf` | `hackrf`         | device index     |
| Soapy          | `[Async]IqSource::from_soapy`  | `soapy`          | device arguments |
| Adalm-Pluto    | `[Async]IqSource::from_pluto`  | `pluto`          | URI              |

All samples are returned as `Complex<f32>` values, regardless of the source.

- The `rtlsdr` feature enables support for RTL-SDR devices (DVB-T dongles). It is based on the [`rtl-sdr-rs`](https://crates.io/crates/rtl-sdr-rs) crate which is a pure Rust implementation of the RTL-SDR driver.
- The `airspy` feature enables support for Airspy devices (R2, Mini, HF+). It is based on the `rs_spy` crate, a pure Rust implementation using `nusb` for USB access. Airspy hardware outputs real samples from a single ADC; the driver performs Fs/4 frequency translation and half-band filtering to produce proper I/Q output.
- The `hackrf` feature enables support for HackRF One devices. It is based on the `rs_hack` crate, a pure Rust implementation using `nusb` for USB access. HackRF outputs interleaved 8-bit signed I/Q samples (Cs8) directly over USB. The driver supports per-element gain control (LNA, VGA, amp) and bias-tee power.
- The `soapy` feature enables support for SoapySDR-compatible devices (LimeSDR, BladeRF, etc.). It is based on the [`soapysdr`](https://crates.io/crates/soapysdr) crate which provides Rust bindings to the SoapySDR C++ library. **The SoapySDR library must be installed separately**.
- The `pluto` feature enables support for Adalm-Pluto devices. It is based on the [`pluto-sdr`](https://crates.io/crates/pluto-sdr) crate which provides Rust bindings to the libiio C library. **The libiio library must be installed separately**.

Contributions to include more SDR frontends (LimeSDR, BladeRF, etc.) **or to port existing ones to pure Rust implementations** are welcome.

## Sync vs Async: Which should I use?

Desperado provides both synchronous (`IqSource`) and asynchronous (`AsyncIqSource`) interfaces. Here's how to choose:

Use Synchronous (`IqSource`) when:

- **Processing files offline**: Reading recorded I/Q files for analysis
- **Simple applications**: You don't need concurrent operations
- **Blocking is acceptable**: Your application can wait for I/O
- **Easier to reason about**: Simpler control flow

Use Asynchronous (`AsyncIqSource`) when:

- **Real-time processing**: Working with live SDR devices
- **Concurrent operations**: Processing multiple streams simultaneously
- **Non-blocking required**: Your application must remain responsive
- **Integrating with async ecosystem**: Using tokio, async-std, etc.

**Performance note**: For single-threaded file processing, synchronous can be faster due to less overhead. For real-time SDR applications, async is typically better.

## Performance Tuning

### Chunk Size Selection

The `chunk_size` parameter determines how many I/Q samples are read in each iteration. Choosing the right size affects both performance and latency:

**General guidelines**:

- **Small chunks (1K-4K samples)**: Lower latency, more overhead, good for interactive applications
- **Medium chunks (8K-16K samples)**: Balanced performance, recommended for most applications
- **Large chunks (32K-64K samples)**: Better throughput, higher latency, good for batch processing

**Example for different use cases**:

```rust,ignore
// Real-time ADS-B decoding (low latency needed)
let source = IqSource::from_file(path, freq, rate, 4096, format)?;

// General SDR processing (balanced)
let source = IqSource::from_file(path, freq, rate, 16384, format)?;

// Batch file processing (maximum throughput)
let source = IqSource::from_file(path, freq, rate, 65536, format)?;
```

### Hardware SDR Performance

**RTL-SDR tips**:

- Sample rates above 2.4 MS/s may cause USB bandwidth issues
- Use manual gain instead of AGC for better performance
- On Linux, consider increasing USB buffer size: `sudo modprobe rtl2832_sdr buffering=1`

**SoapySDR tips**:

- Check device-specific documentation for optimal buffer sizes
- Some devices benefit from specific stream arguments
- Monitor for dropped samples with verbose logging

**PlutoSDR tips**:

- Buffer sizes should match your processing requirements
- Network latency affects performance for IP-connected devices
- Use USB 3.0 connections when possible

**Airspy tips**:

- The R2 supports sample rates of 2.5 and 10 MSPS; the Mini supports 3 and 6 MSPS
- Uses pure-Rust USB access (nusb) — no system driver needed
- Gain modes: manual (LNA/Mixer/IF), linearity, or sensitivity

**HackRF tips**:

- Supports up to 20 MSPS with 8-bit I/Q samples
- Uses pure-Rust USB access (nusb) — no system driver needed
- Control LNA (0–40 dB), VGA (0–62 dB), and 14 dB RF amplifier independently
- Bias-tee output available for powering external LNAs

### Format Considerations

Different I/Q formats have different performance characteristics:

| Format | Bandwidth | Precision | Use Case                               |
| ------ | --------- | --------- | -------------------------------------- |
| Cu8    | Lowest    | 8-bit     | RTL-SDR, bandwidth-limited scenarios   |
| Cs8    | Low       | 8-bit     | Signed 8-bit devices                   |
| Cs16   | Medium    | 16-bit    | Higher dynamic range, Airspy, PlutoSDR |
| Cf32   | Highest   | 32-bit    | Pre-processed files, maximum precision |

**Recommendation**: Use the native format of your source when possible to avoid unnecessary conversions.

## Troubleshooting

### RTL-SDR kernel modules (Linux)

If the RTL kernel modules are installed you will need to temporarily unload them before using this library as follows:

```bash
sudo rmmod rtl2832_sdr
sudo rmmod dvb_usb_rtl28xxu
sudo rmmod rtl2832
sudo rmmod rtl8xxxu
```

Failure to do so will result in the following USB error:

```sh
thread 'main' panicked at 'Unable to open SDR device!: Usb(Busy)'
```

### RTL-SDR Device Detection

To list available RTL-SDR devices:

```bash
rtl_test
# or using the workspace example:
cargo run --example rtl_sdr -p desperado --features rtlsdr
```

### Airspy Device Detection

To list available Airspy devices and query firmware versions:

```bash
cargo run --example airspy_info -p rs-spy
# or stream a few samples with the integration test:
cargo run --example airspy_test -p desperado --features airspy
```

If you get an "I/O Error", the device may need time to reset after the previous run. Unplug and replug the device, or wait ~30 seconds.

### HackRF Device Detection

To list available HackRF devices and query firmware/board info:

```bash
cargo run --example hackrf_info -p rs-hack
```

If the device appears hung, unplug and replug it. On Linux, ensure you have a udev rule granting access to the USB device (vendor `1d50`, product `6089`):

```bash
# /etc/udev/rules.d/52-hackrf.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="6089", MODE="0666"
```

### SoapySDR Device Detection

To list available SoapySDR devices:

```bash
SoapySDRUtil --find
```

If your device isn't detected, ensure the appropriate SoapySDR module is installed (e.g., `SoapyRTLSDR`, `SoapyHackRF`, `SoapyLimeSDR`).

### Adalm-Pluto Device Detection

To list available Adalm-Pluto devices:

```bash
iio_info -s
```

## Finding I/Q Sample Files

If you need I/Q samples for testing or development:

### Public Datasets

- **[IQEngine](https://iqengine.org/)** - Public repository of RF recordings with metadata
- **[Signal Identification Wiki](https://www.sigidwiki.com/)** - Sample files for various signal types

### Capturing Your Own

With RTL-SDR:

```bash
rtl_sdr -f 1090000000 -s 2400000 -n 24000000 adsb_sample.iq
```

With SoapySDR:

```bash
SoapySDRUtil --rate=2.4e6 --freq=1090e6 --output=adsb_sample.iq
```

### File Formats

Desperado supports raw I/Q files in various formats:

- **Cu8** (Complex unsigned 8-bit): Most common for RTL-SDR, 2 bytes per sample
- **Cs8** (Complex signed 8-bit): 2 bytes per sample
- **Cs16** (Complex signed 16-bit): 4 bytes per sample, common for SoapySDR/Pluto
- **Cf32** (Complex 32-bit float): 8 bytes per sample, high precision

## Contributing

Contributions are welcome! Here are some ways you can help:

- **Add SDR device support**: LimeSDR, BladeRF, etc.  
  Consider pure Rust implementations where possible!
- **Improve documentation**: Fix typos, add examples, clarify explanations
- **Report bugs**: Open an issue with details and reproduction steps
- **Add tests**: Help improve test coverage
- **Share your project**: Using Desperado? Add it to the "Projects Using Desperado" section!

Please ensure:

- Code follows Rust conventions (`cargo fmt`, `cargo clippy`)
- All tests pass (`cargo test --all-features`)
- New features include documentation and tests

For major changes, please open an issue first to discuss the approach.

## License

This project is licensed under the MIT License. See the [license.md](license.md) file for details.
