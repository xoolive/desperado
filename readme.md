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
desperado = { version = "0.1", features = ["soapy"] }   # For SoapySDR-compatible devices
# or
desperado = { version = "0.1", features = ["pluto"] }   # For Adalm-Pluto devices
```

### Available features

- **`rtlsdr`**: RTL-SDR device support (DVB-T dongles)
- **`soapy`**: SoapySDR device support (HackRF, LimeSDR, etc.)
- **`pluto`**: Adalm-Pluto SDR support

The following features are only needed for examples:

- **`clap`**: Command-line argument parsing for examples
- **`waterfall`**: Waterfall plot visualization example
- **`audio`**: FM demodulation examples with audio output

## Projects using desperado

- **[jet1090](https://github.com/xoolive/jet1090)** - Real-time ADS-B decoder for tracking aircraft
- **[ship162](https://github.com/xoolive/ship162)** - AIS decoder for tracking maritime vessels

If you're using Desperado in your project, feel free to open a PR to add it here!

## Usage

### Basic example (synchronous version)

```rust
use desperado::{IqFormat, IqSource};

fn main() -> Result<(), Box<dyn std::error::Error>> {
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

### RTL-SDR (sync and async version)

Access to RTL-SDR devices is provided with the `rtlsdr` feature enabled.

```rust
use desperado::AsyncIqSource;
use futures::StreamExt;

#[tokio::main]
async fn main() -> tokio::io::Result<()> {
    let device_index = 0;
    let sample_rate = 2_400_000;
    let center_freq = 1_090_000_000;
    let gain = Some(496);

    let reader = AsyncIqSource::from_rtlsdr(device_index, center_freq, sample_rate, gain).await?;

    while let Some(samples) = reader.next().await {
        // Process samples...
    }

    Ok(())
}
```

### More data sources

Desperado supports various data sources: the following table summarizes the available sources.

Methods are available in both synchronous (`IqSource`) and asynchronous (`AsyncIqSource`) versions.

| **Frontend**   | Method name                    | Source format    | Optional Feature | Identifier       |
| -------------- | ------------------------------ | ---------------- | ---------------- | ---------------- |
| I/Q File       | `[Async]IqSource::from_file`   | any              |                  | file name        |
| Standard Input | `[Async]IqSource::from_stdin`  | any              |                  |
| TCP socket     | `[Async]IqSource::from_tcp`    | any              |                  | address and port |
| RTL-SDR        | `[Async]IqSource::from_rtlsdr` | `IqFormat::Cu8`  | `rtlsdr`         | device index     |
| Soapy          | `[Async]IqSource::from_soapy`  | `IqFormat::Cs16` | `soapy`          | device arguments |
| Adalm-Pluto    | `[Async]IqSource::from_pluto`  | `IqFormat::Cs16` | `pluto`          | URI              |

All samples are returned as `Complex<f32>` values, regardless of the source.

Contributions to include more SDR frontends (LimeSDR, HackRF, etc.) are welcome.
