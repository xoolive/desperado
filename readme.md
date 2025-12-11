<div align="center">
  <img src="logo.png" alt="Desperado Logo" width="200"/>
</div>

# Desperado

Desperado is a library designed to factorize and reuse code for reading I/Q samples from files, SDR devices, and other sources.

Desperado provides a unified interface for iterating (synchronously) and streaming (asynchronously) complex I/Q samples in `Complex<f32>` format.

The library intentionally does not include demodulation, focusing instead on providing a consistent interface over various sources.

The name “Desperado” is a playful nod to DSP (Digital Signal Processing), and tips its hat to <https://www.youtube.com/watch?v=-q93wc3-deU>.

## Usage

### Basic Example (synchronous version)

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
