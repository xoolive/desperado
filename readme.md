# Desperado

Desperado is a library designed to factorize and reuse code for reading I/Q samples from files, SDR devices, and other sources.

Desperado provides a unified interface for iterating (synchronously) and streaming (asynchronously) complex I/Q samples in `Complex<f32>` format.

The library intentionally does not include demodulation, focusing instead on providing a consistent interface over various sources.

While basic digital signal processing (DSP) utilities (such as those used in waterfall visualizations) may be added, the core goal is to keep the interface simple and extensible. A Python wrapper with the same interface is also planned for future development.

The name “Desperado” is a playful nod to DSP (Digital Signal Processing), and tips its hat to <https://www.youtube.com/watch?v=-q93wc3-deU>.

## Usage

### Basic Example

```rust
use desperado::{IqFormat, IqSource};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create an IQ source from a binary file
    let path = "sample.iq";
    let sample_rate = 96_000;
    let center_freq = 162_000_000;
    let chunk_size = 8136;
    let reader = IqSource::from_file(path, center_freq, sample_rate, chunk_size, IqFormat::Cu8)?;


    for samples in reader {
        for s in samples? {  // samples is a Result<Vec<Complex<f32>>, _>
            println!("  I: {}, Q: {}", s.re, s.im);
        }
    }
    Ok(())
}
```

### RTL-SDR (async version)

```rust
use desperado::{IqFormat, IqSource, RtlSdrConfig};


fn main() -> Result<(), Box<dyn std::error::Error>> {
    let device_index = 0;
    let sample_rate = 2_400_000;
    let center_freq = 1_090_000_000;
    let gain = Some(496);

    let reader = from_rtlsdr(
        device_index: usize,
        center_freq: u32,
        sample_rate: u32,
        gain: Option<i32>,
    ).await?;

    while let Some(samples) = reader.next().await {
        // Process samples...
    }

    Ok(())
}
```
