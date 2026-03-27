/// rtl_sdr — RTL-SDR I/Q sample recorder, mirrors the C `rtl_sdr` utility.
///
/// Writes raw CU8 (interleaved u8 I/Q) bytes to stdout so the output can be
/// piped directly to dabradio or any other decoder.
///
/// Two read strategies:
///   --sync  : one `read_sync` call at a time (default matching `-S` in C rtl_sdr)
///   --async : uses `into_async_reader` with a dedicated USB thread (C rtl_sdr default)
///
/// Example — async mode piped to dabradio:
///   ./rtl_sdr --async -f 183648000 -s 2048000 -g 49.6 - | dabradio - 183648000
#[cfg(all(feature = "rtlsdr", feature = "clap"))]
use clap::Parser;
#[cfg(all(feature = "rtlsdr", feature = "clap"))]
use rtl_sdr_rs::{DEFAULT_ASYNC_BUF_NUMBER, DEFAULT_BUF_LENGTH, RtlSdr, TunerGain};
#[cfg(all(feature = "rtlsdr", feature = "clap"))]
use std::io::{self, Write};

#[cfg(all(feature = "rtlsdr", feature = "clap"))]
#[derive(Parser, Debug)]
#[command(
    name = "rtl_sdr",
    about = "RTL-SDR I/Q recorder — writes raw CU8 bytes to stdout"
)]
struct Args {
    /// Center frequency in Hz (e.g. 183648000)
    #[arg(short = 'f', long)]
    frequency: u32,

    /// Sample rate in Hz (default: 2048000)
    #[arg(short = 's', long, default_value = "2048000")]
    sample_rate: u32,

    /// Tuner gain in dB; 0 = auto (default: 0)
    #[arg(short = 'g', long, default_value = "0.0")]
    gain: f32,

    /// Frequency correction in PPM (default: 0)
    #[arg(short = 'p', long, default_value = "0")]
    ppm: i32,

    /// Device index (default: 0)
    #[arg(short = 'd', long, default_value = "0")]
    device: usize,

    /// Number of samples to read (0 = infinite)
    #[arg(short = 'n', long, default_value = "0")]
    num_samples: u64,

    /// Output block size in bytes (default: 16*16384 = 262144)
    #[arg(short = 'b', long, default_value = "262144")]
    block_size: usize,

    /// Use synchronous read_sync loop (like `rtl_sdr -S`).
    /// Default (no flag) uses into_async_reader.
    #[arg(short = 'S', long = "sync", default_value = "false")]
    sync_mode: bool,

    /// Output filename or "-" for stdout (ignored, always stdout)
    output: Option<String>,
}

#[cfg(all(feature = "rtlsdr", feature = "clap"))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Validate block size
    if args.block_size == 0 || args.block_size % 512 != 0 {
        eprintln!("Block size must be a non-zero multiple of 512");
        std::process::exit(1);
    }

    let mut sdr = RtlSdr::open_with_index(args.device)?;
    sdr.set_sample_rate(args.sample_rate)?;
    let _ = sdr.set_tuner_bandwidth(args.sample_rate);
    let _ = sdr.set_freq_correction(args.ppm);
    sdr.set_center_freq(args.frequency)?;

    if args.gain == 0.0 {
        sdr.set_tuner_gain(TunerGain::Auto)?;
        eprintln!(
            "Tuned to {} Hz, {} sps, gain auto",
            args.frequency, args.sample_rate
        );
    } else {
        let gain_tenths = (args.gain * 10.0) as i32;
        sdr.set_tuner_gain(TunerGain::Manual(gain_tenths))?;
        eprintln!(
            "Tuned to {} Hz, {} sps, gain {:.1} dB",
            args.frequency, args.sample_rate, args.gain
        );
    }

    sdr.reset_buffer()?;

    let stdout = io::stdout();
    let mut out = io::BufWriter::with_capacity(args.block_size * 4, stdout.lock());

    let infinite = args.num_samples == 0;
    let mut remaining = args.num_samples;

    if args.sync_mode {
        eprintln!("Read mode: sync (read_sync loop)");
        let mut buf = vec![0u8; args.block_size];
        loop {
            let n = sdr.read_sync(&mut buf)?;
            if n == 0 {
                continue;
            }
            let chunk = &buf[..n];
            if !infinite {
                // num_samples counts IQ sample pairs; each pair = 2 bytes
                let samples_in_chunk = n as u64 / 2;
                if samples_in_chunk >= remaining {
                    out.write_all(&chunk[..(remaining as usize * 2)])?;
                    out.flush()?;
                    break;
                }
                remaining -= samples_in_chunk;
            }
            out.write_all(chunk)?;
        }
    } else {
        eprintln!("Read mode: async (into_multi_transfer_reader — 15 concurrent libusb transfers)");
        // Use 15 concurrent libusb bulk transfers so the RTL-SDR hardware FIFO
        // never sees a gap between transfers, matching C rtl_sdr behaviour.
        let handle =
            sdr.into_multi_transfer_reader(DEFAULT_ASYNC_BUF_NUMBER, DEFAULT_BUF_LENGTH)?;
        for result in handle {
            let bytes = result?;
            if !infinite {
                let samples_in_chunk = bytes.len() as u64 / 2;
                if samples_in_chunk >= remaining {
                    out.write_all(&bytes[..(remaining as usize * 2)])?;
                    break;
                }
                remaining -= samples_in_chunk;
            }
            out.write_all(&bytes)?;
        }
        out.flush()?;
    }

    Ok(())
}

#[cfg(not(all(feature = "rtlsdr", feature = "clap")))]
fn main() {}
