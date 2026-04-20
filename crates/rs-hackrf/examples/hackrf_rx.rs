//! Stream RX samples from HackRF to a file.
//!
//! This example demonstrates synchronous bulk streaming from a HackRF device.
//! Samples are written to a file in raw 8-bit signed I/Q format (compatible
//! with GNU Radio, Inspectrum, hackrf_transfer, and other SDR tools).
//!
//! # Usage
//!
//! ```sh
//! # Stream 1 second of samples at 100 MHz to a file
//! cargo run --example hackrf_rx -- -f 100000000 -o samples.raw -d 1
//!
//! # Stream with custom gains and sample rate
//! cargo run --example hackrf_rx -- -f 433920000 -o samples.raw -d 5 --lna 24 --vga 20
//!
//! # Stream to stdout (pipe to other tools)
//! cargo run --example hackrf_rx -- -f 100000000 -d 1 | inspectrum /dev/stdin
//! ```
//!
//! # Output Format
//!
//! - Raw 8-bit signed I/Q pairs: [I0, Q0, I1, Q1, ...]
//! - Each I and Q sample is an `i8` value (-128 to 127)
//! - Compatible with hackrf_transfer -r output

use rs_hackrf::transport::board_id_name;
use rs_hackrf::{HackRf, RECOMMENDED_BUFFER_SIZE};
use std::env;
use std::fs::File;
use std::io::{self, Write};
use std::process;
use std::time::{Duration, Instant};

struct Args {
    freq_hz: Option<u64>,
    sample_rate: u32,
    output_file: Option<String>,
    duration_secs: f64,
    lna_gain: u32,
    vga_gain: u32,
    amp_enable: bool,
    help: bool,
}

fn parse_args() -> Args {
    let mut args = Args {
        freq_hz: None,
        sample_rate: 10_000_000, // 10 MSPS default (matches hackrf_transfer)
        output_file: None,
        duration_secs: 1.0,
        lna_gain: 16, // Sensible default
        vga_gain: 20, // Sensible default
        amp_enable: false,
        help: false,
    };

    let argv: Vec<String> = env::args().collect();
    let mut i = 1;

    while i < argv.len() {
        match argv[i].as_str() {
            "-h" | "--help" => args.help = true,
            "-f" | "--freq" => {
                i += 1;
                if i < argv.len() {
                    args.freq_hz = argv[i].parse().ok();
                }
            }
            "-r" | "--rate" => {
                i += 1;
                if i < argv.len() {
                    args.sample_rate = argv[i].parse().unwrap_or(10_000_000);
                }
            }
            "-o" | "--output" => {
                i += 1;
                if i < argv.len() {
                    args.output_file = Some(argv[i].clone());
                }
            }
            "-d" | "--duration" => {
                i += 1;
                if i < argv.len() {
                    args.duration_secs = argv[i].parse().unwrap_or(1.0);
                }
            }
            "--lna" => {
                i += 1;
                if i < argv.len() {
                    args.lna_gain = argv[i].parse().unwrap_or(16);
                }
            }
            "--vga" => {
                i += 1;
                if i < argv.len() {
                    args.vga_gain = argv[i].parse().unwrap_or(20);
                }
            }
            "-a" | "--amp" => args.amp_enable = true,
            _ => eprintln!("Unknown argument: {}", argv[i]),
        }
        i += 1;
    }

    args
}

fn print_help() {
    println!("hackrf_rx - Stream RX samples from HackRF to file");
    println!();
    println!("USAGE:");
    println!("    hackrf_rx [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -h, --help              Show this help message");
    println!("    -f, --freq <HZ>         Set frequency in Hz (required)");
    println!("    -r, --rate <HZ>         Sample rate in Hz (default: 10000000)");
    println!("    -o, --output <FILE>     Output file (default: stdout)");
    println!("    -d, --duration <SECS>   Recording duration in seconds (default: 1.0)");
    println!("    --lna <GAIN>            LNA gain in dB, 0-40, 8 dB steps (default: 16)");
    println!("    --vga <GAIN>            VGA gain in dB, 0-62, 2 dB steps (default: 20)");
    println!("    -a, --amp               Enable RF amplifier (+14 dB)");
    println!();
    println!("EXAMPLES:");
    println!("    # Record 5 seconds of FM broadcast at 100 MHz");
    println!("    hackrf_rx -f 100000000 -o fm.raw -d 5 --lna 24 --vga 20");
    println!();
    println!("    # Record ADS-B at 1090 MHz with amp");
    println!("    hackrf_rx -f 1090000000 -o adsb.raw -d 10 --lna 32 --vga 40 --amp");
    println!();
    println!("OUTPUT FORMAT:");
    println!("    - Raw 8-bit signed I/Q pairs: [I0, Q0, I1, Q1, ...]");
    println!("    - Compatible with hackrf_transfer, GNU Radio, Inspectrum");
}

fn main() {
    tracing_subscriber::fmt::init();

    let args = parse_args();

    if args.help {
        print_help();
        process::exit(0);
    }

    if args.freq_hz.is_none() {
        eprintln!("Error: Frequency is required. Use -f <HZ> to set frequency.");
        eprintln!("Use --help for usage information.");
        process::exit(1);
    }

    match run(&args) {
        Ok(stats) => {
            eprintln!();
            eprintln!("Recording complete:");
            eprintln!(
                "  Total bytes: {} ({:.2} MB)",
                stats.total_bytes,
                stats.total_bytes as f64 / 1_000_000.0
            );
            eprintln!("  Total I/Q samples: {}", stats.total_bytes / 2);
            eprintln!("  Duration: {:.2} s", stats.duration_secs);
            eprintln!(
                "  Effective rate: {:.3} MSPS",
                (stats.total_bytes / 2) as f64 / stats.duration_secs / 1_000_000.0
            );
            process::exit(0);
        }
        Err(e) => {
            eprintln!("Error: {}", e);
            process::exit(1);
        }
    }
}

struct Stats {
    total_bytes: usize,
    duration_secs: f64,
}

fn run(args: &Args) -> Result<Stats, Box<dyn std::error::Error>> {
    eprintln!("Opening HackRF device...");
    let mut hackrf = HackRf::open_first()?;

    // Device info
    let board_id = hackrf.board_id()?;
    eprintln!("Board: {} (ID: {})", board_id_name(board_id), board_id);

    let version = hackrf.version().unwrap_or_else(|_| "(unknown)".to_string());
    eprintln!("Firmware: {}", version);

    // Configure
    let freq_hz = args.freq_hz.unwrap();
    eprintln!(
        "Setting sample rate to {:.3} MSPS...",
        args.sample_rate as f64 / 1_000_000.0
    );
    hackrf.set_sample_rate(args.sample_rate)?;

    eprintln!(
        "Setting frequency to {:.3} MHz...",
        freq_hz as f64 / 1_000_000.0
    );
    hackrf.set_freq(freq_hz)?;

    eprintln!("Setting LNA gain to {} dB...", args.lna_gain);
    hackrf.set_lna_gain(args.lna_gain)?;

    eprintln!("Setting VGA gain to {} dB...", args.vga_gain);
    hackrf.set_vga_gain(args.vga_gain)?;

    if args.amp_enable {
        eprintln!("Enabling RF amplifier...");
        hackrf.set_amp_enable(true)?;
    }

    // Open output
    let mut output: Box<dyn Write> = match &args.output_file {
        Some(path) => {
            eprintln!("Writing to: {}", path);
            Box::new(File::create(path)?)
        }
        None => {
            eprintln!("Writing to stdout...");
            Box::new(io::stdout())
        }
    };

    // Calculate expected data
    // HackRF: 2 bytes per I/Q sample (8-bit I + 8-bit Q)
    let expected_bytes = (args.sample_rate as f64 * 2.0 * args.duration_secs) as usize;
    eprintln!(
        "Expected data: {:.2} MB ({:.2} seconds at {:.3} MSPS)",
        expected_bytes as f64 / 1_000_000.0,
        args.duration_secs,
        args.sample_rate as f64 / 1_000_000.0
    );

    // Start streaming
    eprintln!("Starting RX streaming...");
    hackrf.start_rx()?;

    let mut buf = vec![0u8; RECOMMENDED_BUFFER_SIZE];
    let mut total_bytes: usize = 0;
    let start_time = Instant::now();
    let duration = Duration::from_secs_f64(args.duration_secs);

    // Read loop
    while start_time.elapsed() < duration {
        match hackrf.read_sync(&mut buf) {
            Ok(n) if n > 0 => {
                output.write_all(&buf[..n])?;
                total_bytes += n;

                let elapsed = start_time.elapsed().as_secs_f64();
                let rate = total_bytes as f64 / elapsed / 1_000_000.0;
                eprint!(
                    "\rReceived: {:.2} MB ({:.2} MB/s)  ",
                    total_bytes as f64 / 1_000_000.0,
                    rate
                );
            }
            Ok(_) => {
                tracing::debug!("Zero-length read");
            }
            Err(e) => {
                eprintln!("\nRead error: {}", e);
                break;
            }
        }
    }

    // Stop streaming
    eprintln!("\nStopping RX...");
    hackrf.stop_rx()?;

    let actual_duration = start_time.elapsed().as_secs_f64();

    Ok(Stats {
        total_bytes,
        duration_secs: actual_duration,
    })
}
