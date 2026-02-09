//! Stream samples from Airspy to a file.
//!
//! This example demonstrates synchronous bulk streaming from an Airspy device.
//! Samples are written to a file in raw 16-bit format (compatible with GNU Radio,
//! Inspectrum, and other SDR tools).
//!
//! # Usage
//!
//! ```sh
//! # Stream 1 second of samples at 100 MHz to a file
//! cargo run --example airspy_rx -- -f 100000000 -o samples.raw -d 1
//!
//! # Stream with custom gain settings
//! cargo run --example airspy_rx -- -f 433920000 -o samples.raw -d 5 --sensitivity 15
//!
//! # Stream to stdout (pipe to other tools)
//! cargo run --example airspy_rx -- -f 100000000 -d 1 | inspectrum /dev/stdin
//! ```
//!
//! # Output Format
//!
//! - Raw 16-bit unsigned samples (little-endian)
//! - Sample values are 12-bit resolution in the upper bits (0-4095 << 4)
//! - Real samples (not IQ) - use external tools for IQ conversion

use rs_spy::{Airspy, RECOMMENDED_BUFFER_SIZE};
use std::env;
use std::fs::File;
use std::io::{self, Write};
use std::process;
use std::time::{Duration, Instant};

struct Args {
    freq_hz: Option<u32>,
    output_file: Option<String>,
    duration_secs: f64,
    sensitivity_gain: Option<u8>,
    linearity_gain: Option<u8>,
    sample_rate_index: u8,
    help: bool,
}

fn parse_args() -> Args {
    let mut args = Args {
        freq_hz: None,
        output_file: None,
        duration_secs: 1.0,
        sensitivity_gain: None,
        linearity_gain: None,
        sample_rate_index: 0,
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
            "-s" | "--sensitivity" => {
                i += 1;
                if i < argv.len() {
                    args.sensitivity_gain = argv[i].parse().ok();
                }
            }
            "-l" | "--linearity" => {
                i += 1;
                if i < argv.len() {
                    args.linearity_gain = argv[i].parse().ok();
                }
            }
            "-r" | "--rate" => {
                i += 1;
                if i < argv.len() {
                    args.sample_rate_index = argv[i].parse().unwrap_or(0);
                }
            }
            _ => eprintln!("Unknown argument: {}", argv[i]),
        }
        i += 1;
    }

    args
}

fn print_help() {
    println!("airspy_rx - Stream samples from Airspy to file");
    println!();
    println!("USAGE:");
    println!("    airspy_rx [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -h, --help              Show this help message");
    println!("    -f, --freq <HZ>         Set frequency in Hz (required)");
    println!("    -o, --output <FILE>     Output file (default: stdout)");
    println!("    -d, --duration <SECS>   Recording duration in seconds (default: 1.0)");
    println!("    -r, --rate <INDEX>      Sample rate index (default: 0 = highest)");
    println!("    -s, --sensitivity <N>   Use sensitivity gain preset (0-21)");
    println!("    -l, --linearity <N>     Use linearity gain preset (0-21)");
    println!();
    println!("EXAMPLES:");
    println!("    # Record 5 seconds of FM broadcast at 100 MHz");
    println!("    airspy_rx -f 100000000 -o fm.raw -d 5 -s 15");
    println!();
    println!("    # Record ADS-B at 1090 MHz with linearity gain");
    println!("    airspy_rx -f 1090000000 -o adsb.raw -d 10 -l 18");
    println!();
    println!("OUTPUT FORMAT:");
    println!("    - Raw 16-bit unsigned samples (little-endian)");
    println!("    - Compatible with GNU Radio, Inspectrum, etc.");
}

fn main() {
    // Initialize tracing for debug output
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
            eprintln!("  Total samples: {}", stats.total_samples);
            eprintln!("  Duration: {:.2} s", stats.duration_secs);
            eprintln!(
                "  Effective rate: {:.3} MSPS",
                stats.total_samples as f64 / stats.duration_secs / 1_000_000.0
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
    total_samples: usize,
    duration_secs: f64,
}

fn run(args: &Args) -> Result<Stats, Box<dyn std::error::Error>> {
    // Open the Airspy device
    eprintln!("Opening Airspy device...");
    let airspy = Airspy::open_first()?;

    // Get device info
    let board_id = airspy.board_id()?;
    eprintln!("Board: {} (ID: {})", board_id_name(board_id), board_id);

    // Show sample rates
    let sample_rates = airspy.supported_sample_rates()?;
    eprintln!("Sample rates:");
    for (i, rate) in sample_rates.iter().enumerate() {
        let marker = if i == args.sample_rate_index as usize {
            " <--"
        } else {
            ""
        };
        eprintln!("  [{}] {:.3} MSPS{}", i, *rate as f64 / 1_000_000.0, marker);
    }

    let sample_rate = sample_rates
        .get(args.sample_rate_index as usize)
        .copied()
        .unwrap_or(sample_rates[0]);

    // Configure the device
    let freq_hz = args.freq_hz.unwrap();
    eprintln!(
        "Setting frequency to {:.3} MHz...",
        freq_hz as f64 / 1_000_000.0
    );
    airspy.set_freq(freq_hz)?;

    // Set gain
    if let Some(gain) = args.sensitivity_gain {
        eprintln!("Setting sensitivity gain to {}...", gain);
        airspy.set_sensitivity_gain(gain)?;
    } else if let Some(gain) = args.linearity_gain {
        eprintln!("Setting linearity gain to {}...", gain);
        airspy.set_linearity_gain(gain)?;
    } else {
        // Default to mid-range sensitivity
        eprintln!("Using default sensitivity gain (10)...");
        airspy.set_sensitivity_gain(10)?;
    }

    // Open output file or use stdout
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
    let expected_bytes = (sample_rate as f64 * 2.0 * args.duration_secs) as usize;
    eprintln!(
        "Expected data: {:.2} MB ({:.2} seconds at {:.3} MSPS)",
        expected_bytes as f64 / 1_000_000.0,
        args.duration_secs,
        sample_rate as f64 / 1_000_000.0
    );

    // Start streaming
    eprintln!("Starting streaming...");
    airspy.start_rx()?;

    // Allocate buffer
    let mut buf = vec![0u8; RECOMMENDED_BUFFER_SIZE];
    let mut total_bytes: usize = 0;
    let start_time = Instant::now();
    let duration = Duration::from_secs_f64(args.duration_secs);

    // Read loop
    while start_time.elapsed() < duration {
        match airspy.read_sync(&mut buf) {
            Ok(n) if n > 0 => {
                output.write_all(&buf[..n])?;
                total_bytes += n;

                // Progress indicator (to stderr so it doesn't mix with data)
                let elapsed = start_time.elapsed().as_secs_f64();
                let rate = total_bytes as f64 / elapsed / 1_000_000.0;
                eprint!(
                    "\rReceived: {:.2} MB ({:.2} MB/s)  ",
                    total_bytes as f64 / 1_000_000.0,
                    rate
                );
            }
            Ok(_) => {
                // Zero bytes read, might be timeout
                tracing::debug!("Zero-length read");
            }
            Err(e) => {
                eprintln!("\nRead error: {}", e);
                break;
            }
        }
    }

    // Stop streaming
    eprintln!("\nStopping streaming...");
    airspy.stop_rx()?;

    let total_samples = total_bytes / 2; // 16-bit samples
    let actual_duration = start_time.elapsed().as_secs_f64();

    Ok(Stats {
        total_bytes,
        total_samples,
        duration_secs: actual_duration,
    })
}

fn board_id_name(id: u32) -> &'static str {
    match id {
        0 => "AIRSPY",
        1 => "AIRSPY MINI",
        2 => "AIRSPY HF+",
        _ => "UNKNOWN",
    }
}
