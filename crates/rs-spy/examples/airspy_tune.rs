//! CLI tool to configure and tune Airspy devices.
//!
//! This is a Phase 2 example demonstrating:
//! - Frequency tuning
//! - Sample rate selection
//! - Gain configuration (manual, linearity, sensitivity presets)
//! - Bias-T control
//!
//! # Usage
//!
//! ```sh
//! # Show device info and current settings
//! cargo run --example airspy_tune
//!
//! # Tune to 100 MHz FM broadcast band
//! cargo run --example airspy_tune -- --freq 100000000
//!
//! # Tune with specific gain settings
//! cargo run --example airspy_tune -- --freq 433920000 --lna 10 --mixer 8 --vga 12
//!
//! # Use linearity gain preset (good for strong signals)
//! cargo run --example airspy_tune -- --freq 100000000 --linearity 15
//!
//! # Use sensitivity gain preset (good for weak signals)
//! cargo run --example airspy_tune -- --freq 100000000 --sensitivity 18
//!
//! # Enable Bias-T (CAUTION: only for devices that need DC power!)
//! cargo run --example airspy_tune -- --freq 1090000000 --bias-t
//! ```

use rs_spy::Airspy;
use std::env;
use std::process;

/// Simple argument parsing for the example
struct Args {
    freq_hz: Option<u32>,
    sample_rate_index: Option<u8>,
    lna_gain: Option<u8>,
    mixer_gain: Option<u8>,
    vga_gain: Option<u8>,
    linearity_gain: Option<u8>,
    sensitivity_gain: Option<u8>,
    bias_t: bool,
    help: bool,
}

fn parse_args() -> Args {
    let mut args = Args {
        freq_hz: None,
        sample_rate_index: None,
        lna_gain: None,
        mixer_gain: None,
        vga_gain: None,
        linearity_gain: None,
        sensitivity_gain: None,
        bias_t: false,
        help: false,
    };

    let argv: Vec<String> = env::args().collect();
    let mut i = 1;

    while i < argv.len() {
        match argv[i].as_str() {
            "-h" | "--help" => {
                args.help = true;
            }
            "-f" | "--freq" => {
                i += 1;
                if i < argv.len() {
                    args.freq_hz = argv[i].parse().ok();
                }
            }
            "-r" | "--rate" => {
                i += 1;
                if i < argv.len() {
                    args.sample_rate_index = argv[i].parse().ok();
                }
            }
            "--lna" => {
                i += 1;
                if i < argv.len() {
                    args.lna_gain = argv[i].parse().ok();
                }
            }
            "--mixer" => {
                i += 1;
                if i < argv.len() {
                    args.mixer_gain = argv[i].parse().ok();
                }
            }
            "--vga" => {
                i += 1;
                if i < argv.len() {
                    args.vga_gain = argv[i].parse().ok();
                }
            }
            "-l" | "--linearity" => {
                i += 1;
                if i < argv.len() {
                    args.linearity_gain = argv[i].parse().ok();
                }
            }
            "-s" | "--sensitivity" => {
                i += 1;
                if i < argv.len() {
                    args.sensitivity_gain = argv[i].parse().ok();
                }
            }
            "-b" | "--bias-t" => {
                args.bias_t = true;
            }
            _ => {
                eprintln!("Unknown argument: {}", argv[i]);
            }
        }
        i += 1;
    }

    args
}

fn print_help() {
    println!("airspy_tune - Configure and tune Airspy devices");
    println!();
    println!("USAGE:");
    println!("    airspy_tune [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -h, --help              Show this help message");
    println!("    -f, --freq <HZ>         Set frequency in Hz (e.g., 100000000 for 100 MHz)");
    println!("    -r, --rate <INDEX>      Set sample rate by index (0 = first rate)");
    println!("    --lna <GAIN>            Set LNA gain (0-14)");
    println!("    --mixer <GAIN>          Set mixer gain (0-15)");
    println!("    --vga <GAIN>            Set VGA gain (0-15)");
    println!("    -l, --linearity <GAIN>  Set linearity gain preset (0-21)");
    println!("    -s, --sensitivity <GAIN> Set sensitivity gain preset (0-21)");
    println!("    -b, --bias-t            Enable Bias-T (DC on antenna port)");
    println!();
    println!("EXAMPLES:");
    println!("    # Tune to FM broadcast band with sensitivity preset");
    println!("    airspy_tune --freq 100000000 --sensitivity 15");
    println!();
    println!("    # Tune to 433 MHz ISM band with manual gains");
    println!("    airspy_tune --freq 433920000 --lna 10 --mixer 8 --vga 12");
    println!();
    println!("    # Tune to ADS-B 1090 MHz with Bias-T for LNA");
    println!("    airspy_tune --freq 1090000000 --linearity 18 --bias-t");
    println!();
    println!("NOTES:");
    println!("    - Airspy Mini frequency range: 24 MHz to 1.8 GHz");
    println!("    - Use --linearity for strong signals (less distortion)");
    println!("    - Use --sensitivity for weak signals (lower noise floor)");
    println!("    - WARNING: --bias-t puts DC voltage on the antenna port!");
}

fn main() {
    // Initialize tracing for debug output
    tracing_subscriber::fmt::init();

    let args = parse_args();

    if args.help {
        print_help();
        process::exit(0);
    }

    match run(&args) {
        Ok(()) => process::exit(0),
        Err(e) => {
            eprintln!("Error: {}", e);
            process::exit(1);
        }
    }
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    // Open the first available Airspy device
    println!("Opening Airspy device...");
    let airspy = Airspy::open_first()?;

    // Get and display device info
    let board_id = airspy.board_id()?;
    println!("Board: {} (ID: {})", board_id_name(board_id), board_id);

    let version = airspy.version().unwrap_or_else(|_| "(unknown)".to_string());
    println!("Firmware: {}", version);

    // Show supported sample rates
    let sample_rates = airspy.supported_sample_rates()?;
    println!("Supported sample rates:");
    for (i, rate) in sample_rates.iter().enumerate() {
        println!("  [{}] {:.3} MSPS", i, *rate as f64 / 1_000_000.0);
    }
    println!();

    // Apply configuration if specified
    let mut configured = false;

    // Set sample rate
    if let Some(rate_index) = args.sample_rate_index {
        if (rate_index as usize) < sample_rates.len() {
            println!(
                "Setting sample rate to index {} ({:.3} MSPS)...",
                rate_index,
                sample_rates[rate_index as usize] as f64 / 1_000_000.0
            );
            airspy.set_sample_rate(rate_index)?;
            println!("  OK");
            configured = true;
        } else {
            eprintln!(
                "Warning: Sample rate index {} out of range (0-{})",
                rate_index,
                sample_rates.len() - 1
            );
        }
    }

    // Set frequency
    if let Some(freq_hz) = args.freq_hz {
        println!(
            "Setting frequency to {} Hz ({:.3} MHz)...",
            freq_hz,
            freq_hz as f64 / 1_000_000.0
        );
        airspy.set_freq(freq_hz)?;
        println!("  OK");
        configured = true;
    }

    // Set gain using presets OR manual values
    if let Some(gain) = args.linearity_gain {
        println!("Setting linearity gain to {}...", gain);
        airspy.set_linearity_gain(gain)?;
        println!("  OK (LNA, Mixer, VGA set from lookup table)");
        configured = true;
    } else if let Some(gain) = args.sensitivity_gain {
        println!("Setting sensitivity gain to {}...", gain);
        airspy.set_sensitivity_gain(gain)?;
        println!("  OK (LNA, Mixer, VGA set from lookup table)");
        configured = true;
    } else {
        // Manual gain settings
        if let Some(lna) = args.lna_gain {
            println!("Setting LNA gain to {}...", lna);
            airspy.set_lna_gain(lna)?;
            println!("  OK");
            configured = true;
        }

        if let Some(mixer) = args.mixer_gain {
            println!("Setting mixer gain to {}...", mixer);
            airspy.set_mixer_gain(mixer)?;
            println!("  OK");
            configured = true;
        }

        if let Some(vga) = args.vga_gain {
            println!("Setting VGA gain to {}...", vga);
            airspy.set_vga_gain(vga)?;
            println!("  OK");
            configured = true;
        }
    }

    // Set Bias-T
    if args.bias_t {
        println!("WARNING: Enabling Bias-T (DC on antenna port)...");
        airspy.set_rf_bias(true)?;
        println!("  OK - Bias-T ENABLED");
        configured = true;
    }

    if !configured {
        println!("No configuration changes requested.");
        println!("Use --help to see available options.");
    } else {
        println!();
        println!("Configuration complete.");
    }

    println!("Closing device...");
    // Device is automatically closed when `airspy` goes out of scope
    Ok(())
}

/// Get human-readable board ID name.
fn board_id_name(id: u32) -> &'static str {
    match id {
        0 => "AIRSPY",
        1 => "AIRSPY MINI",
        2 => "AIRSPY HF+",
        _ => "UNKNOWN",
    }
}
