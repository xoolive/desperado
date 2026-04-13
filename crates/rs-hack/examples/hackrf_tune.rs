//! CLI tool to configure and tune HackRF devices.
//!
//! This is a Phase 2 example demonstrating:
//! - Frequency tuning
//! - Sample rate configuration
//! - LNA and VGA gain control
//! - RF amplifier enable
//! - Bias tee control
//!
//! # Usage
//!
//! ```sh
//! # Show device info
//! cargo run --example hackrf_tune
//!
//! # Tune to 100 MHz FM broadcast band
//! cargo run --example hackrf_tune -- --freq 100000000
//!
//! # Tune with specific gain settings
//! cargo run --example hackrf_tune -- --freq 433920000 --lna 24 --vga 20
//!
//! # Set sample rate and enable RF amp
//! cargo run --example hackrf_tune -- --freq 1090000000 --rate 8000000 --amp
//!
//! # Enable bias tee (CAUTION: DC on antenna port!)
//! cargo run --example hackrf_tune -- --freq 1090000000 --bias-t
//! ```

use rs_hack::HackRf;
use rs_hack::transport::board_id_name;
use std::env;
use std::process;

struct Args {
    freq_hz: Option<u64>,
    sample_rate: Option<u32>,
    lna_gain: Option<u32>,
    vga_gain: Option<u32>,
    amp_enable: bool,
    bias_t: bool,
    help: bool,
}

fn parse_args() -> Args {
    let mut args = Args {
        freq_hz: None,
        sample_rate: None,
        lna_gain: None,
        vga_gain: None,
        amp_enable: false,
        bias_t: false,
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
                    args.sample_rate = argv[i].parse().ok();
                }
            }
            "--lna" => {
                i += 1;
                if i < argv.len() {
                    args.lna_gain = argv[i].parse().ok();
                }
            }
            "--vga" => {
                i += 1;
                if i < argv.len() {
                    args.vga_gain = argv[i].parse().ok();
                }
            }
            "-a" | "--amp" => args.amp_enable = true,
            "-b" | "--bias-t" => args.bias_t = true,
            _ => eprintln!("Unknown argument: {}", argv[i]),
        }
        i += 1;
    }

    args
}

fn print_help() {
    println!("hackrf_tune - Configure and tune HackRF devices");
    println!();
    println!("USAGE:");
    println!("    hackrf_tune [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    -h, --help              Show this help message");
    println!("    -f, --freq <HZ>         Set frequency in Hz (1 MHz - 6 GHz)");
    println!("    -r, --rate <HZ>         Set sample rate in Hz (2M, 4M, 8M, 10M, 16M, 20M)");
    println!("    --lna <GAIN>            Set LNA gain in dB (0-40, 8 dB steps)");
    println!("    --vga <GAIN>            Set VGA/baseband gain in dB (0-62, 2 dB steps)");
    println!("    -a, --amp               Enable RF amplifier (+14 dB)");
    println!("    -b, --bias-t            Enable bias tee (DC on antenna port)");
    println!();
    println!("EXAMPLES:");
    println!("    # Tune to FM broadcast band");
    println!("    hackrf_tune --freq 100000000 --lna 24 --vga 20");
    println!();
    println!("    # Tune to 1090 MHz for ADS-B with RF amp");
    println!("    hackrf_tune --freq 1090000000 --rate 8000000 --amp");
    println!();
    println!("NOTES:");
    println!("    - HackRF One frequency range: ~1 MHz to 6 GHz");
    println!("    - Default sample rate: 10 MSPS if not specified");
    println!("    - WARNING: --bias-t puts DC voltage on the antenna port!");
}

fn main() {
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
    println!("Opening HackRF device...");
    let hackrf = HackRf::open_first()?;

    // Display device info
    let board_id = hackrf.board_id()?;
    println!("Board: {} (ID: {})", board_id_name(board_id), board_id);

    let version = hackrf.version().unwrap_or_else(|_| "(unknown)".to_string());
    println!("Firmware: {}", version);
    println!();

    let mut configured = false;

    // Set sample rate
    if let Some(rate) = args.sample_rate {
        println!(
            "Setting sample rate to {} Hz ({:.3} MSPS)...",
            rate,
            rate as f64 / 1_000_000.0
        );
        hackrf.set_sample_rate(rate)?;
        println!("  OK");
        configured = true;
    }

    // Set frequency
    if let Some(freq_hz) = args.freq_hz {
        println!(
            "Setting frequency to {} Hz ({:.3} MHz)...",
            freq_hz,
            freq_hz as f64 / 1_000_000.0
        );
        hackrf.set_freq(freq_hz)?;
        println!("  OK");
        configured = true;
    }

    // Set LNA gain
    if let Some(gain) = args.lna_gain {
        println!("Setting LNA gain to {} dB...", gain);
        hackrf.set_lna_gain(gain)?;
        println!("  OK (rounded to {} dB)", gain & !0x07);
        configured = true;
    }

    // Set VGA gain
    if let Some(gain) = args.vga_gain {
        println!("Setting VGA gain to {} dB...", gain);
        hackrf.set_vga_gain(gain)?;
        println!("  OK (rounded to {} dB)", gain & !0x01);
        configured = true;
    }

    // Enable RF amplifier
    if args.amp_enable {
        println!("Enabling RF amplifier (+14 dB)...");
        hackrf.set_amp_enable(true)?;
        println!("  OK");
        configured = true;
    }

    // Enable bias tee
    if args.bias_t {
        println!("WARNING: Enabling bias tee (DC on antenna port)...");
        hackrf.set_antenna_enable(true)?;
        println!("  OK - Bias tee ENABLED");
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
    Ok(())
}
