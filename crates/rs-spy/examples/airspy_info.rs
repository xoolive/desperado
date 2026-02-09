//! Simple CLI tool to discover and query Airspy devices.
//!
//! This is a Phase 1 example showing device discovery and firmware version reporting.
//!
//! NOTE: After each run, the device may need time to reset.
//! If you get "I/O Error", unplug and replug the device or wait 30 seconds.

use rs_spy::{Airspy, AirspyErrorCode};
use std::process;

fn main() {
    // Initialize tracing
    tracing_subscriber::fmt::init();

    match run() {
        Ok(()) => process::exit(0),
        Err(e) => {
            eprintln!("Error: {}", e);
            eprintln!("Error Code: {} (-5)", AirspyErrorCode::NotFound.name());
            eprintln!("\nTroubleshooting:");
            eprintln!("- If you see 'I/O Error', the device may be hung");
            eprintln!("- Try unplugging and replugging the Airspy device");
            eprintln!("- Or restart the system");
            process::exit(1);
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    println!("Scanning for Airspy devices...\n");

    // List all devices
    let devices = Airspy::list_devices()?;

    if devices.is_empty() {
        println!("No Airspy devices found.");
        println!("Error Code: {} (-5)", AirspyErrorCode::NotFound.name());
        return Ok(());
    }

    println!("Found AirSpy board 1");

    // Open the first device
    let airspy = Airspy::open_first()?;

    // Read board ID first (seems most reliable)
    let board_id = airspy.board_id()?;
    println!(
        "Board ID Number: {} ({})",
        board_id,
        board_id_name(board_id)
    );

    // Get firmware version
    let version = match airspy.version() {
        Ok(v) => v,
        Err(e) => {
            tracing::warn!("Failed to read version: {}", e);
            String::from("(not found)")
        }
    };
    println!("Firmware Version: {}", version);

    // Try to get part IDs and serial number
    match airspy.board_partid_serialno() {
        Ok((part_id_1, part_id_2, serial_no)) => {
            println!("Part ID Number: 0x{:08X} 0x{:08X}", part_id_1, part_id_2);
            println!("Serial Number: 0x{:016X}", serial_no);
        }
        Err(e) => {
            tracing::debug!("Failed to read partid/serial: {}", e);
            println!("Part ID Number: (not found)");
            println!("Serial Number: (not found)");
        }
    }

    // Try to get supported sample rates
    match airspy.supported_sample_rates() {
        Ok(sample_rates) => {
            if !sample_rates.is_empty() {
                println!("Supported sample rates:");
                for rate in sample_rates {
                    println!("\t{:.6} MSPS", rate as f64 / 1_000_000.0);
                }
            } else {
                println!("Supported sample rates: (not found)");
            }
        }
        Err(e) => {
            tracing::debug!("Failed to read sample rates: {}", e);
            println!("Supported sample rates: (not found)");
        }
    }

    println!("Close board 1");
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
