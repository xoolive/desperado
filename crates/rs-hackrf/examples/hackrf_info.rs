//! Simple CLI tool to discover and query HackRF devices.
//!
//! This is a Phase 1 example showing device discovery and firmware version reporting.
//! Output format matches the C `hackrf_info` tool as closely as possible.

use rs_hackrf::transport::{BOARD_REV_GSG, board_id_name, board_rev_name};
use rs_hackrf::{HackRf, HackRfErrorCode};
use std::process;

fn main() {
    tracing_subscriber::fmt::init();

    match run() {
        Ok(()) => process::exit(0),
        Err(e) => {
            eprintln!("Error: {}", e);
            eprintln!("Error Code: {} (-5)", HackRfErrorCode::NotFound.name());
            eprintln!("\nTroubleshooting:");
            eprintln!("- If you see 'I/O Error', the device may be hung");
            eprintln!("- Try unplugging and replugging the HackRF device");
            process::exit(1);
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    // List all connected HackRF devices
    let devices = HackRf::list_devices()?;

    if devices.is_empty() {
        println!("No HackRF boards found.");
        return Ok(());
    }

    for (index, serial) in devices.iter().enumerate() {
        if index > 0 {
            println!();
        }

        println!("Found HackRF");
        println!("Index: {}", index);
        println!("Serial number: {}", serial);

        // Open this device
        let hackrf = HackRf::open_by_index(index)?;

        // Board ID
        let board_id = hackrf.board_id()?;
        println!(
            "Board ID Number: {} ({})",
            board_id,
            board_id_name(board_id)
        );

        // Firmware version + USB API version
        let version = hackrf.version().unwrap_or_else(|_| "(unknown)".to_string());
        let usb_api = hackrf.usb_api_version();
        println!(
            "Firmware Version: {} (API:{}.{:02})",
            version,
            (usb_api >> 8) & 0xFF,
            usb_api & 0xFF
        );

        // Part ID and serial number
        match hackrf.board_partid_serialno() {
            Ok((part0, part1, _serial)) => {
                println!("Part ID Number: 0x{:08x} 0x{:08x}", part0, part1);
            }
            Err(e) => {
                tracing::debug!("Failed to read partid/serial: {}", e);
                println!("Part ID Number: (not available)");
            }
        }

        // Board revision (requires USB API >= 0x0106 and supported board IDs)
        if usb_api >= 0x0106 && (board_id == 2 || board_id == 4 || board_id == 5) {
            match hackrf.board_rev() {
                Ok(rev) => {
                    println!("Hardware Revision: {}", board_rev_name(rev));
                    if rev > 0 {
                        if rev & BOARD_REV_GSG != 0 {
                            println!(
                                "Hardware appears to have been manufactured by Great Scott Gadgets."
                            );
                        } else {
                            println!(
                                "Hardware does not appear to have been manufactured by Great Scott Gadgets."
                            );
                        }
                    }
                }
                Err(e) => {
                    tracing::debug!("Failed to read board rev: {}", e);
                }
            }
        }

        // Supported platform
        if usb_api >= 0x0106 {
            match hackrf.supported_platform() {
                Ok(platform) => {
                    println!("Hardware supported by installed firmware:");
                    if platform & rs_hackrf::transport::PLATFORM_JAWBREAKER != 0 {
                        println!("    Jawbreaker");
                    }
                    if platform & rs_hackrf::transport::PLATFORM_RAD1O != 0 {
                        println!("    rad1o");
                    }
                    if platform & rs_hackrf::transport::PLATFORM_HACKRF1_OG != 0
                        || platform & rs_hackrf::transport::PLATFORM_HACKRF1_R9 != 0
                    {
                        println!("    HackRF One");
                    }
                }
                Err(e) => {
                    tracing::debug!("Failed to read supported platform: {}", e);
                }
            }
        }
    }

    Ok(())
}
