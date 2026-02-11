//! Simple Airspy test - list devices and stream a few samples
//!
//! Usage: cargo run --example airspy_test --features airspy

use desperado::Gain;
use desperado::airspy::{AirspyConfig, AirspySdrReader, list_devices};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Airspy Integration Test ===\n");

    // List available devices
    println!("Scanning for Airspy devices...");
    match list_devices() {
        Ok(devices) => {
            if devices.is_empty() {
                println!("No Airspy devices found.");
                return Ok(());
            }
            println!("Found {} device(s):", devices.len());
            for dev in &devices {
                println!(
                    "  [{}] {} - {}",
                    dev.index,
                    dev.board_name(),
                    dev.firmware_version
                );
                println!(
                    "      Part ID: 0x{:08X} 0x{:08X}",
                    dev.part_id[0], dev.part_id[1]
                );
                println!("      Serial:  0x{:016X}", dev.serial_number);
                println!(
                    "      Sample rates: {:?}",
                    dev.supported_sample_rates
                        .iter()
                        .map(|r| format!("{:.3} MSPS", *r as f64 / 1e6))
                        .collect::<Vec<_>>()
                );
            }
        }
        Err(e) => {
            println!("Error listing devices: {}", e);
            return Ok(());
        }
    }

    println!("\n--- Testing streaming at 6 MSPS ---");

    // Create config for 100 MHz, 6 MSPS, auto gain
    let config = AirspyConfig::new(0, 100_000_000, 6_000_000, Gain::Auto);
    println!(
        "Config: freq={} MHz, sample_rate={} MSPS, gain={:?}",
        config.center_freq as f64 / 1e6,
        config.sample_rate as f64 / 1e6,
        config.gain
    );

    // Create reader
    println!("\nOpening device and starting stream...");
    let reader = AirspySdrReader::new(&config)?;

    // Get device info
    if let Ok(info) = reader.device_info() {
        println!(
            "Connected to: {} - {}",
            info.board_name(),
            info.firmware_version
        );
    }

    // Read a few chunks
    println!("\nReading samples...");
    let mut total_samples = 0usize;
    let mut chunks = 0usize;

    for result in reader.take(5) {
        match result {
            Ok(samples) => {
                total_samples += samples.len();
                chunks += 1;

                // Show first few samples from first chunk
                if chunks == 1 && !samples.is_empty() {
                    println!("First 5 samples: {:?}", &samples[..5.min(samples.len())]);
                }

                // Calculate rough power
                let power: f32 =
                    samples.iter().map(|s| s.norm_sqr()).sum::<f32>() / samples.len() as f32;
                println!(
                    "  Chunk {}: {} samples, avg power: {:.4}",
                    chunks,
                    samples.len(),
                    power
                );
            }
            Err(e) => {
                println!("Error reading samples: {}", e);
                break;
            }
        }
    }

    println!("\nTotal: {} samples in {} chunks", total_samples, chunks);

    println!("\n=== Test Complete ===");

    Ok(())
}
