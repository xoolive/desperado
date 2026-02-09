//! Diagnostic tool to examine raw bytes from Airspy
//!
//! This captures raw bytes from Airspy and displays them in multiple formats
//! to determine whether the device outputs REAL samples or I/Q samples.
//!
//! Usage: cargo run --example airspy_raw_bytes --features airspy

use desperado::airspy::{list_devices, AirspyConfig};
use desperado::Gain;
use rs_spy::{Airspy, RECOMMENDED_BUFFER_SIZE};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Airspy Raw Bytes Diagnostic ===\n");

    // List available devices
    println!("Scanning for Airspy devices...");
    let devices = list_devices()?;
    if devices.is_empty() {
        println!("No Airspy devices found.");
        return Ok(());
    }

    println!("Found {} device(s)\n", devices.len());

    // Use first device
    let device_idx = 0;
    println!("Opening device 0...");

    // Create config
    let config = AirspyConfig::new(device_idx, 100_000_000, 6_000_000, Gain::Auto);

    // Manually open device using rs_spy to get raw access
    let device =
        Airspy::open_by_index(device_idx).map_err(|e| format!("Failed to open device: {}", e))?;

    // Set frequency and gain
    device
        .set_freq(config.center_freq)
        .map_err(|e| format!("Failed to set frequency: {}", e))?;

    device
        .set_sensitivity_gain(17)
        .map_err(|e| format!("Failed to set gain: {}", e))?;

    // Start RX
    device
        .start_rx()
        .map_err(|e| format!("Failed to start RX: {}", e))?;

    println!("Device ready. Reading raw bytes...\n");

    // Read several chunks and analyze patterns
    let mut buf = vec![0u8; RECOMMENDED_BUFFER_SIZE];
    let mut chunk_count = 0;

    for _ in 0..5 {
        match device.read_sync(&mut buf) {
            Ok(bytes_read) => {
                if bytes_read == 0 {
                    println!("No more data");
                    break;
                }

                chunk_count += 1;
                println!("=== Chunk {} ({} bytes) ===", chunk_count, bytes_read);

                // Show first 64 bytes as hex pairs
                println!("\nRaw bytes (hex, first 64 bytes):");
                for (i, chunk) in buf[..bytes_read.min(64)].chunks(2).enumerate() {
                    if i % 8 == 0 {
                        print!("  ");
                    }
                    print!("{:02x}{:02x} ", chunk[0], chunk[1]);
                    if i % 8 == 7 {
                        println!();
                    }
                }
                println!("\n");

                // Interpret as i16 real samples (little-endian)
                println!("Interpretation 1: REAL i16 samples (LE)");
                let real_samples: Vec<i16> = buf[..bytes_read.min(64)]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect();
                println!(
                    "  First 16 samples: {:?}",
                    &real_samples[..real_samples.len().min(16)]
                );
                println!(
                    "  Min/Max: {}/{}, RMS: {:.2}",
                    real_samples.iter().min().unwrap_or(&0),
                    real_samples.iter().max().unwrap_or(&0),
                    (real_samples
                        .iter()
                        .map(|s| (*s as f32).powi(2))
                        .sum::<f32>()
                        / real_samples.len() as f32)
                        .sqrt()
                );

                // Interpret as i16 I/Q pairs (little-endian)
                println!("\nInterpretation 2: I/Q pairs of i16 (LE)");
                let iq_samples: Vec<(i16, i16)> = buf[..bytes_read.min(64)]
                    .chunks_exact(4)
                    .map(|c| {
                        let i = i16::from_le_bytes([c[0], c[1]]);
                        let q = i16::from_le_bytes([c[2], c[3]]);
                        (i, q)
                    })
                    .collect();
                println!("  First 8 I/Q pairs:");
                for (idx, (i, q)) in iq_samples.iter().take(8).enumerate() {
                    println!("    [{}] I={:6}, Q={:6}", idx, i, q);
                }
                let avg_magnitude = iq_samples
                    .iter()
                    .map(|(i, q)| ((*i as f32).powi(2) + (*q as f32).powi(2)).sqrt())
                    .sum::<f32>()
                    / iq_samples.len() as f32;
                println!("  Avg magnitude: {:.2}", avg_magnitude);

                // Statistics about byte values
                println!("\nByte statistics:");
                let min_byte = buf[..bytes_read].iter().min().unwrap_or(&0);
                let max_byte = buf[..bytes_read].iter().max().unwrap_or(&0);
                let avg_byte = buf[..bytes_read].iter().map(|b| *b as u64).sum::<u64>() as f32
                    / bytes_read as f32;
                println!(
                    "  Min: {}, Max: {}, Avg: {:.2}",
                    min_byte, max_byte, avg_byte
                );

                // Check for patterns
                println!("\nPattern analysis:");
                let zeros = buf[..bytes_read].iter().filter(|&&b| b == 0).count();
                let ones = buf[..bytes_read].iter().filter(|&&b| b == 255).count();
                println!(
                    "  Zero bytes: {} ({:.2}%)",
                    zeros,
                    zeros as f32 / bytes_read as f32 * 100.0
                );
                println!(
                    "  0xFF bytes: {} ({:.2}%)",
                    ones,
                    ones as f32 / bytes_read as f32 * 100.0
                );

                // Entropy check
                let mut freq = [0u32; 256];
                for &byte in &buf[..bytes_read] {
                    freq[byte as usize] += 1;
                }
                let entropy = freq
                    .iter()
                    .filter(|&&c| c > 0)
                    .map(|&c| {
                        let p = c as f32 / bytes_read as f32;
                        -p * p.log2()
                    })
                    .sum::<f32>();
                println!("  Shannon entropy: {:.2} bits/byte (0-8 range)", entropy);

                println!();
            }
            Err(e) => {
                println!("Error reading: {}", e);
                break;
            }
        }
    }

    device
        .stop_rx()
        .map_err(|e| format!("Failed to stop RX: {}", e))?;

    println!("=== Diagnostic Complete ===");
    println!("\nInterpretation guide:");
    println!("- If I/Q interpretation shows balanced magnitude, data is likely I/Q");
    println!("- If REAL interpretation shows signals, data is likely REAL samples");
    println!("- High entropy (>7) suggests data is properly mixed/modulated");
    println!("- Check for obvious patterns that indicate one format over another");

    Ok(())
}
