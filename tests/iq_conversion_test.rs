//! Integration tests for I/Q format conversion
//!
//! These tests verify that the `convert_bytes_to_complex` function correctly
//! converts various I/Q formats to Complex<f32> samples.

mod helpers;

use desperado::{IqFormat, IqSource};
use std::fs;

#[test]
fn test_iq_conversion_cu8_sine_wave() {
    // Generate a sine wave at 1 kHz with 96 kHz sample rate
    let samples = helpers::generate_sine_wave_cu8(1000.0, 96000, 96);
    
    // Write to temporary file
    let temp_path = "/tmp/test_sine_cu8.iq";
    fs::write(temp_path, &samples).expect("Failed to write test file");
    
    // Read back using IqSource
    let mut iq_source = IqSource::from_file(
        temp_path,
        162_000_000, // center freq (arbitrary for test)
        96_000,      // sample rate
        96,          // chunk size
        IqFormat::Cu8,
    )
    .expect("Failed to create IQ source");
    
    // Get first chunk
    let chunk = iq_source.next().expect("No data").expect("Read error");
    
    // Verify we got the expected number of samples
    assert_eq!(chunk.len(), 96);
    
    // Verify samples are complex numbers (not all zero)
    let non_zero = chunk.iter().any(|c| c.norm() > 0.01);
    assert!(non_zero, "All samples are near zero");
    
    // Clean up
    fs::remove_file(temp_path).ok();
}

#[test]
fn test_iq_conversion_dc_signal() {
    // Generate DC signal (constant I=0, Q=0 in normalized space)
    let samples = helpers::generate_dc_signal_cu8(100, 0.0, 0.0);
    
    let temp_path = "/tmp/test_dc_cu8.iq";
    fs::write(temp_path, &samples).expect("Failed to write test file");
    
    let mut iq_source = IqSource::from_file(
        temp_path,
        162_000_000,
        96_000,
        100,
        IqFormat::Cu8,
    )
    .expect("Failed to create IQ source");
    
    let chunk = iq_source.next().expect("No data").expect("Read error");
    
    // All samples should be near (0, 0)
    for sample in &chunk {
        assert!(sample.re.abs() < 0.01, "I component not near zero: {}", sample.re);
        assert!(sample.im.abs() < 0.01, "Q component not near zero: {}", sample.im);
    }
    
    fs::remove_file(temp_path).ok();
}

#[test]
fn test_convert_cu8_boundaries() {
    // Test boundary values for Cu8 format
    // Cu8 conversion: (byte - 127.5) / 128.0
    // Expected: 0 → -0.996, 127 → -0.004, 128 → 0.004, 255 → 0.996
    
    let samples = vec![
        0, 0,       // I=0, Q=0 → (-0.996, -0.996)
        127, 127,   // I=127, Q=127 → (-0.004, -0.004)
        128, 128,   // I=128, Q=128 → (0.004, 0.004)
        255, 255,   // I=255, Q=255 → (0.996, 0.996)
    ];
    
    let temp_path = "/tmp/test_cu8_boundaries.iq";
    fs::write(temp_path, &samples).expect("Failed to write test file");
    
    let mut iq_source = IqSource::from_file(
        temp_path,
        162_000_000,
        96_000,
        4, // 4 samples
        IqFormat::Cu8,
    )
    .expect("Failed to create IQ source");
    
    let chunk = iq_source.next().expect("No data").expect("Read error");
    
    assert_eq!(chunk.len(), 4);
    
    // Check boundary values with tolerance
    let epsilon = 0.01;
    assert!((chunk[0].re + 0.996).abs() < epsilon, "Sample 0 I: expected ~-0.996, got {}", chunk[0].re);
    assert!((chunk[0].im + 0.996).abs() < epsilon, "Sample 0 Q: expected ~-0.996, got {}", chunk[0].im);
    assert!((chunk[1].re + 0.004).abs() < epsilon, "Sample 1 I: expected ~-0.004, got {}", chunk[1].re);
    assert!((chunk[1].im + 0.004).abs() < epsilon, "Sample 1 Q: expected ~-0.004, got {}", chunk[1].im);
    assert!((chunk[2].re - 0.004).abs() < epsilon, "Sample 2 I: expected ~0.004, got {}", chunk[2].re);
    assert!((chunk[2].im - 0.004).abs() < epsilon, "Sample 2 Q: expected ~0.004, got {}", chunk[2].im);
    assert!((chunk[3].re - 0.996).abs() < epsilon, "Sample 3 I: expected ~0.996, got {}", chunk[3].re);
    assert!((chunk[3].im - 0.996).abs() < epsilon, "Sample 3 Q: expected ~0.996, got {}", chunk[3].im);
    
    fs::remove_file(temp_path).ok();
}

#[test]
fn test_convert_cs8_sign_handling() {
    // Test signed 8-bit conversion
    // Cs8 conversion: (byte as i8) / 128.0
    // Expected: -128 → -1.0, -1 → -0.0078, 0 → 0.0, 127 → 0.992
    
    let samples = vec![
        0x80u8, 0x80u8,  // I=-128, Q=-128 → (-1.0, -1.0)
        0xFFu8, 0xFFu8,  // I=-1, Q=-1 → (-0.0078, -0.0078)
        0, 0,            // I=0, Q=0 → (0.0, 0.0)
        127, 127,        // I=127, Q=127 → (0.992, 0.992)
    ];
    
    let temp_path = "/tmp/test_cs8_sign.iq";
    fs::write(temp_path, &samples).expect("Failed to write test file");
    
    let mut iq_source = IqSource::from_file(
        temp_path,
        162_000_000,
        96_000,
        4,
        IqFormat::Cs8,
    )
    .expect("Failed to create IQ source");
    
    let chunk = iq_source.next().expect("No data").expect("Read error");
    
    assert_eq!(chunk.len(), 4);
    
    let epsilon = 0.01;
    assert!((chunk[0].re + 1.0).abs() < epsilon, "Sample 0 I: expected ~-1.0, got {}", chunk[0].re);
    assert!((chunk[0].im + 1.0).abs() < epsilon, "Sample 0 Q: expected ~-1.0, got {}", chunk[0].im);
    assert!((chunk[1].re + 0.0078).abs() < epsilon, "Sample 1 I: expected ~-0.0078, got {}", chunk[1].re);
    assert!((chunk[1].im + 0.0078).abs() < epsilon, "Sample 1 Q: expected ~-0.0078, got {}", chunk[1].im);
    assert!(chunk[2].re.abs() < epsilon, "Sample 2 I: expected ~0.0, got {}", chunk[2].re);
    assert!(chunk[2].im.abs() < epsilon, "Sample 2 Q: expected ~0.0, got {}", chunk[2].im);
    assert!((chunk[3].re - 0.992).abs() < epsilon, "Sample 3 I: expected ~0.992, got {}", chunk[3].re);
    assert!((chunk[3].im - 0.992).abs() < epsilon, "Sample 3 Q: expected ~0.992, got {}", chunk[3].im);
    
    fs::remove_file(temp_path).ok();
}

#[test]
fn test_convert_cs16_endianness() {
    // Test 16-bit little-endian conversion
    // Cs16 conversion: i16::from_le_bytes / 32768.0
    // Expected: -32768 → -1.0, 0 → 0.0, 32767 → 0.999
    
    let samples = vec![
        0x00u8, 0x80u8, 0x00u8, 0x80u8,  // I=-32768, Q=-32768 (LE) → (-1.0, -1.0)
        0x00u8, 0x00u8, 0x00u8, 0x00u8,  // I=0, Q=0 → (0.0, 0.0)
        0xFFu8, 0x7Fu8, 0xFFu8, 0x7Fu8,  // I=32767, Q=32767 (LE) → (0.999, 0.999)
    ];
    
    let temp_path = "/tmp/test_cs16_endian.iq";
    fs::write(temp_path, &samples).expect("Failed to write test file");
    
    let mut iq_source = IqSource::from_file(
        temp_path,
        162_000_000,
        96_000,
        3,
        IqFormat::Cs16,
    )
    .expect("Failed to create IQ source");
    
    let chunk = iq_source.next().expect("No data").expect("Read error");
    
    assert_eq!(chunk.len(), 3);
    
    let epsilon = 0.01;
    assert!((chunk[0].re + 1.0).abs() < epsilon, "Sample 0 I: expected ~-1.0, got {}", chunk[0].re);
    assert!((chunk[0].im + 1.0).abs() < epsilon, "Sample 0 Q: expected ~-1.0, got {}", chunk[0].im);
    assert!(chunk[1].re.abs() < epsilon, "Sample 1 I: expected ~0.0, got {}", chunk[1].re);
    assert!(chunk[1].im.abs() < epsilon, "Sample 1 Q: expected ~0.0, got {}", chunk[1].im);
    assert!((chunk[2].re - 0.999).abs() < epsilon, "Sample 2 I: expected ~0.999, got {}", chunk[2].re);
    assert!((chunk[2].im - 0.999).abs() < epsilon, "Sample 2 Q: expected ~0.999, got {}", chunk[2].im);
    
    fs::remove_file(temp_path).ok();
}

#[test]
fn test_convert_cf32_float_precision() {
    // Test 32-bit float format (direct copy, no conversion)
    // Cf32 conversion: f32::from_le_bytes (little-endian)
    
    // Create test samples: (0.5, -0.5), (-1.0, 1.0), (0.0, 0.0)
    let mut samples = Vec::new();
    samples.extend_from_slice(&0.5f32.to_le_bytes());
    samples.extend_from_slice(&(-0.5f32).to_le_bytes());
    samples.extend_from_slice(&(-1.0f32).to_le_bytes());
    samples.extend_from_slice(&1.0f32.to_le_bytes());
    samples.extend_from_slice(&0.0f32.to_le_bytes());
    samples.extend_from_slice(&0.0f32.to_le_bytes());
    
    let temp_path = "/tmp/test_cf32_float.iq";
    fs::write(temp_path, &samples).expect("Failed to write test file");
    
    let mut iq_source = IqSource::from_file(
        temp_path,
        162_000_000,
        96_000,
        3,
        IqFormat::Cf32,
    )
    .expect("Failed to create IQ source");
    
    let chunk = iq_source.next().expect("No data").expect("Read error");
    
    assert_eq!(chunk.len(), 3);
    
    // Cf32 should preserve exact float values
    let epsilon = 1e-6;
    assert!((chunk[0].re - 0.5).abs() < epsilon, "Sample 0 I: expected 0.5, got {}", chunk[0].re);
    assert!((chunk[0].im + 0.5).abs() < epsilon, "Sample 0 Q: expected -0.5, got {}", chunk[0].im);
    assert!((chunk[1].re + 1.0).abs() < epsilon, "Sample 1 I: expected -1.0, got {}", chunk[1].re);
    assert!((chunk[1].im - 1.0).abs() < epsilon, "Sample 1 Q: expected 1.0, got {}", chunk[1].im);
    assert!(chunk[2].re.abs() < epsilon, "Sample 2 I: expected 0.0, got {}", chunk[2].re);
    assert!(chunk[2].im.abs() < epsilon, "Sample 2 Q: expected 0.0, got {}", chunk[2].im);
    
    fs::remove_file(temp_path).ok();
}

#[test]
fn test_convert_empty_buffer() {
    // Test edge case with empty file (0 samples)
    let samples: Vec<u8> = vec![];
    
    let temp_path = "/tmp/test_empty.iq";
    fs::write(temp_path, &samples).expect("Failed to write test file");
    
    let mut iq_source = IqSource::from_file(
        temp_path,
        162_000_000,
        96_000,
        100,
        IqFormat::Cu8,
    )
    .expect("Failed to create IQ source");
    
    // Should return None (no data) rather than error
    let result = iq_source.next();
    assert!(result.is_none(), "Expected None for empty file, got {:?}", result);
    
    fs::remove_file(temp_path).ok();
}
