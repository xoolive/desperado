//! Unit and integration tests for the iqread module

use desperado::{IqFormat, IqSource};
use std::fs;
use tempfile::NamedTempFile;

/// Helper to create a temp file with the given bytes and return (NamedTempFile, path String).
/// The NamedTempFile must be kept alive for the duration of the test.
fn temp_iq(data: &[u8]) -> (NamedTempFile, String) {
    let f = NamedTempFile::new().expect("Failed to create temp file");
    let path = f.path().to_str().unwrap().to_string();
    fs::write(&path, data).expect("Failed to write test file");
    (f, path)
}

#[test]
fn test_iqformat_bytes_per_sample_cu8() {
    // Cu8 format: 2 bytes per sample (1 byte I, 1 byte Q)
    let format = IqFormat::Cu8;

    let samples = vec![127, 127, 128, 128, 255, 255]; // 3 samples
    let (_tmp, path) = temp_iq(&samples);

    let mut iq_source = IqSource::from_file(&path, 162_000_000, 96_000, 3, format)
        .expect("Failed to create IQ source");

    let chunk = iq_source.next().expect("No data").expect("Read error");
    assert_eq!(chunk.len(), 3, "Should read exactly 3 samples from 6 bytes");
}

#[test]
fn test_iqformat_bytes_per_sample_cs8() {
    // Cs8 format: 2 bytes per sample (1 byte I, 1 byte Q)
    let format = IqFormat::Cs8;

    let samples = vec![0, 0, 127, 127]; // 2 samples
    let (_tmp, path) = temp_iq(&samples);

    let mut iq_source = IqSource::from_file(&path, 162_000_000, 96_000, 2, format)
        .expect("Failed to create IQ source");

    let chunk = iq_source.next().expect("No data").expect("Read error");
    assert_eq!(chunk.len(), 2, "Should read exactly 2 samples from 4 bytes");
}

#[test]
fn test_iqformat_bytes_per_sample_cs16() {
    // Cs16 format: 4 bytes per sample (2 bytes I, 2 bytes Q)
    let format = IqFormat::Cs16;

    let samples = vec![0, 0, 0, 0, 0xFF, 0x7F, 0xFF, 0x7F]; // 2 samples
    let (_tmp, path) = temp_iq(&samples);

    let mut iq_source = IqSource::from_file(&path, 162_000_000, 96_000, 2, format)
        .expect("Failed to create IQ source");

    let chunk = iq_source.next().expect("No data").expect("Read error");
    assert_eq!(chunk.len(), 2, "Should read exactly 2 samples from 8 bytes");
}

#[test]
fn test_iqformat_bytes_per_sample_cf32() {
    // Cf32 format: 8 bytes per sample (4 bytes I, 4 bytes Q)
    let format = IqFormat::Cf32;

    let mut samples = Vec::new();
    samples.extend_from_slice(&0.5f32.to_le_bytes());
    samples.extend_from_slice(&(-0.5f32).to_le_bytes());
    samples.extend_from_slice(&1.0f32.to_le_bytes());
    samples.extend_from_slice(&(-1.0f32).to_le_bytes());
    // Total: 16 bytes = 2 samples

    let (_tmp, path) = temp_iq(&samples);

    let mut iq_source = IqSource::from_file(&path, 162_000_000, 96_000, 2, format)
        .expect("Failed to create IQ source");

    let chunk = iq_source.next().expect("No data").expect("Read error");
    assert_eq!(
        chunk.len(),
        2,
        "Should read exactly 2 samples from 16 bytes"
    );
}

#[test]
fn test_expanduser_with_tilde() {
    // Test that paths starting with ~ are expanded to home directory
    // Since expanduser() is private, we test it indirectly

    // Create a test file in the actual home directory with a unique name
    let home = dirs::home_dir().expect("Could not get home directory");
    let unique_name = format!(".desperado_test_{}.iq", std::process::id());
    let test_file = home.join(&unique_name);

    // Write a small test file
    let samples = vec![127, 127, 128, 128]; // 2 Cu8 samples
    fs::write(&test_file, &samples).expect("Failed to write test file");

    // Try to open with tilde path
    let tilde_path = format!("~/{}", unique_name);
    let result = IqSource::from_file(&tilde_path, 162_000_000, 96_000, 2, IqFormat::Cu8);

    // Should successfully open the file
    assert!(
        result.is_ok(),
        "Failed to open file with tilde path: {:?}",
        result.err()
    );

    // Verify we can read from it
    let mut iq_source = result.unwrap();
    let chunk = iq_source.next().expect("No data").expect("Read error");
    assert_eq!(chunk.len(), 2, "Should read 2 samples from tilde path");

    // Clean up
    fs::remove_file(&test_file).ok();
}

#[test]
fn test_expanduser_without_tilde() {
    // Test that regular paths work normally
    let samples = vec![127, 127];
    let (_tmp, path) = temp_iq(&samples);

    let result = IqSource::from_file(&path, 162_000_000, 96_000, 1, IqFormat::Cu8);
    assert!(result.is_ok(), "Failed to open file with regular path");
}

#[test]
fn test_expanduser_nonexistent_file() {
    // Test that nonexistent files return appropriate error
    let result = IqSource::from_file(
        "~/nonexistent_file_12345.iq",
        162_000_000,
        96_000,
        100,
        IqFormat::Cu8,
    );

    assert!(result.is_err(), "Should return error for nonexistent file");
    let err = result.err().unwrap();
    // Check that it's an I/O error
    assert!(matches!(err, desperado::Error::Io(_)));
}

#[test]
fn test_iqread_integration_multiple_chunks() {
    // Integration test: Read multiple chunks from a file
    let mut samples = Vec::new();

    // Create 3 chunks of 10 samples each (Cu8 = 2 bytes per sample)
    for chunk_idx in 0..3 {
        for sample_idx in 0..10 {
            let value = (chunk_idx * 10 + sample_idx) as u8;
            samples.push(value);
            samples.push(value);
        }
    }

    let (_tmp, path) = temp_iq(&samples);

    let mut iq_source = IqSource::from_file(
        &path,
        162_000_000,
        96_000,
        10, // chunk size
        IqFormat::Cu8,
    )
    .expect("Failed to create IQ source");

    // Read all 3 chunks
    for chunk_idx in 0..3 {
        let chunk = iq_source
            .next()
            .unwrap_or_else(|| panic!("Chunk {} missing", chunk_idx))
            .expect("Read error");

        assert_eq!(
            chunk.len(),
            10,
            "Chunk {} should have 10 samples",
            chunk_idx
        );
    }

    // Fourth read should return None (EOF)
    assert!(
        iq_source.next().is_none(),
        "Should reach EOF after 3 chunks"
    );
}

#[test]
fn test_iqread_integration_partial_chunk() {
    // Integration test: File size not exactly divisible by chunk size
    // Current behavior: read_exact will fail on partial chunk (UnexpectedEof)
    // Create 25 samples, request chunks of 10
    let mut samples = Vec::new();
    for i in 0..25 {
        samples.push(i as u8);
        samples.push(i as u8);
    }

    let (_tmp, path) = temp_iq(&samples);

    let mut iq_source = IqSource::from_file(&path, 162_000_000, 96_000, 10, IqFormat::Cu8)
        .expect("Failed to create IQ source");

    // First two chunks should be full (10 samples each)
    for i in 0..2 {
        let chunk = iq_source
            .next()
            .expect("Missing chunk")
            .expect("Read error");
        assert_eq!(chunk.len(), 10, "Chunk {} should be full", i);
    }

    // Third attempt will encounter UnexpectedEof (only 5 samples = 10 bytes remaining)
    // The Iterator impl treats UnexpectedEof as None (end of stream)
    let result = iq_source.next();
    assert!(result.is_none(), "Partial chunk should result in EOF");
}

#[test]
fn test_iqread_from_stdin_creation() {
    // Test that stdin source can be created (not testing actual reading from stdin)
    let result = IqSource::from_stdin(162_000_000, 96_000, 100, IqFormat::Cu8);
    assert!(result.is_ok(), "Failed to create stdin IQ source");
}

#[test]
fn test_iqread_from_tcp_invalid_address() {
    // Test that invalid TCP addresses return appropriate errors
    let result = IqSource::from_tcp(
        "invalid.nonexistent.host.test",
        12345,
        162_000_000,
        96_000,
        100,
        IqFormat::Cu8,
    );

    // Should fail to connect
    assert!(
        result.is_err(),
        "Should return error for invalid TCP address"
    );
}
