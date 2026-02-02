//! Test helper utilities for generating synthetic I/Q signals

use std::f32::consts::PI;

/// Generate a complex sine wave at a specific frequency
///
/// # Arguments
/// * `frequency` - Frequency in Hz
/// * `sample_rate` - Sample rate in Hz
/// * `num_samples` - Number of samples to generate
///
/// # Returns
/// Vector of complex samples as interleaved I/Q bytes (format: Cu8)
pub fn generate_sine_wave_cu8(frequency: f32, sample_rate: u32, num_samples: usize) -> Vec<u8> {
    let mut buffer = Vec::with_capacity(num_samples * 2);
    let angular_freq = 2.0 * PI * frequency / sample_rate as f32;

    for n in 0..num_samples {
        let phase = angular_freq * n as f32;
        let i = phase.cos();
        let q = phase.sin();

        // Convert from [-1, 1] to [0, 255] for Cu8 format
        let i_byte = ((i + 1.0) * 127.5) as u8;
        let q_byte = ((q + 1.0) * 127.5) as u8;

        buffer.push(i_byte);
        buffer.push(q_byte);
    }

    buffer
}

/// Generate a complex sine wave at a specific frequency (Cs16 format)
///
/// # Arguments
/// * `frequency` - Frequency in Hz
/// * `sample_rate` - Sample rate in Hz
/// * `num_samples` - Number of samples to generate
///
/// # Returns
/// Vector of complex samples as interleaved I/Q bytes (format: Cs16, little-endian)
pub fn generate_sine_wave_cs16(frequency: f32, sample_rate: u32, num_samples: usize) -> Vec<u8> {
    let mut buffer = Vec::with_capacity(num_samples * 4);
    let angular_freq = 2.0 * PI * frequency / sample_rate as f32;

    for n in 0..num_samples {
        let phase = angular_freq * n as f32;
        let i = phase.cos();
        let q = phase.sin();

        // Convert from [-1, 1] to [-32768, 32767] for Cs16 format
        let i_sample = (i * 32767.0) as i16;
        let q_sample = (q * 32767.0) as i16;

        // Write as little-endian
        buffer.extend_from_slice(&i_sample.to_le_bytes());
        buffer.extend_from_slice(&q_sample.to_le_bytes());
    }

    buffer
}

/// Generate a DC signal (constant value)
///
/// # Arguments
/// * `num_samples` - Number of samples to generate
/// * `i_value` - I component value (0.0 = center for Cu8)
/// * `q_value` - Q component value (0.0 = center for Cu8)
///
/// # Returns
/// Vector of complex samples as interleaved I/Q bytes (format: Cu8)
pub fn generate_dc_signal_cu8(num_samples: usize, i_value: f32, q_value: f32) -> Vec<u8> {
    let mut buffer = Vec::with_capacity(num_samples * 2);

    // Convert from [-1, 1] to [0, 255] for Cu8 format
    let i_byte = ((i_value + 1.0) * 127.5).clamp(0.0, 255.0) as u8;
    let q_byte = ((q_value + 1.0) * 127.5).clamp(0.0, 255.0) as u8;

    for _ in 0..num_samples {
        buffer.push(i_byte);
        buffer.push(q_byte);
    }

    buffer
}

/// Generate a complex sine wave (Cf32 format)
///
/// # Arguments
/// * `frequency` - Frequency in Hz
/// * `sample_rate` - Sample rate in Hz
/// * `num_samples` - Number of samples to generate
///
/// # Returns
/// Vector of complex samples as interleaved I/Q bytes (format: Cf32, little-endian)
pub fn generate_sine_wave_cf32(frequency: f32, sample_rate: u32, num_samples: usize) -> Vec<u8> {
    let mut buffer = Vec::with_capacity(num_samples * 8);
    let angular_freq = 2.0 * PI * frequency / sample_rate as f32;

    for n in 0..num_samples {
        let phase = angular_freq * n as f32;
        let i = phase.cos();
        let q = phase.sin();

        buffer.extend_from_slice(&i.to_le_bytes());
        buffer.extend_from_slice(&q.to_le_bytes());
    }

    buffer
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_sine_wave_cu8_length() {
        let samples = generate_sine_wave_cu8(1000.0, 96000, 100);
        assert_eq!(samples.len(), 200); // 100 samples * 2 bytes per sample
    }

    #[test]
    fn test_generate_sine_wave_cu8_center() {
        // DC signal at 0 Hz should produce constant I=255 (cos(0)=1.0), Q≈127 (sin(0)=0.0)
        let samples = generate_sine_wave_cu8(0.0, 96000, 10);
        for i in (0..samples.len()).step_by(2) {
            assert_eq!(samples[i], 255, "I component should be 255 for DC signal");
            assert!(
                (samples[i + 1] as i16 - 127).abs() <= 1,
                "Q component should be ~127 for DC signal"
            );
        }
    }

    #[test]
    fn test_generate_sine_wave_cs16_length() {
        let samples = generate_sine_wave_cs16(1000.0, 96000, 100);
        assert_eq!(samples.len(), 400); // 100 samples * 4 bytes per sample
    }

    #[test]
    fn test_generate_dc_signal_cu8() {
        let samples = generate_dc_signal_cu8(50, 0.0, 0.0);
        assert_eq!(samples.len(), 100); // 50 samples * 2 bytes

        // All values should be 127 (center)
        for &byte in &samples {
            assert_eq!(byte, 127);
        }
    }

    #[test]
    fn test_generate_dc_signal_cu8_max() {
        let samples = generate_dc_signal_cu8(10, 1.0, 1.0);

        // All values should be 255 (max)
        for &byte in &samples {
            assert_eq!(byte, 255);
        }
    }

    #[test]
    fn test_generate_sine_wave_cf32_length() {
        let samples = generate_sine_wave_cf32(1000.0, 96000, 100);
        assert_eq!(samples.len(), 800); // 100 samples * 8 bytes per sample
    }
}
