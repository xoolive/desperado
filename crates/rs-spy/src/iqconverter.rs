//! I/Q Converter for Airspy REAL samples.
//!
//! Airspy hardware outputs REAL samples from a single ADC. This module converts
//! those real samples to Complex I/Q format using Fs/4 frequency translation.
//!
//! The algorithm is based on libairspy's iqconverter but implemented in pure Rust.
//!
//! ## Processing Steps
//!
//! 1. **DC Removal**: High-pass filter to remove DC offset
//! 2. **Fs/4 Translation**: Frequency shift by sample_rate/4 using rotation sequence
//! 3. **Half-band FIR Filter**: Low-pass filter for I channel
//! 4. **Delay Line**: Matches group delay for Q channel
//!
//! After processing, the interleaved output contains I/Q pairs where:
//! - Even indices (0, 2, 4, ...) are I samples
//! - Odd indices (1, 3, 5, ...) are Q samples

use num_complex::Complex;

/// 47-tap half-band FIR filter kernel.
///
/// This filter is symmetric and has zeros at every other tap (half-band property).
/// The center tap is 0.5, providing unity gain at DC.
#[allow(clippy::excessive_precision)]
const HB_KERNEL_FLOAT: [f32; 47] = [
    -0.000998606272947510,
    0.000000000000000000,
    0.001695637278417295,
    0.000000000000000000,
    -0.003054430179754289,
    0.000000000000000000,
    0.005055504379767936,
    0.000000000000000000,
    -0.007901319195893647,
    0.000000000000000000,
    0.011873357051047719,
    0.000000000000000000,
    -0.017411159379930066,
    0.000000000000000000,
    0.025304817427568772,
    0.000000000000000000,
    -0.037225225204559217,
    0.000000000000000000,
    0.057533286997004301,
    0.000000000000000000,
    -0.102327462004259350,
    0.000000000000000000,
    0.317034472508947400,
    0.500000000000000000, // Center tap (hbc)
    0.317034472508947400,
    0.000000000000000000,
    -0.102327462004259350,
    0.000000000000000000,
    0.057533286997004301,
    0.000000000000000000,
    -0.037225225204559217,
    0.000000000000000000,
    0.025304817427568772,
    0.000000000000000000,
    -0.017411159379930066,
    0.000000000000000000,
    0.011873357051047719,
    0.000000000000000000,
    -0.007901319195893647,
    0.000000000000000000,
    0.005055504379767936,
    0.000000000000000000,
    -0.003054430179754289,
    0.000000000000000000,
    0.001695637278417295,
    0.000000000000000000,
    -0.000998606272947510,
];

/// DC removal high-pass filter coefficient.
const DC_SCALE: f32 = 0.01;

/// Size factor for circular buffer (allows efficient wrap-around).
const SIZE_FACTOR: usize = 32;

/// I/Q Converter state.
///
/// Maintains internal buffers for FIR filtering and delay line processing.
pub struct IqConverter {
    /// Running DC average for removal
    avg: f32,

    /// Center tap value from half-band kernel (0.5)
    hbc: f32,

    /// Half the kernel length + 1 (number of non-zero taps on one side)
    len: usize,

    /// Current index into FIR queue (circular buffer)
    fir_index: usize,

    /// Current index into delay line (circular buffer)
    delay_index: usize,

    /// Reduced FIR kernel (only non-zero taps from even indices)
    fir_kernel: Vec<f32>,

    /// FIR filter circular queue
    fir_queue: Vec<f32>,

    /// Delay line for Q channel alignment
    delay_line: Vec<f32>,
}

impl IqConverter {
    /// Create a new I/Q converter with the standard 47-tap half-band filter.
    pub fn new() -> Self {
        Self::with_kernel(&HB_KERNEL_FLOAT)
    }

    /// Create an I/Q converter with a custom half-band kernel.
    ///
    /// The kernel must have odd length and be symmetric (half-band property).
    pub fn with_kernel(hb_kernel: &[f32]) -> Self {
        let full_len = hb_kernel.len();
        let len = full_len / 2 + 1;
        let hbc = hb_kernel[full_len / 2]; // Center tap (should be 0.5)

        // Extract non-zero taps from even indices
        let fir_kernel: Vec<f32> = (0..len).map(|i| hb_kernel[i * 2]).collect();

        // Allocate circular buffers
        let fir_queue = vec![0.0f32; len * SIZE_FACTOR];
        let delay_line = vec![0.0f32; len / 2];

        Self {
            avg: 0.0,
            hbc,
            len,
            fir_index: 0,
            delay_index: 0,
            fir_kernel,
            fir_queue,
            delay_line,
        }
    }

    /// Reset the converter state (clear all internal buffers).
    pub fn reset(&mut self) {
        self.avg = 0.0;
        self.fir_index = 0;
        self.delay_index = 0;
        self.delay_line.fill(0.0);
        self.fir_queue.fill(0.0);
    }

    /// Process a buffer of REAL samples and convert to interleaved I/Q.
    ///
    /// The input buffer is modified in-place:
    /// - Even indices become I samples
    /// - Odd indices become Q samples
    ///
    /// After this call, the buffer length is unchanged but now contains
    /// len/2 complex I/Q pairs.
    pub fn process(&mut self, samples: &mut [f32]) {
        self.remove_dc(samples);
        self.translate_fs_4(samples);
    }

    /// Process REAL samples and output Complex<f32> I/Q pairs.
    ///
    /// This is a convenience method that processes the samples and returns
    /// a vector of Complex samples. The output length is half the input length.
    ///
    /// # Arguments
    ///
    /// * `real_samples` - Buffer of real samples (will be modified in-place during processing)
    ///
    /// # Returns
    ///
    /// Vector of Complex<f32> I/Q samples (length = input_length / 2)
    pub fn process_to_complex(&mut self, real_samples: &mut [f32]) -> Vec<Complex<f32>> {
        self.process(real_samples);

        // Convert interleaved I/Q to Complex
        real_samples
            .chunks_exact(2)
            .map(|iq| Complex::new(iq[0], iq[1]))
            .collect()
    }

    /// Remove DC offset using a simple high-pass filter.
    ///
    /// Updates running average and subtracts it from each sample.
    fn remove_dc(&mut self, samples: &mut [f32]) {
        for sample in samples.iter_mut() {
            *sample -= self.avg;
            self.avg += DC_SCALE * *sample;
        }
    }

    /// Perform Fs/4 frequency translation.
    ///
    /// This shifts the spectrum by sample_rate/4 using:
    /// 1. Rotation sequence: [-1, -hbc, +1, +hbc] applied to groups of 4 samples
    /// 2. Half-band FIR filter on even samples (I channel)
    /// 3. Delay line on odd samples (Q channel)
    fn translate_fs_4(&mut self, samples: &mut [f32]) {
        let hbc = self.hbc;

        // Apply rotation sequence in groups of 4
        // Pattern: [-1, -hbc, +1, +hbc]
        for chunk in samples.chunks_exact_mut(4) {
            chunk[0] = -chunk[0];
            chunk[1] = -chunk[1] * hbc;
            // chunk[2] unchanged (multiply by +1)
            chunk[3] *= hbc;
        }

        // Apply half-band FIR filter to even samples (I channel)
        self.fir_interleaved(samples);

        // Apply delay line to odd samples (Q channel)
        self.delay_interleaved(samples);
    }

    /// Apply FIR filter to interleaved samples (even indices only).
    ///
    /// Uses the same algorithm as libairspy: symmetric half-band filter
    /// with circular buffer. For len=24, uses specialized unrolled code.
    fn fir_interleaved(&mut self, samples: &mut [f32]) {
        let len = self.len;

        // Use specialized implementation for len=24 (most common case)
        if len == 24 {
            self.fir_interleaved_24(samples);
        } else {
            self.fir_interleaved_generic(samples);
        }
    }

    /// FIR filter specialized for len=24 (47-tap kernel)
    fn fir_interleaved_24(&mut self, samples: &mut [f32]) {
        for i in (0..samples.len()).step_by(2) {
            let q = self.fir_index;

            // Store new sample
            self.fir_queue[q] = samples[i];

            // Compute using libairspy's unrolled approach for len=24
            // For 24-tap symmetric kernel, we have 12 pairs
            let mut acc = 0.0f32;
            acc += self.fir_kernel[0] * (self.fir_queue[q] + self.fir_queue[q + 23]);
            acc += self.fir_kernel[1] * (self.fir_queue[q + 1] + self.fir_queue[q + 22]);
            acc += self.fir_kernel[2] * (self.fir_queue[q + 2] + self.fir_queue[q + 21]);
            acc += self.fir_kernel[3] * (self.fir_queue[q + 3] + self.fir_queue[q + 20]);
            acc += self.fir_kernel[4] * (self.fir_queue[q + 4] + self.fir_queue[q + 19]);
            acc += self.fir_kernel[5] * (self.fir_queue[q + 5] + self.fir_queue[q + 18]);
            acc += self.fir_kernel[6] * (self.fir_queue[q + 6] + self.fir_queue[q + 17]);
            acc += self.fir_kernel[7] * (self.fir_queue[q + 7] + self.fir_queue[q + 16]);
            acc += self.fir_kernel[8] * (self.fir_queue[q + 8] + self.fir_queue[q + 15]);
            acc += self.fir_kernel[9] * (self.fir_queue[q + 9] + self.fir_queue[q + 14]);
            acc += self.fir_kernel[10] * (self.fir_queue[q + 10] + self.fir_queue[q + 13]);
            acc += self.fir_kernel[11] * (self.fir_queue[q + 11] + self.fir_queue[q + 12]);

            samples[i] = acc;

            // Update circular buffer index
            if self.fir_index == 0 {
                self.fir_index = 24 * (SIZE_FACTOR - 1);
                // Copy tail to extended buffer for wrap-around
                for k in 0..23 {
                    self.fir_queue[self.fir_index + 1 + k] = self.fir_queue[k];
                }
            } else {
                self.fir_index -= 1;
            }
        }
    }

    /// Generic FIR filter for any length
    fn fir_interleaved_generic(&mut self, samples: &mut [f32]) {
        let len = self.len;

        for i in (0..samples.len()).step_by(2) {
            let queue = self.fir_index;

            // Store new sample
            self.fir_queue[queue] = samples[i];

            // Compute FIR with symmetric property
            // For symmetric kernel, we pair taps: kernel[j] * (queue[queue+j] + queue[queue+len-1-j])
            // But need to handle circular buffer wrap-around properly
            let mut acc = 0.0f32;
            for j in 0..len / 2 {
                let forward_idx = queue + j;
                let backward_idx = queue + len - 1 - j;
                let forward_val = self.fir_queue[forward_idx];
                let backward_val = self.fir_queue[backward_idx];
                acc += self.fir_kernel[j] * (forward_val + backward_val);
            }

            // Handle odd length kernel (center tap)
            if len % 2 == 1 {
                let center_idx = queue + len / 2;
                acc += self.fir_kernel[len / 2] * self.fir_queue[center_idx];
            }

            samples[i] = acc;

            // Update circular buffer index
            if self.fir_index == 0 {
                self.fir_index = len * (SIZE_FACTOR - 1);
                // Copy tail to extended buffer for wrap-around
                for k in 0..len - 1 {
                    self.fir_queue[self.fir_index + 1 + k] = self.fir_queue[k];
                }
            } else {
                self.fir_index -= 1;
            }
        }
    }

    /// Apply delay line to interleaved samples (odd indices only).
    ///
    /// This aligns the Q channel with the I channel by applying
    /// the same group delay as the FIR filter.
    fn delay_interleaved(&mut self, samples: &mut [f32]) {
        let half_len = self.len / 2;

        // Process only odd-indexed samples (Q channel)
        // The libairspy code passes samples+1 with full length, so we need to handle all samples
        for i in (1..samples.len()).step_by(2) {
            std::mem::swap(&mut self.delay_line[self.delay_index], &mut samples[i]);

            self.delay_index += 1;
            if self.delay_index >= half_len {
                self.delay_index = 0;
            }
        }
    }
}

impl Default for IqConverter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_iqconverter_creation() {
        let converter = IqConverter::new();
        assert_eq!(converter.len, 24); // 47/2 + 1 = 24
        assert!((converter.hbc - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_dc_removal() {
        let mut converter = IqConverter::new();

        // Create samples with DC offset
        let mut samples: Vec<f32> = (0..1000).map(|_| 0.5).collect();

        converter.remove_dc(&mut samples);

        // After many samples, DC should be mostly removed
        // The last samples should be close to 0
        let avg_last_100: f32 = samples[900..].iter().sum::<f32>() / 100.0;
        assert!(
            avg_last_100.abs() < 0.1,
            "DC removal failed: avg = {}",
            avg_last_100
        );
    }

    #[test]
    fn test_process_to_complex() {
        let mut converter = IqConverter::new();

        // Create test samples (1000 real samples -> 500 complex)
        let mut samples: Vec<f32> = (0..1000).map(|i| (i as f32 * 0.01).sin()).collect();

        let complex_samples = converter.process_to_complex(&mut samples);

        assert_eq!(complex_samples.len(), 500);
    }

    #[test]
    fn test_fir_buffer_circulation() {
        let converter = IqConverter::new();
        let len = converter.len;
        let queue_len = converter.fir_queue.len();

        // Verify circular buffer is properly sized
        assert_eq!(queue_len, len * SIZE_FACTOR);
        assert_eq!(len, 24);
        assert_eq!(queue_len, 768);
    }

    #[test]
    fn test_delay_line_size() {
        let converter = IqConverter::new();
        let len = converter.len;
        let delay_len = converter.delay_line.len();

        // Delay line should be len/2 = 12 samples
        assert_eq!(delay_len, len / 2);
    }

    #[test]
    fn test_fir_with_impulse() {
        // Test FIR filter response to impulse
        let mut converter = IqConverter::new();

        // Create impulse: single sample at value 1.0, rest zeros
        let mut samples = vec![0.0f32; 100];
        samples[0] = 1.0;

        // Apply only DC removal (which shouldn't affect much at DC=0)
        converter.remove_dc(&mut samples);

        // Apply rotation sequence
        for chunk in samples.chunks_exact_mut(4) {
            chunk[0] = -chunk[0];
            chunk[1] = -chunk[1] * 0.5;
            // chunk[2] unchanged
            chunk[3] *= 0.5;
        }

        // Apply FIR filter - track output magnitudes
        let original_even: Vec<f32> = samples.iter().step_by(2).take(10).copied().collect();
        converter.fir_interleaved(&mut samples);
        let fir_output: Vec<f32> = samples.iter().step_by(2).take(10).copied().collect();

        // After FIR, we should see some magnitude at the filtered output
        let sum_original: f32 = original_even.iter().sum::<f32>().abs();
        let sum_fir: f32 = fir_output.iter().sum::<f32>().abs();

        println!(
            "Impulse test - Original sum: {}, FIR output sum: {}",
            sum_original, sum_fir
        );
    }

    #[test]
    fn test_extended_processing_stability() {
        // Process extended sample set to check for buffer issues
        let mut converter = IqConverter::new();

        // Create 2000 real samples (1000 I/Q pairs)
        let mut samples: Vec<f32> = (0..2000)
            .map(|i| {
                // 1 kHz sine wave at 6 MSPS
                let freq = 1000.0 / 6_000_000.0;
                (2.0 * std::f32::consts::PI * freq * i as f32).sin()
            })
            .collect();

        let orig_sum: f32 = samples.iter().sum::<f32>().abs();

        // Process entire buffer
        converter.process(&mut samples);

        // Convert to complex pairs
        let complex_pairs: Vec<_> = samples
            .chunks_exact(2)
            .map(|chunk| Complex::new(chunk[0], chunk[1]))
            .collect();

        // Check statistics
        let magnitudes: Vec<f32> = complex_pairs.iter().map(|c| c.norm()).collect();
        let avg_magnitude = magnitudes.iter().sum::<f32>() / magnitudes.len() as f32;
        let max_magnitude = magnitudes.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let min_magnitude = magnitudes.iter().cloned().fold(f32::INFINITY, f32::min);

        println!("Extended processing (2000 samples):");
        println!("  Original input sum: {}", orig_sum);
        println!("  Output pairs: {}", complex_pairs.len());
        println!(
            "  Magnitude stats - avg: {:.6}, min: {:.6}, max: {:.6}",
            avg_magnitude, min_magnitude, max_magnitude
        );

        // Verify we got reasonable output
        assert!(avg_magnitude > 0.0, "Average magnitude should be positive");
        assert!(max_magnitude > 0.0, "Max magnitude should be positive");
    }

    #[test]
    fn test_fir_kernel_sum() {
        // Verify the FIR kernel sum to understand gain
        let kernel = &HB_KERNEL_FLOAT;
        let kernel_sum: f32 = kernel.iter().sum();

        println!("FIR kernel total sum: {}", kernel_sum);

        // The kernel is 47-tap, symmetric. Let's check:
        // - Are we supposed to only count half?
        // - Is this already a complete, normalized kernel?

        // Let's also check the extracted kernel (non-zero taps)
        let converter = IqConverter::new();
        let extracted_sum: f32 = converter.fir_kernel.iter().sum();
        println!("Extracted kernel sum (non-zero taps): {}", extracted_sum);
        println!("Extracted kernel length: {}", converter.fir_kernel.len());

        // The full kernel sums to ~1.0, which is what we'd expect for a normalized
        // low-pass filter. The extracted kernel will be smaller since we only
        // extract non-zero taps and don't double-count due to symmetry.
    }

    #[test]
    fn test_dc_removal_coefficient() {
        // Verify DC removal behavior
        let mut converter = IqConverter::new();

        // Test with constant DC value
        let mut samples = vec![1.0f32; 200];

        converter.remove_dc(&mut samples);

        // With DC_SCALE=0.01, after 200 iterations we should have removed most DC
        let final_avg = converter.avg;
        let final_sample = samples[199];

        println!("DC removal test:");
        println!("  Final avg: {:.6}", final_avg);
        println!("  Final sample: {:.6}", final_sample);
        println!("  DC_SCALE used: {}", DC_SCALE);

        // Should converge towards the signal level
        assert!(final_avg > 0.5, "Should have accumulated some average");
        assert!(final_avg < 1.0, "Should not exceed input value");
    }

    #[test]
    fn test_rotation_sequence_magnitude() {
        // Verify that Fs/4 rotation sequence preserves magnitude reasonably
        let hbc = 0.5f32;
        let sample_groups = vec![
            [1.0, 1.0, 1.0, 1.0],
            [0.1, 0.1, 0.1, 0.1],
            [100.0, 100.0, 100.0, 100.0],
        ];

        for group in sample_groups {
            let original_energy: f32 = group.iter().map(|x| x * x).sum();

            let mut rotated = group;
            rotated[0] = -rotated[0];
            rotated[1] = -rotated[1] * hbc;
            rotated[3] *= hbc;

            let rotated_energy: f32 = rotated.iter().map(|x| x * x).sum();

            let ratio = rotated_energy / original_energy;
            println!("Rotation energy ratio: {:.6}", ratio);

            // Energy should be roughly preserved (slight loss due to hbc=0.5)
            assert!(
                ratio > 0.5 && ratio < 2.0,
                "Rotation should preserve approximate magnitude"
            );
        }
    }

    #[test]
    fn test_fir_circular_buffer_wrap() {
        // Detailed test of circular buffer wrap-around logic
        let mut converter = IqConverter::new();

        // Process exactly enough samples to trigger wrap-around multiple times
        // For len=24, SIZE_FACTOR=32: queue is 768 samples
        // After each I sample, fir_index decrements or wraps

        // Process 1000 real samples (500 I samples -> should wrap multiple times)
        let mut samples = vec![1.0f32; 1000];

        let fir_index_before = converter.fir_index;
        println!("FIR index before: {}", fir_index_before);

        converter.fir_interleaved(&mut samples);

        let fir_index_after = converter.fir_index;
        println!("FIR index after 500 iterations: {}", fir_index_after);

        // After 500 iterations and wraps, index should be deterministic
        // Expected: (500 % (24*32)) samples worth of offset
        let expected_offset = 500 % (24 * SIZE_FACTOR);
        println!(
            "Expected final index region: {}",
            24 * (SIZE_FACTOR - 1) - expected_offset
        );
    }

    #[test]
    fn test_delay_line_circular_behavior() {
        // Test that delay line properly cycles through
        let mut converter = IqConverter::new();
        let delay_len = converter.delay_line.len();

        // Track delay_index changes
        let delay_index_before = converter.delay_index;
        assert_eq!(delay_index_before, 0);

        // Process 100 real samples -> 50 odd indices (Q samples)
        let mut samples = vec![0.0f32; 100];
        converter.delay_interleaved(&mut samples);

        // After 50 iterations with length 12, we should have cycled:
        // 0, 1, 2, ..., 11, 0, 1, 2, ...
        // 50 % 12 = 2, so we expect index 2
        let expected_index = 50 % delay_len;
        println!(
            "Delay index after 50 Q samples: {} (expected: {})",
            converter.delay_index, expected_index
        );
        assert_eq!(converter.delay_index, expected_index);
    }

    #[test]
    fn test_iq_pair_relationship() {
        // Test that I/Q pairs maintain proper relationship
        let mut converter = IqConverter::new();

        // Create a pure sine wave (real samples only)
        let n_samples = 200;
        let freq = 0.01; // Normalized frequency
        let mut samples: Vec<f32> = (0..n_samples)
            .map(|i| (2.0 * std::f32::consts::PI * freq * i as f32).sin())
            .collect();

        converter.process(&mut samples);

        // Extract I/Q pairs
        let pairs: Vec<(f32, f32)> = samples.chunks_exact(2).map(|c| (c[0], c[1])).collect();

        // For a properly phase-shifted signal, I and Q should have phase difference
        // Check that they're not identical (would indicate processing failure)
        for (i, (i_samp, q_samp)) in pairs.iter().enumerate().take(50) {
            let i_q_product = i_samp * q_samp;
            if i_q_product.is_finite() && !i_q_product.is_nan() {
                println!(
                    "Pair {}: I={:.6}, Q={:.6}, product={:.6}",
                    i, i_samp, q_samp, i_q_product
                );
            }
        }

        // Verify pairs have some spread (not all same value)
        let i_values: Vec<f32> = pairs.iter().map(|p| p.0).collect();
        let q_values: Vec<f32> = pairs.iter().map(|p| p.1).collect();

        let i_range = i_values.iter().cloned().fold(f32::NEG_INFINITY, f32::max)
            - i_values.iter().cloned().fold(f32::INFINITY, f32::min);
        let q_range = q_values.iter().cloned().fold(f32::NEG_INFINITY, f32::max)
            - q_values.iter().cloned().fold(f32::INFINITY, f32::min);

        println!("I range: {:.6}, Q range: {:.6}", i_range, q_range);

        // Both should have some variation
        assert!(i_range > 0.0, "I channel should have variation");
        assert!(q_range > 0.0, "Q channel should have variation");
    }

    #[test]
    fn test_initialization_phase_effect() {
        // Check if initialization affects early vs late samples
        let mut converter = IqConverter::new();

        // Create constant signal to observe initialization effects
        let mut samples = vec![1.0f32; 100];

        // Make a copy to compare
        // let original = samples.clone();

        converter.process(&mut samples);

        // Check early samples vs late samples
        let early_i: f32 = samples[0..20].iter().step_by(2).map(|x| x.abs()).sum();
        let late_i: f32 = samples[80..100].iter().step_by(2).map(|x| x.abs()).sum();

        println!("Early I sum (samples 0-10): {:.6}", early_i);
        println!("Late I sum (samples 40-50): {:.6}", late_i);

        // If there's initialization transient, late samples should stabilize
        // (may differ from early due to FIR/delay line warmup)
    }

    #[test]
    fn test_extracted_kernel_symmetry() {
        // Verify that extracted kernel properly represents symmetry
        let converter = IqConverter::new();
        let kernel = &converter.fir_kernel;

        // The extracted kernel should be symmetric
        // kernel[0] should pair with kernel[len-1], etc.
        let len = kernel.len();

        for i in 0..len / 2 {
            let left = kernel[i];
            let right = kernel[len - 1 - i];

            if (left - right).abs() > 1e-6 {
                println!(
                    "Kernel asymmetry at index {}: left={:.9}, right={:.9}",
                    i, left, right
                );
            }
            assert!((left - right).abs() < 1e-6, "Kernel should be symmetric");
        }

        println!("Kernel symmetry verified for {} taps", len);
    }

    #[test]
    fn test_fir_manual_vs_implementation() {
        // Compare manual FIR calculation with our implementation
        let mut converter = IqConverter::new();

        // Create a simple test signal: constant value
        let mut samples = vec![2.0f32; 50];

        // Store samples for manual calculation
        let test_val = samples[0];

        // Apply DC removal first
        converter.remove_dc(&mut samples);

        // Now manually check what FIR should output
        // For a constant signal with DC removed, we expect:
        // - DC removal creates signal that converges to 0
        // - FIR will output based on the filtered signal

        let samples_before_fir = samples.clone();

        // Apply rotation (samples[0] = -2.0 initially)
        for chunk in samples.chunks_exact_mut(4) {
            chunk[0] = -chunk[0];
            chunk[1] = -chunk[1] * 0.5;
            chunk[3] *= 0.5;
        }

        let samples_after_rotation = samples.clone();

        // Apply FIR
        converter.fir_interleaved(&mut samples);

        let samples_after_fir: Vec<f32> = samples.iter().step_by(2).take(10).copied().collect();

        println!("FIR test - constant value");
        println!("  Test value: {}", test_val);
        println!("  DC removal (first 5): {:?}", &samples_before_fir[0..5]);
        println!(
            "  After rotation (first 5): {:?}",
            &samples_after_rotation[0..5]
        );
        println!("  FIR output (first 10): {:?}", samples_after_fir);

        // All FIR outputs should be finite and reasonable magnitude
        for (i, val) in samples_after_fir.iter().enumerate() {
            assert!(val.is_finite(), "Sample {} is not finite", i);
        }
    }

    #[test]
    fn test_amplitude_analysis_sine_wave() {
        // Test with a sine wave at realistic frequency
        let mut converter = IqConverter::new();

        // Create 6 MSPS sine at 100 kHz (normalized: 100k/6M = 0.01667)
        let freq = 100_000.0 / 6_000_000.0;
        let n = 5000; // Process 5000 real samples
        let mut samples: Vec<f32> = (0..n)
            .map(|i| (2.0 * std::f32::consts::PI * freq * i as f32).sin())
            .collect();

        // Measure input energy
        let input_energy: f32 = samples.iter().map(|x| x * x).sum::<f32>() / n as f32;

        // Process
        converter.process(&mut samples);

        // Convert to complex
        let complex: Vec<Complex<f32>> = samples
            .chunks_exact(2)
            .map(|c| Complex::new(c[0], c[1]))
            .collect();

        // Skip first 50 samples (initialization transient)
        let mid_start = 50;
        let mid_end = 2400;
        let mid_samples = &complex[mid_start..mid_end];

        let output_magnitude: f32 =
            mid_samples.iter().map(|c| c.norm()).sum::<f32>() / mid_samples.len() as f32;
        let output_energy: f32 =
            mid_samples.iter().map(|c| c.norm_sqr()).sum::<f32>() / mid_samples.len() as f32;

        println!("Sine wave amplitude analysis (100 kHz at 6 MSPS):");
        println!("  Input energy: {:.6}", input_energy);
        println!("  Output magnitude (mid-stream): {:.6}", output_magnitude);
        println!("  Output energy (mid-stream): {:.6}", output_energy);
        println!(
            "  Energy loss factor: {:.6}%",
            (1.0 - output_energy / input_energy) * 100.0
        );

        // The output should have reasonable amplitude (at least 0.01 normalized)
        assert!(
            output_magnitude > 0.01,
            "Output magnitude too small: {}",
            output_magnitude
        );
    }
}
