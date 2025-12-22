//! Adaptive audio resampling for real-time streaming applications.
//!
//! This module provides adaptive resampling capabilities using the `rubato` crate,
//! designed for real-time audio streaming where the sample rate needs to dynamically
//! adjust to maintain buffer levels.
//!
//! # Feature Flag
//!
//! This module is only available when the `adaptive` feature is enabled.
//!
//! # Example
//!
//! ```no_run
//! use desperado::dsp::resampler::AdaptiveResampler;
//!
//! // Resample from 240kHz to 48kHz (mono)
//! let mut resampler = AdaptiveResampler::new(
//!     48_000.0 / 240_000.0,  // Initial ratio
//!     1,                      // Adjustment interval
//!     1,                      // Mono (1 channel)
//! ).unwrap();
//!
//! let input = vec![0.0; 1000];
//! let output = resampler.process(&input);
//! ```

use rubato::{
    Resampler, SincFixedOut, SincInterpolationParameters, SincInterpolationType, WindowFunction,
};

/// Adaptive resampler for real-time audio streaming.
///
/// Uses a PI (Proportional-Integral) controller to dynamically adjust the resampling
/// ratio based on buffer fill level. This helps maintain a stable buffer in streaming
/// applications where there might be slight mismatches between input and output rates.
///
/// The resampler uses `rubato::SincFixedOut` for high-quality sinc interpolation.
///
/// # Example
///
/// ```no_run
/// use desperado::dsp::resampler::AdaptiveResampler;
///
/// // Create mono resampler from FM bandwidth to audio rate
/// let mut resampler = AdaptiveResampler::new(
///     48_000.0 / 240_000.0,  // ratio
///     1,                      // adjustment interval
///     1,                      // channels
/// ).unwrap();
///
/// // Process audio
/// let input = vec![0.5; 240];
/// let output = resampler.process(&input);
///
/// // Adjust based on buffer fill (0.0 = empty, 1.0 = full)
/// resampler.adjust_ratio(0.3); // 30% full
/// ```
pub struct AdaptiveResampler {
    resampler: SincFixedOut<f32>,
    target_fill: f64,
    alpha: f64,
    k_p: f64,
    k_i: f64,
    smoothed_error: f64,
    integral_error: f64,
    adjustment_interval: usize,
    adjustment_counter: usize,
    leftover: Vec<f32>,
    resample_ratio: f64,
    nominal_ratio: f64,
    channels: usize,
}

impl AdaptiveResampler {
    /// Create a new adaptive resampler.
    ///
    /// # Arguments
    ///
    /// * `initial_ratio` - Initial resampling ratio (output_rate / input_rate)
    /// * `adjustment_interval` - How often to adjust the ratio (in process() calls)
    /// * `channels` - Number of audio channels (1 = mono, 2 = stereo, etc.)
    ///
    /// # Returns
    ///
    /// Returns `Ok(AdaptiveResampler)` on success, or `Err(String)` if resampler
    /// creation fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use desperado::dsp::resampler::AdaptiveResampler;
    ///
    /// // Mono resampler
    /// let mono = AdaptiveResampler::new(0.2, 1, 1).unwrap();
    ///
    /// // Stereo resampler
    /// let stereo = AdaptiveResampler::new(0.2, 5, 2).unwrap();
    /// ```
    pub fn new(
        initial_ratio: f64,
        adjustment_interval: usize,
        channels: usize,
    ) -> Result<Self, String> {
        let params = SincInterpolationParameters {
            sinc_len: 256,
            f_cutoff: 0.95,
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 160,
            window: WindowFunction::BlackmanHarris2,
        };

        let output_frames = 1024;
        let max_resample_ratio_relative = 1.02;

        let resampler = SincFixedOut::<f32>::new(
            initial_ratio,
            max_resample_ratio_relative,
            params,
            output_frames,
            channels,
        )
        .map_err(|e| format!("Failed to create resampler: {:?}", e))?;

        Ok(Self {
            resampler,
            target_fill: 0.3,
            alpha: 0.95,
            k_p: 0.0001,
            k_i: 1e-7,
            smoothed_error: 0.0,
            integral_error: 0.0,
            adjustment_interval,
            adjustment_counter: 0,
            leftover: Vec::new(),
            resample_ratio: initial_ratio,
            nominal_ratio: initial_ratio,
            channels,
        })
    }

    /// Create a new adaptive resampler with custom PI controller parameters.
    ///
    /// # Arguments
    ///
    /// * `initial_ratio` - Initial resampling ratio
    /// * `adjustment_interval` - Adjustment frequency
    /// * `channels` - Number of channels
    /// * `target_fill` - Target buffer fill level (0.0-1.0)
    /// * `k_p` - Proportional gain for PI controller
    /// * `k_i` - Integral gain for PI controller
    ///
    /// # Example
    ///
    /// ```no_run
    /// use desperado::dsp::resampler::AdaptiveResampler;
    ///
    /// // More aggressive adaptation
    /// let resampler = AdaptiveResampler::with_params(
    ///     0.2, 5, 2,    // ratio, interval, channels
    ///     0.4,          // target 40% buffer fill
    ///     0.002,        // higher proportional gain
    ///     5e-6,         // higher integral gain
    /// ).unwrap();
    /// ```
    pub fn with_params(
        initial_ratio: f64,
        adjustment_interval: usize,
        channels: usize,
        target_fill: f64,
        k_p: f64,
        k_i: f64,
    ) -> Result<Self, String> {
        let mut resampler = Self::new(initial_ratio, adjustment_interval, channels)?;
        resampler.target_fill = target_fill;
        resampler.k_p = k_p;
        resampler.k_i = k_i;
        Ok(resampler)
    }

    /// Process audio samples through the adaptive resampler.
    ///
    /// Input is expected to be interleaved for multi-channel audio (L, R, L, R, ...).
    /// Output is also interleaved.
    ///
    /// # Arguments
    ///
    /// * `input` - Interleaved audio samples
    ///
    /// # Returns
    ///
    /// Resampled audio samples (interleaved if multi-channel)
    ///
    /// # Example
    ///
    /// ```no_run
    /// use desperado::dsp::resampler::AdaptiveResampler;
    ///
    /// let mut resampler = AdaptiveResampler::new(0.2, 1, 1).unwrap();
    /// let input = vec![0.1, 0.2, 0.3, 0.4, 0.5];
    /// let output = resampler.process(&input);
    /// ```
    #[allow(clippy::needless_range_loop)]
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        self.leftover.extend_from_slice(input);
        let mut output = Vec::new();

        loop {
            let input_frames_needed = self.resampler.input_frames_next();
            let samples_needed = input_frames_needed * self.channels;

            if self.leftover.len() < samples_needed {
                break;
            }

            let chunk: Vec<f32> = self.leftover.drain(..samples_needed).collect();

            // Deinterleave for rubato (index-based loop needed for multi-channel interleaving)
            #[allow(clippy::needless_range_loop)]
            let mut channels_data: Vec<Vec<f32>> =
                vec![Vec::with_capacity(input_frames_needed); self.channels];
            for frame_idx in 0..input_frames_needed {
                for ch in 0..self.channels {
                    let idx = frame_idx * self.channels + ch;
                    channels_data[ch].push(chunk[idx]);
                }
            }

            match self.resampler.process(&channels_data, None) {
                Ok(output_blocks) => {
                    let out_frames = output_blocks[0].len();
                    // Reinterleave (index-based loop needed for multi-channel interleaving)
                    #[allow(clippy::needless_range_loop)]
                    for i in 0..out_frames {
                        for ch in 0..self.channels {
                            output.push(output_blocks[ch][i]);
                        }
                    }
                }
                Err(_) => break,
            }
        }

        output
    }

    /// Adjust the resampling ratio based on buffer fill level.
    ///
    /// Uses a PI controller to smoothly adjust the ratio to maintain the target
    /// buffer fill level. Should be called periodically (e.g., every few process() calls).
    ///
    /// # Arguments
    ///
    /// * `buffer_fill` - Current buffer fill ratio (0.0 = empty, 1.0 = full)
    ///
    /// # Example
    ///
    /// ```ignore
    /// use desperado::dsp::resampler::AdaptiveResampler;
    ///
    /// let mut resampler = AdaptiveResampler::new(0.2, 5, 1).unwrap();
    ///
    /// // In streaming loop:
    /// let output = resampler.process(&input_data);
    /// let fill_ratio = buffer.len() as f64 / buffer.capacity() as f64;
    /// resampler.adjust_ratio(fill_ratio);
    /// ```
    pub fn adjust_ratio(&mut self, buffer_fill: f64) {
        self.adjustment_counter += 1;
        if self.adjustment_counter < self.adjustment_interval {
            return;
        }
        self.adjustment_counter = 0;

        let error = self.target_fill - buffer_fill;

        // Exponential smoothing
        self.smoothed_error = self.alpha * self.smoothed_error + (1.0 - self.alpha) * error;

        // Deadband to prevent jitter
        if self.smoothed_error.abs() < 0.01 {
            self.smoothed_error = 0.0;
        }

        // Integral term with anti-windup
        if self.smoothed_error.abs() < 0.15 {
            self.integral_error += self.smoothed_error;
            self.integral_error = self.integral_error.clamp(-100.0, 100.0);
        }

        // PI controller
        self.resample_ratio += self.k_p * self.smoothed_error + self.k_i * self.integral_error;

        // Clamp to reasonable range
        let min_ratio = self.nominal_ratio * 0.97;
        let max_ratio = self.nominal_ratio * 1.03;
        self.resample_ratio = self.resample_ratio.clamp(min_ratio, max_ratio);

        let _ = self.resampler.set_resample_ratio(self.resample_ratio, true);
    }

    /// Get the current resampling ratio.
    pub fn ratio(&self) -> f64 {
        self.resample_ratio
    }

    /// Get the nominal (initial) resampling ratio.
    pub fn nominal_ratio(&self) -> f64 {
        self.nominal_ratio
    }

    /// Get the current PI controller error.
    pub fn error(&self) -> f64 {
        self.smoothed_error
    }

    /// Get the number of leftover samples in the internal buffer.
    pub fn leftover_samples(&self) -> usize {
        self.leftover.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adaptive_resampler_creation() {
        let resampler = AdaptiveResampler::new(0.2, 1, 1);
        assert!(resampler.is_ok());

        let resampler = resampler.unwrap();
        assert_eq!(resampler.ratio(), 0.2);
        assert_eq!(resampler.nominal_ratio(), 0.2);
    }

    #[test]
    fn test_adaptive_resampler_stereo() {
        let resampler = AdaptiveResampler::new(0.2, 1, 2);
        assert!(resampler.is_ok());
    }

    #[test]
    fn test_adaptive_resampler_with_params() {
        let resampler = AdaptiveResampler::with_params(0.2, 5, 1, 0.4, 0.002, 5e-6);
        assert!(resampler.is_ok());

        let resampler = resampler.unwrap();
        assert_eq!(resampler.target_fill, 0.4);
        assert_eq!(resampler.k_p, 0.002);
        assert_eq!(resampler.k_i, 5e-6);
    }

    #[test]
    fn test_adaptive_resampler_process() {
        let mut resampler = AdaptiveResampler::new(1.0, 1, 1).unwrap();

        // Provide enough samples for processing
        let input = vec![0.5; 2048];
        let output = resampler.process(&input);

        // Should produce output
        assert!(!output.is_empty());
    }

    #[test]
    fn test_adaptive_resampler_ratio_adjustment() {
        let mut resampler = AdaptiveResampler::new(0.2, 1, 1).unwrap();
        let initial_ratio = resampler.ratio();

        // Simulate buffer too full (should decrease ratio)
        resampler.adjust_ratio(0.8);

        // Ratio should change after adjustment
        let adjusted_ratio = resampler.ratio();
        assert_ne!(initial_ratio, adjusted_ratio);
    }

    #[test]
    fn test_adaptive_resampler_leftover() {
        let mut resampler = AdaptiveResampler::new(1.0, 1, 1).unwrap();

        // Small input that won't be fully processed
        let input = vec![0.5; 10];
        let _ = resampler.process(&input);

        // Should have leftover samples
        assert!(resampler.leftover_samples() > 0);
    }
}
