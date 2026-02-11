//! Automatic Gain Control (AGC)
//!
//! This module provides a liquid-dsp compatible AGC implementation for
//! normalizing signal amplitude in software-defined radio applications.
//!
//! # Overview
//!
//! The AGC automatically adjusts gain to maintain a target output amplitude.
//! It tracks the signal envelope and applies inverse gain to normalize the
//! signal level. This is essential for digital demodulation where signal
//! strength varies.
//!
//! # Design
//!
//! The implementation follows liquid-dsp's `agc_crcf` design:
//! - Exponential moving average for envelope tracking
//! - Logarithmic gain domain for stable operation
//! - Configurable bandwidth controls attack/decay rate
//!
//! # Example
//!
//! ```
//! use desperado::dsp::agc::Agc;
//!
//! // Create AGC with 0.01 bandwidth
//! let mut agc = Agc::new(0.01);
//!
//! // Process samples
//! let mut samples = vec![(1.0, 0.5), (0.8, 0.6), (1.2, 0.4)];
//! for sample in samples.iter_mut() {
//!     let (out_i, out_q) = agc.execute(sample.0, sample.1);
//!     *sample = (out_i, out_q);
//! }
//! ```
//!
//! # Reference
//!
//! This implementation is based on liquid-dsp's `agc_crcf` object.
//! See: <https://github.com/jgaeddert/liquid-dsp>

/// Automatic Gain Control for complex signals.
///
/// The AGC normalizes signal amplitude by tracking the envelope and
/// applying inverse gain. This provides consistent signal levels for
/// downstream processing like demodulation.
#[derive(Debug, Clone)]
pub struct Agc {
    /// Target output amplitude (default 1.0)
    target: f32,

    /// Loop bandwidth (controls attack/decay rate)
    bandwidth: f32,

    /// Current gain value
    gain: f32,

    /// Envelope estimate (for tracking)
    envelope: f32,

    /// Minimum gain (to prevent instability)
    gain_min: f32,

    /// Maximum gain (to prevent noise amplification)
    gain_max: f32,

    /// AGC locked flag
    is_locked: bool,
}

impl Agc {
    /// Create a new AGC with specified bandwidth.
    ///
    /// # Arguments
    ///
    /// * `bandwidth` - Loop bandwidth [0, 1]. Lower values provide smoother
    ///   gain adjustment but slower response to level changes.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::agc::Agc;
    ///
    /// let agc = Agc::new(0.01);
    /// ```
    pub fn new(bandwidth: f32) -> Self {
        Self {
            target: 1.0,
            bandwidth: bandwidth.clamp(0.0, 1.0),
            gain: 1.0,
            envelope: 1.0,
            gain_min: 1e-6,
            gain_max: 1e6,
            is_locked: false,
        }
    }

    /// Create AGC with custom target amplitude.
    ///
    /// # Arguments
    ///
    /// * `bandwidth` - Loop bandwidth [0, 1]
    /// * `target` - Target output amplitude (default 1.0)
    pub fn with_target(bandwidth: f32, target: f32) -> Self {
        Self {
            target,
            bandwidth: bandwidth.clamp(0.0, 1.0),
            gain: 1.0,
            envelope: 1.0,
            gain_min: 1e-6,
            gain_max: 1e6,
            is_locked: false,
        }
    }

    /// Set the loop bandwidth.
    ///
    /// # Arguments
    ///
    /// * `bandwidth` - Loop bandwidth [0, 1]
    pub fn set_bandwidth(&mut self, bandwidth: f32) {
        self.bandwidth = bandwidth.clamp(0.0, 1.0);
    }

    /// Get the current bandwidth.
    pub fn get_bandwidth(&self) -> f32 {
        self.bandwidth
    }

    /// Set the target output amplitude.
    pub fn set_target(&mut self, target: f32) {
        self.target = target;
    }

    /// Get the target output amplitude.
    pub fn get_target(&self) -> f32 {
        self.target
    }

    /// Set gain limits.
    ///
    /// # Arguments
    ///
    /// * `min` - Minimum gain value
    /// * `max` - Maximum gain value
    pub fn set_gain_limits(&mut self, min: f32, max: f32) {
        self.gain_min = min.max(1e-10);
        self.gain_max = max.min(1e10);
    }

    /// Get the current gain value.
    pub fn get_gain(&self) -> f32 {
        self.gain
    }

    /// Set the gain directly (useful for initialization).
    pub fn set_gain(&mut self, gain: f32) {
        self.gain = gain.clamp(self.gain_min, self.gain_max);
    }

    /// Get the current envelope estimate.
    pub fn get_envelope(&self) -> f32 {
        self.envelope
    }

    /// Lock the AGC.
    ///
    /// When locked, the gain is not updated (useful for stable operation
    /// after acquisition).
    pub fn lock(&mut self) {
        self.is_locked = true;
    }

    /// Unlock the AGC.
    pub fn unlock(&mut self) {
        self.is_locked = false;
    }

    /// Check if the AGC is locked.
    pub fn is_locked(&self) -> bool {
        self.is_locked
    }

    /// Reset the AGC state.
    pub fn reset(&mut self) {
        self.gain = 1.0;
        self.envelope = 1.0;
    }

    /// Process a complex sample through the AGC.
    ///
    /// # Arguments
    ///
    /// * `i` - In-phase (real) component
    /// * `q` - Quadrature (imaginary) component
    ///
    /// # Returns
    ///
    /// Normalized (I, Q) tuple.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::agc::Agc;
    ///
    /// let mut agc = Agc::new(0.01);
    /// let (out_i, out_q) = agc.execute(0.5, 0.3);
    /// ```
    pub fn execute(&mut self, i: f32, q: f32) -> (f32, f32) {
        // Apply current gain
        let out_i = i * self.gain;
        let out_q = q * self.gain;

        // Update gain if not locked
        if !self.is_locked {
            // Compute output magnitude (squared for efficiency)
            let mag_sq = out_i * out_i + out_q * out_q;
            let mag = mag_sq.sqrt();

            // Update envelope estimate (exponential moving average)
            self.envelope = (1.0 - self.bandwidth) * self.envelope + self.bandwidth * mag;

            // Compute error (in log domain for stability)
            if self.envelope > 1e-10 {
                let error = self.target / self.envelope;

                // Update gain with bandwidth-scaled error
                self.gain *= 1.0 + self.bandwidth * (error - 1.0);

                // Clamp gain to limits
                self.gain = self.gain.clamp(self.gain_min, self.gain_max);
            }
        }

        (out_i, out_q)
    }

    /// Process a real sample through the AGC.
    ///
    /// # Arguments
    ///
    /// * `x` - Input sample
    ///
    /// # Returns
    ///
    /// Normalized output sample.
    pub fn execute_real(&mut self, x: f32) -> f32 {
        let (out, _) = self.execute(x, 0.0);
        out
    }

    /// Process a batch of complex samples.
    ///
    /// # Arguments
    ///
    /// * `input` - Input samples as (I, Q) tuples
    ///
    /// # Returns
    ///
    /// Vector of normalized (I, Q) tuples.
    pub fn execute_batch(&mut self, input: &[(f32, f32)]) -> Vec<(f32, f32)> {
        input.iter().map(|&(i, q)| self.execute(i, q)).collect()
    }

    /// Get the current signal level in dB.
    ///
    /// This is useful for monitoring AGC operation.
    pub fn get_signal_level_db(&self) -> f32 {
        20.0 * self.envelope.log10()
    }

    /// Get the current gain in dB.
    pub fn get_gain_db(&self) -> f32 {
        20.0 * self.gain.log10()
    }
}

impl Default for Agc {
    fn default() -> Self {
        Self::new(0.01)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_agc_creation() {
        let agc = Agc::new(0.01);
        assert!((agc.get_bandwidth() - 0.01).abs() < 1e-6);
        assert!((agc.get_target() - 1.0).abs() < 1e-6);
        assert!((agc.get_gain() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_agc_unity_gain() {
        let mut agc = Agc::new(0.01);

        // With unit input, output should be approximately unit
        for _ in 0..1000 {
            let (out_i, out_q) = agc.execute(1.0, 0.0);
            // After settling, output magnitude should be close to target
            let mag = (out_i * out_i + out_q * out_q).sqrt();
            if agc.get_envelope() > 0.9 {
                assert!(
                    (mag - 1.0).abs() < 0.2,
                    "Output magnitude {} too far from target",
                    mag
                );
            }
        }
    }

    #[test]
    fn test_agc_amplification() {
        let mut agc = Agc::new(0.1); // Faster bandwidth for test

        // With small input, gain should increase
        for _ in 0..1000 {
            agc.execute(0.1, 0.0);
        }

        // Gain should be greater than 1 to amplify small signal
        assert!(
            agc.get_gain() > 1.0,
            "Gain {} should be > 1",
            agc.get_gain()
        );
    }

    #[test]
    fn test_agc_attenuation() {
        let mut agc = Agc::new(0.1); // Faster bandwidth for test

        // With large input, gain should decrease
        for _ in 0..1000 {
            agc.execute(10.0, 0.0);
        }

        // Gain should be less than 1 to attenuate large signal
        assert!(
            agc.get_gain() < 1.0,
            "Gain {} should be < 1",
            agc.get_gain()
        );
    }

    #[test]
    fn test_agc_lock() {
        let mut agc = Agc::new(0.1);

        // Process some samples
        for _ in 0..100 {
            agc.execute(0.5, 0.0);
        }

        let gain_before = agc.get_gain();

        // Lock and process more
        agc.lock();
        assert!(agc.is_locked());

        for _ in 0..100 {
            agc.execute(2.0, 0.0); // Different amplitude
        }

        // Gain should not have changed
        assert!(
            (agc.get_gain() - gain_before).abs() < 1e-6,
            "Gain changed while locked"
        );

        // Unlock and verify gain changes again
        agc.unlock();
        for _ in 0..100 {
            agc.execute(2.0, 0.0);
        }
        assert!(
            (agc.get_gain() - gain_before).abs() > 0.01,
            "Gain should change after unlock"
        );
    }

    #[test]
    fn test_agc_reset() {
        let mut agc = Agc::new(0.1);

        // Process samples to change state
        for _ in 0..100 {
            agc.execute(0.1, 0.0);
        }

        assert!((agc.get_gain() - 1.0).abs() > 0.1);

        // Reset
        agc.reset();
        assert!((agc.get_gain() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_agc_bandwidth_clamping() {
        let mut agc = Agc::new(0.5);

        agc.set_bandwidth(2.0);
        assert!((agc.get_bandwidth() - 1.0).abs() < 1e-6);

        agc.set_bandwidth(-0.5);
        assert!((agc.get_bandwidth() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_agc_complex_signal() {
        let mut agc = Agc::new(0.1);

        // Process complex signal with varying magnitude
        for i in 0..1000 {
            let phase = (i as f32) * 0.1;
            let amp = 0.5 + 0.3 * (i as f32 * 0.01).sin();
            let (out_i, out_q) = agc.execute(amp * phase.cos(), amp * phase.sin());

            // Output should be roughly normalized
            if i > 500 {
                let out_mag = (out_i * out_i + out_q * out_q).sqrt();
                assert!(
                    out_mag < 2.0 && out_mag > 0.1,
                    "Output magnitude {} out of range",
                    out_mag
                );
            }
        }
    }
}
