//! Numerically Controlled Oscillator (NCO) with Phase-Locked Loop (PLL)
//!
//! This module provides a liquid-dsp compatible NCO implementation for
//! carrier frequency generation and tracking in software-defined radio applications.
//!
//! # Overview
//!
//! The NCO generates complex sinusoids at a programmable frequency. When combined
//! with the integrated PLL, it can track an incoming carrier signal by adjusting
//! its frequency and phase based on a phase error signal.
//!
//! # Design
//!
//! The implementation follows liquid-dsp's NCO design:
//! - Phase is represented as a value in [0, 1) representing one full cycle
//! - Frequency is in cycles per sample (normalized to sample rate)
//! - The PLL uses a second-order loop filter with proportional (beta) and
//!   integral (alpha) gains
//!
//! # Example
//!
//! ```
//! use desperado::dsp::nco::Nco;
//!
//! // Create NCO at 57 kHz with 171 kHz sample rate
//! let mut nco = Nco::new(57000.0, 171000.0);
//!
//! // Set PLL bandwidth to 0.03 Hz for very narrow tracking
//! nco.set_pll_bandwidth(0.03, 171000.0);
//!
//! // Mix a real sample down to baseband
//! let input_sample = 1.0f32;
//! let (i, q) = nco.mix_down(input_sample);
//!
//! // Step the NCO phase forward
//! nco.step();
//! ```
//!
//! # Reference
//!
//! This implementation is based on liquid-dsp's `nco_crcf` object.
//! See: <https://github.com/jgaeddert/liquid-dsp>

use std::f64::consts::PI;

/// Numerically Controlled Oscillator with integrated Phase-Locked Loop.
///
/// The NCO generates complex sinusoids and can track an input carrier using
/// the PLL functionality. This is commonly used for carrier recovery in
/// coherent demodulation systems like BPSK/QPSK and RDS decoding.
#[derive(Debug, Clone)]
pub struct Nco {
    /// Current phase in cycles [0, 1)
    phase: f64,

    /// Frequency in cycles per sample
    frequency: f64,

    /// PLL frequency proportion (integral gain)
    /// Higher alpha = faster frequency tracking, more noise
    alpha: f64,

    /// PLL phase proportion (proportional gain)
    /// Typically beta = sqrt(alpha) for critical damping
    beta: f64,
}

impl Nco {
    /// Create a new NCO at the specified frequency.
    ///
    /// # Arguments
    ///
    /// * `frequency_hz` - Initial oscillator frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::nco::Nco;
    ///
    /// // Create NCO at 57 kHz with 171 kHz sample rate
    /// let nco = Nco::new(57000.0, 171000.0);
    /// ```
    pub fn new(frequency_hz: f64, sample_rate: f64) -> Self {
        let frequency = frequency_hz / sample_rate;

        Self {
            phase: 0.0,
            frequency,
            alpha: 0.1, // Default bandwidth, will be overridden by set_pll_bandwidth
            beta: 0.1_f64.sqrt(),
        }
    }

    /// Set the PLL loop bandwidth.
    ///
    /// The bandwidth controls how quickly the PLL tracks frequency and phase
    /// changes. Lower bandwidth provides better noise rejection but slower
    /// acquisition; higher bandwidth tracks faster but passes more noise.
    ///
    /// The PLL uses a second-order loop filter with:
    /// - alpha (frequency gain) = bandwidth (normalized)
    /// - beta (phase gain) = sqrt(alpha)
    ///
    /// This provides critically damped response.
    ///
    /// # Arguments
    ///
    /// * `bandwidth_hz` - Loop bandwidth in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::nco::Nco;
    ///
    /// let mut nco = Nco::new(57000.0, 171000.0);
    ///
    /// // Very narrow bandwidth for RDS (0.03 Hz as used by redsea)
    /// nco.set_pll_bandwidth(0.03, 171000.0);
    /// ```
    pub fn set_pll_bandwidth(&mut self, bandwidth_hz: f64, sample_rate: f64) {
        // Normalize bandwidth to sample rate
        // liquid-dsp uses this directly as alpha
        self.alpha = bandwidth_hz / sample_rate;
        self.beta = self.alpha.sqrt();
    }

    /// Set the PLL gains directly (advanced).
    ///
    /// For non-standard loop filter designs, you can set alpha and beta directly.
    ///
    /// # Arguments
    ///
    /// * `alpha` - Frequency (integral) gain
    /// * `beta` - Phase (proportional) gain
    pub fn set_pll_gains(&mut self, alpha: f64, beta: f64) {
        self.alpha = alpha;
        self.beta = beta;
    }

    /// Get the current phase in radians [0, 2*PI).
    pub fn get_phase_radians(&self) -> f64 {
        self.phase * 2.0 * PI
    }

    /// Get the current phase in cycles [0, 1).
    pub fn get_phase(&self) -> f64 {
        self.phase
    }

    /// Set the phase directly in cycles [0, 1).
    pub fn set_phase(&mut self, phase: f64) {
        self.phase = phase - phase.floor();
    }

    /// Set the phase in radians.
    pub fn set_phase_radians(&mut self, phase_rad: f64) {
        self.set_phase(phase_rad / (2.0 * PI));
    }

    /// Get the current frequency in Hz.
    ///
    /// # Arguments
    ///
    /// * `sample_rate` - Sample rate in Hz (needed to convert from normalized)
    pub fn get_frequency_hz(&self, sample_rate: f64) -> f64 {
        self.frequency * sample_rate
    }

    /// Get the current frequency in cycles per sample (normalized).
    pub fn get_frequency(&self) -> f64 {
        self.frequency
    }

    /// Set the frequency in Hz.
    ///
    /// # Arguments
    ///
    /// * `frequency_hz` - Frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    pub fn set_frequency_hz(&mut self, frequency_hz: f64, sample_rate: f64) {
        self.frequency = frequency_hz / sample_rate;
    }

    /// Set the frequency in cycles per sample (normalized).
    pub fn set_frequency(&mut self, frequency: f64) {
        self.frequency = frequency;
    }

    /// Adjust the frequency by a delta (in cycles per sample).
    pub fn adjust_frequency(&mut self, delta: f64) {
        self.frequency += delta;
    }

    /// Adjust the phase by a delta (in cycles).
    pub fn adjust_phase(&mut self, delta: f64) {
        self.phase += delta;
        // Wrap to [0, 1)
        self.phase -= self.phase.floor();
    }

    /// Get the current complex phasor exp(j * 2*PI * phase).
    ///
    /// Returns (cos(theta), sin(theta)) = (I, Q) = (real, imag).
    pub fn get_complex(&self) -> (f32, f32) {
        let theta = 2.0 * PI * self.phase;
        (theta.cos() as f32, theta.sin() as f32)
    }

    /// Compute sin and cos of current phase.
    ///
    /// Returns (sin(theta), cos(theta)).
    pub fn sincos(&self) -> (f32, f32) {
        let theta = 2.0 * PI * self.phase;
        (theta.sin() as f32, theta.cos() as f32)
    }

    /// Step the NCO phase forward by one sample.
    ///
    /// This advances the phase by the current frequency.
    pub fn step(&mut self) {
        self.phase += self.frequency;
        // Wrap phase to [0, 1)
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        } else if self.phase < 0.0 {
            self.phase += 1.0;
        }
    }

    /// Mix an input sample down by the NCO frequency.
    ///
    /// Multiplies the input by exp(-j * theta), effectively shifting the
    /// input spectrum down by the NCO frequency.
    ///
    /// For a real input x, the output is:
    /// - I = x * cos(theta)
    /// - Q = -x * sin(theta)
    ///
    /// # Arguments
    ///
    /// * `input` - Real input sample
    ///
    /// # Returns
    ///
    /// (I, Q) tuple representing the complex baseband signal.
    pub fn mix_down(&self, input: f32) -> (f32, f32) {
        let theta = 2.0 * PI * self.phase;
        let cos_theta = theta.cos() as f32;
        let sin_theta = theta.sin() as f32;
        // exp(-j*theta) = cos(theta) - j*sin(theta)
        // x * exp(-j*theta) = x*cos(theta) - j*x*sin(theta)
        (input * cos_theta, -input * sin_theta)
    }

    /// Mix a complex input sample down by the NCO frequency.
    ///
    /// Multiplies the complex input (i, q) by exp(-j * theta).
    ///
    /// # Arguments
    ///
    /// * `i` - In-phase (real) component
    /// * `q` - Quadrature (imaginary) component
    ///
    /// # Returns
    ///
    /// (I, Q) tuple after frequency shift.
    pub fn mix_down_complex(&self, i: f32, q: f32) -> (f32, f32) {
        let theta = 2.0 * PI * self.phase;
        let cos_theta = theta.cos() as f32;
        let sin_theta = theta.sin() as f32;
        // (i + jq) * (cos - j*sin) = i*cos + q*sin + j(q*cos - i*sin)
        (i * cos_theta + q * sin_theta, q * cos_theta - i * sin_theta)
    }

    /// Mix an input sample up by the NCO frequency.
    ///
    /// Multiplies the input by exp(+j * theta), effectively shifting the
    /// input spectrum up by the NCO frequency.
    ///
    /// # Arguments
    ///
    /// * `input` - Real input sample
    ///
    /// # Returns
    ///
    /// (I, Q) tuple.
    pub fn mix_up(&self, input: f32) -> (f32, f32) {
        let theta = 2.0 * PI * self.phase;
        let cos_theta = theta.cos() as f32;
        let sin_theta = theta.sin() as f32;
        // x * exp(j*theta) = x*cos(theta) + j*x*sin(theta)
        (input * cos_theta, input * sin_theta)
    }

    /// Update the PLL with a phase error.
    ///
    /// This is the core PLL loop filter update. Given a phase error (in radians),
    /// the NCO frequency and phase are adjusted to track the input signal.
    ///
    /// The loop filter is second-order:
    /// - Frequency is adjusted by `error * alpha` (integral path)
    /// - Phase is adjusted by `error * beta` (proportional path)
    ///
    /// # Arguments
    ///
    /// * `phase_error` - Phase error in radians (from phase detector)
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::nco::Nco;
    ///
    /// let mut nco = Nco::new(57000.0, 171000.0);
    /// nco.set_pll_bandwidth(0.03, 171000.0);
    ///
    /// // Typical PLL loop: compute phase error, update PLL, step NCO
    /// let received_phase = 0.1; // From received signal
    /// let local_phase = nco.get_phase_radians();
    /// let phase_error = received_phase - local_phase;
    ///
    /// nco.pll_step(phase_error);
    /// nco.step();
    /// ```
    pub fn pll_step(&mut self, phase_error: f64) {
        // Convert phase error from radians to cycles
        let error_cycles = phase_error / (2.0 * PI);

        // Second-order loop filter (matches liquid-dsp nco_pll_step):
        // 1. Adjust frequency by alpha * error (integral)
        self.adjust_frequency(error_cycles * self.alpha);

        // 2. Adjust phase by beta * error (proportional)
        self.adjust_phase(error_cycles * self.beta);
    }

    /// Update the PLL with a phase error (error already in cycles, not radians).
    ///
    /// This variant is useful when you've already computed the error in cycles.
    pub fn pll_step_cycles(&mut self, phase_error_cycles: f64) {
        // Second-order loop filter:
        // 1. Adjust frequency by alpha * error (integral)
        self.adjust_frequency(phase_error_cycles * self.alpha);

        // 2. Adjust phase by beta * error (proportional)
        self.adjust_phase(phase_error_cycles * self.beta);
    }

    /// Reset the NCO to initial state.
    ///
    /// Sets phase to 0 but retains the current frequency.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Reset the NCO completely.
    ///
    /// Sets both phase and frequency to 0.
    pub fn reset_full(&mut self) {
        self.phase = 0.0;
        self.frequency = 0.0;
    }
}

impl Default for Nco {
    fn default() -> Self {
        Self {
            phase: 0.0,
            frequency: 0.0,
            alpha: 0.1,
            beta: 0.1_f64.sqrt(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_nco_creation() {
        let nco = Nco::new(57000.0, 171000.0);
        assert!((nco.get_frequency() - 1.0 / 3.0).abs() < 1e-10);
        assert_eq!(nco.get_phase(), 0.0);
    }

    #[test]
    fn test_nco_step() {
        let mut nco = Nco::new(57000.0, 171000.0);
        nco.step();
        // After one step, phase should be 1/3 cycle
        assert!((nco.get_phase() - 1.0 / 3.0).abs() < 1e-10);

        nco.step();
        // After two steps, phase should be 2/3 cycle
        assert!((nco.get_phase() - 2.0 / 3.0).abs() < 1e-10);

        nco.step();
        // After three steps, phase should wrap to 0
        assert!(nco.get_phase() < 1e-10);
    }

    #[test]
    fn test_nco_get_complex() {
        let nco = Nco::new(57000.0, 171000.0);
        let (cos_val, sin_val) = nco.get_complex();
        // At phase 0: cos=1, sin=0
        assert!((cos_val - 1.0).abs() < 1e-6);
        assert!(sin_val.abs() < 1e-6);
    }

    #[test]
    fn test_nco_mix_down() {
        let nco = Nco::new(57000.0, 171000.0);
        let (i, q) = nco.mix_down(1.0);
        // At phase 0: mix_down returns (1*cos(0), -1*sin(0)) = (1, 0)
        assert!((i - 1.0).abs() < 1e-6);
        assert!(q.abs() < 1e-6);
    }

    #[test]
    fn test_pll_bandwidth() {
        let mut nco = Nco::new(57000.0, 171000.0);
        nco.set_pll_bandwidth(0.03, 171000.0);

        // Check that alpha is properly normalized
        let expected_alpha = 0.03 / 171000.0;
        assert!((nco.alpha - expected_alpha).abs() < 1e-15);
        assert!((nco.beta - expected_alpha.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_pll_tracking() {
        // Simulate PLL tracking a slightly offset frequency
        let sample_rate = 171000.0;
        let mut nco_tx = Nco::new(57010.0, sample_rate); // TX at 57010 Hz
        let mut nco_rx = Nco::new(57000.0, sample_rate); // RX starts at 57000 Hz
        nco_rx.set_pll_bandwidth(10.0, sample_rate); // 10 Hz bandwidth for test

        // Run PLL for some samples
        for _ in 0..10000 {
            // Get phases
            let tx_phase = nco_tx.get_phase_radians();
            let rx_phase = nco_rx.get_phase_radians();

            // Compute phase error
            let mut phase_error = tx_phase - rx_phase;
            // Wrap to [-PI, PI]
            while phase_error > PI {
                phase_error -= 2.0 * PI;
            }
            while phase_error < -PI {
                phase_error += 2.0 * PI;
            }

            // Update PLL
            nco_rx.pll_step(phase_error);

            // Step both NCOs
            nco_tx.step();
            nco_rx.step();
        }

        // After PLL tracking, frequencies should be close
        let freq_error =
            (nco_tx.get_frequency_hz(sample_rate) - nco_rx.get_frequency_hz(sample_rate)).abs();
        assert!(
            freq_error < 1.0,
            "Frequency error {} Hz too large",
            freq_error
        );
    }

    #[test]
    fn test_phase_wrapping() {
        let mut nco = Nco::new(100000.0, 100000.0); // 1 cycle per sample

        // Step 10 times - phase should stay in [0, 1)
        for _ in 0..10 {
            nco.step();
            assert!(nco.get_phase() >= 0.0);
            assert!(nco.get_phase() < 1.0);
        }
    }

    #[test]
    fn test_negative_frequency() {
        let mut nco = Nco::new(-10000.0, 100000.0); // -0.1 cycles per sample

        nco.step();
        // Phase should wrap to positive
        assert!(nco.get_phase() >= 0.0);
        assert!(nco.get_phase() < 1.0);
        // Should be 0.9 (wrapped from -0.1)
        assert!((nco.get_phase() - 0.9).abs() < 1e-10);
    }
}
