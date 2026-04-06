//! Second-Order PLL Loop Filter
//!
//! This module provides a reusable loop filter for Phase-Locked Loops (PLLs)
//! and timing synchronizers. It implements a standard second-order loop filter
//! with proportional (P) and integral (I) paths.
//!
//! # Overview
//!
//! A second-order loop filter processes an error signal (phase or timing error)
//! and produces corrections to two state variables:
//! - **Frequency/Rate adjustment** (integral path): accumulates error to track
//!   slow frequency/timing drifts
//! - **Phase adjustment** (proportional path): immediately responds to error
//!   for fast phase/timing adjustments
//!
//! The design is based on liquid-dsp's loop filter architecture and is used in:
//! - NCO phase-locked loops for carrier recovery (nco.rs)
//! - Symbol synchronizers for timing recovery (symsync.rs)
//!
//! # Example
//!
//! ```
//! use desperado::dsp::pll_filter::PllFilter;
//!
//! // Create PLL filter with bandwidth 0.03 Hz at 171 kHz sample rate
//! let mut pll = PllFilter::from_bandwidth(0.03, 171000.0);
//!
//! // Process phase error
//! let phase_error = 0.1; // radians
//! let (freq_adj, phase_adj) = pll.update(phase_error);
//!
//! println!("Frequency adjustment: {}", freq_adj);
//! println!("Phase adjustment: {}", phase_adj);
//! ```

/// Second-order PLL loop filter.
///
/// Implements a standard second-order loop filter with proportional and
/// integral paths. Given an error signal, computes adjustments to frequency
/// and phase based on the loop gains (alpha and beta).
#[derive(Debug, Clone, Copy)]
pub struct PllFilter {
    /// Frequency/rate gain (integral path)
    /// Also called "alpha" or bandwidth-related gain
    alpha: f64,

    /// Phase gain (proportional path)
    /// Also called "beta" - typically sqrt(alpha) for critical damping
    beta: f64,
}

impl PllFilter {
    /// Create a PLL filter from bandwidth and sample rate.
    ///
    /// This is the most common way to configure a PLL. The bandwidth determines
    /// the loop's speed of response. Lower bandwidth provides better noise rejection
    /// but slower acquisition; higher bandwidth tracks faster but passes more noise.
    ///
    /// # Arguments
    ///
    /// * `bandwidth_hz` - Loop bandwidth in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    ///
    /// PllFilter configured for the given bandwidth (uses critically damped design)
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::pll_filter::PllFilter;
    ///
    /// // RDS PLL: narrow 0.03 Hz bandwidth at 171 kHz sample rate
    /// let pll = PllFilter::from_bandwidth(0.03, 171000.0);
    /// ```
    pub fn from_bandwidth(bandwidth_hz: f64, sample_rate: f64) -> Self {
        let alpha = bandwidth_hz / sample_rate;
        let beta = alpha.sqrt(); // Critical damping

        Self { alpha, beta }
    }

    /// Create a PLL filter from direct gains.
    ///
    /// For advanced use cases, you can set alpha and beta directly.
    ///
    /// # Arguments
    ///
    /// * `alpha` - Frequency (integral) gain
    /// * `beta` - Phase (proportional) gain
    ///
    /// # Returns
    ///
    /// PllFilter with the specified gains
    pub fn from_gains(alpha: f64, beta: f64) -> Self {
        Self { alpha, beta }
    }

    /// Get the alpha (frequency) gain.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Get the beta (phase) gain.
    pub fn beta(&self) -> f64 {
        self.beta
    }

    /// Update the loop filter with an error signal.
    ///
    /// Computes the frequency and phase adjustments based on the current error.
    /// This is typically called once per sample.
    ///
    /// # Arguments
    ///
    /// * `error` - Error signal (phase error in radians, or timing error in cycles)
    ///
    /// # Returns
    ///
    /// Tuple of (frequency_adjustment, phase_adjustment) in compatible units.
    /// For phase errors in radians: divide by 2π to get cycle adjustments.
    /// For timing errors in cycles: use directly.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::pll_filter::PllFilter;
    /// use std::f64::consts::PI;
    ///
    /// let pll = PllFilter::from_bandwidth(10.0, 100000.0);
    ///
    /// // Phase error in radians
    /// let phase_error_rad = 0.1;
    /// let (freq_adj_rad, phase_adj_rad) = pll.update(phase_error_rad);
    ///
    /// // Convert radians to cycles
    /// let freq_adj_cycles = freq_adj_rad / (2.0 * PI);
    /// let phase_adj_cycles = phase_adj_rad / (2.0 * PI);
    /// ```
    pub fn update(&self, error: f64) -> (f64, f64) {
        let freq_adj = error * self.alpha;
        let phase_adj = error * self.beta;

        (freq_adj, phase_adj)
    }

    /// Update with error already in cycles (not radians).
    ///
    /// This is a convenience method for systems that work directly with cycle units.
    ///
    /// # Arguments
    ///
    /// * `error_cycles` - Error in cycles
    ///
    /// # Returns
    ///
    /// Tuple of (frequency_adjustment_cycles, phase_adjustment_cycles)
    pub fn update_cycles(&self, error_cycles: f64) -> (f64, f64) {
        self.update(error_cycles) // Same calculation, just different units
    }

    /// Set new gains (for adaptive control).
    ///
    /// Allows changing the loop filter gains after creation for adaptive systems.
    pub fn set_gains(&mut self, alpha: f64, beta: f64) {
        self.alpha = alpha;
        self.beta = beta;
    }

    /// Set bandwidth (for adaptive control).
    ///
    /// Reconfigures the loop for a new bandwidth with critically damped response.
    pub fn set_bandwidth(&mut self, bandwidth_hz: f64, sample_rate: f64) {
        self.alpha = bandwidth_hz / sample_rate;
        self.beta = self.alpha.sqrt();
    }
}

impl Default for PllFilter {
    fn default() -> Self {
        // Default: 0.1 Hz bandwidth at 100 kHz sample rate
        Self::from_bandwidth(0.1, 100000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pll_filter_creation() {
        let pll = PllFilter::from_bandwidth(0.03, 171000.0);

        let expected_alpha = 0.03 / 171000.0;
        assert!((pll.alpha() - expected_alpha).abs() < 1e-15);
        assert!((pll.beta() - expected_alpha.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_pll_filter_gains() {
        let pll = PllFilter::from_gains(0.001, 0.05);

        assert!((pll.alpha() - 0.001).abs() < 1e-10);
        assert!((pll.beta() - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_pll_filter_update() {
        let pll = PllFilter::from_gains(0.1, 0.2);

        let error = 1.0;
        let (freq_adj, phase_adj) = pll.update(error);

        assert!((freq_adj - 0.1).abs() < 1e-10);
        assert!((phase_adj - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_pll_filter_update_cycles() {
        let pll = PllFilter::from_gains(0.1, 0.2);

        let error_cycles = 1.0;
        let (freq_adj_cycles, phase_adj_cycles) = pll.update_cycles(error_cycles);

        assert!((freq_adj_cycles - 0.1).abs() < 1e-10);
        assert!((phase_adj_cycles - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_pll_filter_set_bandwidth() {
        let mut pll = PllFilter::default();

        pll.set_bandwidth(0.5, 100000.0);
        let expected_alpha = 0.5 / 100000.0;

        assert!((pll.alpha() - expected_alpha).abs() < 1e-15);
        assert!((pll.beta() - expected_alpha.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_pll_filter_set_gains() {
        let mut pll = PllFilter::default();

        pll.set_gains(0.02, 0.03);

        assert!((pll.alpha() - 0.02).abs() < 1e-10);
        assert!((pll.beta() - 0.03).abs() < 1e-10);
    }

    #[test]
    fn test_pll_filter_default() {
        let pll = PllFilter::default();

        // Default: 0.1 Hz at 100 kHz
        let expected_alpha = 0.1 / 100000.0;
        assert!((pll.alpha() - expected_alpha).abs() < 1e-15);
    }

    #[test]
    fn test_pll_filter_zero_error() {
        let pll = PllFilter::from_bandwidth(10.0, 100000.0);

        let (freq_adj, phase_adj) = pll.update(0.0);

        assert_eq!(freq_adj, 0.0);
        assert_eq!(phase_adj, 0.0);
    }
}
