//! Polyphase Symbol Synchronizer
//!
//! This module provides a liquid-dsp compatible symbol synchronizer for
//! timing recovery in digital communication systems.
//!
//! # Overview
//!
//! The symbol synchronizer recovers the optimal sampling instants for a
//! digitally modulated signal. It uses a polyphase filter bank approach
//! combined with a timing error detector and loop filter.
//!
//! # Design
//!
//! The implementation follows liquid-dsp's `symsync_crcf` design:
//! - Polyphase matched filter bank with `npfb` phases
//! - Derivative matched filter bank for timing error detection
//! - Matched filter TED: `error = Re(conj(mf) * dmf)`
//! - Second-order loop filter for smooth timing tracking
//!
//! # Example
//!
//! ```
//! use desperado::dsp::symsync::SymSync;
//!
//! // Create synchronizer for RDS: 3 samples/symbol, 32 filter phases
//! let mut symsync = SymSync::new_rnyquist(
//!     3,      // samples per symbol
//!     32,     // number of filter phases
//!     3,      // filter semi-length (symbols)
//!     0.8,    // rolloff factor (beta)
//!     0.01,   // loop bandwidth
//! );
//!
//! // Process samples
//! let input_samples = vec![(0.5, 0.3), (0.6, 0.2), (0.7, 0.4)];
//! let mut output = Vec::new();
//! for sample in input_samples {
//!     if let Some((i, q)) = symsync.push(sample.0, sample.1) {
//!         output.push((i, q));
//!     }
//! }
//! ```
//!
//! # Reference
//!
//! This implementation is based on liquid-dsp's `symsync_crcf` object.
//! See: <https://github.com/jgaeddert/liquid-dsp>
//!
//! References:
//! - \[Mengali:1997\] Umberto Mengali and Aldo N. D'Andrea,
//!   "Synchronization Techniques for Digital Receivers"
//! - \[harris:2001\] frederic j. harris and Michael Rice,
//!   "Multirate Digital Filters for Symbol Timing Synchronization
//!   in Software Defined Radios"

use std::f32::consts::PI;

/// Root-raised-cosine filter design.
///
/// Generates coefficients for an RRC filter used in symbol synchronization.
/// This matches liquid-dsp's liquid_firdes_rrcos() implementation.
fn design_rrc_filter(k: usize, m: usize, beta: f32) -> Vec<f32> {
    // Filter length: 2*k*m + 1
    let h_len = 2 * k * m + 1;
    let mut h = vec![0.0f32; h_len];

    let kf = k as f32;
    let mf = m as f32;

    for (n, coef) in h.iter_mut().enumerate() {
        let nf = n as f32;
        // z is the normalized time in symbol periods, centered at m
        let z = nf / kf - mf;

        // Check for special condition where z equals zero
        if z.abs() < 1e-5 {
            *coef = 1.0 - beta + 4.0 * beta / PI;
        } else {
            let g = 1.0 - 16.0 * beta * beta * z * z;
            let g_squared = g * g;

            // Check for special condition where 16*beta^2*z^2 equals 1
            if g_squared < 1e-5 {
                let g1 = 1.0 + 2.0 / PI;
                let g2 = (0.25 * PI / beta).sin();
                let g3 = 1.0 - 2.0 / PI;
                let g4 = (0.25 * PI / beta).cos();
                *coef = beta / 2.0_f32.sqrt() * (g1 * g2 + g3 * g4);
            } else {
                let t1 = ((1.0 + beta) * PI * z).cos();
                let t2 = ((1.0 - beta) * PI * z).sin();
                let t3 = 1.0 / (4.0 * beta * z);
                let t4 = 4.0 * beta / (PI * (1.0 - 16.0 * beta * beta * z * z));
                *coef = t4 * (t1 + t2 * t3);
            }
        }
    }

    // Note: liquid-dsp does NOT normalize the filter coefficients.
    // The filter has natural gain that gives proper signal amplitude.

    h
}

/// Compute derivative of filter coefficients.
///
/// Uses central differences for interior points.
fn compute_derivative_filter(h: &[f32]) -> Vec<f32> {
    let h_len = h.len();
    let mut dh = vec![0.0f32; h_len];

    for i in 0..h_len {
        if i == 0 {
            dh[i] = h[i + 1] - h[h_len - 1];
        } else if i == h_len - 1 {
            dh[i] = h[0] - h[i - 1];
        } else {
            dh[i] = h[i + 1] - h[i - 1];
        }
    }

    // Find max of h*dh for normalization (matching liquid-dsp)
    let hdh_max = h
        .iter()
        .zip(dh.iter())
        .map(|(a, b)| (a * b).abs())
        .fold(0.0f32, f32::max);

    if hdh_max > 1e-10 {
        let scale = 0.06 / hdh_max;
        for coef in &mut dh {
            *coef *= scale;
        }
    }

    dh
}

/// Polyphase filter bank.
///
/// Stores filter coefficients decomposed into `npfb` sub-filters.
#[derive(Debug, Clone)]
struct PolyphaseFilterBank {
    /// Number of filter phases
    npfb: usize,
    /// Sub-filter length
    h_sub_len: usize,
    /// Filter coefficients [npfb][h_sub_len]
    filters: Vec<Vec<f32>>,
    /// Input buffer (ring buffer)
    buffer_i: Vec<f32>,
    buffer_q: Vec<f32>,
    /// Write index into buffer
    buf_idx: usize,
}

impl PolyphaseFilterBank {
    /// Create a new polyphase filter bank from prototype filter.
    fn new(h: &[f32], npfb: usize) -> Self {
        let h_len = h.len();
        // Sub-filter length
        let h_sub_len = h_len.div_ceil(npfb);

        // Decompose into polyphase components
        let mut filters = vec![vec![0.0f32; h_sub_len]; npfb];

        for (i, &coef) in h.iter().enumerate() {
            let phase = i % npfb;
            let idx = i / npfb;
            if idx < h_sub_len {
                filters[phase][idx] = coef;
            }
        }

        let buffer_len = h_sub_len;

        Self {
            npfb,
            h_sub_len,
            filters,
            buffer_i: vec![0.0; buffer_len],
            buffer_q: vec![0.0; buffer_len],
            buf_idx: 0,
        }
    }

    /// Push a new sample into the filter bank.
    fn push(&mut self, i: f32, q: f32) {
        self.buffer_i[self.buf_idx] = i;
        self.buffer_q[self.buf_idx] = q;
        self.buf_idx = (self.buf_idx + 1) % self.h_sub_len;
    }

    /// Execute filter at given phase index.
    fn execute(&self, phase: usize) -> (f32, f32) {
        let filter = &self.filters[phase.min(self.npfb - 1)];
        let mut sum_i = 0.0f32;
        let mut sum_q = 0.0f32;

        for (j, &coef) in filter.iter().enumerate() {
            // Read from buffer in reverse order (convolution)
            let idx = (self.buf_idx + self.h_sub_len - 1 - j) % self.h_sub_len;
            sum_i += self.buffer_i[idx] * coef;
            sum_q += self.buffer_q[idx] * coef;
        }

        (sum_i, sum_q)
    }

    /// Reset filter state.
    fn reset(&mut self) {
        self.buffer_i.fill(0.0);
        self.buffer_q.fill(0.0);
        self.buf_idx = 0;
    }
}

/// Symbol Synchronizer with polyphase filter bank.
///
/// Performs timing recovery for digital communication signals using a
/// polyphase matched filter approach with timing error detection and
/// loop filtering.
#[derive(Debug, Clone)]
pub struct SymSync {
    /// Samples per symbol (input)
    k: usize,

    /// Number of polyphase filter phases
    npfb: usize,

    /// Matched filter bank
    mf: PolyphaseFilterBank,

    /// Derivative matched filter bank (for timing error detection)
    dmf: PolyphaseFilterBank,

    /// Timing phase [0, 1) representing fractional symbol timing
    tau: f32,

    /// Soft filterbank index
    bf: f32,

    /// Hard filterbank index
    b: i32,

    /// Resampling rate (nominal = k)
    rate: f32,

    /// Fractional delay step
    del: f32,

    /// Instantaneous timing error
    q: f32,

    /// Filtered timing error
    q_hat: f32,

    /// Loop filter bandwidth
    bandwidth: f32,

    /// Rate adjustment factor
    rate_adjustment: f32,

    /// Loop filter coefficients (simplified second-order)
    alpha: f32,

    /// Synchronizer locked flag
    is_locked: bool,
}

impl SymSync {
    /// Create a symbol synchronizer from external filter coefficients.
    ///
    /// # Arguments
    ///
    /// * `k` - Samples per symbol
    /// * `npfb` - Number of polyphase filter phases (typically 32)
    /// * `h` - Matched filter coefficients
    /// * `bandwidth` - Loop filter bandwidth [0, 1]
    pub fn new(k: usize, npfb: usize, h: &[f32], bandwidth: f32) -> Self {
        // Compute derivative filter
        let dh = compute_derivative_filter(h);

        // Create polyphase filter banks
        let mf = PolyphaseFilterBank::new(h, npfb);
        let dmf = PolyphaseFilterBank::new(&dh, npfb);

        let rate = k as f32;

        // Compute loop filter coefficients from bandwidth
        let alpha = 1.0 - bandwidth;
        let rate_adjustment = 0.5 * bandwidth;

        Self {
            k,
            npfb,
            mf,
            dmf,
            tau: 0.0,
            bf: 0.0,
            b: 0,
            rate,
            del: rate,
            q: 0.0,
            q_hat: 0.0,
            bandwidth,
            rate_adjustment,
            alpha,
            is_locked: false,
        }
    }

    /// Create a symbol synchronizer with root-raised-cosine matched filter.
    ///
    /// This is the most common configuration for pulse-shaped signals.
    ///
    /// # Arguments
    ///
    /// * `k` - Samples per symbol (e.g., 3 for RDS at 171 kHz / 57 kHz)
    /// * `npfb` - Number of polyphase filter phases (typically 32)
    /// * `m` - Filter semi-length in symbols (typically 3)
    /// * `beta` - Rolloff factor [0, 1] (e.g., 0.8 for RDS)
    /// * `bandwidth` - Loop filter bandwidth [0, 1] (e.g., 0.01)
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::symsync::SymSync;
    ///
    /// // RDS symbol synchronizer
    /// let symsync = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.01);
    /// ```
    pub fn new_rnyquist(k: usize, npfb: usize, m: usize, beta: f32, bandwidth: f32) -> Self {
        // Design RRC filter at upsampled rate
        let h = design_rrc_filter(k * npfb, m, beta);
        Self::new(k, npfb, &h, bandwidth)
    }

    /// Set the loop filter bandwidth.
    ///
    /// # Arguments
    ///
    /// * `bandwidth` - Loop bandwidth in [0, 1]. Lower values provide
    ///   smoother timing but slower adaptation.
    pub fn set_bandwidth(&mut self, bandwidth: f32) {
        self.bandwidth = bandwidth.clamp(0.0, 1.0);
        self.alpha = 1.0 - self.bandwidth;
        self.rate_adjustment = 0.5 * self.bandwidth;
    }

    /// Get the current timing offset.
    ///
    /// Returns a value in [0, 1) representing the fractional symbol timing.
    pub fn get_tau(&self) -> f32 {
        self.tau
    }

    /// Get the current resampling rate.
    pub fn get_rate(&self) -> f32 {
        self.rate
    }

    /// Lock the synchronizer.
    ///
    /// When locked, the timing offset is not updated (useful after acquisition).
    pub fn lock(&mut self) {
        self.is_locked = true;
    }

    /// Unlock the synchronizer.
    ///
    /// Allows timing tracking to resume.
    pub fn unlock(&mut self) {
        self.is_locked = false;
    }

    /// Check if the synchronizer is locked.
    pub fn is_locked(&self) -> bool {
        self.is_locked
    }

    /// Reset the synchronizer state.
    pub fn reset(&mut self) {
        self.mf.reset();
        self.dmf.reset();
        self.tau = 0.0;
        self.bf = 0.0;
        self.b = 0;
        self.rate = self.k as f32;
        self.del = self.rate;
        self.q = 0.0;
        self.q_hat = 0.0;
    }

    /// Push a sample and potentially get an output symbol.
    ///
    /// This is the main processing function. Call it once per input sample.
    /// When the timing is right, it returns the synchronized symbol.
    ///
    /// # Arguments
    ///
    /// * `i` - In-phase (real) component of input sample
    /// * `q` - Quadrature (imaginary) component of input sample
    ///
    /// # Returns
    ///
    /// `Some((i, q))` when a symbol is ready, `None` otherwise.
    ///
    /// # Example
    ///
    /// ```
    /// use desperado::dsp::symsync::SymSync;
    ///
    /// let mut symsync = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.01);
    /// let mut symbols = Vec::new();
    ///
    /// // Process input samples
    /// let input_samples = vec![(0.5, 0.3), (0.6, 0.2), (0.7, 0.4)];
    /// for (i, q) in input_samples {
    ///     if let Some(sym) = symsync.push(i, q) {
    ///         symbols.push(sym);
    ///     }
    /// }
    /// ```
    pub fn push(&mut self, i: f32, q: f32) -> Option<(f32, f32)> {
        // Push sample into both filter banks
        self.mf.push(i, q);
        self.dmf.push(i, q);

        // Check if we should output a symbol (b < npfb means we're at a symbol point)
        if self.b < self.npfb as i32 {
            // Compute matched filter output
            let (mf_i, mf_q) = self.mf.execute(self.b as usize);

            // Scale by samples/symbol
            let out_i = mf_i / self.k as f32;
            let out_q = mf_q / self.k as f32;

            // Update timing if not locked
            if !self.is_locked {
                // Compute derivative matched filter output
                let (dmf_i, dmf_q) = self.dmf.execute(self.b as usize);

                // Timing error detector: Re(conj(mf) * dmf)
                // = mf_i * dmf_i + mf_q * dmf_q
                self.q = mf_i * dmf_i + mf_q * dmf_q;

                // Clip timing error
                self.q = self.q.clamp(-1.0, 1.0);

                // Simple loop filter (first-order approximation)
                // Full liquid-dsp uses IIR second-order SOS filter
                self.q_hat = self.alpha * self.q_hat + (1.0 - self.alpha) * self.q;

                // Update rate and timing
                self.rate += self.rate_adjustment * self.q_hat;
                self.del = self.rate + self.q_hat;
            }

            // Update timing state: advance by del (nominally k samples/symbol)
            self.tau += self.del;
            self.bf = self.tau * self.npfb as f32;
            self.b = self.bf.round() as i32;

            // Fall through to do the rollover below (like liquid-dsp)
            // Don't return early - the rollover happens unconditionally
            // after outputting a symbol, not on subsequent calls.

            // Do the rollover (same as liquid-dsp after while loop exits)
            self.tau -= 1.0;
            self.bf -= self.npfb as f32;
            self.b -= self.npfb as i32;

            return Some((out_i, out_q));
        }

        // No symbol output this sample - do exactly ONE rollover
        // This corresponds to liquid-dsp's rollover after the while loop
        // when no symbols were output (because b >= npfb on entry)
        self.tau -= 1.0;
        self.bf -= self.npfb as f32;
        self.b -= self.npfb as i32;

        None
    }

    /// Process a batch of samples.
    ///
    /// # Arguments
    ///
    /// * `input` - Input samples as (I, Q) tuples
    ///
    /// # Returns
    ///
    /// Vector of synchronized symbols.
    pub fn execute(&mut self, input: &[(f32, f32)]) -> Vec<(f32, f32)> {
        let mut output = Vec::with_capacity(input.len() / self.k + 1);

        for &(i, q) in input {
            if let Some(sym) = self.push(i, q) {
                output.push(sym);
            }
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rrc_filter_design() {
        let h = design_rrc_filter(4, 3, 0.35);
        // Filter should have 2*4*3 + 1 = 25 coefficients
        assert_eq!(h.len(), 25);
        // Filter should be roughly symmetric
        let mid = h.len() / 2;
        for i in 0..mid {
            assert!(
                (h[i] - h[h.len() - 1 - i]).abs() < 0.01,
                "Filter not symmetric at index {}",
                i
            );
        }
    }

    #[test]
    fn test_derivative_filter() {
        let h = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let dh = compute_derivative_filter(&h);
        assert_eq!(dh.len(), h.len());
        // Derivative of symmetric filter should be antisymmetric
    }

    #[test]
    fn test_polyphase_filterbank_creation() {
        let h = design_rrc_filter(3 * 32, 3, 0.8);
        let pfb = PolyphaseFilterBank::new(&h, 32);
        assert_eq!(pfb.npfb, 32);
        assert_eq!(pfb.filters.len(), 32);
    }

    #[test]
    fn test_symsync_creation() {
        let ss = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.01);
        assert_eq!(ss.k, 3);
        assert_eq!(ss.npfb, 32);
        assert!(!ss.is_locked());
    }

    #[test]
    fn test_symsync_basic_operation() {
        let mut ss = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.01);

        // Push samples and check that we get outputs
        let mut output_count = 0;
        for i in 0..100 {
            let phase = 2.0 * PI * (i as f32) / 3.0;
            let sample_i = phase.cos();
            let sample_q = phase.sin();

            if ss.push(sample_i, sample_q).is_some() {
                output_count += 1;
            }
        }

        // With 3 samples/symbol, we should get roughly 100/3 = 33 symbols
        assert!(
            output_count > 20,
            "Expected ~33 outputs, got {}",
            output_count
        );
        assert!(
            output_count < 50,
            "Expected ~33 outputs, got {}",
            output_count
        );
    }

    #[test]
    fn test_symsync_lock() {
        let mut ss = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.01);

        ss.lock();
        assert!(ss.is_locked());

        ss.unlock();
        assert!(!ss.is_locked());
    }

    #[test]
    fn test_symsync_reset() {
        let mut ss = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.01);

        // Push some samples
        for _ in 0..10 {
            ss.push(1.0, 0.0);
        }

        // Reset
        ss.reset();

        // State should be back to initial
        assert_eq!(ss.tau, 0.0);
        assert_eq!(ss.b, 0);
    }

    #[test]
    fn test_symsync_bandwidth() {
        let mut ss = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.01);

        ss.set_bandwidth(0.05);
        assert!((ss.bandwidth - 0.05).abs() < 1e-6);

        // Bandwidth should be clamped
        ss.set_bandwidth(2.0);
        assert!((ss.bandwidth - 1.0).abs() < 1e-6);

        ss.set_bandwidth(-0.5);
        assert!((ss.bandwidth - 0.0).abs() < 1e-6);
    }
}
