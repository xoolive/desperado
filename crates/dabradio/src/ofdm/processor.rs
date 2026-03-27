//! OFDM processor: frame synchronisation, FFT, symbol extraction.
//! Reads a continuous IQ stream and produces OFDM frames (arrays of frequency-domain symbols).
//!
//! Follows the welle.io architecture:
//! 1. Initial sync: null symbol detection (power drop) → PRS correlation
//! 2. Subsequent frames: read null + PRS correlation (no re-detection)
//! 3. Fine frequency estimation from cyclic prefix correlation (all 75 symbols)
//! 4. Coarse frequency estimation from PRS carrier-offset search
//! 5. Per-sample frequency correction with continuous phase across frames
//! 6. Symbol extraction (FFT of each useful part)

use crate::constants::*;
use num_complex::Complex;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::PI;
use std::sync::Arc;

use tracing::{debug, trace};

/// One DAB transmission frame worth of frequency-domain symbols.
/// `symbols[0]` = Phase Reference Symbol (PRS), `symbols[1..=75]` = data symbols.
pub struct OfdmFrame {
    pub symbols: Vec<Vec<Complex<f32>>>,
    #[allow(dead_code)] // semantically useful, not yet consumed
    pub coarse_freq_offset: i32, // in carriers
}

/// OFDM processor state machine.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SyncState {
    /// Need initial null detection (first frame or after sync loss).
    NeedNullDetect,
    /// Have found null boundary; need PRS correlation to find exact timing.
    NeedPrsCorrelation,
    /// Synced — ready to process frame data.
    Synced,
    /// Waiting to skip null symbol (have processed frame but need more samples).
    NeedNullSkip,
}

/// OFDM processor: consumes IQ samples and produces OFDM frames.
pub struct OfdmProcessor {
    fft: Arc<dyn Fft<f32>>,
    ifft: Arc<dyn Fft<f32>>,
    /// PRS reference in frequency domain, conjugated for correlation.
    prs_ref_conj: Vec<Complex<f32>>,
    buffer: Vec<Complex<f32>>,
    state: SyncState,
    frame_count: usize,
    /// Accumulated fine frequency correction in Hz (fractional carrier).
    fine_freq_hz: f32,
    /// Coarse frequency correction in whole carriers.
    coarse_freq_carriers: i32,
    /// SNR from the last PRS timing, used to gate coarse freq estimation.
    last_prs_snr: f32,
    /// Running phase for frequency correction — continuous across frames.
    /// This matches welle.io's `localPhase` which never resets.
    running_phase: f32,
    /// True if we just came from null detection (buffer at guard start).
    /// False if we came from successful frame processing (buffer at useful start).
    from_null_detect: bool,
    /// Count of consecutive PRS correlation failures (for re-acquisition).
    prs_fail_count: u32,
}

impl OfdmProcessor {
    pub fn new() -> Self {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(T_U);
        let ifft = planner.plan_fft_inverse(T_U);

        // Build conjugated PRS reference for correlation
        let prs_ref = phase_reference_table();
        let prs_ref_conj: Vec<Complex<f32>> = prs_ref.iter().map(|c| c.conj()).collect();

        Self {
            fft,
            ifft,
            prs_ref_conj,
            buffer: Vec::with_capacity(T_F * 2),
            state: SyncState::NeedNullDetect,
            frame_count: 0,
            fine_freq_hz: 0.0,
            coarse_freq_carriers: 0,
            last_prs_snr: 0.0,
            running_phase: 0.0,
            from_null_detect: true,
            prs_fail_count: 0,
        }
    }

    /// Apply frequency correction to a slice of samples in-place.
    /// Uses the running phase accumulator for continuity across calls.
    fn apply_freq_correction(&mut self, samples: &mut [Complex<f32>]) {
        let total_freq_hz =
            self.fine_freq_hz + self.coarse_freq_carriers as f32 * CARRIER_DIFF as f32;
        let phase_step = -2.0 * PI * total_freq_hz / SAMPLE_RATE as f32;
        for s in samples.iter_mut() {
            let (sin_p, cos_p) = self.running_phase.sin_cos();
            *s = Complex::new(s.re * cos_p - s.im * sin_p, s.re * sin_p + s.im * cos_p);
            self.running_phase += phase_step;
            // Wrap to [-PI, PI) for numerical stability
            if self.running_phase > PI {
                self.running_phase -= 2.0 * PI;
            } else if self.running_phase < -PI {
                self.running_phase += 2.0 * PI;
            }
        }
    }

    /// Estimate fine frequency from cyclic prefix correlation over data symbols.
    ///
    /// Like welle.io, correlates guard interval samples with their T_U-later
    /// copies across all data symbols. The phase of the correlation gives the
    /// fractional-carrier frequency error.
    ///
    /// `symbols_data` contains T_S samples per symbol for `num_symbols` symbols.
    fn estimate_fine_freq_from_symbols(symbols_data: &[Vec<Complex<f32>>]) -> f32 {
        let mut corr = Complex::new(0.0f32, 0.0);
        for sym in symbols_data {
            if sym.len() < T_S {
                continue;
            }
            // Guard interval: sym[0..T_G]
            // Matching useful part end: sym[T_U..T_S] = sym[T_U..T_U+T_G]
            for n in 0..T_G {
                let a = sym[T_U + n]; // end of useful part
                let b = sym[n]; // guard interval
                // welle.io: FreqCorr += buf[i] * conj(buf[i - T_U])
                //   where i is in [T_u..T_s), so buf[i] is end-of-useful,
                //   buf[i-T_u] is start-of-symbol (guard interval).
                corr += a * b.conj();
            }
        }

        // arg(corr) = 2π·Δf·T_U/fs (for a * conj(b) where b is T_U earlier)
        // Δf = arg(corr) / (2π) * fs / T_U = arg(corr) / (2π) * CARRIER_DIFF * T_U / T_U
        //    = arg(corr) / (2π) * CARRIER_DIFF
        // But welle.io uses: fineCorrector += 0.1 * arg(FreqCorr) / M_PI * (carrierDiff / 2)
        // which is: 0.1 * arg / PI * 500 = 0.1 * arg * 500 / PI
        // Our formula: arg(corr) * fs / (2*PI*T_U) = arg(corr) * 2048000 / (2*PI*2048)
        //            = arg(corr) * 1000 / (2*PI) = arg(corr) / (2*PI) * carrierDiff
        // These are the same! (arg/PI * 500 = arg / (2*PI) * 1000)
        let phase = corr.im.atan2(corr.re);
        phase * SAMPLE_RATE as f32 / (2.0 * PI * T_U as f32)
    }

    /// Estimate coarse (integer carrier) frequency offset from PRS.
    ///
    /// Searches carrier offsets in [-36, +36] and picks the offset that maximises
    /// `|Σ recv[k+d] · conj(ref[k])|²` over all 1536 active carriers.
    fn estimate_coarse_freq(&self, prs_spectrum: &[Complex<f32>]) -> i32 {
        if prs_spectrum.len() != T_U {
            return 0;
        }

        const SEARCH_RANGE: i32 = 36;

        let mut best_offset = 0i32;
        let mut best_corr = 0.0f32;

        for offset in -SEARCH_RANGE..=SEARCH_RANGE {
            let corr = self.correlate_prs_at_offset(prs_spectrum, offset);
            if corr > best_corr {
                best_corr = corr;
                best_offset = offset;
            }
        }

        best_offset
    }

    /// Feed IQ samples into the processor. Call repeatedly with chunks.
    /// Returns complete frames when enough data has been accumulated and synced.
    ///
    /// Frame processing follows welle.io's architecture:
    /// - Initial sync: null detect → PRS correlate → process frame
    /// - Subsequent frames: consume null samples → PRS correlate → process frame
    /// - Frequency correction phase is continuous across all operations
    pub fn process(&mut self, samples: &[Complex<f32>]) -> Vec<OfdmFrame> {
        self.buffer.extend_from_slice(samples);
        let mut frames = Vec::new();

        loop {
            match self.state {
                SyncState::NeedNullDetect => {
                    if !self.find_null_symbol() {
                        break;
                    }
                    // After find_null_symbol(), buffer[0] ≈ PRS guard start.
                    // Apply frequency correction to the consumed null samples
                    // is not possible (they're gone), but we do need to advance
                    // the phase accumulator. We don't know exactly how many null
                    // samples were consumed, so we just let the phase drift here.
                    // This is fine for initial sync — we'll re-estimate freq anyway.
                    self.from_null_detect = true;
                    self.state = SyncState::NeedPrsCorrelation;
                }

                SyncState::NeedPrsCorrelation => {
                    // After null detection: buffer[0] ≈ PRS guard start
                    // After frame+drain: buffer[0] ≈ PRS guard start (after NeedNullSkip)
                    trace!(
                        frame_count = self.frame_count,
                        buffer_len = self.buffer.len(),
                        from_null_detect = self.from_null_detect,
                        "Entering PRS correlation state"
                    );

                    // Use PRS correlation to find exact timing
                    let (prs_start, prs_snr) = self.find_prs_start();
                    if prs_start < 0 {
                        // Not enough data — wait for more
                        break;
                    }

                    let best_lag = prs_start - T_G as i32;

                    // For frame-to-frame tracking, if PRS SNR is low, the correlation is unreliable.
                    // In this case, trust the timing and just drain T_G samples to get to useful start.
                    // For initial sync (from_null_detect), we need good PRS correlation.
                    let drain_count = if self.from_null_detect {
                        // Initial sync - use PRS correlation result
                        if prs_snr < 5.0 || best_lag.abs() > 1500 {
                            debug!(
                                snr = format!("{:.2}", prs_snr),
                                best_lag,
                                frame_count = self.frame_count,
                                "PRS correlation failed during initial sync, re-acquiring"
                            );
                            self.state = SyncState::NeedNullDetect;
                            continue;
                        }
                        prs_start as usize
                    } else {
                        // Frame-to-frame tracking: trust timing, just skip guard interval.
                        // PRS correlation during tracking is unreliable because the
                        // frequency-corrected PRS doesn't match the ideal reference
                        // (the correction removes the carrier offset but introduces
                        // a starting-phase dependency). Instead, we trust that the
                        // null skip placed us correctly and just drain T_G to reach
                        // the PRS useful start. If timing drifts, the CP correlation
                        // in process_frame will detect it via fine_freq changes.
                        //
                        // Fall back to re-acquisition only if PRS SNR is extremely
                        // poor AND the lag suggests we're completely lost.
                        if prs_snr > 10.0 && best_lag.abs() < 100 {
                            // Good PRS correlation, use for fine timing
                            self.prs_fail_count = 0;
                            prs_start as usize
                        } else {
                            // PRS correlation is weak during tracking — this is common
                            // early in a stream (before freq correction converges) and
                            // does not mean we've lost frame sync.  Trust that the null
                            // skip placed us at the right spot and drain exactly T_G to
                            // reach the PRS useful start.  Only fall back to full
                            // re-acquisition after several consecutive failures.
                            self.prs_fail_count += 1;
                            if self.prs_fail_count >= 5 {
                                debug!(
                                    snr = format!("{:.1}", prs_snr),
                                    best_lag,
                                    consecutive = self.prs_fail_count,
                                    "PRS lost after 5 consecutive weak correlations, re-acquiring"
                                );
                                self.prs_fail_count = 0;
                                self.state = SyncState::NeedNullDetect;
                                continue;
                            }
                            debug!(
                                snr = format!("{:.1}", prs_snr),
                                best_lag,
                                consecutive = self.prs_fail_count,
                                "PRS weak during tracking — trusting cyclic-prefix timing"
                            );
                            T_G
                        }
                    };

                    // Check that we have enough data for the full frame BEFORE
                    // draining the PRS guard. This prevents the state machine from
                    // splitting PRS correlation and frame processing across process()
                    // calls, which would cause issues if new USB data appended between
                    // calls has a sample gap.
                    let frame_data_len = T_U + (L_SYMBOLS - 1) * T_S;
                    if self.buffer.len() < drain_count + frame_data_len {
                        // Not enough data yet — wait without changing state.
                        // PRS correlation will be redone on next call with fresh data.
                        break;
                    }

                    // Drain up to the PRS useful-part start
                    if drain_count > 0 {
                        let mut drained: Vec<Complex<f32>> = self.buffer[..drain_count].to_vec();
                        self.apply_freq_correction(&mut drained);
                        self.buffer.drain(..drain_count);
                        trace!(
                            drain_count,
                            best_lag,
                            from_null_detect = self.from_null_detect,
                            "PRS sync: drained to PRS useful start"
                        );
                    }
                    self.last_prs_snr = prs_snr;
                    self.state = SyncState::Synced;
                }

                SyncState::Synced => {
                    // buffer[0] = PRS useful-part start.
                    let frame_data_len = T_U + (L_SYMBOLS - 1) * T_S;
                    if self.buffer.len() < frame_data_len {
                        break;
                    }

                    // Frequency-correct the entire frame and extract symbols
                    let frame = self.process_frame(frame_data_len);

                    if let Some(frame) = frame {
                        // Drain the processed frame data
                        trace!(
                            frame_data_len,
                            buffer_len_before = self.buffer.len(),
                            "Draining frame data"
                        );
                        self.buffer.drain(..frame_data_len);
                        self.frame_count += 1;
                        frames.push(frame);

                        // After the frame, we're at the null symbol start.
                        // Transition to NeedNullSkip to handle the null symbol.
                        self.state = SyncState::NeedNullSkip;
                        // Continue to process NeedNullSkip state
                    } else {
                        break;
                    }
                }

                SyncState::NeedNullSkip => {
                    // We're at the null symbol start. Skip it while maintaining phase.
                    if self.buffer.len() < T_NULL {
                        // Not enough samples for null skip, wait for more data
                        break;
                    }

                    trace!(
                        t_null = T_NULL,
                        buffer_len = self.buffer.len(),
                        "Skipping null symbol"
                    );
                    // Frequency-correct and drain the null symbol samples
                    let mut null_samples: Vec<Complex<f32>> = self.buffer[..T_NULL].to_vec();
                    self.apply_freq_correction(&mut null_samples);
                    self.buffer.drain(..T_NULL);

                    trace!(
                        buffer_len_after_null = self.buffer.len(),
                        "After null skip, ready for PRS correlation"
                    );
                    // After skipping null, buffer[0] ≈ PRS guard start
                    // Go directly to PRS correlation
                    self.from_null_detect = false;
                    self.state = SyncState::NeedPrsCorrelation;
                }
            }
        }

        frames
    }

    /// Process a complete frame starting at buffer[0] = PRS useful start.
    ///
    /// Steps:
    /// 1. Frequency-correct the entire frame (advancing running_phase)
    /// 2. Estimate fine frequency from cyclic prefixes
    /// 3. Estimate coarse frequency from PRS
    /// 4. FFT all 76 symbols
    fn process_frame(&mut self, frame_data_len: usize) -> Option<OfdmFrame> {
        // ----------------------------------------------------------
        // Step 1: Estimate fine frequency from cyclic prefixes FIRST
        // ----------------------------------------------------------
        // We need the frequency estimate BEFORE applying correction, because
        // on the first frame (and whenever freq changes rapidly), the current
        // correction may be far off. The CP correlation works even on uncorrected
        // data since it measures phase difference over T_U samples, which gives
        // the total frequency offset including any uncorrected residual.
        //
        // welle.io handles this differently: it applies correction sample-by-sample
        // as data is read (with the PREVIOUS frame's estimate), then updates the
        // estimate after the frame. This works because welle.io's estimate converges
        // slowly (0.1 gain). But for the first frame, the correction is zero and the
        // full offset remains uncorrected.
        //
        // Our approach: estimate the full residual frequency offset from the raw
        // (uncorrected) data's CP correlation, then apply it all at once. This gives
        // immediate correction even on the first frame.

        // Extract raw data symbols from buffer for CP correlation (before any correction)
        let mut raw_symbol_bufs: Vec<Vec<Complex<f32>>> = Vec::with_capacity(L_SYMBOLS - 1);
        for i in 1..L_SYMBOLS {
            let sym_start = T_U + (i - 1) * T_S;
            let sym_end = sym_start + T_S;
            if sym_end > frame_data_len || sym_end > self.buffer.len() {
                break;
            }
            raw_symbol_bufs.push(self.buffer[sym_start..sym_end].to_vec());
        }
        let raw_fine_freq = Self::estimate_fine_freq_from_symbols(&raw_symbol_bufs);

        // The raw CP correlation gives the TOTAL frequency offset (including any
        // already-known offset). The new residual is: raw estimate - current correction.
        let current_total_hz =
            self.fine_freq_hz + self.coarse_freq_carriers as f32 * CARRIER_DIFF as f32;
        let residual_hz = raw_fine_freq - current_total_hz;

        // For the first frame, apply the full estimated offset immediately.
        // For subsequent frames, only update if the estimate is plausible
        // (within ±200 Hz of the current estimate — larger deviations indicate
        // a timing error rather than a real frequency change).
        if self.frame_count == 0 {
            // First frame: apply the full CP estimate as our correction
            self.fine_freq_hz = raw_fine_freq;
            trace!(
                raw_fine_freq = format!("{:.1}", raw_fine_freq),
                "First frame: applying full CP frequency estimate"
            );
        } else if residual_hz.abs() < 200.0 {
            // Plausible residual — gradual convergence
            self.fine_freq_hz += 0.1 * residual_hz;
            trace!(
                raw_fine_freq = format!("{:.1}", raw_fine_freq),
                residual_hz = format!("{:.1}", residual_hz),
                new_fine_hz = format!("{:.1}", self.fine_freq_hz),
                "Frame-to-frame freq update"
            );
        } else {
            // Large residual — likely timing error, ignore this estimate
            trace!(
                raw_fine_freq = format!("{:.1}", raw_fine_freq),
                residual_hz = format!("{:.1}", residual_hz),
                "Ignoring implausible CP frequency estimate"
            );
        }

        // Migrate fine to coarse if fine exceeds half a carrier
        let half_carrier = CARRIER_DIFF as f32 / 2.0;
        if self.fine_freq_hz > half_carrier {
            self.coarse_freq_carriers += 1;
            self.fine_freq_hz -= CARRIER_DIFF as f32;
        } else if self.fine_freq_hz < -half_carrier {
            self.coarse_freq_carriers -= 1;
            self.fine_freq_hz += CARRIER_DIFF as f32;
        }

        // ----------------------------------------------------------
        // Step 2: Apply frequency correction with updated estimate
        // ----------------------------------------------------------
        let mut corrected = self.buffer[..frame_data_len].to_vec();
        let phase_before = self.running_phase;
        self.apply_freq_correction(&mut corrected);
        trace!(
            frame = self.frame_count,
            fine_freq_hz = format!("{:.1}", self.fine_freq_hz),
            phase_before = format!("{:.4}", phase_before),
            phase_after = format!("{:.4}", self.running_phase),
            "process_frame: applied freq correction"
        );

        // ----------------------------------------------------------
        // Step 3: Estimate coarse frequency from PRS
        // ----------------------------------------------------------
        {
            let mut prs_spec: Vec<Complex<f32>> = corrected[..T_U].to_vec();
            self.fft.process(&mut prs_spec);

            let coarse = self.estimate_coarse_freq(&prs_spec);

            trace!(
                frame = self.frame_count,
                coarse_est = coarse,
                current_coarse = self.coarse_freq_carriers,
                fine_hz = format!("{:.1}", self.fine_freq_hz),
                prs_snr = format!("{:.1}", self.last_prs_snr),
                "Freq estimation"
            );

            if self.last_prs_snr > 20.0 || self.frame_count == 0 {
                self.coarse_freq_carriers = coarse;
            }
        }

        // ----------------------------------------------------------
        // Step 4: FFT all 76 symbols
        // ----------------------------------------------------------
        let mut symbols = Vec::with_capacity(L_SYMBOLS);

        // PRS: corrected[0..T_U] (no guard interval)
        {
            let mut prs: Vec<Complex<f32>> = corrected[..T_U].to_vec();
            self.fft.process(&mut prs);
            symbols.push(prs);
        }

        // Data symbols 1..75: skip guard interval (T_G) before FFT
        for i in 1..L_SYMBOLS {
            let useful_start = T_U + (i - 1) * T_S + T_G;
            let useful_end = useful_start + T_U;
            if useful_end > corrected.len() {
                break;
            }
            let mut sym: Vec<Complex<f32>> = corrected[useful_start..useful_end].to_vec();
            self.fft.process(&mut sym);
            symbols.push(sym);
        }

        if symbols.len() == L_SYMBOLS {
            Some(OfdmFrame {
                symbols,
                coarse_freq_offset: self.coarse_freq_carriers,
            })
        } else {
            None
        }
    }

    /// Compute PRS correlation magnitude-squared at a specific carrier offset.
    fn correlate_prs_at_offset(&self, prs_spectrum: &[Complex<f32>], offset: i32) -> f32 {
        let mut corr = Complex::new(0.0f32, 0.0);
        for k in 1..=(K / 2) as i32 {
            let pos_bin = k + offset;
            if pos_bin > 0 && (pos_bin as usize) < T_U {
                corr += prs_spectrum[pos_bin as usize] * self.prs_ref_conj[k as usize];
            }
            let neg_bin = T_U as i32 - k + offset;
            if neg_bin > 0 && (neg_bin as usize) < T_U {
                corr +=
                    prs_spectrum[neg_bin as usize] * self.prs_ref_conj[(T_U as i32 - k) as usize];
            }
        }
        corr.norm_sqr()
    }

    /// Find the exact start of the PRS useful part using IFFT-based correlation.
    ///
    /// Expects buffer[0] to be approximately the PRS guard interval start.
    /// Returns `(sample_offset, snr)` where sample_offset is from buffer[0] to the PRS
    /// useful-part start, or `(-1, 0.0)` if correlation is too weak.
    fn find_prs_start(&self) -> (i32, f32) {
        if self.buffer.len() < T_S + T_G {
            return (-1, 0.0);
        }

        if T_G + T_U > self.buffer.len() {
            return (-1, 0.0);
        }

        // Window at the expected useful part location: buffer[T_G..T_G+T_U]
        let win_start = T_G;
        let mut fft_buf: Vec<Complex<f32>> = self.buffer[win_start..win_start + T_U].to_vec();

        // No frequency correction for PRS correlation. The frequency offset (~100-200 Hz)
        // is much smaller than the carrier spacing (1000 Hz), so it only adds a phase
        // rotation to each carrier without shifting the FFT bins. The IFFT correlation
        // magnitude is unaffected by this rotation.

        // Debug: compute signal power in the window
        let window_power: f32 = fft_buf.iter().map(|c| c.norm_sqr()).sum::<f32>() / T_U as f32;

        self.fft.process(&mut fft_buf);

        let mut corr_buf: Vec<Complex<f32>> = fft_buf
            .iter()
            .zip(self.prs_ref_conj.iter())
            .map(|(a, b)| a * b)
            .collect();
        self.ifft.process(&mut corr_buf);

        let mut best_lag = 0i32;
        let mut best_mag = 0.0f32;
        let mut sum_mag = 0.0f32;

        for (i, c) in corr_buf.iter().enumerate() {
            let mag = c.norm_sqr();
            sum_mag += mag.sqrt();
            if mag > best_mag {
                best_mag = mag;
                best_lag = if i <= T_U / 2 {
                    i as i32
                } else {
                    i as i32 - T_U as i32
                };
            }
        }

        let avg_mag = sum_mag / T_U as f32;
        let snr = if avg_mag > 0.0 {
            best_mag.sqrt() / avg_mag
        } else {
            0.0
        };

        let prs_start = win_start as i32 + best_lag;

        trace!(
            buffer_len = self.buffer.len(),
            window_power = format!("{:.1}", window_power),
            best_lag,
            best_mag = format!("{:.1}", best_mag.sqrt()),
            avg_mag = format!("{:.1}", avg_mag),
            snr = format!("{:.2}", snr),
            "find_prs_start"
        );

        (prs_start.max(0), snr)
    }

    /// Find the null symbol by detecting a power drop (initial acquisition only).
    /// Consumes samples up to and including the null symbol.
    ///
    /// After detection, buffer[0] ≈ start of PRS guard interval.
    fn find_null_symbol(&mut self) -> bool {
        if self.buffer.len() < T_NULL + T_S + T_F / 4 {
            return false;
        }

        let window = 128;
        let search_len = self.buffer.len().min(T_F * 2).saturating_sub(T_NULL + T_S);
        if search_len < window * 2 {
            return false;
        }

        let mut best_pos = 0usize;
        let mut best_ratio = 0.0f32;

        for pos in (T_NULL / 2..search_len).step_by(window / 4) {
            if pos < window || pos + window > self.buffer.len() {
                continue;
            }
            let before_power: f32 = self.buffer[pos - window..pos]
                .iter()
                .map(|s| s.norm_sqr())
                .sum();
            let after_power: f32 = self.buffer[pos..pos + window]
                .iter()
                .map(|s| s.norm_sqr())
                .sum();

            if before_power > 0.0 {
                let ratio = after_power / before_power;
                if ratio > best_ratio {
                    best_ratio = ratio;
                    best_pos = pos;
                }
            }
        }

        trace!(
            buffer_len = self.buffer.len(),
            search_len,
            best_pos,
            best_ratio = format!("{:.2}", best_ratio),
            "find_null_symbol"
        );

        if best_ratio < 4.0 {
            let discard = self.buffer.len().min(T_F / 2);
            // Advance phase for discarded samples to maintain continuity
            let total_freq_hz =
                self.fine_freq_hz + self.coarse_freq_carriers as f32 * CARRIER_DIFF as f32;
            let phase_step = -2.0 * PI * total_freq_hz / SAMPLE_RATE as f32;
            self.running_phase += phase_step * discard as f32;
            self.running_phase = self.running_phase.rem_euclid(2.0 * PI);
            if self.running_phase > PI {
                self.running_phase -= 2.0 * PI;
            }
            self.buffer.drain(..discard);
            return false;
        }

        if best_pos >= self.buffer.len() {
            return false;
        }

        // CRITICAL: Advance running_phase for all drained samples to maintain phase
        // continuity. Without this, the frequency correction applied in process_frame()
        // starts with the wrong phase, causing constellation rotation that gets worse
        // for later symbols (MSC).
        let total_freq_hz =
            self.fine_freq_hz + self.coarse_freq_carriers as f32 * CARRIER_DIFF as f32;
        let phase_step = -2.0 * PI * total_freq_hz / SAMPLE_RATE as f32;
        self.running_phase += phase_step * best_pos as f32;
        // Wrap to [-PI, PI) for numerical stability
        self.running_phase = self.running_phase.rem_euclid(2.0 * PI);
        if self.running_phase > PI {
            self.running_phase -= 2.0 * PI;
        }

        trace!(
            drained = best_pos,
            phase_advance = format!("{:.4}", phase_step * best_pos as f32),
            running_phase = format!("{:.4}", self.running_phase),
            "find_null_symbol: advancing phase for drained samples"
        );

        self.buffer.drain(..best_pos);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex;

    /// Test IFFT-based PRS timing with a synthetic signal.
    #[test]
    fn test_prs_ifft_timing() {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(T_U);
        let ifft = planner.plan_fft_inverse(T_U);

        let prs_ref = phase_reference_table();
        let prs_ref_conj: Vec<Complex<f32>> = prs_ref.iter().map(|c| c.conj()).collect();

        // Create time-domain PRS by IFFT of frequency-domain reference
        let mut prs_time = prs_ref.clone();
        ifft.process(&mut prs_time);
        for s in prs_time.iter_mut() {
            *s /= T_U as f32;
        }

        // Test 1: PRS at offset 0 — IFFT peak should be at 0
        {
            let mut test_sig = prs_time.clone();
            fft.process(&mut test_sig);
            let mut corr: Vec<Complex<f32>> = test_sig
                .iter()
                .zip(prs_ref_conj.iter())
                .map(|(a, b)| a * b)
                .collect();
            ifft.process(&mut corr);
            let (peak_idx, _) = corr
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.norm_sqr().partial_cmp(&b.norm_sqr()).unwrap())
                .unwrap();
            assert_eq!(peak_idx, 0, "PRS at offset 0 should give IFFT peak at 0");
        }

        // Test 2: PRS circularly shifted by 100 samples — peak should be at 100
        {
            let shift = 100usize;
            let mut shifted = vec![Complex::new(0.0f32, 0.0); T_U];
            for i in 0..T_U {
                shifted[(i + shift) % T_U] = prs_time[i];
            }
            fft.process(&mut shifted);
            let mut corr: Vec<Complex<f32>> = shifted
                .iter()
                .zip(prs_ref_conj.iter())
                .map(|(a, b)| a * b)
                .collect();
            ifft.process(&mut corr);
            let (peak_idx, _) = corr
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.norm_sqr().partial_cmp(&b.norm_sqr()).unwrap())
                .unwrap();
            assert_eq!(
                peak_idx, shift,
                "PRS shifted by {} should give IFFT peak at {}",
                shift, shift
            );
        }

        // Test 3: PRS shifted by T_G — simulating guard interval prepended
        {
            let shift = T_G;
            let mut shifted = vec![Complex::new(0.0f32, 0.0); T_U];
            for i in 0..T_U {
                shifted[(i + shift) % T_U] = prs_time[i];
            }
            fft.process(&mut shifted);
            let mut corr: Vec<Complex<f32>> = shifted
                .iter()
                .zip(prs_ref_conj.iter())
                .map(|(a, b)| a * b)
                .collect();
            ifft.process(&mut corr);
            let (peak_idx, _) = corr
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.norm_sqr().partial_cmp(&b.norm_sqr()).unwrap())
                .unwrap();
            assert_eq!(
                peak_idx, T_G,
                "PRS shifted by T_G should give IFFT peak at T_G"
            );
        }

        // Test 4: PRS with frequency offset of 3 carriers
        {
            let freq_offset_carriers = 3;
            let freq_hz = freq_offset_carriers as f32 * CARRIER_DIFF as f32;
            let mut sig = prs_time.clone();
            let phase_step = 2.0 * PI * freq_hz / SAMPLE_RATE as f32;
            let mut phase = 0.0f32;
            for s in sig.iter_mut() {
                let (sin_p, cos_p) = phase.sin_cos();
                let rotated =
                    Complex::new(s.re * cos_p - s.im * sin_p, s.re * sin_p + s.im * cos_p);
                *s = rotated;
                phase += phase_step;
            }
            fft.process(&mut sig);
            let mut corr: Vec<Complex<f32>> = sig
                .iter()
                .zip(prs_ref_conj.iter())
                .map(|(a, b)| a * b)
                .collect();
            ifft.process(&mut corr);
            let peak_mag: f32 = corr.iter().map(|c| c.norm_sqr()).fold(0.0f32, f32::max);
            let sum_mag: f32 = corr.iter().map(|c| c.norm_sqr().sqrt()).sum();
            let snr = peak_mag.sqrt() * T_U as f32 / sum_mag;
            eprintln!(
                "  Freq offset test: snr={:.1} (should be low for 3-carrier offset)",
                snr
            );
        }
    }
}
