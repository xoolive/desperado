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

/// Search range for coarse frequency estimation (in carriers).
/// Matches welle.io's `#define SEARCH_RANGE (2 * 36)`.
const SEARCH_RANGE: usize = 2 * 36;

/// Number of adjacent-carrier phase differences to correlate against the
/// PRS reference in the phase-difference coarse frequency estimator.
/// Matches welle.io's `#define CORRELATION_LENGTH 24`.
const CORRELATION_LENGTH: usize = 24;

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
    /// Phase-difference reference for coarse frequency estimation.
    /// `ref_arg[j]` = `arg(prs_ref[T_u + j] * conj(prs_ref[T_u + j + 1]))` for
    /// `j` in `0..CORRELATION_LENGTH`. This matches welle.io's `refArg`.
    ref_arg: Vec<f32>,
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
    /// FIC decode ratio (0–100%), set by external code.
    /// Used to gate coarse frequency correction: only apply coarse when FIC
    /// decoding is poor (< 50%), matching welle.io's approach.
    /// This prevents a reception glitch from corrupting an already-working
    /// frequency estimate.
    fic_decode_ratio: u8,
    /// Whether initial coarse correction has been applied (from getMiddle or
    /// phase-difference method). Reset on sync loss.
    initial_coarse_applied: bool,
    /// Long-term average signal level (L1 norm), used for null detection.
    /// Matches welle.io's `sLevel` which is computed with exponential smoothing.
    s_level: f32,
    /// Whether initial warm-up has been done (discard first samples for AGC settling).
    /// Matches welle.io's initial `for (i = 0; i < T_F / 2; i++) getSample(0)`.
    warmup_done: bool,
    /// Total number of samples consumed (for warm-up tracking).
    samples_consumed: usize,
}

impl OfdmProcessor {
    pub fn new() -> Self {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(T_U);
        let ifft = planner.plan_fft_inverse(T_U);

        // Build conjugated PRS reference for correlation
        let prs_ref = phase_reference_table();
        let prs_ref_conj: Vec<Complex<f32>> = prs_ref.iter().map(|c| c.conj()).collect();

        // Build phase-difference reference for coarse frequency estimation.
        // ref_arg[j] = arg(prs_ref[(T_u + j) % T_u] * conj(prs_ref[(T_u + j + 1) % T_u]))
        // This matches welle.io's refArg initialization (ofdm-processor.cpp:97-101).
        let ref_arg: Vec<f32> = (0..CORRELATION_LENGTH)
            .map(|j| {
                let a = prs_ref[(T_U + j) % T_U];
                let b = prs_ref[(T_U + j + 1) % T_U];
                (a * b.conj()).arg()
            })
            .collect();

        Self {
            fft,
            ifft,
            prs_ref_conj,
            ref_arg,
            buffer: Vec::with_capacity(T_F * 2),
            state: SyncState::NeedNullDetect,
            frame_count: 0,
            fine_freq_hz: 0.0,
            coarse_freq_carriers: 0,
            last_prs_snr: 0.0,
            running_phase: 0.0,
            from_null_detect: true,
            prs_fail_count: 0,
            fic_decode_ratio: 0,
            initial_coarse_applied: false,
            s_level: 0.0,
            warmup_done: false,
            samples_consumed: 0,
        }
    }

    /// Update the FIC decode ratio (0–100%). Call this after each frame's FIC
    /// processing to allow the OFDM processor to gate coarse frequency correction.
    /// Matches welle.io's `ficHandler.getFicDecodeRatioPercent()` feedback loop.
    pub fn set_fic_decode_ratio(&mut self, ratio_percent: u8) {
        self.fic_decode_ratio = ratio_percent.min(100);
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

    /// Estimate coarse (integer carrier) frequency offset from PRS using
    /// phase-difference correlation (welle.io's CorrelatePRS method).
    ///
    /// Instead of correlating carrier values directly (which is destroyed by
    /// timing offsets that add linear phase across carriers), this computes
    /// `arg(fft[k] * conj(fft[k+1]))` — the phase difference between adjacent
    /// carriers — and correlates that pattern with a precomputed reference.
    /// Phase differences are insensitive to linear phase ramps from timing
    /// offsets.
    ///
    /// Searches carrier offsets in `[-SEARCH_RANGE/2, +SEARCH_RANGE/2)`.
    fn estimate_coarse_freq_phase_diff(&self, prs_spectrum: &[Complex<f32>]) -> i32 {
        if prs_spectrum.len() != T_U {
            return 0;
        }

        // Compute phase differences for the search window.
        // welle.io: correlationVector[i] = arg(fft[baseIndex] * conj(fft[baseIndex+1]))
        //   where baseIndex = T_u - SEARCH_RANGE/2 + i
        let mut corr_vec = vec![0.0f32; SEARCH_RANGE + CORRELATION_LENGTH];
        #[allow(clippy::needless_range_loop)]
        for i in 0..(SEARCH_RANGE + CORRELATION_LENGTH) {
            let base = T_U + i - SEARCH_RANGE / 2;
            let a = prs_spectrum[base % T_U];
            let b = prs_spectrum[(base + 1) % T_U];
            corr_vec[i] = (a * b.conj()).arg();
        }

        // Slide the reference pattern across the correlation vector to find best match.
        // welle.io: sum += abs(refArg[j] * correlationVector[i + j])
        let mut best_sum = 0.0f32;
        let mut best_index = 0usize;
        for i in 0..SEARCH_RANGE {
            let mut sum = 0.0f32;
            for j in 0..CORRELATION_LENGTH {
                sum += (self.ref_arg[j] * corr_vec[i + j]).abs();
                // welle.io updates best inside the inner loop (slightly different
                // semantics — it picks the partial sum that first exceeds the max).
                // We match that behavior for compatibility.
                if sum > best_sum {
                    best_sum = sum;
                    best_index = i;
                }
            }
        }

        // Map index back to carrier offset.
        // welle.io: return T_u - SEARCH_RANGE/2 + index - T_u
        //         = index - SEARCH_RANGE/2
        let offset = best_index as i32 - (SEARCH_RANGE as i32) / 2;

        trace!(
            best_index,
            best_sum = format!("{:.2}", best_sum),
            offset,
            "estimate_coarse_freq_phase_diff"
        );

        offset
    }

    /// Estimate coarse frequency offset by finding where spectral energy is
    /// concentrated — no reference needed. This is welle.io's `getMiddle` method.
    ///
    /// Computes a sliding K-wide window sum of `|fft[k]|` and finds the maximum.
    /// The offset of that maximum relative to the expected center gives the
    /// coarse carrier offset.
    ///
    /// This is the most robust initial sync method because it needs no PRS
    /// reference — just the knowledge that DAB has K=1536 active carriers.
    fn get_middle(prs_spectrum: &[Complex<f32>]) -> i32 {
        if prs_spectrum.len() != T_U {
            return 0;
        }

        // Compute initial sum over K carriers starting at index 40 relative to center.
        let mut sum: f32 = 0.0;
        for i in 40..(K + 40) {
            sum += prs_spectrum[(T_U / 2 + i) % T_U].norm();
        }

        let mut max_sum = sum;
        let mut max_index = 40i32;

        // Slide the window: remove one carrier from the left, add one on the right.
        for i in 41..(T_U - (K - 40)) {
            sum -= prs_spectrum[(T_U / 2 + i - 1) % T_U].norm();
            sum += prs_spectrum[(T_U / 2 + i - 1 + K) % T_U].norm();
            if sum > max_sum {
                max_sum = sum;
                max_index = i as i32;
            }
        }

        // Expected center of active carriers: (T_U - K) / 2
        let expected = (T_U - K) / 2;
        let offset = max_index - expected as i32;

        trace!(
            max_index,
            expected,
            max_sum = format!("{:.1}", max_sum),
            offset,
            "get_middle"
        );

        offset
    }

    /// Brute-force search for the best coarse frequency offset by trying
    /// PRS IFFT correlation at each candidate offset from -5 to +5 and
    /// returning the one with the highest SNR.
    ///
    /// This is used as a fallback when `getMiddle` returns an implausible
    /// result (e.g., on noisy/partial PRS data). More expensive than
    /// `getMiddle` but works even on weak signals because it uses the
    /// PRS reference for correlation.
    ///
    /// Returns `Some(offset)` if a clear winner is found (SNR > 5 and at
    /// least 2x better than the next candidate), or `None` if inconclusive.
    fn brute_force_coarse_search(
        prs_spectrum: &[Complex<f32>],
        prs_ref_conj: &[Complex<f32>],
        ifft: &Arc<dyn Fft<f32>>,
    ) -> Option<i32> {
        if prs_spectrum.len() != T_U {
            return None;
        }

        let mut best_snr = 0.0f32;
        let mut best_offset = 0i32;
        let mut second_snr = 0.0f32;

        for d in -5..=5i32 {
            // Build correlation buffer with this candidate offset
            let mut corr_buf: Vec<Complex<f32>> = (0..T_U)
                .map(|k| {
                    let ref_idx = ((k as i32 - d).rem_euclid(T_U as i32)) as usize;
                    prs_spectrum[k] * prs_ref_conj[ref_idx]
                })
                .collect();
            ifft.process(&mut corr_buf);

            // Find peak and compute SNR
            let mut peak_mag = 0.0f32;
            let mut sum_mag = 0.0f32;
            for c in corr_buf.iter() {
                let mag = c.norm_sqr();
                sum_mag += mag.sqrt();
                if mag > peak_mag {
                    peak_mag = mag;
                }
            }
            let avg_mag = sum_mag / T_U as f32;
            let snr = if avg_mag > 0.0 {
                peak_mag.sqrt() / avg_mag
            } else {
                0.0
            };

            trace!(d, snr = format!("{:.2}", snr), "brute_force_coarse_search");

            if snr > best_snr {
                second_snr = best_snr;
                best_snr = snr;
                best_offset = d;
            } else if snr > second_snr {
                second_snr = snr;
            }
        }

        // Require the best to be clearly better than the second-best
        // and have a reasonable absolute SNR
        if best_snr > 5.0 && (second_snr < 1.0 || best_snr > second_snr * 1.5) {
            debug!(
                best_offset,
                best_snr = format!("{:.1}", best_snr),
                second_snr = format!("{:.1}", second_snr),
                "brute_force_coarse_search: clear winner"
            );
            Some(best_offset)
        } else {
            debug!(
                best_snr = format!("{:.1}", best_snr),
                second_snr = format!("{:.1}", second_snr),
                "brute_force_coarse_search: inconclusive"
            );
            None
        }
    }

    /// Feed IQ samples into the processor. Call repeatedly with chunks.
    /// Returns complete frames when enough data has been accumulated and synced.
    ///
    /// Frame processing follows welle.io's architecture:
    /// - Initial warm-up: discard first T_F/2 samples while computing s_level
    /// - Initial sync: null detect → PRS correlate → process frame
    /// - Subsequent frames: consume null samples → PRS correlate → process frame
    /// - Frequency correction phase is continuous across all operations
    pub fn process(&mut self, samples: &[Complex<f32>]) -> Vec<OfdmFrame> {
        self.buffer.extend_from_slice(samples);
        let mut frames = Vec::new();

        // --- Warm-up phase (matches welle.io's initial sLevel computation) ---
        // Discard the first T_F/2 samples to let the hardware's AGC settle.
        // Use these samples to compute the long-term signal level (s_level)
        // needed for null detection.
        if !self.warmup_done {
            let warmup_target = T_F / 2;
            let remaining = warmup_target - self.samples_consumed;
            let available = self.buffer.len().min(remaining);
            if available > 0 {
                // Accumulate L1 sum for fast mean-based initialization.
                // Using arithmetic mean instead of EMA with tiny alpha (0.00001)
                // because the EMA only converges to ~63% of the true level after
                // T_F/2 samples. The mean gives exact convergence immediately.
                for s in &self.buffer[..available] {
                    let l1 = s.re.abs() + s.im.abs();
                    self.s_level += l1;
                }
                self.samples_consumed += available;
                self.buffer.drain(..available);
            }
            if self.samples_consumed < warmup_target {
                return frames; // Need more samples for warm-up
            }
            // Convert accumulated sum to mean L1 norm per sample.
            self.s_level /= self.samples_consumed as f32;
            self.warmup_done = true;
            debug!(
                s_level = format!("{:.4}", self.s_level),
                warmup_samples = self.samples_consumed,
                "AGC warm-up complete"
            );
        }

        loop {
            match self.state {
                SyncState::NeedNullDetect => {
                    // Reset initial coarse flag so getMiddle runs again on re-acquisition.
                    // Also reset frequency estimates — if we lost sync, the old estimates
                    // may be wildly wrong and would poison the next acquisition attempt.
                    self.initial_coarse_applied = false;
                    self.coarse_freq_carriers = 0;
                    self.fine_freq_hz = 0.0;
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

                    // -------------------------------------------------------
                    // Initial coarse frequency estimation using getMiddle.
                    //
                    // On first acquisition (or re-acquisition after sync loss),
                    // find where spectral energy is concentrated BEFORE doing
                    // PRS correlation. This breaks the chicken-and-egg problem
                    // where PRS correlation needs coarse correction but coarse
                    // correction needs good PRS correlation.
                    //
                    // getMiddle needs no PRS reference — just the knowledge
                    // that DAB has K=1536 active carriers — so it works even
                    // with large frequency offsets that destroy IFFT-based PRS
                    // correlation.
                    // -------------------------------------------------------
                    if self.from_null_detect && !self.initial_coarse_applied {
                        // Need at least T_G + T_U samples to FFT the PRS
                        if self.buffer.len() >= T_G + T_U {
                            let mut prs_spec: Vec<Complex<f32>> =
                                self.buffer[T_G..T_G + T_U].to_vec();
                            self.fft.process(&mut prs_spec);

                            let middle_offset = Self::get_middle(&prs_spec);
                            // Sanity check: real hardware offsets are at most a few
                            // carriers. getMiddle can return wild values (100+) when
                            // it sees a partial/misaligned PRS or noise. Clamp to ±5
                            // to prevent catastrophic divergence while still covering
                            // any realistic tuner frequency offset.
                            if middle_offset != 0 && middle_offset.abs() <= 5 {
                                debug!(middle_offset, "Initial coarse freq from getMiddle");
                                self.coarse_freq_carriers += middle_offset;
                            } else if middle_offset.abs() > 5 {
                                debug!(
                                    middle_offset,
                                    "getMiddle implausible, trying brute-force PRS search"
                                );
                                // getMiddle failed (noisy data). Fall back to trying
                                // PRS IFFT correlation at each candidate coarse offset
                                // from -5 to +5 and picking the one with best SNR.
                                // This is more expensive but very robust.
                                let best = Self::brute_force_coarse_search(
                                    &prs_spec,
                                    &self.prs_ref_conj,
                                    &self.ifft,
                                );
                                if let Some(offset) = best {
                                    debug!(offset, "Brute-force PRS search found coarse offset");
                                    self.coarse_freq_carriers += offset;
                                }
                            }
                            self.initial_coarse_applied = true;
                        }
                    }

                    // Use PRS correlation to find exact timing
                    let (prs_start, prs_snr) = self.find_prs_start();
                    debug!(
                        prs_start,
                        prs_snr = format!("{:.2}", prs_snr),
                        from_null_detect = self.from_null_detect,
                        "PRS correlation"
                    );
                    if prs_start == i32::MIN {
                        // Not enough data — wait for more
                        break;
                    }

                    let best_lag = prs_start - T_G as i32;

                    // For frame-to-frame tracking, if PRS SNR is low, the correlation is unreliable.
                    // In this case, trust the timing and just drain T_G samples to get to useful start.
                    // For initial sync (from_null_detect), we need good PRS correlation.
                    let drain_count = if self.from_null_detect {
                        // Initial sync - use PRS correlation result
                        // If prs_start < 0, we've already passed the PRS useful start — re-acquire.
                        if prs_snr < 5.0 || best_lag.abs() > 1500 || prs_start < 0 {
                            debug!(
                                snr = format!("{:.2}", prs_snr),
                                best_lag,
                                prs_start,
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
                        if prs_snr > 10.0 && best_lag.abs() < 300 {
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
                        debug!(
                            frame = self.frame_count,
                            coarse_offset = frame.coarse_freq_offset,
                            "produced frame"
                        );
                        self.buffer.drain(..frame_data_len);
                        self.frame_count += 1;
                        frames.push(frame);

                        // After the frame, we're at the null symbol start.
                        // Transition to NeedNullSkip to handle the null symbol.
                        self.state = SyncState::NeedNullSkip;
                        // Continue to process NeedNullSkip state
                    } else {
                        debug!("process_frame returned None, re-acquiring");
                        self.state = SyncState::NeedNullDetect;
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

    /// Process a complete frame starting at `buffer[0]` = PRS useful start.
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

            // Use phase-difference approach (robust to timing offsets) instead
            // of the old coherent-sum method.
            let coarse = self.estimate_coarse_freq_phase_diff(&prs_spec);

            trace!(
                frame = self.frame_count,
                coarse_est = coarse,
                current_coarse = self.coarse_freq_carriers,
                fine_hz = format!("{:.1}", self.fine_freq_hz),
                prs_snr = format!("{:.1}", self.last_prs_snr),
                "Freq estimation"
            );

            debug!(
                coarse_est = coarse,
                prs_snr = format!("{:.1}", self.last_prs_snr),
                frame = self.frame_count,
                raw_fine_hz = format!("{:.1}", raw_fine_freq),
                fine_hz = format!("{:.1}", self.fine_freq_hz),
                coarse_carriers = self.coarse_freq_carriers,
                fic_ratio = self.fic_decode_ratio,
                "freq estimation"
            );

            // Gate coarse correction:
            // - Only when FIC decoding is poor (< 50%), matching welle.io.
            // - The phase-diff estimator is robust to timing offsets but can
            //   give noisy results when PRS SNR is very low. Require SNR > 5
            //   (much lower than the old > 15 gate that caused the chicken-and-egg
            //   problem — getMiddle handles the initial large offset).
            // - Limit per-frame correction to ±3 carriers. The phase-diff method
            //   should only need small adjustments. Large jumps indicate noise.
            if self.fic_decode_ratio < 50
                && coarse != 0
                && self.last_prs_snr > 5.0
                && coarse.abs() <= 3
            {
                // Sanity: don't apply corrections that would push total
                // coarse beyond ±35 carriers (35 kHz), matching welle.io.
                let new_coarse = self.coarse_freq_carriers + coarse;
                if new_coarse.abs() <= 35 {
                    debug!(
                        correction = coarse,
                        new_coarse, "applying coarse freq correction"
                    );
                    self.coarse_freq_carriers = new_coarse;
                }
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
    #[allow(dead_code)] // Retained for testing; production uses phase-diff method
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
    /// Expects `buffer[0]` to be approximately the PRS guard interval start.
    /// Returns `(sample_offset, snr)` where sample_offset is from `buffer[0]` to the PRS
    /// useful-part start, or `(i32::MIN, 0.0)` if not enough data yet.
    /// May return negative values if the PRS start was already passed (caller should re-acquire).
    ///
    /// Accounts for `coarse_freq_carriers` by shifting the PRS reference so that
    /// the correlation works even with a coarse frequency offset (where the FFT
    /// bins are shifted by the offset amount).
    fn find_prs_start(&self) -> (i32, f32) {
        if self.buffer.len() < T_S + T_G {
            return (i32::MIN, 0.0);
        }

        if T_G + T_U > self.buffer.len() {
            return (i32::MIN, 0.0);
        }

        // Window at the expected useful part location: buffer[T_G..T_G+T_U]
        let win_start = T_G;
        let mut fft_buf: Vec<Complex<f32>> = self.buffer[win_start..win_start + T_U].to_vec();

        // Debug: compute signal power in the window
        let window_power: f32 = fft_buf.iter().map(|c| c.norm_sqr()).sum::<f32>() / T_U as f32;

        self.fft.process(&mut fft_buf);

        // Build correlation buffer accounting for coarse frequency offset.
        // When the signal has a coarse offset of `d` carriers, FFT bin `k` in the
        // received signal corresponds to reference bin `k - d`. So we correlate:
        //   corr[k] = fft_buf[k] * prs_ref_conj[(k - d) mod T_U]
        let d = self.coarse_freq_carriers;
        let mut corr_buf: Vec<Complex<f32>> = (0..T_U)
            .map(|k| {
                let ref_idx = ((k as i32 - d).rem_euclid(T_U as i32)) as usize;
                fft_buf[k] * self.prs_ref_conj[ref_idx]
            })
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

        debug!(
            buf = self.buffer.len(),
            win_power = format!("{:.4}", window_power),
            best_lag,
            best_mag = format!("{:.2}", best_mag.sqrt()),
            snr = format!("{:.2}", snr),
            prs_start,
            "find_prs_start"
        );

        (prs_start, snr)
    }

    /// Find the null symbol using welle.io's two-phase approach:
    /// 1. Scan for a power dip (null symbol) where the 50-sample average drops
    ///    below 50% of the long-term signal level
    /// 2. Then find the end of the null (power rises above 75% of s_level)
    ///
    /// After detection, `buffer[0]` ≈ start of PRS guard interval.
    /// Uses and updates `self.s_level` throughout for adaptive thresholding.
    fn find_null_symbol(&mut self) -> bool {
        // Need enough samples to search through (at least one frame)
        if self.buffer.len() < T_NULL + T_S {
            return false;
        }

        let window = 50usize;
        let search_limit = self.buffer.len().min(T_F * 2);

        // Phase 1: Build initial window and find the null dip
        if search_limit < window + T_NULL {
            return false;
        }

        // Initialize the sliding window sum over the first `window` samples
        let mut current_strength: f32 = 0.0;
        for i in 0..window {
            current_strength += self.buffer[i].re.abs() + self.buffer[i].im.abs();
        }

        // Update s_level from these initial samples too
        for i in 0..window {
            let l1 = self.buffer[i].re.abs() + self.buffer[i].im.abs();
            self.s_level = 0.00001 * l1 + (1.0 - 0.00001) * self.s_level;
        }

        // Scan for null dip: sliding window average < 0.60 * s_level
        // We use 0.60 instead of welle.io's 0.50 because our push-based model
        // doesn't apply frequency correction during null scanning, resulting in
        // a higher noise floor in the null period (null/signal ratio ~0.51-0.55
        // for Airspy Mini live data vs ~0.20 in file recordings).
        let mut pos = window;
        let mut found_null = false;
        let mut counter = 0u32;
        while pos < search_limit {
            let sample = self.buffer[pos];
            let l1 = sample.re.abs() + sample.im.abs();
            self.s_level = 0.00001 * l1 + (1.0 - 0.00001) * self.s_level;

            // Update sliding window: add new, remove oldest
            current_strength += l1;
            if pos >= window {
                current_strength -=
                    self.buffer[pos - window].re.abs() + self.buffer[pos - window].im.abs();
            }

            pos += 1;
            counter += 1;

            let window_avg = current_strength / window as f32;
            if self.s_level > 0.0 && window_avg <= 0.60 * self.s_level {
                found_null = true;
                break;
            }

            if counter > T_F as u32 {
                // Hopeless — no null found in one full frame
                debug!(
                    s_level = format!("{:.4}", self.s_level),
                    counter, "null detect: no null dip found"
                );
                // Discard some samples and try again
                let discard = self.buffer.len().min(T_F / 2);
                self.buffer.drain(..discard);
                return false;
            }
        }

        if !found_null {
            // Not enough samples to find null
            return false;
        }

        let null_start_pos = pos;
        trace!(
            null_start_pos,
            s_level = format!("{:.4}", self.s_level),
            window_avg = format!("{:.4}", current_strength / window as f32),
            "null detect: found null dip"
        );

        // Phase 2: Find end of null (power rises above 0.75 * s_level)
        // IMPORTANT: Do NOT update s_level during null period scanning.
        // The null period has very low power which would drag s_level down,
        // lowering the end-of-null threshold and causing false triggers on
        // noise spikes within the null. Freeze s_level at its pre-null value.
        let s_level_frozen = self.s_level;
        let mut null_counter = 0u32;
        while pos < search_limit {
            let sample = self.buffer[pos];
            let l1 = sample.re.abs() + sample.im.abs();
            // Continue updating s_level for phase 1 consistency in future calls,
            // but use the frozen value for threshold comparison.
            self.s_level = 0.00001 * l1 + (1.0 - 0.00001) * self.s_level;

            current_strength += l1;
            if pos >= window {
                current_strength -=
                    self.buffer[pos - window].re.abs() + self.buffer[pos - window].im.abs();
            }

            pos += 1;
            null_counter += 1;

            if s_level_frozen > 0.0 && current_strength / window as f32 >= 0.75 * s_level_frozen {
                // Validate null length: must be close to T_NULL (2656 samples).
                // Accept >= 50% of T_NULL to account for timing jitter and
                // the sliding window lag (~50 samples). Values like 137 or 411
                // are clearly false triggers on noise.
                let min_null_len = (T_NULL / 2) as u32;
                if null_counter < min_null_len {
                    debug!(
                        null_start = null_start_pos,
                        null_len = null_counter,
                        min_null_len,
                        "null detect: null period too short, false trigger"
                    );
                    // This was a false null dip (e.g. AGC transient, noise dip).
                    // Continue scanning from here — don't discard samples.
                    // Reset to phase 1 scanning by breaking out and returning false
                    // so we retry with more data next time.
                    let discard = pos.min(self.buffer.len());
                    self.buffer.drain(..discard);
                    return false;
                }

                debug!(
                    null_start = null_start_pos,
                    null_end = pos,
                    null_len = null_counter,
                    s_level = format!("{:.4}", s_level_frozen),
                    "null detect OK"
                );

                // Drain everything up to the end-of-null position.
                // The PRS guard interval starts approximately here.
                self.buffer.drain(..pos);
                return true;
            }

            if null_counter > (T_NULL + T_NULL / 4) as u32 {
                // Null period too long — probably not a real null
                debug!(
                    null_start = null_start_pos,
                    null_counter, "null detect: null period too long, re-syncing"
                );
                let discard = self.buffer.len().min(T_F / 2);
                self.buffer.drain(..discard);
                return false;
            }
        }

        // Not enough samples to find end of null
        false
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
            trace!(
                snr = format!("{:.1}", snr),
                "freq offset test (should be low for 3-carrier offset)"
            );
        }
    }
}
