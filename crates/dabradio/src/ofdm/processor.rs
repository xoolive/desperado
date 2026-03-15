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
                    self.state = SyncState::NeedPrsCorrelation;
                }

                SyncState::NeedPrsCorrelation => {
                    // We need T_U samples to run PRS correlation.
                    // After null detection or null consumption, buffer[0] ≈ PRS guard start.
                    let (prs_start, prs_snr) = self.find_prs_start();
                    if prs_start < 0 {
                        // Not enough data — wait for more
                        break;
                    }
                    if prs_snr < 5.0 {
                        // PRS correlation too weak — frame alignment likely lost
                        // (e.g. due to dropped samples). Re-acquire from null detection.
                        debug!(
                            snr = format!("{:.2}", prs_snr),
                            "PRS correlation failed, re-acquiring sync"
                        );
                        self.state = SyncState::NeedNullDetect;
                        continue;
                    }
                    // Drain up to the PRS useful-part start
                    if prs_start > 0 {
                        // Apply freq correction to the drained samples to keep phase continuous
                        let drain_count = prs_start as usize;
                        let mut drained: Vec<Complex<f32>> = self.buffer[..drain_count].to_vec();
                        self.apply_freq_correction(&mut drained);
                        self.buffer.drain(..drain_count);
                    }
                    self.last_prs_snr = prs_snr;
                    self.state = SyncState::Synced;
                }

                SyncState::Synced => {
                    // buffer[0] = PRS useful-part start.
                    // Frame layout:
                    //   PRS useful: [0, T_U)
                    //   75 data symbols: each T_S samples (guard + useful)
                    //   Total: T_U + 75 * T_S = 193448
                    let frame_data_len = T_U + (L_SYMBOLS - 1) * T_S;
                    if self.buffer.len() < frame_data_len {
                        break;
                    }

                    // Frequency-correct the entire frame and extract symbols
                    let frame = self.process_frame(frame_data_len);

                    if let Some(frame) = frame {
                        // Drain the processed frame data
                        self.buffer.drain(..frame_data_len);
                        self.frame_count += 1;
                        frames.push(frame);

                        // After the frame, we expect T_NULL samples of null symbol,
                        // then T_G samples of PRS guard, then T_U of PRS useful.
                        //
                        // welle.io reads T_NULL for the null symbol, then T_U for PRS
                        // correlation (which starts at the PRS guard).
                        //
                        // We consume ONLY T_NULL samples here so that buffer[0] is at
                        // the PRS guard interval start. find_prs_start() expects
                        // buffer[0] ≈ guard start and looks at buffer[T_G..T_G+T_U]
                        // for the PRS useful part.
                        if self.buffer.len() >= T_NULL {
                            // Apply freq correction to null samples for phase continuity
                            let mut null_samples: Vec<Complex<f32>> =
                                self.buffer[..T_NULL].to_vec();
                            self.apply_freq_correction(&mut null_samples);
                            self.buffer.drain(..T_NULL);
                            // Go directly to PRS correlation (no null re-detection)
                            self.state = SyncState::NeedPrsCorrelation;
                        } else {
                            // Not enough data for null consumption yet — wait for more
                            self.state = SyncState::NeedPrsCorrelation;
                            break;
                        }
                    } else {
                        break;
                    }
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
        // Step 1: Frequency-correct the frame data
        // ----------------------------------------------------------
        let mut corrected = self.buffer[..frame_data_len].to_vec();
        self.apply_freq_correction(&mut corrected);

        // ----------------------------------------------------------
        // Step 2: Extract raw data symbols (with guard) for fine freq estimation
        // ----------------------------------------------------------
        // welle.io reads each data symbol as T_S samples and accumulates
        // CP correlation across all 75 symbols.
        let mut data_symbol_bufs: Vec<Vec<Complex<f32>>> = Vec::with_capacity(L_SYMBOLS - 1);
        for i in 1..L_SYMBOLS {
            let sym_start = T_U + (i - 1) * T_S;
            let sym_end = sym_start + T_S;
            if sym_end > corrected.len() {
                break;
            }
            data_symbol_bufs.push(corrected[sym_start..sym_end].to_vec());
        }

        let fine_freq = Self::estimate_fine_freq_from_symbols(&data_symbol_bufs);
        // welle.io ALWAYS uses 0.1 gain: fineCorrector += 0.1 * arg(FreqCorr) / PI * (carrierDiff / 2)
        // This makes fine frequency converge gradually over ~20 frames.
        // Each frame's residual error is within the cyclic prefix tolerance.
        self.fine_freq_hz += 0.1 * fine_freq;

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

        if best_ratio < 4.0 {
            let discard = self.buffer.len().min(T_F / 2);
            self.buffer.drain(..discard);
            return false;
        }

        if best_pos >= self.buffer.len() {
            return false;
        }
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
