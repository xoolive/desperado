//! RDS DSP Processing Chain
//!
//! This module implements the DSP components for RDS decoding:
//! - Stereo decoder with PLL locked to 19 kHz pilot
//! - Complex I/Q resampler with pilot-coherent mixing
//! - RDS decoder with carrier PLL, AGC, symbol sync, and biphase decoding
//!
//! References:
//! - redsea (<https://github.com/windytan/redsea>)
//! - liquid-dsp (<https://github.com/jgaeddert/liquid-dsp>)

use super::parser::{RdsGroupJson, RdsParser};
use desperado::dsp::filters::StatefulLowPassFir;
use desperado::dsp::{agc::Agc, nco::Nco, symsync::SymSync};
use rubato::{
    Resampler, SincFixedOut, SincInterpolationParameters, SincInterpolationType, WindowFunction,
};
use std::f32::consts::PI;
use tracing::{debug, info, trace, warn};

/// Stereo decoder using a complex PLL locked to the 19 kHz pilot tone.
pub struct StereoDecoderPLL {
    sample_rate: f32,
    pll_phase: f64,
    pll_freq: f64,
    nominal_freq: f64,
    kp: f64,
    ki: f64,
    error_lpf_state: f64,
    error_lpf_alpha: f64,
    pilot_hi: StatefulLowPassFir,
    pilot_lo: StatefulLowPassFir,
    diff_lowpass: StatefulLowPassFir,
}

impl StereoDecoderPLL {
    pub fn new(sample_rate: f32) -> Self {
        let nominal = 19_000.0_f64;
        let fc = 50.0;
        let error_lpf_alpha = 2.0 * std::f64::consts::PI * fc
            / (sample_rate as f64 + 2.0 * std::f64::consts::PI * fc);

        Self {
            sample_rate,
            pll_phase: 0.0,
            pll_freq: nominal,
            nominal_freq: nominal,
            kp: 0.15,
            ki: 0.0005,
            error_lpf_state: 0.0,
            error_lpf_alpha,
            pilot_hi: StatefulLowPassFir::new(19_700.0, sample_rate, 257),
            pilot_lo: StatefulLowPassFir::new(18_300.0, sample_rate, 257),
            diff_lowpass: StatefulLowPassFir::new(15_000.0, sample_rate, 257),
        }
    }

    pub fn process(&mut self, input: &[f32]) -> (Vec<f32>, Vec<f32>, Vec<f64>) {
        let n = input.len();
        let pilot_hi = self.pilot_hi.process(input);
        let pilot_lo = self.pilot_lo.process(input);
        let mut pilot_band = Vec::with_capacity(n);
        for i in 0..n {
            pilot_band.push(pilot_hi[i] - pilot_lo[i]);
        }

        let mut lmr_raw = Vec::with_capacity(n);
        let mut pilot_phases = Vec::with_capacity(n);
        let phase_inc = 2.0 * std::f64::consts::PI / self.sample_rate as f64;

        for &p in &pilot_band {
            let p = p as f64;
            let raw_error = p * self.pll_phase.cos();
            self.error_lpf_state += self.error_lpf_alpha * (raw_error - self.error_lpf_state);
            let error = self.error_lpf_state;

            self.pll_freq += self.ki * error;
            self.pll_freq = self
                .pll_freq
                .clamp(self.nominal_freq * 0.99, self.nominal_freq * 1.01);

            self.pll_phase += phase_inc * self.pll_freq + self.kp * error;
            while self.pll_phase > std::f64::consts::PI {
                self.pll_phase -= 2.0 * std::f64::consts::PI;
            }
            while self.pll_phase < -std::f64::consts::PI {
                self.pll_phase += 2.0 * std::f64::consts::PI;
            }

            pilot_phases.push(self.pll_phase);
            let carrier_38 = (2.0 * self.pll_phase).sin();
            lmr_raw.push(input[lmr_raw.len()] as f64 * carrier_38);
        }

        let lmr_f32: Vec<f32> = lmr_raw.iter().map(|&v| v as f32).collect();
        let lmr = self.diff_lowpass.process(&lmr_f32);

        let mut left = Vec::with_capacity(n);
        let mut right = Vec::with_capacity(n);
        for i in 0..n {
            let lpr = input[i];
            let lmr_val = lmr[i];
            left.push(0.5 * (lpr + lmr_val));
            right.push(0.5 * (lpr - lmr_val));
        }
        (left, right, pilot_phases)
    }

    /// Get the current PLL phase (for 19 kHz pilot).
    /// Multiply by 3 to get phase for 57 kHz RDS carrier.
    pub fn pilot_phase(&self) -> f64 {
        self.pll_phase
    }

    /// Get the current PLL frequency (should be ~19 kHz when locked).
    pub fn pilot_freq(&self) -> f64 {
        self.pll_freq
    }
}

/// Stateful FIR filter with ring buffer for sample-by-sample processing
struct StatefulFir {
    coeffs: Vec<f32>,
    buffer: Vec<f32>,
    write_pos: usize,
}

impl StatefulFir {
    fn new(cutoff_freq: f32, sample_rate: f32, taps: usize) -> Self {
        // Design windowed-sinc filter (same as LowPassFir)
        let mut coeffs = Vec::with_capacity(taps);
        let mid = (taps / 2) as isize;
        let norm_cutoff = cutoff_freq / sample_rate;

        for n in 0..taps {
            let x = n as isize - mid;
            let sinc = if x == 0 {
                2.0 * norm_cutoff
            } else {
                (2.0 * norm_cutoff * PI * x as f32).sin() / (PI * x as f32)
            };
            // Blackman window
            let window = 0.42 - 0.5 * ((2.0 * PI * n as f32) / (taps as f32 - 1.0)).cos()
                + 0.08 * ((4.0 * PI * n as f32) / (taps as f32 - 1.0)).cos();
            coeffs.push(sinc * window);
        }

        // Normalize
        let norm: f32 = coeffs.iter().sum();
        for v in coeffs.iter_mut() {
            *v /= norm;
        }

        Self {
            coeffs,
            buffer: vec![0.0; taps],
            write_pos: 0,
        }
    }

    fn push(&mut self, sample: f32) -> f32 {
        // Add sample to ring buffer
        self.buffer[self.write_pos] = sample;
        self.write_pos = (self.write_pos + 1) % self.buffer.len();

        // Compute filtered output (convolution)
        let mut acc = 0.0_f32;
        let len = self.coeffs.len();
        for i in 0..len {
            let buf_idx = (self.write_pos + len - 1 - i) % len;
            acc += self.buffer[buf_idx] * self.coeffs[i];
        }
        acc
    }
}

// Note: RrcFilter and PolyphaseSymSync structs have been removed.
// Now using desperado::dsp::symsync::SymSync instead.

/// RDS resampler: input rate → output rate
/// Default: 250 kHz → 171 kHz (exactly 3 samples per RDS symbol after decimation by 24)
/// This matches redsea's architecture: 171000 / 24 = 7125 Hz, 7125 / 2375 = 3.0 exactly
/// Now also does coherent mixing down using pilot-derived 57 kHz carrier
pub struct RdsResamplerCustom {
    resampler_i: SincFixedOut<f32>,
    resampler_q: SincFixedOut<f32>,
    leftover_i: Vec<f32>,
    leftover_q: Vec<f32>,
    input_rate: f32,
}

impl RdsResamplerCustom {
    pub fn new(input_rate: f32, output_rate: f32) -> Self {
        let ratio = output_rate as f64 / input_rate as f64;

        let params_i = SincInterpolationParameters {
            sinc_len: 128, // Good quality for this ratio
            f_cutoff: 0.9, // Cutoff just below Nyquist
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 128,
            window: WindowFunction::BlackmanHarris2,
        };

        let params_q = SincInterpolationParameters {
            sinc_len: 128,
            f_cutoff: 0.9,
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 128,
            window: WindowFunction::BlackmanHarris2,
        };

        // Output 1024 frames at a time (1 channel each for I and Q)
        let resampler_i = SincFixedOut::<f32>::new(ratio, 1.1, params_i, 1024, 1).unwrap();
        let resampler_q = SincFixedOut::<f32>::new(ratio, 1.1, params_q, 1024, 1).unwrap();

        debug!(
            "[RDS-RESAMP] Created {}kHz → {}kHz complex resampler (ratio: {:.4})",
            input_rate / 1000.0,
            output_rate / 1000.0,
            ratio
        );

        Self {
            resampler_i,
            resampler_q,
            leftover_i: Vec::new(),
            leftover_q: Vec::new(),
            input_rate,
        }
    }

    /// Process MPX signal with pilot phases to produce coherent baseband I/Q
    /// pilot_phases: phase of 19 kHz pilot at each sample (multiply by 3 for 57 kHz)
    pub fn process_with_pilot(
        &mut self,
        input: &[f32],
        pilot_phases: &[f64],
    ) -> (Vec<f32>, Vec<f32>) {
        // Mix down to baseband using pilot-derived 57 kHz carrier
        // 57 kHz = 3 × 19 kHz, so phase_57 = 3 × phase_19
        // The RDS carrier may have a phase offset relative to the pilot
        // Try adding 90° (π/2) to compensate
        let mut i_mixed = Vec::with_capacity(input.len());
        let mut q_mixed = Vec::with_capacity(input.len());

        // Debug: print pilot phase statistics once
        static DEBUG_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let count = DEBUG_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if count == 100 && pilot_phases.len() >= 10 {
            // Check pilot phase rate of change using average over several samples to avoid wrap issues
            let mut total_delta = 0.0;
            let mut valid_deltas = 0;
            for i in 1..pilot_phases.len().min(100) {
                let mut delta = pilot_phases[i] - pilot_phases[i - 1];
                // Handle phase wrapping
                if delta > std::f64::consts::PI {
                    delta -= 2.0 * std::f64::consts::PI;
                } else if delta < -std::f64::consts::PI {
                    delta += 2.0 * std::f64::consts::PI;
                }
                total_delta += delta;
                valid_deltas += 1;
            }
            let avg_delta = if valid_deltas > 0 {
                total_delta / valid_deltas as f64
            } else {
                0.0
            };
            let measured_freq = avg_delta * self.input_rate as f64 / (2.0 * std::f64::consts::PI);
            let expected_delta = 2.0 * std::f64::consts::PI * 19000.0 / self.input_rate as f64;
            trace!(
                "[RDS-PILOT-DBG] avg_delta: {:.5}, expected: {:.5}, measured_freq: {:.1} Hz",
                avg_delta, expected_delta, measured_freq
            );
        }

        for (idx, &x) in input.iter().enumerate() {
            let pilot_phase = pilot_phases.get(idx).copied().unwrap_or(0.0);
            // The stereo decoder PLL locks sin(pll_phase) to the pilot
            // For 57 kHz, we need to adjust phase relationship
            // Try using sin() for I instead of cos() to match the stereo carrier convention
            let phase_57 = 3.0 * pilot_phase; // 57 kHz = 3 × 19 kHz pilot

            // Mix down: the RDS carrier is sin(3*pilot_phase), so to put data on I:
            //   I = input * sin(3φ) → demodulates the in-phase (data) component
            //   Q = input * cos(3φ) → quadrature component
            let sin_val = phase_57.sin() as f32;
            let cos_val = phase_57.cos() as f32;

            i_mixed.push(x * sin_val);
            q_mixed.push(x * cos_val);
        }

        // Add to leftovers
        self.leftover_i.extend_from_slice(&i_mixed);
        self.leftover_q.extend_from_slice(&q_mixed);

        let mut output_i = Vec::new();
        let mut output_q = Vec::new();

        loop {
            let input_frames_needed = self.resampler_i.input_frames_next();

            if self.leftover_i.len() < input_frames_needed {
                break;
            }

            // Take exactly what we need for I
            let input_chunk_i: Vec<Vec<f32>> =
                vec![self.leftover_i[..input_frames_needed].to_vec()];
            let input_chunk_q: Vec<Vec<f32>> =
                vec![self.leftover_q[..input_frames_needed].to_vec()];
            self.leftover_i.drain(0..input_frames_needed);
            self.leftover_q.drain(0..input_frames_needed);

            // Process I
            match self.resampler_i.process(&input_chunk_i, None) {
                Ok(resampled) => {
                    if !resampled.is_empty() {
                        output_i.extend_from_slice(&resampled[0]);
                    }
                }
                Err(e) => {
                    warn!("[RDS-RESAMP] Error I: {:?}", e);
                    break;
                }
            }

            // Process Q
            match self.resampler_q.process(&input_chunk_q, None) {
                Ok(resampled) => {
                    if !resampled.is_empty() {
                        output_q.extend_from_slice(&resampled[0]);
                    }
                }
                Err(e) => {
                    warn!("[RDS-RESAMP] Error Q: {:?}", e);
                    break;
                }
            }
        }

        (output_i, output_q)
    }

    /// Simple real-only resampling (for use with independent NCO approach)
    #[allow(dead_code)]
    fn process_real(&mut self, input: &[f32]) -> Vec<f32> {
        // Use I resampler for real signal
        self.leftover_i.extend_from_slice(input);

        let mut output = Vec::new();

        loop {
            let input_frames_needed = self.resampler_i.input_frames_next();

            if self.leftover_i.len() < input_frames_needed {
                break;
            }

            let input_chunk: Vec<Vec<f32>> = vec![self.leftover_i[..input_frames_needed].to_vec()];
            self.leftover_i.drain(0..input_frames_needed);

            match self.resampler_i.process(&input_chunk, None) {
                Ok(resampled) => {
                    if !resampled.is_empty() {
                        output.extend_from_slice(&resampled[0]);
                    }
                }
                Err(e) => {
                    warn!("[RDS-RESAMP] Error: {:?}", e);
                    break;
                }
            }
        }

        output
    }
}

/// RDS decoder with NCO mixer, PLL, and proper biphase decoding (matching redsea architecture)
pub struct RdsDecoder {
    // NCO with integrated PLL (from desperado::dsp::nco)
    nco: Nco,

    // Lowpass filter for baseband (complex I/Q)
    lpf_i: StatefulFir,
    lpf_q: StatefulFir,

    // AGC (from desperado::dsp::agc)
    agc: Agc,

    // Polyphase symbol synchronizer (from desperado::dsp::symsync)
    symsync: SymSync,

    // Decimation
    decimate_ratio: usize,
    decimate_counter: usize,

    // Biphase decoder state
    prev_psk_symbol: (f32, f32),
    biphase_clock: usize,
    biphase_polarity: usize,
    biphase_history: Vec<f32>,
    prev_decoded_bit: bool, // For differential decoding

    // Bit output
    bits: Vec<u8>,
    rds_parser: RdsParser,

    debug_counter: u64,

    // PLL acquisition state
    pll_locked: bool,        // True when PLL has acquired lock
    pll_lock_counter: usize, // Count consecutive low phase errors

    print_json_output: bool,
    json_output_cache: Vec<RdsGroupJson>,
}

impl RdsDecoder {
    /// Create RDS decoder expecting 171 kHz input (from RdsResampler)
    /// This matches redsea's architecture exactly:
    /// - 171 kHz input rate
    /// - Decimate by 24 → 7125 Hz
    /// - 7125 / 2375 = 3.0 samples per symbol exactly
    pub fn new(_sample_rate: f32, verbose: bool) -> Self {
        let mut rds_parser = RdsParser::new();
        rds_parser.set_verbose(verbose);
        rds_parser.set_json_mode(true);

        // Fixed parameters matching redsea exactly
        let sample_rate = 171_000.0_f32; // We now expect 171 kHz input (from resampler)
        let decimate_ratio = 24_usize; // Fixed: 171000 / 24 = 7125 Hz
        let decimated_rate = 7125.0_f32; // Exactly 3 samples per 2375 Hz symbol
        let psk_symbol_rate = 2375.0_f32;

        debug!(
            "[RDS-DSP] Sample rate: {} Hz, Decimate ratio: {}, Decimated rate: {} Hz",
            sample_rate, decimate_ratio, decimated_rate
        );
        debug!(
            "[RDS-DSP] Samples per PSK symbol: {:.2}, PSK rate: {:.1} Hz",
            decimated_rate / psk_symbol_rate,
            psk_symbol_rate
        );

        // Lowpass at 2400 Hz (matching redsea)
        let lpf_cutoff = 2400.0;
        // Use 255 taps for good selectivity
        let lpf_taps = 255;

        // NCO for fine phase tracking (IQ path: 57 kHz already removed by pilot-coherent mixing)
        // In IQ mode, we only need to track residual phase, so frequency is 0
        // The NCO runs at the DECIMATED rate (7125 Hz) for phase rotation at sample level
        let decimated_rate = 7125.0_f64;
        let mut nco = Nco::new(0.0, decimated_rate); // Zero frequency for IQ path
        // PLL bandwidth: coefficients must match the PLL UPDATE rate (symbol rate ~2375 Hz),
        // since pll_step() is called once per symbol, not once per decimated sample.
        let symbol_rate = 2375.0_f64;
        nco.set_pll_bandwidth(2.0, symbol_rate); // 2 Hz acquisition bandwidth at symbol rate

        debug!(
            "[RDS-DSP] NCO: freq=0 Hz (IQ path), PLL bandwidth=2Hz at symbol rate {}",
            symbol_rate
        );

        // AGC with bandwidth matching redsea (~500 Hz at 171 kHz = 0.003)
        // redsea: agc.init(kAGCBandwidth_Hz / kTargetSampleRate_Hz, kAGCInitialGain)
        // kAGCBandwidth_Hz = 500.0, so 500/171000 ≈ 0.003
        // kAGCInitialGain = 0.08
        let mut agc = Agc::new(0.003);
        agc.set_gain(0.08); // Match redsea initial gain
        debug!("[RDS-DSP] AGC: bandwidth=0.003, initial_gain=0.08");

        // Polyphase symbol synchronizer (matching redsea's liquid-dsp symsync)
        // redsea: symsync.init(LIQUID_FIRFILT_RRC, kSamplesPerSymbol, kSymsyncDelay, kSymsyncBeta, 32)
        //         symsync.setBandwidth(kSymsyncBandwidth_Hz / kTargetSampleRate_Hz)
        // kSymsyncBandwidth_Hz = 2200.0, so 2200/171000 ≈ 0.013
        let symsync = SymSync::new_rnyquist(3, 32, 3, 0.8, 0.013);
        debug!("[RDS-DSP] SymSync: k=3, npfb=32, m=3, beta=0.8, bandwidth=0.013");

        Self {
            nco,
            lpf_i: StatefulFir::new(lpf_cutoff, sample_rate, lpf_taps),
            lpf_q: StatefulFir::new(lpf_cutoff, sample_rate, lpf_taps),
            agc,
            symsync,
            decimate_ratio,
            decimate_counter: 0,
            prev_psk_symbol: (0.0, 0.0),
            biphase_clock: 0,
            biphase_polarity: 0,
            biphase_history: vec![0.0; 128],
            prev_decoded_bit: false,
            bits: Vec::new(),
            rds_parser,
            debug_counter: 0,
            pll_locked: false,
            pll_lock_counter: 0,
            print_json_output: true,
            json_output_cache: Vec::new(),
        }
    }

    pub fn process(&mut self, input: &[f32]) {
        // Debug: check input signal statistics
        if self.debug_counter == 0 {
            let input_rms: f32 =
                (input.iter().map(|x| x * x).sum::<f32>() / input.len() as f32).sqrt();
            debug!(
                "[RDS-DSP] Input RMS: {:.4}, Input len: {}",
                input_rms,
                input.len()
            );
        }

        // Process each sample through the DSP chain with Costas loop PLL

        for &x in input {
            // Step 1: Mix down to baseband using NCO
            let (i_mixed, q_mixed) = self.nco.mix_down(x);

            // Step 2: Lowpass filter (applied at full rate, before decimation)
            let i_filt = self.lpf_i.push(i_mixed);
            let q_filt = self.lpf_q.push(q_mixed);

            // Step 3: Advance NCO phase
            self.nco.step();

            // Step 4: Decimation - only process every N samples
            self.decimate_counter += 1;
            if self.decimate_counter < self.decimate_ratio {
                continue;
            }
            self.decimate_counter = 0;

            // Step 5: AGC (using new AGC module)
            let (i_agc, q_agc) = self.agc.execute(i_filt, q_filt);

            // Step 6: Polyphase symbol synchronizer
            let symbol_opt = self.symsync.push(i_agc, q_agc);

            // Only process when we have a symbol output from symsync
            let psk_symbol = match symbol_opt {
                Some(sym) => sym,
                None => continue,
            };

            // Carrier PLL: Use liquid-dsp BPSK modem phase error calculation
            // liquid-dsp formula: phase_error = imag(r * conj(x_hat))
            // For BPSK: x_hat = +1 or -1, so phase_error = Q * sign(I)
            let (si, sq) = (psk_symbol.0 as f64, psk_symbol.1 as f64);
            let sym_mag = (si * si + sq * sq).sqrt();

            // Lower threshold to 0.001 - any signal is useful for phase tracking
            if sym_mag > 0.001 {
                // Compute phase error as angle from nearest BPSK constellation point (0° or 180°)
                // For symbol at angle θ:
                //   If I >= 0: it's a +1 symbol, error = θ - 0° = θ
                //   If I < 0: it's a -1 symbol, error = θ - 180° (wrapped to ±π)
                let symbol_phase = sq.atan2(si); // Phase in radians
                let phase_error_rad = if si >= 0.0 {
                    // Should be at 0°
                    symbol_phase
                } else {
                    // Should be at 180° (π radians)
                    // Wrap to [-π, π]
                    let err = symbol_phase - std::f64::consts::PI;
                    if err < -std::f64::consts::PI {
                        err + 2.0 * std::f64::consts::PI
                    } else if err > std::f64::consts::PI {
                        err - 2.0 * std::f64::consts::PI
                    } else {
                        err
                    }
                };

                // PLL multiplier: Reduced gain since phase error is now accurate
                let scaled_error = phase_error_rad * 0.5;
                self.nco.pll_step(scaled_error);

                // Lock detection: if phase error is consistently small
                let phase_error_deg = phase_error_rad.to_degrees();
                if phase_error_deg.abs() < 20.0 {
                    self.pll_lock_counter += 1;
                    if self.pll_lock_counter > 500 && !self.pll_locked {
                        self.pll_locked = true;
                        // Narrow bandwidth once locked for better tracking
                        // PLL updates at symbol rate (~2375 Hz), not decimated rate
                        let symbol_rate = 2375.0_f64;
                        self.nco.set_pll_bandwidth(0.5, symbol_rate);
                        debug!(
                            "[RDS-PLL] Locked! Narrowing bandwidth to 0.5 Hz. Phase error: {:.1}°, freq: {:.2} Hz",
                            phase_error_deg,
                            self.nco.get_frequency_hz(7125.0_f64)
                        );
                    }
                } else {
                    if self.pll_lock_counter > 0 {
                        self.pll_lock_counter -= 1;
                    }
                    if self.pll_lock_counter == 0 && self.pll_locked {
                        self.pll_locked = false;
                        // Widen bandwidth for re-acquisition
                        // PLL updates at symbol rate (~2375 Hz)
                        let symbol_rate = 2375.0_f64;
                        self.nco.set_pll_bandwidth(10.0, symbol_rate);
                        debug!(
                            "[RDS-PLL] Lost lock, widening bandwidth to 100 Hz. Phase error: {:.1}°",
                            phase_error_deg
                        );
                    }
                }
            }

            self.debug_counter += 1;

            // Biphase (Manchester) decoding using subtraction at mid-bit
            // (matching redsea's BiphaseDecoder::push)
            let (pi, _pq) = self.prev_psk_symbol;
            let (ci, _cq) = psk_symbol;
            let biphase_i = (ci - pi) * 0.5;

            // Clock polarity detection uses abs(real) only (redsea convention)
            let biphase_mag = biphase_i.abs();
            self.biphase_history[self.biphase_clock] = biphase_mag;

            // Biphase bit decision from sign of real component
            let biphase_bit = biphase_i >= 0.0;

            // Output at mid-bit positions
            if self.biphase_clock % 2 == self.biphase_polarity {
                // Differential decode: RDS encoding order is source→CRC→differential→biphase
                // So decoder must do: biphase→differential→CRC
                let decoded_bit = biphase_bit != self.prev_decoded_bit;
                self.prev_decoded_bit = biphase_bit;
                self.bits.push(if decoded_bit { 1 } else { 0 });
            }

            self.biphase_clock += 1;

            // Every 128 symbols (~54 ms), check clock polarity (matching redsea)
            if self.biphase_clock >= 128 {
                let mut even_sum = 0.0_f32;
                let mut odd_sum = 0.0_f32;
                for i in 0..64 {
                    even_sum += self.biphase_history[i * 2];
                    odd_sum += self.biphase_history[i * 2 + 1];
                }
                if even_sum > odd_sum {
                    self.biphase_polarity = 0;
                } else if odd_sum > even_sum {
                    self.biphase_polarity = 1;
                }
                // Reset history (matching redsea)
                self.biphase_history.fill(0.0);
                self.biphase_clock = 0;
            }

            self.prev_psk_symbol = psk_symbol;
        }

        // Feed to RDS parser
        self.process_bits();
    }

    /// Process pre-mixed complex I/Q baseband (already at 171 kHz, coherent with pilot)
    /// This skips the NCO mixing and uses the pilot-derived 57 kHz carrier
    pub fn process_iq(&mut self, input_i: &[f32], input_q: &[f32]) {
        if input_i.is_empty() {
            return;
        }

        // Process each sample through the DSP chain
        // Mixing already done by caller using pilot-derived carrier
        for idx in 0..input_i.len() {
            let i_in = input_i[idx];
            let q_in = input_q[idx];

            // Step 1: Lowpass filter (at 171 kHz rate)
            let i_filt = self.lpf_i.push(i_in);
            let q_filt = self.lpf_q.push(q_in);

            // Step 2: Decimation - only process every N samples
            self.decimate_counter += 1;
            if self.decimate_counter < self.decimate_ratio {
                continue;
            }
            self.decimate_counter = 0;

            // Step 3: AGC
            let (i_agc, q_agc) = self.agc.execute(i_filt, q_filt);

            // Step 4: Fine carrier phase tracking
            // The pilot-derived 57 kHz gives us frequency lock, but we need to track the phase
            // Complex rotation: (I + jQ) * e^(-j*theta)
            let (sin_rot, cos_rot) = self.nco.sincos();
            let i_rot = i_agc * cos_rot + q_agc * sin_rot;
            let q_rot = q_agc * cos_rot - i_agc * sin_rot;

            // Step carrier NCO (at decimated rate, 7125 Hz)
            self.nco.step();

            // Step 5: Polyphase symbol synchronizer
            let symbol_opt = self.symsync.push(i_rot, q_rot);

            // Only process when we have a symbol
            if let Some(psk_symbol) = symbol_opt {
                let psk_symbol = (psk_symbol.0, psk_symbol.1);

                // Update carrier PLL based on symbol phase error
                // For BPSK: phase_error = Q * sign(I) ≈ sin(θ) for small angles
                let mag_sq = psk_symbol.0 * psk_symbol.0 + psk_symbol.1 * psk_symbol.1;
                if mag_sq > 0.01 {
                    let phase_error = psk_symbol.1 * psk_symbol.0.signum();
                    let phase_error_rad = (phase_error / mag_sq.sqrt()) as f64 * 0.5;
                    self.nco.pll_step(phase_error_rad);
                }

                self.debug_counter += 1;
                if self.debug_counter.is_multiple_of(5000) {
                    let mag = (psk_symbol.0 * psk_symbol.0 + psk_symbol.1 * psk_symbol.1).sqrt();
                    let sym_phase = psk_symbol.1.atan2(psk_symbol.0).to_degrees();
                    debug!(
                        "[RDS-DSP-IQ] AGC: {:.2}, I: {:.3}, Q: {:.3}, mag: {:.3}, sym_ph: {:.1}°, nco_f: {:.4} Hz",
                        self.agc.get_gain(),
                        psk_symbol.0,
                        psk_symbol.1,
                        mag,
                        sym_phase,
                        self.nco.get_frequency_hz(7125.0)
                    );
                }

                // Biphase (Manchester) decoding using subtraction at mid-bit
                // (matching redsea's BiphaseDecoder::push)
                let (pi, _pq) = self.prev_psk_symbol;
                let (ci, _cq) = psk_symbol;
                let biphase_i = (ci - pi) * 0.5;

                // Clock polarity detection uses abs(real) only (redsea convention)
                let biphase_mag = biphase_i.abs();
                self.biphase_history[self.biphase_clock] = biphase_mag;

                // Biphase bit decision from sign of real component
                let biphase_bit = biphase_i >= 0.0;

                // Output at mid-bit positions
                if self.biphase_clock % 2 == self.biphase_polarity {
                    // Differential decode: RDS encoding order is source→CRC→differential→biphase
                    // So decoder must do: biphase→differential→CRC
                    let decoded_bit = biphase_bit != self.prev_decoded_bit;
                    self.prev_decoded_bit = biphase_bit;
                    self.bits.push(if decoded_bit { 1 } else { 0 });
                }

                self.biphase_clock += 1;

                // Every 128 symbols (~54 ms), check clock polarity (matching redsea)
                if self.biphase_clock >= 128 {
                    let mut even_sum = 0.0_f32;
                    let mut odd_sum = 0.0_f32;
                    for i in 0..64 {
                        even_sum += self.biphase_history[i * 2];
                        odd_sum += self.biphase_history[i * 2 + 1];
                    }
                    if even_sum > odd_sum {
                        self.biphase_polarity = 0;
                    } else if odd_sum > even_sum {
                        self.biphase_polarity = 1;
                    }
                    // Reset history (matching redsea)
                    self.biphase_history.fill(0.0);
                    self.biphase_clock = 0;
                }

                self.prev_psk_symbol = psk_symbol;
            }
        }

        // Feed to RDS parser
        self.process_bits();
    }

    pub fn take_json_outputs(&mut self) -> Vec<RdsGroupJson> {
        let mut out = Vec::new();
        out.append(&mut self.json_output_cache);
        out.append(&mut self.rds_parser.take_json_outputs());
        out
    }

    pub fn set_print_json_output(&mut self, enabled: bool) {
        self.print_json_output = enabled;
    }

    pub fn clock_time(&self) -> Option<super::ClockTimeInfo> {
        self.rds_parser.station_info().clock_time.clone()
    }

    pub fn program_id(&self) -> u16 {
        self.rds_parser.program_id()
    }

    pub fn program_type(&self) -> String {
        self.rds_parser.program_type_name().to_string()
    }

    pub fn language(&self) -> Option<String> {
        self.rds_parser.language_name().map(|s| s.to_string())
    }

    pub fn station_name(&self) -> Option<String> {
        self.rds_parser.station_name()
    }

    pub fn radio_text(&self) -> Option<String> {
        self.rds_parser.radio_text()
    }

    pub fn stats(&self) -> (u64, u32, u32, u32) {
        self.rds_parser.stats()
    }

    /// Get the first alternative frequency (if available) in Hz
    /// Returns None if no valid alternative frequencies are decoded yet
    pub fn alt_frequency(&self) -> Option<u32> {
        let af_list = &self.rds_parser.station_info().af_list;
        // Valid AF frequencies are 1-204 (in 10kHz units)
        // 225-249 are special codes, 0 is filler
        af_list
            .iter()
            .find(|&&f| (1..=204).contains(&f))
            .map(|&f| u32::from(f) * 10_000) // Convert to Hz
    }

    /// Return all alternative frequencies as a Vec<u32> in Hz
    pub fn alt_frequencies(&self) -> Vec<u32> {
        self.rds_parser
            .station_info()
            .af_list
            .iter()
            .map(|&f| u32::from(f) * 10_000) // Convert to Hz
            .collect()
    }

    /// Process accumulated bits through the RDS parser and display results
    fn process_bits(&mut self) {
        if self.bits.len() >= 104 {
            self.rds_parser.push_bits(&self.bits);

            let outputs = self.rds_parser.take_json_outputs();
            if self.print_json_output {
                for json_out in outputs {
                    if let Ok(json_str) = serde_json::to_string(&json_out) {
                        info!(target: "fmradio::rds_json", "{}", json_str);
                    }
                }
            } else {
                self.json_output_cache.extend(outputs);
            }

            self.bits.clear();
        }
    }
}
