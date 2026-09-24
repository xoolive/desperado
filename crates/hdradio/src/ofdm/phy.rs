//! FM-hybrid NRSC-5 OFDM acquisition and symbol extraction.
//!
//! This module deliberately contains only physical-layer mechanics: cyclic
//! prefix timing, carrier-frequency estimation, derotation, and 2048-point
//! FFTs. Carrier partitioning and FEC live in later transport modules.

use std::f32::consts::PI;
use std::sync::Arc;

use num_complex::Complex;
use rustfft::{Fft, FftPlanner};

/// NRSC-5 FM complex sample rate.
pub const SAMPLE_RATE_HZ: u32 = 744_188;
/// Number of time-domain samples in an FM OFDM useful symbol.
pub const FFT_LEN: usize = 2_048;
/// Cyclic-prefix length in samples.
pub const CP_LEN: usize = 112;
/// Complete OFDM symbol duration in samples.
pub const SYMBOL_LEN: usize = FFT_LEN + CP_LEN;
/// Symbols in an NRSC-5 logical-channel block.
pub const BLOCK_SYMBOLS: usize = 32;

/// Frequency-domain symbols extracted from one synchronized OFDM block.
#[derive(Debug, Clone)]
pub struct OfdmBlock {
    /// FFT-shifted, CFO-corrected frequency-domain symbols.
    pub symbols: Vec<Vec<Complex<f32>>>,
    /// CP-derived carrier-frequency offset in Hz.
    pub frequency_offset_hz: f32,
    /// Timing position selected within the incoming buffer.
    pub timing_offset_samples: usize,
    /// Peak-to-mean CP correlation power ratio.
    pub timing_metric: f32,
}

/// Last observed acquisition metrics.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PhyMetrics {
    /// Number of output blocks since construction/reset.
    pub locked_blocks: u64,
    /// CP-derived residual frequency offset in Hz.
    pub frequency_offset_hz: f32,
    /// Peak-to-mean correlation ratio for the latest block.
    pub timing_metric: f32,
}

/// Streaming CP synchronizer and OFDM FFT front end.
pub struct OfdmReceiver {
    fft: Arc<dyn Fft<f32>>,
    buffer: Vec<Complex<f32>>,
    pulse_shape: Vec<f32>,
    fine_sync: bool,
    correction_phase: f32,
    previous_angle: f32,
    feedback_sampling_error: i32,
    feedback_angle: f32,
    keep_extra: usize,
    pending_timing: Option<usize>,
    metrics: PhyMetrics,
}

impl OfdmReceiver {
    /// Construct a receiver in the unlocked state.
    pub fn new() -> Self {
        let mut planner = FftPlanner::new();
        Self {
            fft: planner.plan_fft_forward(FFT_LEN),
            buffer: Vec::with_capacity((BLOCK_SYMBOLS * 3) * SYMBOL_LEN),
            pulse_shape: pulse_shape(),
            fine_sync: false,
            correction_phase: 0.0,
            previous_angle: 0.0,
            feedback_sampling_error: 0,
            feedback_angle: 0.0,
            keep_extra: 0,
            pending_timing: None,
            metrics: PhyMetrics::default(),
        }
    }

    /// Consume input and return every complete, synchronized block currently
    /// buffered. The first lock searches one full OFDM-symbol period; thereafter
    /// each result leaves the buffer at the next expected block boundary.
    pub fn process(&mut self, input: &[Complex<f32>]) -> Vec<OfdmBlock> {
        const ACQUISITION_SAMPLES: usize = (BLOCK_SYMBOLS + 1) * SYMBOL_LEN;

        // The previous block's synchronization decision determines how much
        // input history is retained. Delay consumption until that decision is
        // available, as the reference acquisition state machine does.
        if let Some(timing) = self.pending_timing.take() {
            let keep = (SYMBOL_LEN as isize + (SYMBOL_LEN / 2) as isize - timing as isize
                + self.keep_extra as isize) as usize;
            self.buffer.drain(..ACQUISITION_SAMPLES - keep);
            self.keep_extra = 0;
        }

        self.buffer.extend_from_slice(input);
        if self.buffer.len() < ACQUISITION_SAMPLES {
            return Vec::new();
        }

        let (timing, angle, metric) = if self.fine_sync {
            let timing = (SYMBOL_LEN as i32 / 2 + self.feedback_sampling_error)
                .clamp(0, SYMBOL_LEN as i32 - 1) as usize;
            self.feedback_sampling_error = 0;
            let angle = self.previous_angle - self.feedback_angle;
            self.feedback_angle = 0.0;
            self.previous_angle = angle;
            (timing, angle, self.metrics.timing_metric)
        } else {
            let (timing, correlation, metric) = self.find_timing();
            let difference = wrap_phase(correlation.arg() - self.previous_angle);
            let factor = if self.previous_angle == 0.0 {
                1.0
            } else {
                0.25
            };
            let angle = self.previous_angle + difference * factor;
            self.previous_angle = angle;
            (timing, angle, metric)
        };

        let timing_adjustment = SYMBOL_LEN as f32 / 2.0 - timing as f32;
        self.correction_phase =
            wrap_phase(self.correction_phase - timing_adjustment * angle / FFT_LEN as f32);
        let phase_step = angle / FFT_LEN as f32;
        let frequency_offset_hz = angle * SAMPLE_RATE_HZ as f32 / (2.0 * PI * FFT_LEN as f32);
        let mut symbols = Vec::with_capacity(BLOCK_SYMBOLS);
        for symbol_index in 0..BLOCK_SYMBOLS {
            let start = timing + symbol_index * SYMBOL_LEN;
            let mut symbol = self.windowed_symbol(
                start,
                phase_step,
                self.correction_phase,
                symbol_index * SYMBOL_LEN,
            );
            self.fft.process(&mut symbol);
            fft_shift(&mut symbol);
            symbols.push(symbol);
        }
        self.correction_phase =
            wrap_phase(self.correction_phase + phase_step * (BLOCK_SYMBOLS * SYMBOL_LEN) as f32);
        self.pending_timing = Some(timing);
        self.metrics.locked_blocks += 1;
        self.metrics.frequency_offset_hz = frequency_offset_hz;
        self.metrics.timing_metric = metric;

        vec![OfdmBlock {
            symbols,
            frequency_offset_hz,
            timing_offset_samples: timing,
            timing_metric: metric,
        }]
    }

    /// Latest physical-layer metrics.
    pub fn metrics(&self) -> PhyMetrics {
        self.metrics
    }

    /// Shift the next acquisition output to the recovered NRSC-5 logical
    /// block boundary. This is equivalent to the reference receiver retaining
    /// `32 - block_offset` symbols after coarse synchronization.
    pub fn align_logical_block(&mut self, block_offset: u8) {
        let extra = (BLOCK_SYMBOLS - usize::from(block_offset) % BLOCK_SYMBOLS) % BLOCK_SYMBOLS;
        self.keep_extra = extra * SYMBOL_LEN;
    }

    /// Enter fine acquisition after an aligned reference block validates.
    pub fn enter_fine_sync(&mut self) {
        self.fine_sync = true;
    }

    /// Leave fine tracking after downstream frame lock is lost while preserving
    /// acquisition history, oscillator phase, and buffered future samples.
    pub fn lose_fine_sync(&mut self) {
        self.fine_sync = false;
    }

    /// Apply fine timing and carrier-loop feedback from reference carriers.
    pub fn adjust_tracking(&mut self, sampling_error: i32, common_angle: f32) {
        self.feedback_sampling_error = sampling_error;
        self.feedback_angle = common_angle;
    }

    /// Clear synchronization history after a source discontinuity.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.fine_sync = false;
        self.correction_phase = 0.0;
        self.previous_angle = 0.0;
        self.feedback_sampling_error = 0;
        self.feedback_angle = 0.0;
        self.keep_extra = 0;
        self.pending_timing = None;
        self.metrics = PhyMetrics::default();
    }

    /// Search one symbol period with the pulse-shaped prefix correlation used
    /// by the reference receiver. Correlations at the same position in all 32
    /// symbols are summed first, then integrated over the 112-sample overlap.
    fn find_timing(&self) -> (usize, Complex<f32>, f32) {
        let filtered = acquisition_filter(&self.buffer);
        let mut sums = vec![Complex::new(0.0, 0.0); SYMBOL_LEN];
        for (position, sum) in sums.iter_mut().enumerate() {
            for symbol in 0..BLOCK_SYMBOLS {
                let index = position + symbol * SYMBOL_LEN;
                *sum += filtered[index] * filtered[index + FFT_LEN].conj();
            }
        }

        let mut best_position = 0;
        let mut best_correlation = Complex::new(0.0, 0.0);
        let mut best_power = -1.0f32;
        let mut total_power = 0.0;
        for position in 0..SYMBOL_LEN {
            let mut correlation = Complex::new(0.0, 0.0);
            for overlap in 0..CP_LEN {
                correlation += sums[(position + overlap) % SYMBOL_LEN]
                    * self.pulse_shape[overlap]
                    * self.pulse_shape[overlap + FFT_LEN];
            }
            let power = correlation.norm_sqr();
            total_power += power;
            if power > best_power {
                best_power = power;
                best_position = position;
                best_correlation = correlation;
            }
        }

        // The 32-tap acquisition filter has a measured group delay of 15.
        let timing = (best_position + SYMBOL_LEN - 15) % SYMBOL_LEN;
        let mean_power = total_power / SYMBOL_LEN as f32;
        let metric = if mean_power > 0.0 {
            best_power / mean_power
        } else {
            0.0
        };
        (timing, best_correlation, metric)
    }

    /// Fold the sine/cosine overlap into a 2048-sample FFT window. NRSC-5's
    /// pulse-shaped symbol cannot be decoded by simply discarding its prefix.
    fn windowed_symbol(
        &self,
        start: usize,
        phase_step: f32,
        phase_start: f32,
        phase_sample_offset: usize,
    ) -> Vec<Complex<f32>> {
        let mut fft_input = vec![Complex::new(0.0, 0.0); FFT_LEN];
        for index in 0..SYMBOL_LEN {
            let phase = phase_start + phase_step * (phase_sample_offset + index) as f32;
            let sample = self.buffer[start + index] * Complex::new(phase.cos(), phase.sin());
            if index < CP_LEN {
                fft_input[index] = sample * self.pulse_shape[index];
            } else if index < FFT_LEN {
                fft_input[index] = sample;
            } else {
                fft_input[index - FFT_LEN] += sample * self.pulse_shape[index];
            }
        }
        fft_input
    }
}

impl Default for OfdmReceiver {
    fn default() -> Self {
        Self::new()
    }
}

fn pulse_shape() -> Vec<f32> {
    (0..SYMBOL_LEN)
        .map(|index| {
            if index < CP_LEN {
                (PI * 0.5 * index as f32 / CP_LEN as f32).sin()
            } else if index < FFT_LEN {
                1.0
            } else {
                (PI * 0.5 * (index - FFT_LEN) as f32 / CP_LEN as f32).cos()
            }
        })
        .collect()
}

fn acquisition_filter(samples: &[Complex<f32>]) -> Vec<Complex<f32>> {
    const TAPS: [f32; 32] = [
        -0.00068564323,
        0.005636965,
        0.009015782,
        -0.015486306,
        -0.035108358,
        0.017446253,
        0.08155813,
        0.007995186,
        -0.13311294,
        -0.07274229,
        0.15914097,
        0.16498782,
        -0.1324499,
        -0.24840122,
        0.05177393,
        0.28215778,
        0.05177393,
        -0.24840122,
        -0.1324499,
        0.16498782,
        0.15914097,
        -0.07274229,
        -0.13311294,
        0.007995186,
        0.08155813,
        0.017446253,
        -0.035108358,
        -0.015486306,
        0.009015782,
        0.005636965,
        -0.00068564323,
        0.0,
    ];
    let mut output = vec![Complex::new(0.0, 0.0); samples.len()];
    for index in 0..samples.len() {
        let count = TAPS.len().min(index + 1);
        for tap in 0..count {
            output[index] += samples[index - tap] * TAPS[tap];
        }
    }
    output
}

fn wrap_phase(phase: f32) -> f32 {
    (phase + PI).rem_euclid(2.0 * PI) - PI
}

fn fft_shift(samples: &mut [Complex<f32>]) {
    samples.rotate_left(samples.len() / 2);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_ofdm_stream(offset: usize, frequency_hz: f32) -> Vec<Complex<f32>> {
        let mut output = vec![Complex::new(0.13, -0.27); offset];
        for symbol_index in 0..(BLOCK_SYMBOLS + 1) {
            let useful: Vec<Complex<f32>> = (0..FFT_LEN)
                .map(|index| {
                    let mut bits = (index + symbol_index * FFT_LEN) as u32;
                    bits ^= bits << 13;
                    bits ^= bits >> 17;
                    bits ^= bits << 5;
                    match bits & 3 {
                        0 => Complex::new(1.0, 1.0),
                        1 => Complex::new(1.0, -1.0),
                        2 => Complex::new(-1.0, 1.0),
                        _ => Complex::new(-1.0, -1.0),
                    }
                })
                .collect();
            output.extend_from_slice(&useful[FFT_LEN - CP_LEN..]);
            output.extend_from_slice(&useful);
        }
        for (sample_index, sample) in output.iter_mut().enumerate() {
            let phase = 2.0 * PI * frequency_hz * sample_index as f32 / SAMPLE_RATE_HZ as f32;
            *sample *= Complex::new(phase.cos(), phase.sin());
        }
        output
    }

    #[test]
    fn acquires_cp_timing_and_frequency() {
        let timing_offset = 287;
        let frequency_hz = 125.0;
        let mut receiver = OfdmReceiver::new();
        let blocks = receiver.process(&make_ofdm_stream(timing_offset, frequency_hz));
        assert_eq!(blocks.len(), 1);
        assert_eq!(blocks[0].symbols.len(), BLOCK_SYMBOLS);
        assert!(blocks[0].timing_offset_samples.abs_diff(timing_offset) <= 1);
        assert!(
            (blocks[0].frequency_offset_hz + frequency_hz).abs() < 1.0,
            "got {} Hz",
            blocks[0].frequency_offset_hz
        );
        assert!(blocks[0].timing_metric > 1.0);
    }

    #[test]
    fn acquisition_is_independent_of_chunk_boundaries() {
        let input = make_ofdm_stream(611, -100.0);
        let mut receiver = OfdmReceiver::new();
        let mut blocks = Vec::new();
        for chunk in input.chunks(733) {
            blocks.extend(receiver.process(chunk));
        }
        assert_eq!(blocks.len(), 1);
        assert!(
            (blocks[0].frequency_offset_hz - 100.0).abs() < 1.0,
            "got {} Hz",
            blocks[0].frequency_offset_hz
        );
    }
}
