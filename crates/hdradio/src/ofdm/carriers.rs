//! FM-hybrid reference equalization and primary-main carrier mapping.

use std::f32::consts::PI;

use num_complex::Complex;

use crate::ofdm::frame_sync::FrameSync;
use crate::ofdm::phy::{BLOCK_SYMBOLS, FFT_LEN, OfdmBlock, SYMBOL_LEN};

/// Number of subcarriers in one FM partition, including its reference carrier.
pub const PARTITION_WIDTH: usize = 19;
/// Data carriers in one partition after excluding the reference carrier.
pub const DATA_CARRIERS: usize = PARTITION_WIDTH - 1;
/// Primary-main partitions per sideband.
pub const PRIMARY_MAIN_PARTITIONS: usize = 10;
/// First lower-sideband carrier in an FFT-shifted spectrum.
pub const LOWER_BAND_START: usize = FFT_LEN / 2 - 546;
/// Last upper-sideband carrier in an FFT-shifted spectrum.
pub const UPPER_BAND_END: usize = FFT_LEN / 2 + 546;

// Second-order Costas gains for loop bandwidth 0.05 and damping 1/sqrt(2).
const COSTAS_ALPHA: f32 = 0.131_774_08;
const COSTAS_BETA: f32 = 0.009_317_835;
const REFERENCE_SYNC: [i8; BLOCK_SYMBOLS] = [
    -1, 1, -1, -1, -1, 1, 1, 0, 1, -1, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, -1, 1, -1, 0, 0, 0, 0, 0, 0,
    0, 0, -1,
];

/// Signed soft QPSK decisions for one 32-symbol primary-main block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PrimaryMainSoftBits {
    /// For every symbol: all lower partitions, then all upper partitions.
    pub bits: Vec<i8>,
}

#[derive(Debug, Clone, Copy, Default)]
struct ReferenceTracker {
    phase: f32,
    frequency: f32,
}

/// Stateful reference-carrier equalizer and primary-main extractor.
#[derive(Debug)]
pub struct PrimaryMainExtractor {
    trackers: Vec<ReferenceTracker>,
    sampling_error: i32,
    common_angle: f32,
}

impl PrimaryMainExtractor {
    /// Construct an equalizer with reset Costas loops.
    pub fn new() -> Self {
        Self {
            trackers: vec![ReferenceTracker::default(); FFT_LEN],
            sampling_error: 0,
            common_angle: 0.0,
        }
    }

    /// Warm reference Costas loops on the coarse-synchronization block before
    /// the acquisition buffer is shifted to a logical block boundary.
    pub fn warm_references(
        &mut self,
        block: &OfdmBlock,
        sync: FrameSync,
        current_service_mode: u8,
    ) {
        // Coarse processing uses the service mode that was active before this
        // block; a newly decoded PSMI applies beginning with the next block.
        let partitions = partitions_per_band(current_service_mode);
        let mut symbols = block.symbols.clone();
        let mut phases = vec![[0.0f32; BLOCK_SYMBOLS]; FFT_LEN];
        self.adjust_sample_timing(sync.coarse_frequency_bins, block.timing_offset_samples);
        // Normal coarse processing corrects nominal references once before the
        // integer-CFO scan.
        for partition in 0..=partitions {
            for nominal_bin in [
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                UPPER_BAND_END - partition * PARTITION_WIDTH,
            ] {
                let bin = shifted_bin(nominal_bin, sync.coarse_frequency_bins);
                self.correct_reference(
                    &mut symbols,
                    &mut phases[bin],
                    bin,
                    sync.coarse_frequency_bins,
                );
            }
        }

        // The integer-CFO search is stateful: every tested candidate advances
        // the Costas loop at its candidate reference bins, then restores only
        // the corrected samples. Preserve that behavior through the winning
        // candidate so the following aligned block starts with identical loop
        // history.
        for candidate in -(2 * PARTITION_WIDTH as i32)..=sync.coarse_frequency_bins {
            for partition in 0..=PRIMARY_MAIN_PARTITIONS {
                for nominal in [
                    LOWER_BAND_START as i32 + partition as i32 * PARTITION_WIDTH as i32,
                    UPPER_BAND_END as i32 - partition as i32 * PARTITION_WIDTH as i32,
                ] {
                    let candidate_bin = nominal + candidate;
                    if !(0..FFT_LEN as i32).contains(&candidate_bin) {
                        continue;
                    }
                    let bin = candidate_bin as usize;
                    self.correct_reference(&mut symbols, &mut phases[bin], bin, candidate);
                    for (symbol_index, symbol) in symbols.iter_mut().enumerate() {
                        let phase = phases[bin][symbol_index];
                        symbol[bin] *= Complex::new(phase.cos(), phase.sin());
                    }
                }
            }
        }
    }

    /// Advance nominal reference loops on a coarse block while the integer-CFO
    /// detector is held off after a recent fine-lock loss.
    pub fn warm_nominal_references(
        &mut self,
        block: &OfdmBlock,
        coarse_frequency_bins: i32,
        current_service_mode: u8,
    ) {
        let partitions = partitions_per_band(current_service_mode);
        let mut symbols = block.symbols.clone();
        let mut phases = vec![[0.0f32; BLOCK_SYMBOLS]; FFT_LEN];
        self.adjust_sample_timing(coarse_frequency_bins, block.timing_offset_samples);
        for partition in 0..=partitions {
            for nominal in [
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                UPPER_BAND_END - partition * PARTITION_WIDTH,
            ] {
                let bin = shifted_bin(nominal, coarse_frequency_bins);
                self.correct_reference(&mut symbols, &mut phases[bin], bin, coarse_frequency_bins);
            }
        }
    }

    /// Equalize the aligned coarse-to-fine transition block using the reset
    /// compatibility mode. The newly decoded PSMI applies on the next block.
    pub fn equalize_transition(
        &mut self,
        block: &OfdmBlock,
        sync: FrameSync,
        current_service_mode: u8,
    ) -> OfdmBlock {
        self.equalize_partitions(block, sync, partitions_per_band(current_service_mode))
    }

    /// Correct references and interpolate phase/gain across every data
    /// partition exactly as the NRSC-5 FM receiver architecture requires.
    pub fn equalize(&mut self, block: &OfdmBlock, sync: FrameSync) -> OfdmBlock {
        self.equalize_partitions(block, sync, partitions_per_band(sync.primary_service_mode))
    }

    fn equalize_partitions(
        &mut self,
        block: &OfdmBlock,
        sync: FrameSync,
        partitions: usize,
    ) -> OfdmBlock {
        let mut symbols = block.symbols.clone();
        let mut phases = vec![[0.0f32; BLOCK_SYMBOLS]; FFT_LEN];
        self.adjust_sample_timing(sync.coarse_frequency_bins, block.timing_offset_samples);

        for partition in 0..=partitions {
            for nominal_bin in [
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                UPPER_BAND_END - partition * PARTITION_WIDTH,
            ] {
                let bin = shifted_bin(nominal_bin, sync.coarse_frequency_bins);
                self.correct_reference(
                    &mut symbols,
                    &mut phases[bin],
                    bin,
                    sync.coarse_frequency_bins,
                );
            }
        }

        self.update_tracking(&phases, partitions, sync.coarse_frequency_bins);

        for partition in 0..partitions {
            let lower_left = shifted_bin(
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                sync.coarse_frequency_bins,
            );
            self.correct_partition(
                &mut symbols,
                &phases,
                lower_left,
                lower_left + PARTITION_WIDTH,
            );

            let upper_left = shifted_bin(
                UPPER_BAND_END - (partition + 1) * PARTITION_WIDTH,
                sync.coarse_frequency_bins,
            );
            self.correct_partition(
                &mut symbols,
                &phases,
                upper_left,
                upper_left + PARTITION_WIDTH,
            );
        }

        OfdmBlock {
            symbols,
            frequency_offset_hz: block.frequency_offset_hz,
            timing_offset_samples: block.timing_offset_samples,
            timing_metric: block.timing_metric,
        }
    }

    /// Latest integer sample correction and common Costas frequency.
    pub fn tracking_feedback(&self) -> (i32, f32) {
        (self.sampling_error, self.common_angle)
    }

    /// Extract ten primary-main partitions from each sideband using the same
    /// per-block MER-derived soft scaling as the reference receiver.
    pub fn extract(&self, block: &OfdmBlock, primary_service_mode: u8) -> PrimaryMainSoftBits {
        let active_partitions = partitions_per_band(primary_service_mode);
        let (lower_scale, upper_scale) = soft_scales(block, active_partitions);
        let capacity = BLOCK_SYMBOLS * PRIMARY_MAIN_PARTITIONS * DATA_CARRIERS * 4;
        let mut bits = Vec::with_capacity(capacity);
        for symbol in &block.symbols {
            for partition in 0..PRIMARY_MAIN_PARTITIONS {
                let start = LOWER_BAND_START + partition * PARTITION_WIDTH;
                append_partition(
                    &mut bits,
                    &symbol[start..start + PARTITION_WIDTH],
                    lower_scale,
                );
            }
            for partition in 0..PRIMARY_MAIN_PARTITIONS {
                let start = UPPER_BAND_END - PRIMARY_MAIN_PARTITIONS * PARTITION_WIDTH
                    + partition * PARTITION_WIDTH;
                append_partition(
                    &mut bits,
                    &symbol[start..start + PARTITION_WIDTH],
                    upper_scale,
                );
            }
        }
        PrimaryMainSoftBits { bits }
    }

    fn adjust_sample_timing(&mut self, coarse_frequency_bins: i32, timing_samples: usize) {
        const MAX_PARTITIONS: usize = 14;
        let adjustment = (SYMBOL_LEN / 2) as f32 - timing_samples as f32;
        for partition in 0..=MAX_PARTITIONS {
            for nominal in [
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                UPPER_BAND_END - partition * PARTITION_WIDTH,
            ] {
                let bin = shifted_bin(nominal, coarse_frequency_bins);
                let carrier = bin as f32 - (FFT_LEN / 2) as f32;
                self.trackers[bin].phase = wrap_phase(
                    self.trackers[bin].phase - adjustment * carrier * 2.0 * PI / FFT_LEN as f32,
                );
            }
        }
    }

    fn correct_reference(
        &mut self,
        symbols: &mut [Vec<Complex<f32>>],
        phases: &mut [f32; BLOCK_SYMBOLS],
        bin: usize,
        coarse_frequency_bins: i32,
    ) {
        let tracker = &mut self.trackers[bin];
        let prefix_rotation = 2.0 * PI * coarse_frequency_bins as f32 * 112.0 / FFT_LEN as f32;
        for (symbol_index, symbol) in symbols.iter_mut().enumerate() {
            let value = symbol[bin];
            let doubled = value
                * value
                * Complex::new((-2.0 * tracker.phase).cos(), (-2.0 * tracker.phase).sin());
            let error = 0.5 * doubled.arg();
            phases[symbol_index] = tracker.phase;
            symbol[bin] *= Complex::new((-tracker.phase).cos(), (-tracker.phase).sin());
            tracker.frequency = (tracker.frequency + COSTAS_BETA * error).clamp(-0.5, 0.5);
            tracker.phase = wrap_phase(
                tracker.phase + tracker.frequency + prefix_rotation + COSTAS_ALPHA * error,
            );
        }

        let polarity = symbols
            .iter()
            .zip(REFERENCE_SYNC)
            .map(|(symbol, sync)| symbol[bin].re * f32::from(sync))
            .sum::<f32>();
        if polarity < 0.0 {
            for (symbol_index, symbol) in symbols.iter_mut().enumerate() {
                phases[symbol_index] += PI;
                symbol[bin] = -symbol[bin];
            }
            tracker.phase = wrap_phase(tracker.phase + PI);
        }
    }

    fn update_tracking(
        &mut self,
        phases: &[[f32; BLOCK_SYMBOLS]],
        partitions: usize,
        coarse_frequency_bins: i32,
    ) {
        let mut sampling_error = 0.0;
        let mut common_angle = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_x2 = 0.0;
        for partition in 0..partitions {
            let lower = shifted_bin(
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                coarse_frequency_bins,
            );
            let upper = shifted_bin(
                UPPER_BAND_END - partition * PARTITION_WIDTH,
                coarse_frequency_bins,
            );
            sampling_error += phase_diff(phases[lower][0], phases[lower + PARTITION_WIDTH][0]);
            sampling_error += phase_diff(phases[upper - PARTITION_WIDTH][0], phases[upper][0]);
        }
        sampling_error = sampling_error / (partitions * 2) as f32 * FFT_LEN as f32
            / PARTITION_WIDTH as f32
            / (2.0 * PI);

        for partition in 0..=partitions {
            for nominal in [
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                UPPER_BAND_END - partition * PARTITION_WIDTH,
            ] {
                let bin = shifted_bin(nominal, coarse_frequency_bins);
                let x = nominal as f32 - (FFT_LEN / 2) as f32;
                let y = self.trackers[bin].frequency;
                common_angle += y;
                sum_xy += x * y;
                sum_x2 += x * x;
            }
        }
        let phase_sampling_error = sampling_error;
        let frequency_sampling_error =
            (sum_xy / sum_x2) * FFT_LEN as f32 / (2.0 * PI) * BLOCK_SYMBOLS as f32;
        sampling_error -= frequency_sampling_error;
        self.sampling_error = sampling_error.round() as i32;
        tracing::trace!(
            phase_sampling_error,
            frequency_sampling_error,
            sampling_error,
            rounded = self.sampling_error,
            angle_sum = common_angle,
            "reference tracking components"
        );
        self.common_angle = common_angle / ((partitions + 1) * 2) as f32;
        for partition in 0..=partitions {
            for nominal in [
                LOWER_BAND_START + partition * PARTITION_WIDTH,
                UPPER_BAND_END - partition * PARTITION_WIDTH,
            ] {
                let bin = shifted_bin(nominal, coarse_frequency_bins);
                self.trackers[bin].frequency -= self.common_angle;
            }
        }
    }

    fn correct_partition(
        &self,
        symbols: &mut [Vec<Complex<f32>>],
        phases: &[[f32; BLOCK_SYMBOLS]],
        left: usize,
        right: usize,
    ) {
        let left_magnitude = symbols
            .iter()
            .map(|symbol| symbol[left].re.abs())
            .sum::<f32>()
            / BLOCK_SYMBOLS as f32;
        let right_magnitude = symbols
            .iter()
            .map(|symbol| symbol[right].re.abs())
            .sum::<f32>()
            / BLOCK_SYMBOLS as f32;
        if left_magnitude <= f32::EPSILON || right_magnitude <= f32::EPSILON {
            return;
        }

        for symbol_index in 0..BLOCK_SYMBOLS {
            let left_phase = Complex::new(
                phases[left][symbol_index].cos(),
                phases[left][symbol_index].sin(),
            );
            let right_phase = Complex::new(
                phases[right][symbol_index].cos(),
                phases[right][symbol_index].sin(),
            );
            for carrier in 1..PARTITION_WIDTH {
                let denominator = right_phase * (carrier as f32 * right_magnitude)
                    + left_phase * ((PARTITION_WIDTH - carrier) as f32 * left_magnitude);
                if denominator.norm_sqr() > f32::EPSILON {
                    let correction =
                        Complex::new(PARTITION_WIDTH as f32, PARTITION_WIDTH as f32) / denominator;
                    symbols[symbol_index][left + carrier] *= correction;
                }
            }
        }
    }
}

impl Default for PrimaryMainExtractor {
    fn default() -> Self {
        Self::new()
    }
}

fn partitions_per_band(primary_service_mode: u8) -> usize {
    const COMPATIBILITY_MODE: [u8; 64] = [
        0, 1, 2, 3, 1, 5, 6, 5, 6, 1, 2, 11, 1, 5, 6, 5, 6, 1, 2, 3, 1, 5, 6, 5, 6, 1, 2, 11, 1, 5,
        6, 5, 6, 1, 2, 3, 1, 5, 6, 5, 6, 1, 2, 11, 1, 5, 6, 5, 6, 1, 2, 3, 1, 5, 6, 5, 6, 1, 2, 11,
        1, 5, 6, 5,
    ];
    match COMPATIBILITY_MODE[usize::from(primary_service_mode.min(63))] {
        2 => 11,
        3 => 12,
        5 | 6 | 11 => 14,
        _ => 10,
    }
}

fn shifted_bin(nominal: usize, offset: i32) -> usize {
    (nominal as i32 + offset) as usize
}

fn soft_scales(block: &OfdmBlock, partitions: usize) -> (f32, f32) {
    let mut lower_error = 0.0f32;
    let mut upper_error = 0.0f32;
    for symbol in &block.symbols {
        for partition in 0..partitions {
            let lower_start = LOWER_BAND_START + partition * PARTITION_WIDTH;
            let upper_start = UPPER_BAND_END - (partition + 1) * PARTITION_WIDTH;
            for carrier in 1..PARTITION_WIDTH {
                lower_error += decision_error(symbol[lower_start + carrier]);
                upper_error += decision_error(symbol[upper_start + carrier]);
            }
        }
    }
    let signal = 2.0 * BLOCK_SYMBOLS as f32 * (partitions * DATA_CARRIERS) as f32;
    let lower_mer = signal / lower_error.max(f32::MIN_POSITIVE);
    let upper_mer = signal / upper_error.max(f32::MIN_POSITIVE);
    (
        (lower_mer * 10.0).clamp(1.0, 127.0),
        (upper_mer * 10.0).clamp(1.0, 127.0),
    )
}

fn decision_error(value: Complex<f32>) -> f32 {
    let ideal = Complex::new(
        if value.re >= 0.0 { 1.0 } else { -1.0 },
        if value.im >= 0.0 { 1.0 } else { -1.0 },
    );
    (ideal - value).norm_sqr()
}

fn append_partition(output: &mut Vec<i8>, partition: &[Complex<f32>], scale: f32) {
    for carrier in &partition[1..] {
        output.push(soft_decision(carrier.re, scale));
        output.push(soft_decision(carrier.im, scale));
    }
}

fn soft_decision(value: f32, scale: f32) -> i8 {
    value
        .clamp(-1.0, 1.0)
        .mul_add(scale, 0.0)
        .round()
        .clamp(-127.0, 127.0) as i8
}

fn phase_diff(left: f32, right: f32) -> f32 {
    let mut difference = left - right;
    while difference > PI * 0.5 {
        difference -= PI;
    }
    while difference < -PI * 0.5 {
        difference += PI;
    }
    difference
}

fn wrap_phase(phase: f32) -> f32 {
    (phase + PI).rem_euclid(2.0 * PI) - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_block() -> OfdmBlock {
        OfdmBlock {
            symbols: (0..BLOCK_SYMBOLS)
                .map(|_| vec![Complex::new(1.0, -1.0); FFT_LEN])
                .collect(),
            frequency_offset_hz: 0.0,
            timing_offset_samples: 0,
            timing_metric: 1.0,
        }
    }

    #[test]
    fn primary_main_has_expected_bit_count() {
        let output = PrimaryMainExtractor::new().extract(&test_block(), 3);
        assert_eq!(
            output.bits.len(),
            BLOCK_SYMBOLS * PRIMARY_MAIN_PARTITIONS * DATA_CARRIERS * 4
        );
    }

    #[test]
    fn compatibility_mode_selects_extended_partitions() {
        assert_eq!(partitions_per_band(3), 12);
        assert_eq!(partitions_per_band(11), 14);
        assert_eq!(partitions_per_band(1), 10);
    }
}
