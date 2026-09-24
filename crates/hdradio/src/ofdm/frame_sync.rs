//! NRSC-5 FM reference-carrier block synchronization.
//!
//! Cyclic-prefix acquisition supplies OFDM symbol timing. This layer tracks
//! the differentially encoded BPSK reference carriers across consecutive
//! 32-symbol blocks to recover integer-carrier offset, block counter, and
//! primary service mode.

use std::f32::consts::PI;

use num_complex::Complex;

use crate::ofdm::carriers::{LOWER_BAND_START, PARTITION_WIDTH, UPPER_BAND_END};
use crate::ofdm::phy::{BLOCK_SYMBOLS, FFT_LEN, OfdmBlock};

const MAX_COARSE_OFFSET_BINS: i32 = 2 * PARTITION_WIDTH as i32;
const REFERENCE_PARTITIONS: usize = 10;
const COSTAS_ALPHA: f32 = 0.131_774_08;
const COSTAS_BETA: f32 = 0.009_317_835;
const LOCK_FAILURE_LIMIT: u8 = 4;

/// A reference-carrier synchronization observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameSync {
    /// Integer carrier displacement from nominal bins.
    pub coarse_frequency_bins: i32,
    /// Position of the first logical-block symbol in the received OFDM block.
    pub block_offset: u8,
    /// NRSC-5 primary service mode indicator.
    pub primary_service_mode: u8,
    /// Four-bit logical-channel block counter.
    pub block_counter: u8,
    /// Number of independently matching reference carriers.
    pub reference_matches: u8,
}

#[derive(Debug, Clone, Copy, Default)]
struct CostasTracker {
    phase: f32,
    frequency: f32,
}

#[derive(Debug, Clone, Copy)]
struct LockedFrame {
    coarse_frequency_bins: i32,
    primary_service_mode: u8,
    failures: u8,
}

/// Finds and continuously tracks FM-hybrid reference carriers.
#[derive(Debug)]
pub struct FrameSynchronizer {
    trackers: Vec<CostasTracker>,
    lock: Option<LockedFrame>,
}

impl FrameSynchronizer {
    /// Construct an unlocked synchronizer.
    pub fn new() -> Self {
        Self {
            trackers: vec![CostasTracker::default(); FFT_LEN],
            lock: None,
        }
    }

    /// Recover synchronization from a 32-symbol OFDM block.
    ///
    /// Acquisition searches integer CFO candidates using stateless reference
    /// loops. Once at least four references agree, each carrier's Costas state
    /// persists across future blocks; resetting that state was insufficient for
    /// low-MER real captures because their phase evolves through every prefix.
    pub fn synchronize(&mut self, block: &OfdmBlock) -> Option<FrameSync> {
        if let Some(locked) = self.lock {
            if let Some(sync) = self.track_locked(block, locked) {
                self.lock = Some(LockedFrame {
                    coarse_frequency_bins: sync.coarse_frequency_bins,
                    primary_service_mode: sync.primary_service_mode,
                    failures: 0,
                });
                return Some(sync);
            }
            let failures = locked.failures + 1;
            if failures < LOCK_FAILURE_LIMIT {
                self.lock = Some(LockedFrame { failures, ..locked });
            } else {
                self.reset();
            }
            return None;
        }

        let sync = self.acquire(block)?;
        self.seed_trackers(block, sync.coarse_frequency_bins);
        self.lock = Some(LockedFrame {
            coarse_frequency_bins: sync.coarse_frequency_bins,
            primary_service_mode: sync.primary_service_mode,
            failures: 0,
        });
        Some(sync)
    }

    /// Clear all continuous reference-carrier state after a discontinuity.
    pub fn reset(&mut self) {
        self.trackers.fill(CostasTracker::default());
        self.lock = None;
    }

    fn acquire(&self, block: &OfdmBlock) -> Option<FrameSync> {
        let mut best = None;
        for coarse_frequency_bins in -MAX_COARSE_OFFSET_BINS..=MAX_COARSE_OFFSET_BINS {
            let matches = decode_references_stateless(block, coarse_frequency_bins);
            best = choose_consensus(best, coarse_frequency_bins, &matches);
        }
        best.filter(|sync| sync.reference_matches >= 4)
    }

    fn track_locked(&mut self, block: &OfdmBlock, locked: LockedFrame) -> Option<FrameSync> {
        let mut matches = Vec::new();
        for bin in reference_bins(locked.coarse_frequency_bins) {
            let values: Vec<Complex<f32>> =
                block.symbols.iter().map(|symbol| symbol[bin]).collect();
            let corrected = costas_correct_stateful(
                &values,
                locked.coarse_frequency_bins,
                &mut self.trackers[bin],
            );
            let rsid = reference_rsid(bin, locked.coarse_frequency_bins)?;
            if let Some(decoded) = decode_corrected_reference(&corrected, rsid) {
                matches.push(decoded);
            }
        }
        choose_consensus(None, locked.coarse_frequency_bins, &matches).filter(|sync| {
            sync.reference_matches >= 4 && sync.primary_service_mode == locked.primary_service_mode
        })
    }

    fn seed_trackers(&mut self, block: &OfdmBlock, coarse_frequency_bins: i32) {
        for bin in reference_bins(coarse_frequency_bins) {
            let values: Vec<Complex<f32>> =
                block.symbols.iter().map(|symbol| symbol[bin]).collect();
            let _ =
                costas_correct_stateful(&values, coarse_frequency_bins, &mut self.trackers[bin]);
        }
    }
}

impl Default for FrameSynchronizer {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy)]
struct ReferenceDecode {
    block_offset: u8,
    primary_service_mode: u8,
    block_counter: u8,
}

fn decode_references_stateless(
    block: &OfdmBlock,
    coarse_frequency_bins: i32,
) -> Vec<ReferenceDecode> {
    reference_bins(coarse_frequency_bins)
        .into_iter()
        .filter_map(|bin| {
            let values: Vec<Complex<f32>> =
                block.symbols.iter().map(|symbol| symbol[bin]).collect();
            let corrected = costas_correct_stateless(&values, coarse_frequency_bins);
            decode_corrected_reference(&corrected, reference_rsid(bin, coarse_frequency_bins)?)
        })
        .collect()
}

fn choose_consensus(
    mut best: Option<FrameSync>,
    coarse_frequency_bins: i32,
    matches: &[ReferenceDecode],
) -> Option<FrameSync> {
    for candidate in matches.iter().copied() {
        let agreeing = matches
            .iter()
            .filter(|other| {
                other.block_offset == candidate.block_offset
                    && other.primary_service_mode == candidate.primary_service_mode
                    && other.block_counter == candidate.block_counter
            })
            .count() as u8;
        let sync = FrameSync {
            coarse_frequency_bins,
            block_offset: candidate.block_offset,
            primary_service_mode: candidate.primary_service_mode,
            block_counter: candidate.block_counter,
            reference_matches: agreeing,
        };
        if best.is_none_or(|current| sync.reference_matches > current.reference_matches) {
            best = Some(sync);
        }
    }
    best
}

fn reference_bins(coarse_frequency_bins: i32) -> Vec<usize> {
    let mut bins = Vec::with_capacity((REFERENCE_PARTITIONS + 1) * 2);
    for partition in 0..=REFERENCE_PARTITIONS {
        for nominal in [
            LOWER_BAND_START as i32 + partition as i32 * PARTITION_WIDTH as i32,
            UPPER_BAND_END as i32 - partition as i32 * PARTITION_WIDTH as i32,
        ] {
            let bin = nominal + coarse_frequency_bins;
            if (0..FFT_LEN as i32).contains(&bin) {
                bins.push(bin as usize);
            }
        }
    }
    bins
}

fn reference_rsid(bin: usize, coarse_frequency_bins: i32) -> Option<u8> {
    for partition in 0..=REFERENCE_PARTITIONS {
        for nominal in [
            LOWER_BAND_START as i32 + partition as i32 * PARTITION_WIDTH as i32,
            UPPER_BAND_END as i32 - partition as i32 * PARTITION_WIDTH as i32,
        ] {
            if nominal + coarse_frequency_bins == bin as i32 {
                return Some(((30 - partition) & 3) as u8);
            }
        }
    }
    None
}

fn decode_corrected_reference(values: &[Complex<f32>], rsid: u8) -> Option<ReferenceDecode> {
    let hard: Vec<u8> = values
        .iter()
        .map(|value| u8::from(value.re > 0.0))
        .collect();
    for inverted in [false, true] {
        let hard: Vec<u8> = if inverted {
            hard.iter().map(|bit| bit ^ 1).collect()
        } else {
            hard.clone()
        };
        for offset in 0..BLOCK_SYMBOLS {
            if !matches_reference_signature(&hard, offset, rsid) {
                continue;
            }
            let differential = differential_decode(&hard, offset);
            return Some(ReferenceDecode {
                block_offset: offset as u8,
                block_counter: (differential[16] << 3)
                    | (differential[17] << 2)
                    | (differential[18] << 1)
                    | differential[19],
                primary_service_mode: (differential[25] << 5)
                    | (differential[26] << 4)
                    | (differential[27] << 3)
                    | (differential[28] << 2)
                    | (differential[29] << 1)
                    | differential[30],
            });
        }
    }
    None
}

fn costas_correct_stateless(
    values: &[Complex<f32>],
    coarse_frequency_bins: i32,
) -> Vec<Complex<f32>> {
    let mut tracker = CostasTracker::default();
    costas_correct_stateful(values, coarse_frequency_bins, &mut tracker)
}

fn costas_correct_stateful(
    values: &[Complex<f32>],
    coarse_frequency_bins: i32,
    tracker: &mut CostasTracker,
) -> Vec<Complex<f32>> {
    let cp_rotation = 2.0 * PI * coarse_frequency_bins as f32 * 112.0 / FFT_LEN as f32;
    values
        .iter()
        .map(|value| {
            let doubled = *value
                * *value
                * Complex::new((-2.0 * tracker.phase).cos(), (-2.0 * tracker.phase).sin());
            let error = 0.5 * doubled.arg();
            let corrected = *value * Complex::new((-tracker.phase).cos(), (-tracker.phase).sin());
            tracker.frequency = (tracker.frequency + COSTAS_BETA * error).clamp(-0.5, 0.5);
            tracker.phase =
                wrap_phase(tracker.phase + tracker.frequency + cp_rotation + COSTAS_ALPHA * error);
            corrected
        })
        .collect()
}

fn wrap_phase(phase: f32) -> f32 {
    (phase + PI).rem_euclid(2.0 * PI) - PI
}

fn matches_reference_signature(bits: &[u8], offset: usize, rsid: u8) -> bool {
    let expected: [Option<u8>; BLOCK_SYMBOLS] = [
        Some(0),
        Some(1),
        Some(0),
        Some(0),
        Some(0),
        Some(1),
        Some(1),
        None,
        Some(1),
        Some(0),
        Some(rsid >> 1),
        Some((rsid >> 1) ^ (rsid & 1)),
        None,
        Some(0),
        Some(0),
        None,
        None,
        None,
        None,
        None,
        Some(0),
        Some(1),
        Some(0),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        Some(0),
    ];
    expected.iter().enumerate().all(|(index, expected)| {
        expected.is_none_or(|expected| bits[(offset + index) % BLOCK_SYMBOLS] == expected)
    })
}

fn differential_decode(bits: &[u8], offset: usize) -> [u8; BLOCK_SYMBOLS] {
    let mut output = [0u8; BLOCK_SYMBOLS];
    let mut previous = 0;
    for (index, output_bit) in output.iter_mut().enumerate() {
        let bit = bits[(offset + index) % BLOCK_SYMBOLS];
        *output_bit = bit ^ previous;
        previous = bit;
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn differential_decoder_recovers_transition_bits() {
        let bits = [
            0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0,
            0, 1, 0,
        ];
        let decoded = differential_decode(&bits, 0);
        assert_eq!(decoded[0], 0);
        assert_eq!(decoded[2], 1);
        assert_eq!(decoded[3], 0);
    }

    #[test]
    fn signature_accepts_dont_care_bits() {
        let mut bits = [0u8; BLOCK_SYMBOLS];
        let rsid = 2;
        for (index, value) in [
            (0, 0),
            (1, 1),
            (2, 0),
            (3, 0),
            (4, 0),
            (5, 1),
            (6, 1),
            (8, 1),
            (9, 0),
            (10, rsid >> 1),
            (11, (rsid >> 1) ^ (rsid & 1)),
            (13, 0),
            (14, 0),
            (20, 0),
            (21, 1),
            (22, 0),
            (31, 0),
        ] {
            bits[index] = value;
        }
        assert!(matches_reference_signature(&bits, 0, rsid));
    }
}
