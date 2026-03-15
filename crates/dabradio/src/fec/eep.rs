//! EEP (Equal Error Protection) depuncturing for DAB FIC and MSC.
//!
//! The DAB convolutional encoder produces 4 output bits per input bit (rate 1/4).
//! Puncturing removes some of these bits to achieve higher code rates.
//! Depuncturing re-inserts zero-valued soft bits (erasures) at the punctured positions.
//!
//! For the FIC (Fast Information Channel) Mode I:
//! - 3 FIC OFDM symbols per frame, each carrying 3072 soft bits (2×K after DQPSK)
//! - Total: 3 × 3072 = 9216 soft bits per frame
//! - These are split into 4 sub-blocks of 2304 punctured soft bits each
//! - Each sub-block encodes 768 bits (3 FIBs of 256 bits) + 6 tail bits
//! - Depuncturing expands 2304 → 3096 rate-1/4 bits for Viterbi
//!
//! For the MSC (Main Service Channel):
//! - Each subchannel uses EEP with L1 blocks at puncture rate PI1 and L2 blocks at PI2
//! - Parameters depend on bitrate and protection level (EEP-A or EEP-B)
//! - Plus 24 tail bits with PI_TAIL pattern
//!
//! Reference: ETSI EN 300 401 §11.1, §11.2, welle.io src/backend/fic-handler.cpp

/// All 24 puncturing vectors from ETSI EN 300 401 Table 31.
/// `P_CODES[i]` is puncturing vector PI_(i+1), with i in 0..23.
/// Each vector has 32 entries; a 1 means "transmitted", 0 means "punctured".
/// Reference: welle.io src/backend/protTables.cpp
pub const P_CODES: [[u8; 32]; 24] = [
    // PI 1: 9 ones
    [
        1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
        0, 0,
    ],
    // PI 2: 10 ones
    [
        1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
        0, 0,
    ],
    // PI 3: 11 ones
    [
        1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
        0, 0,
    ],
    // PI 4: 12 ones
    [
        1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0,
        0, 0,
    ],
    // PI 5: 13 ones
    [
        1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0,
        0, 0,
    ],
    // PI 6: 14 ones
    [
        1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0,
        0, 0,
    ],
    // PI 7: 15 ones
    [
        1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0,
        0, 0,
    ],
    // PI 8: 16 ones
    [
        1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0,
    ],
    // PI 9: 17 ones
    [
        1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0,
    ],
    // PI 10: 18 ones
    [
        1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0,
    ],
    // PI 11: 19 ones
    [
        1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
        0, 0,
    ],
    // PI 12: 20 ones
    [
        1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1,
        0, 0,
    ],
    // PI 13: 21 ones
    [
        1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1,
        0, 0,
    ],
    // PI 14: 22 ones
    [
        1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1,
        0, 0,
    ],
    // PI 15: 23 ones
    [
        1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1,
        0, 0,
    ],
    // PI 16: 24 ones
    [
        1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1,
        1, 0,
    ],
    // PI 17: 25 ones
    [
        1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1,
        1, 0,
    ],
    // PI 18: 26 ones
    [
        1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1,
        1, 0,
    ],
    // PI 19: 27 ones
    [
        1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1,
        1, 0,
    ],
    // PI 20: 28 ones
    [
        1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1,
        1, 0,
    ],
    // PI 21: 29 ones
    [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1,
        1, 0,
    ],
    // PI 22: 30 ones
    [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 0,
    ],
    // PI 23: 31 ones
    [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 0,
    ],
    // PI 24: 32 ones (no puncturing)
    [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1,
    ],
];

/// Puncturing vector PI_16 (index 15) from ETSI EN 300 401 Table 31.
/// Kept for backwards compatibility with FIC code. Same as P_CODES[15].
pub const PI_16: [u8; 32] = P_CODES[15];

/// Puncturing vector PI_15 (index 14). Same as P_CODES[14].
pub const PI_15: [u8; 32] = P_CODES[14];

/// Tail puncturing pattern (for the 6 tail bits × 4 = 24 coded bits).
/// 24 entries: 12 ones, 12 zeros. Pattern: [1,1,0,0] repeated 6 times.
pub const PI_TAIL: [u8; 24] = [
    1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
];

/// Depuncture one FIC sub-block of 2304 punctured soft bits → 3096 rate-1/4 soft bits.
///
/// The FIC uses:
/// - 21 blocks of PI_16: each block covers 128 coded positions (the 32-entry pattern repeated 4×),
///   consuming 21 × (24×4) = 21 × 96 = 2016 transmitted bits
/// - 3 blocks of PI_15: each block covers 128 coded positions,
///   consuming 3 × (23×4) = 3 × 92 = 276 transmitted bits
/// - 1 tail block of 24 coded positions, consuming 12 transmitted bits
///   Total consumed: 2016 + 276 + 12 = 2304 (matches input size)
///   Total output: 21×128 + 3×128 + 24 = 2688 + 384 + 24 = 3096
pub fn depuncture_fic(soft_bits: &[i8]) -> Vec<i8> {
    let mut depunctured = Vec::with_capacity(3096);
    let mut read_idx = 0;

    // Apply PI_16 for 21 blocks, each block = 4 repetitions of the 32-entry pattern = 128 positions
    for _ in 0..21 {
        for _ in 0..4 {
            for &p in &PI_16 {
                if p == 1 {
                    if read_idx < soft_bits.len() {
                        depunctured.push(soft_bits[read_idx]);
                        read_idx += 1;
                    } else {
                        depunctured.push(0);
                    }
                } else {
                    depunctured.push(0); // erasure
                }
            }
        }
    }

    // Apply PI_15 for 3 blocks, each block = 4 repetitions
    for _ in 0..3 {
        for _ in 0..4 {
            for &p in &PI_15 {
                if p == 1 {
                    if read_idx < soft_bits.len() {
                        depunctured.push(soft_bits[read_idx]);
                        read_idx += 1;
                    } else {
                        depunctured.push(0);
                    }
                } else {
                    depunctured.push(0); // erasure
                }
            }
        }
    }

    // Apply tail (24 coded positions)
    for &p in &PI_TAIL {
        if p == 1 {
            if read_idx < soft_bits.len() {
                depunctured.push(soft_bits[read_idx]);
                read_idx += 1;
            } else {
                depunctured.push(0);
            }
        } else {
            depunctured.push(0);
        }
    }

    depunctured
}

/// EEP parameters for MSC subchannel depuncturing.
///
/// Given the subchannel bitrate (kbps), protection level (0-3 from FIG 0/1),
/// and EEP option (0=A, 1=B), returns (L1, L2, pi1_index, pi2_index) where:
/// - L1, L2 are the number of 128-bit blocks for each puncture rate
/// - pi1_index, pi2_index index into P_CODES (0-based, so PI n → P_CODES[n-1])
///
/// Returns None for unsupported combinations.
///
/// Reference: ETSI EN 300 401 §11.2, Tables 7/8, welle.io src/backend/eep-protection.cpp
pub fn eep_params(bitrate: u16, protection_level: u8, eep_option: u8) -> Option<EepParams> {
    let n = bitrate as i32 / 8; // n = bitrate/8 for EEP-A, bitrate/32 for EEP-B

    match eep_option {
        0 => {
            // EEP-A (option 0)
            let (l1, l2, pi1, pi2) = match protection_level {
                0 => (6 * n - 3, 3, 24 - 1, 23 - 1),         // EEP 1-A
                1 => (2 * n - 3, 4 * n + 3, 14 - 1, 13 - 1), // EEP 2-A
                2 => (6 * n - 3, 3, 8 - 1, 7 - 1),           // EEP 3-A
                3 => (4 * n - 3, 2 * n + 3, 3 - 1, 2 - 1),   // EEP 4-A
                _ => return None,
            };
            if l1 < 0 || l2 < 0 {
                return None;
            }
            Some(EepParams {
                l1: l1 as usize,
                l2: l2 as usize,
                pi1_index: pi1 as usize,
                pi2_index: pi2 as usize,
            })
        }
        1 => {
            // EEP-B (option 1)
            let n_b = bitrate as i32 / 32;
            let (l1, l2, pi1, pi2) = match protection_level {
                0 => (24 * n_b - 3, 3, 10 - 1, 9 - 1), // EEP 1-B
                1 => (24 * n_b - 3, 3, 6 - 1, 5 - 1),  // EEP 2-B
                2 => (24 * n_b - 3, 3, 4 - 1, 3 - 1),  // EEP 3-B
                3 => (24 * n_b - 3, 3, 2 - 1, 0),      // EEP 4-B, PI 2 / PI 1
                _ => return None,
            };
            if l1 < 0 || l2 < 0 {
                return None;
            }
            Some(EepParams {
                l1: l1 as usize,
                l2: l2 as usize,
                pi1_index: pi1 as usize,
                pi2_index: pi2 as usize,
            })
        }
        _ => None,
    }
}

/// Parameters for EEP depuncturing.
#[derive(Debug, Clone, Copy)]
pub struct EepParams {
    pub l1: usize,        // Number of 128-bit blocks at puncture rate PI1
    pub l2: usize,        // Number of 128-bit blocks at puncture rate PI2
    pub pi1_index: usize, // Index into P_CODES for first region
    pub pi2_index: usize, // Index into P_CODES for second region
}

/// Count the number of ones in a puncturing pattern over a given number of 128-bit blocks.
/// Each block = 4 repetitions of 32-entry pattern.
fn count_punctured_ones(pi_index: usize, num_blocks: usize) -> usize {
    let pi = &P_CODES[pi_index];
    let ones_per_32: usize = pi.iter().map(|&x| x as usize).sum();
    ones_per_32 * 4 * num_blocks // 4 repetitions per block
}

/// Compute the expected punctured (input) size for an MSC subchannel.
pub fn msc_punctured_size(params: &EepParams) -> usize {
    count_punctured_ones(params.pi1_index, params.l1)
        + count_punctured_ones(params.pi2_index, params.l2)
        + 12 // tail: 12 ones from PI_TAIL
}

/// Compute the expected depunctured (output) size for an MSC subchannel.
pub fn msc_depunctured_size(params: &EepParams) -> usize {
    (params.l1 + params.l2) * 128 + 24 // 128 coded bits per block + 24 tail bits
}

/// Depuncture an MSC subchannel's soft bits using the given EEP parameters.
///
/// The MSC uses:
/// - L1 blocks with puncture pattern P_CODES[pi1_index]
/// - L2 blocks with puncture pattern P_CODES[pi2_index]
/// - 1 tail block of 24 coded positions with PI_TAIL
///
/// Each block = 4 repetitions of the 32-entry puncture pattern = 128 coded positions.
pub fn depuncture_msc(soft_bits: &[i8], params: &EepParams) -> Vec<i8> {
    let out_size = msc_depunctured_size(params);
    let mut depunctured = Vec::with_capacity(out_size);
    let mut read_idx = 0;

    // L1 blocks with PI1
    let pi1 = &P_CODES[params.pi1_index];
    for _ in 0..params.l1 {
        for _ in 0..4 {
            for &p in pi1 {
                if p == 1 {
                    if read_idx < soft_bits.len() {
                        depunctured.push(soft_bits[read_idx]);
                        read_idx += 1;
                    } else {
                        depunctured.push(0);
                    }
                } else {
                    depunctured.push(0); // erasure
                }
            }
        }
    }

    // L2 blocks with PI2
    let pi2 = &P_CODES[params.pi2_index];
    for _ in 0..params.l2 {
        for _ in 0..4 {
            for &p in pi2 {
                if p == 1 {
                    if read_idx < soft_bits.len() {
                        depunctured.push(soft_bits[read_idx]);
                        read_idx += 1;
                    } else {
                        depunctured.push(0);
                    }
                } else {
                    depunctured.push(0); // erasure
                }
            }
        }
    }

    // Tail (24 coded positions)
    for &p in &PI_TAIL {
        if p == 1 {
            if read_idx < soft_bits.len() {
                depunctured.push(soft_bits[read_idx]);
                read_idx += 1;
            } else {
                depunctured.push(0);
            }
        } else {
            depunctured.push(0);
        }
    }

    depunctured
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_depuncture_fic_output_size() {
        let input = vec![0i8; 2304];
        let output = depuncture_fic(&input);
        assert_eq!(output.len(), 3096);
    }

    #[test]
    fn test_depuncture_fic_consumes_all_input() {
        // Verify that exactly 2304 input bits are consumed
        let mut input = vec![42i8; 2304];
        // Add a sentinel that should NOT be consumed
        input.push(-1);
        let output = depuncture_fic(&input[..2304]);
        assert_eq!(output.len(), 3096);

        // Count non-zero entries (should be 2304 — all the transmitted bits)
        let nonzero = output.iter().filter(|&&x| x != 0).count();
        assert_eq!(nonzero, 2304);
    }

    #[test]
    fn test_pi_16_ones_count() {
        let ones: u32 = PI_16.iter().map(|&x| x as u32).sum();
        assert_eq!(ones, 24);
    }

    #[test]
    fn test_pi_15_ones_count() {
        let ones: u32 = PI_15.iter().map(|&x| x as u32).sum();
        assert_eq!(ones, 23);
    }

    #[test]
    fn test_pi_tail_ones_count() {
        let ones: u32 = PI_TAIL.iter().map(|&x| x as u32).sum();
        assert_eq!(ones, 12);
    }

    #[test]
    fn test_p_codes_ones_count() {
        // Verify each P_CODES[i] has (i+1)+8 ones (PI 1 has 9, PI 24 has 32)
        let expected_ones = [
            9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
            31, 32,
        ];
        for (i, &expected) in expected_ones.iter().enumerate() {
            let ones: u32 = P_CODES[i].iter().map(|&x| x as u32).sum();
            assert_eq!(
                ones,
                expected,
                "P_CODES[{}] (PI {}) should have {} ones",
                i,
                i + 1,
                expected
            );
        }
    }

    #[test]
    fn test_p_codes_match_legacy_constants() {
        // PI_16 and PI_15 should match the P_CODES entries
        assert_eq!(PI_16, P_CODES[15]);
        assert_eq!(PI_15, P_CODES[14]);
    }

    #[test]
    fn test_eep_2a_88kbps_params() {
        // All services in our test ensemble are 88 kbps EEP 2-A
        let params = eep_params(88, 1, 0).unwrap(); // protection_level=1 → EEP 2-A, option=0 → A
        assert_eq!(params.l1, 19); // 2*11 - 3 = 19
        assert_eq!(params.l2, 47); // 4*11 + 3 = 47
        assert_eq!(params.pi1_index, 13); // PI 14
        assert_eq!(params.pi2_index, 12); // PI 13
    }

    #[test]
    fn test_eep_2a_88kbps_sizes() {
        let params = eep_params(88, 1, 0).unwrap();
        // Punctured size: 19*(22*4) + 47*(21*4) + 12 = 1672 + 3948 + 12 = 5632
        assert_eq!(msc_punctured_size(&params), 5632);
        // Depunctured size: (19+47)*128 + 24 = 66*128 + 24 = 8448 + 24 = 8472
        assert_eq!(msc_depunctured_size(&params), 8472);
    }

    #[test]
    fn test_depuncture_msc_88kbps() {
        let params = eep_params(88, 1, 0).unwrap();
        let input = vec![42i8; msc_punctured_size(&params)];
        let output = depuncture_msc(&input, &params);
        assert_eq!(output.len(), msc_depunctured_size(&params));

        // All 5632 input bits should appear in output (non-zero)
        let nonzero = output.iter().filter(|&&x| x != 0).count();
        assert_eq!(nonzero, 5632);
    }
}
