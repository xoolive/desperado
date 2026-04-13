//! Viterbi convolutional decoder for DAB.
//!
//! DAB uses a rate 1/4 mother code with constraint length K=7 (64 states).
//! Generator polynomials (octal): G1=133, G2=171, G3=145, G4=133.
//! Note: G1 == G4 in the DAB standard.
//!
//! For the FIC, the code is punctured to rate 1/3 (one of the four output
//! streams is discarded per puncturing pattern).
//!
//! Reference: ETSI EN 300 401 §11.1, welle.io src/backend/viterbi.cpp

/// Generator polynomials for the rate-1/4 convolutional code (octal).
///
/// DAB standard (ETSI EN 300 401) uses the "official" polynomials:
///   G1=0155, G2=0117, G3=0123, G4=0155
/// where the input bit enters at the LSB of the shift register.
///
/// The "bit-reversed" form {0133, 0171, 0145, 0133} has the input at MSB.
/// Both produce the same trellis but with different state numbering.
/// We must match the convention used by the DAB transmitter.
///
/// welle.io uses {0155, 0117, 0123, 0155} with a butterfly decoder where
/// the state register has the newest bit at LSB. We need our build_output_table
/// and state transitions to be consistent.
const POLY: [u8; 4] = [
    0o155, // G1 (official DAB standard)
    0o117, // G2
    0o123, // G3
    0o155, // G4 (same as G1)
];

const K: usize = 7; // Constraint length
const NUM_STATES: usize = 1 << (K - 1); // 64 states

/// Compute the parity (number of 1-bits mod 2) of a byte.
#[inline]
fn parity(x: u8) -> u8 {
    let mut v = x;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    v & 1
}

/// Build the branch output table.
/// For each state (0..63) and input bit (0 or 1), compute the 4-bit output.
/// Returns `output[state][input]` as a [u8; 4] (one bit per generator).
///
/// Convention (matching welle.io / DAB standard):
/// - State = upper (K-1) = 6 bits of shift register
/// - Input bit enters at LSB (bit 0)
/// - Register = (state << 1) | input
fn build_output_table() -> Vec<[[u8; 4]; 2]> {
    let mut table = vec![[[0u8; 4]; 2]; NUM_STATES];
    for (state, entry) in table.iter_mut().enumerate() {
        for input in 0..2u8 {
            // Register: state bits at positions 6..1, input at position 0
            let reg = ((state as u8) << 1) | input;
            for (g, poly) in POLY.iter().enumerate() {
                entry[input as usize][g] = parity(reg & poly);
            }
        }
    }
    table
}

/// Viterbi decoder for the DAB convolutional code.
///
/// Initialization: all states start with metric 0 except a small penalty for non-start states.
/// This matches welle.io's behavior where non-start states are initialized to 63 (a small
/// Hamming distance penalty). In our correlation metric (higher = better), this translates
/// to initializing non-start states to a small negative value.
///
/// Renormalization: after each step, metrics are shifted down to prevent overflow.
/// welle.io renormalizes when any metric exceeds 137 (in unsigned 16-bit space).
/// We renormalize by subtracting the maximum metric when it gets large.
pub fn viterbi_decode_with_metric(soft_bits: &[i8]) -> (Vec<u8>, i32) {
    let n_steps = soft_bits.len() / 4;
    if n_steps == 0 {
        return (Vec::new(), 0);
    }

    let output_table = build_output_table();

    // Path metrics: use i32 to avoid overflow with accumulated soft decisions.
    // Match welle.io: start state = 0, all other states = small penalty.
    // In welle.io (Hamming distance, lower=better), non-start states init to 63.
    // In our correlation metric (higher=better), equivalent is a small negative penalty.
    // The value -63 is chosen to match welle.io's penalty magnitude.
    let mut prev_metrics = vec![-63i32; NUM_STATES];
    let mut curr_metrics = vec![i32::MIN / 2; NUM_STATES];
    prev_metrics[0] = 0; // Start in state 0

    // Survivor paths: for each step, for each state, store the predecessor state
    let mut survivors = vec![vec![0u8; NUM_STATES]; n_steps];

    // Renormalization threshold: prevent metrics from growing unbounded.
    // With soft bits in [-127, 127] and 4 per step, max branch metric per step is 4*127=508.
    // Renormalize when max metric exceeds this to keep values manageable.
    const RENORM_THRESHOLD: i32 = 100_000;

    for (step, survivor_row) in survivors.iter_mut().enumerate() {
        // Reset current metrics
        for m in curr_metrics.iter_mut() {
            *m = i32::MIN / 2;
        }

        let base = step * 4;
        let s0 = soft_bits[base] as i32;
        let s1 = soft_bits[base + 1] as i32;
        let s2 = soft_bits[base + 2] as i32;
        let s3 = soft_bits[base + 3] as i32;

        for state in 0..NUM_STATES {
            if prev_metrics[state] <= i32::MIN / 2 + 1000 {
                continue; // Unreachable state
            }

            for input in 0..2u8 {
                let next_state = ((state << 1) | input as usize) & (NUM_STATES - 1);
                let out = &output_table[state][input as usize];

                // Branch metric: correlation between received soft bits and expected output.
                // Convention: positive soft bit = likely '1', negative = likely '0'.
                // (welle.io: ibits[i] = -real(r1)*scale, so transmitted 1 → positive soft)
                // For expected output bit b:
                //   b=1: we want positive soft → metric += +soft
                //   b=0: we want negative soft → metric += -soft
                let mut branch_metric = 0i32;
                branch_metric += if out[0] == 1 { s0 } else { -s0 };
                branch_metric += if out[1] == 1 { s1 } else { -s1 };
                branch_metric += if out[2] == 1 { s2 } else { -s2 };
                branch_metric += if out[3] == 1 { s3 } else { -s3 };

                let candidate = prev_metrics[state] + branch_metric;
                if candidate > curr_metrics[next_state] {
                    curr_metrics[next_state] = candidate;
                    survivor_row[next_state] = state as u8;
                }
            }
        }

        std::mem::swap(&mut prev_metrics, &mut curr_metrics);

        // Renormalize: subtract the maximum metric to keep values bounded
        // This is equivalent to welle.io's renormalization (subtract min in distance domain)
        let max_metric = prev_metrics.iter().copied().max().unwrap_or(0);
        if max_metric > RENORM_THRESHOLD {
            for m in prev_metrics.iter_mut() {
                if *m > i32::MIN / 2 + 1000 {
                    *m -= max_metric;
                }
            }
        }
    }

    // Traceback: find the best final state (ideally state 0 for terminated codes,
    // but DAB FIC is not tail-biting, so pick the best overall state)
    let mut best_state = 0usize;
    let mut best_metric = prev_metrics[0];
    for (state, &metric) in prev_metrics.iter().enumerate() {
        if metric > best_metric {
            best_metric = metric;
            best_state = state;
        }
    }

    // Trace back through survivors
    let mut decoded = vec![0u8; n_steps];
    let mut state = best_state;
    for step in (0..n_steps).rev() {
        // The input bit that led to `state` is the LSB of `state`
        // (because next_state = (prev_state << 1 | input) & mask)
        decoded[step] = (state & 1) as u8;
        state = survivors[step][state] as usize;
    }

    (decoded, best_metric)
}

/// Same as viterbi_decode_with_metric but always traces back from state 0 (for tail-terminated codes).
/// Uses the same initialization (small penalty for non-start states).
pub fn viterbi_decode_state0(soft_bits: &[i8]) -> (Vec<u8>, i32) {
    let n_steps = soft_bits.len() / 4;
    if n_steps == 0 {
        return (Vec::new(), 0);
    }

    let output_table = build_output_table();

    // Match welle.io: small penalty for non-start states
    let mut prev_metrics = vec![-63i32; NUM_STATES];
    let mut curr_metrics = vec![i32::MIN / 2; NUM_STATES];
    prev_metrics[0] = 0;

    let mut survivors = vec![vec![0u8; NUM_STATES]; n_steps];

    const RENORM_THRESHOLD: i32 = 100_000;

    for (step, survivor_row) in survivors.iter_mut().enumerate() {
        for m in curr_metrics.iter_mut() {
            *m = i32::MIN / 2;
        }

        let base = step * 4;
        let s0 = soft_bits[base] as i32;
        let s1 = soft_bits[base + 1] as i32;
        let s2 = soft_bits[base + 2] as i32;
        let s3 = soft_bits[base + 3] as i32;

        for state in 0..NUM_STATES {
            if prev_metrics[state] <= i32::MIN / 2 + 1000 {
                continue;
            }

            for input in 0..2u8 {
                let next_state = ((state << 1) | input as usize) & (NUM_STATES - 1);
                let out = &output_table[state][input as usize];

                let mut branch_metric = 0i32;
                branch_metric += if out[0] == 1 { s0 } else { -s0 };
                branch_metric += if out[1] == 1 { s1 } else { -s1 };
                branch_metric += if out[2] == 1 { s2 } else { -s2 };
                branch_metric += if out[3] == 1 { s3 } else { -s3 };

                let candidate = prev_metrics[state] + branch_metric;
                if candidate > curr_metrics[next_state] {
                    curr_metrics[next_state] = candidate;
                    survivor_row[next_state] = state as u8;
                }
            }
        }

        std::mem::swap(&mut prev_metrics, &mut curr_metrics);

        // Renormalize to prevent overflow
        let max_metric = prev_metrics.iter().copied().max().unwrap_or(0);
        if max_metric > RENORM_THRESHOLD {
            for m in prev_metrics.iter_mut() {
                if *m > i32::MIN / 2 + 1000 {
                    *m -= max_metric;
                }
            }
        }
    }

    // Always trace back from state 0 (tail-terminated code)
    let best_metric = prev_metrics[0];
    let mut decoded = vec![0u8; n_steps];
    let mut state = 0usize;
    for step in (0..n_steps).rev() {
        decoded[step] = (state & 1) as u8;
        state = survivors[step][state] as usize;
    }

    (decoded, best_metric)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parity() {
        assert_eq!(parity(0b0000_0000), 0);
        assert_eq!(parity(0b0000_0001), 1);
        assert_eq!(parity(0b0000_0011), 0);
        assert_eq!(parity(0b0111_1111), 1);
    }

    #[test]
    fn test_output_table_state0() {
        let table = build_output_table();
        // State 0, input 0: register = 0b0_000000, all polys give parity(0) = 0
        assert_eq!(table[0][0], [0, 0, 0, 0]);
        // State 0, input 1: register = 0b1_000000 = 0x40
        // G1=0o133=0x5B: 0x40 & 0x5B = 0x40 = parity(1) = 1
        // G2=0o171=0x79: 0x40 & 0x79 = 0x40 = parity(1) = 1
        // G3=0o145=0x65: 0x40 & 0x65 = 0x40 = parity(1) = 1
        // G4=0o133=0x5B: same as G1 = 1
        assert_eq!(table[0][1], [1, 1, 1, 1]);
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        // Encode a short message, then decode and verify
        let message = [1u8, 0, 1, 1, 0, 0, 1, 0];
        let table = build_output_table();

        // Encode
        let mut encoded = Vec::new();
        let mut state = 0usize;
        for &bit in &message {
            let out = table[state][bit as usize];
            for &b in &out {
                // Convert to soft: 1 -> +100 (positive = likely '1'), 0 -> -100
                encoded.push(if b == 1 { 100i8 } else { -100i8 });
            }
            state = ((state << 1) | bit as usize) & (NUM_STATES - 1);
        }
        // Add tail bits to flush the encoder
        for _ in 0..(K - 1) {
            let out = table[state][0];
            for &b in &out {
                encoded.push(if b == 1 { 100i8 } else { -100i8 });
            }
            state = (state << 1) & (NUM_STATES - 1);
        }

        let (decoded, _) = viterbi_decode_with_metric(&encoded);
        assert_eq!(&decoded[..message.len()], &message);
    }

    /// Encode random data with the rate-1/4 code, puncture with PI_16 pattern [1,1,1,0],
    /// depuncture, and check autocorrelation at lag 4.
    /// This establishes a baseline for what autocorrelation to expect from correctly coded data.
    #[test]
    fn test_autocorrelation_of_coded_data() {
        use crate::fec::eep;

        let table = build_output_table();
        // Generate 774 pseudo-random bits (768 data + 6 tail) to match FIC sub-block
        let mut message = Vec::with_capacity(774);
        let mut lfsr: u32 = 0xACE1; // arbitrary seed
        for _ in 0..768 {
            lfsr ^= lfsr << 13;
            lfsr ^= lfsr >> 17;
            lfsr ^= lfsr << 5;
            message.push((lfsr & 1) as u8);
        }
        // 6 tail bits (zeros)
        message.extend(std::iter::repeat_n(0u8, 6));

        // Encode to rate-1/4 → 774 * 4 = 3096 hard bits
        let mut encoded_hard = Vec::with_capacity(3096);
        let mut state = 0usize;
        for &bit in &message {
            let out = table[state][bit as usize];
            encoded_hard.extend_from_slice(&out);
            state = ((state << 1) | bit as usize) & (NUM_STATES - 1);
        }
        assert_eq!(encoded_hard.len(), 3096);

        // Puncture with FIC pattern (PI_16 × 21 blocks, PI_15 × 3 blocks, PI_TAIL)
        // to get 2304 transmitted bits
        let mut punctured = Vec::new();
        let mut idx = 0;
        // 21 blocks × 4 reps × 32 positions with PI_16
        for _ in 0..21 {
            for _ in 0..4 {
                for &p in &eep::PI_16 {
                    if p == 1 {
                        punctured.push(encoded_hard[idx]);
                    }
                    idx += 1;
                }
            }
        }
        // 3 blocks × 4 reps × 32 positions with PI_15
        for _ in 0..3 {
            for _ in 0..4 {
                for &p in &eep::PI_15 {
                    if p == 1 {
                        punctured.push(encoded_hard[idx]);
                    }
                    idx += 1;
                }
            }
        }
        // Tail: 24 positions with PI_TAIL
        for &p in &eep::PI_TAIL {
            if p == 1 {
                punctured.push(encoded_hard[idx]);
            }
            idx += 1;
        }
        assert_eq!(idx, 3096);
        assert_eq!(punctured.len(), 2304);

        // Convert hard bits to soft bits: 1 → +100 (positive = likely '1'), 0 → -100
        let soft_punctured: Vec<i8> = punctured
            .iter()
            .map(|&b| if b == 1 { 100i8 } else { -100i8 })
            .collect();

        // Depuncture
        let depunctured = eep::depuncture_fic(&soft_punctured);
        assert_eq!(depunctured.len(), 3096);

        // Check autocorrelation at lag 4
        let mut ac_sum = 0i64;
        let mut ac_pow = 0i64;
        for i in 0..depunctured.len() - 4 {
            ac_sum += depunctured[i] as i64 * depunctured[i + 4] as i64;
            ac_pow += (depunctured[i] as i64) * (depunctured[i] as i64);
        }
        let ac_coeff = ac_sum as f64 / ac_pow.max(1) as f64;
        eprintln!(
            "  Autocorrelation at lag 4 of properly encoded+punctured+depunctured data: {:.4}",
            ac_coeff
        );

        // Decode with Viterbi and verify roundtrip
        let (decoded, metric) = viterbi_decode_with_metric(&depunctured);
        assert_eq!(&decoded[..768], &message[..768]);
        eprintln!("  Viterbi metric for clean coded data: {}", metric);

        // Path metric as fraction of achievable
        let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
        eprintln!(
            "  Path metric fraction: {:.1}%",
            metric as f64 / achievable as f64 * 100.0
        );

        // Test with inverted soft bits
        let inverted: Vec<i8> = depunctured.iter().map(|&x| x.saturating_neg()).collect();
        let (_inv_bits, inv_metric) = viterbi_decode_with_metric(&inverted);
        eprintln!(
            "  Inverted metric: {}, diff: {}",
            inv_metric,
            (metric - inv_metric).abs()
        );

        // Now test with noisy data: add ±30 noise to soft bits
        let noisy: Vec<i8> = depunctured
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                if x == 0 {
                    0 // keep erasures
                } else {
                    let noise = ((i as i32 * 7 + 13) % 61 - 30) as i8;
                    x.saturating_add(noise)
                }
            })
            .collect();
        let (noisy_decoded, noisy_metric) = viterbi_decode_with_metric(&noisy);
        let noisy_achievable: i32 = noisy.iter().map(|&x| (x as i32).abs()).sum();
        let noisy_correct = noisy_decoded
            .iter()
            .zip(message.iter())
            .take(768)
            .filter(|&(&a, &b)| a == b)
            .count();
        let noisy_inverted: Vec<i8> = noisy.iter().map(|&x| x.saturating_neg()).collect();
        let (_ni_bits, ni_metric) = viterbi_decode_with_metric(&noisy_inverted);
        eprintln!(
            "  Noisy metric: {} / {} achievable ({:.1}%), correct={}/768, inv_diff={}",
            noisy_metric,
            noisy_achievable,
            noisy_metric as f64 / noisy_achievable as f64 * 100.0,
            noisy_correct,
            (noisy_metric - ni_metric).abs()
        );

        // Test with truly random soft bits (should give ~50% metric)
        let random_soft: Vec<i8> = (0..3096)
            .map(|i| {
                if eep::PI_16[i % 32] == 0 && i < 21 * 128 {
                    0 // simulate erasure
                } else {
                    ((i as i32 * 997 + 311) % 255 - 127) as i8
                }
            })
            .collect();
        let (_rand_decoded, rand_metric) = viterbi_decode_with_metric(&random_soft);
        let rand_achievable: i32 = random_soft.iter().map(|&x| (x as i32).abs()).sum();
        let rand_inverted: Vec<i8> = random_soft.iter().map(|&x| x.saturating_neg()).collect();
        let (_ri_bits, ri_metric) = viterbi_decode_with_metric(&rand_inverted);
        eprintln!(
            "  Random metric: {} / {} achievable ({:.1}%), inv_diff={}",
            rand_metric,
            rand_achievable,
            rand_metric as f64 / rand_achievable as f64 * 100.0,
            (rand_metric - ri_metric).abs()
        );
    }
}
