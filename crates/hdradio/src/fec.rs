//! NRSC-5 convolutional FEC primitives.

/// Constraint length of the FM logical-channel convolutional code.
const CONSTRAINT_LENGTH: usize = 7;
const STATE_COUNT: usize = 1 << (CONSTRAINT_LENGTH - 1);
/// NRSC-5 FM rate-1/3 generator polynomials (octal).
const GENERATORS: [u8; 3] = [0o133, 0o171, 0o165];
const TAIL_BITING_EXTRA: usize = 32;

/// Decode a rate-1/3, K=7 tail-biting convolutional code from signed soft
/// bits. Positive values favour a transmitted one; negative values favour a
/// transmitted zero. The returned metric is the best path correlation score.
pub fn decode_tail_biting(soft_bits: &[i8]) -> Option<(Vec<u8>, i32)> {
    if soft_bits.is_empty() || !soft_bits.len().is_multiple_of(3) {
        return None;
    }
    let steps = soft_bits.len() / 3;
    if steps < TAIL_BITING_EXTRA {
        return None;
    }
    let output = transition_table();
    let extended_steps = steps + 2 * TAIL_BITING_EXTRA;
    let mut previous = vec![0i32; STATE_COUNT];
    let mut survivors = vec![vec![0u8; STATE_COUNT]; extended_steps];

    // Match the reference decoder's circular extension: begin 32 bits before
    // the frame, traverse the complete frame, then continue for 32 bits.
    for (extended_step, survivor) in survivors.iter_mut().enumerate() {
        let step = (steps - TAIL_BITING_EXTRA + extended_step) % steps;
        let received = &soft_bits[step * 3..step * 3 + 3];
        let mut current = vec![i32::MIN / 4; STATE_COUNT];
        for state in 0..STATE_COUNT {
            for input in 0..=1u8 {
                let next = next_state(state, input);
                let metric =
                    previous[state] + branch_metric(received, output[state][input as usize]);
                // The reference butterfly chooses the odd predecessor when
                // path metrics tie (`sum0 > sum1`, otherwise path 1).
                if metric >= current[next] {
                    current[next] = metric;
                    survivor[next] = state as u8;
                }
            }
        }
        previous = current;
    }

    let mut state = 0;
    let mut best_metric = previous[0];
    for (candidate, &metric) in previous.iter().enumerate().skip(1) {
        if metric > best_metric {
            state = candidate;
            best_metric = metric;
        }
    }
    // Discard the final circular convergence tail.
    for extended_step in (steps + TAIL_BITING_EXTRA..extended_steps).rev() {
        state = survivors[extended_step][state] as usize;
    }
    let mut decoded = vec![0u8; steps];
    for step in (0..steps).rev() {
        let path_step = step + TAIL_BITING_EXTRA;
        decoded[step] = ((state >> (CONSTRAINT_LENGTH - 2)) & 1) as u8;
        state = survivors[path_step][state] as usize;
    }
    Some((decoded, best_metric))
}

fn transition_table() -> Vec<[[u8; 3]; 2]> {
    (0..STATE_COUNT)
        .map(|state| {
            std::array::from_fn(|input| {
                let register = state as u8 | ((input as u8) << 6);
                GENERATORS.map(|generator| parity(register & generator))
            })
        })
        .collect()
}

fn next_state(state: usize, input: u8) -> usize {
    (state >> 1) | ((input as usize) << (CONSTRAINT_LENGTH - 2))
}

fn branch_metric(received: &[i8], expected: [u8; 3]) -> i32 {
    received
        .iter()
        .zip(expected)
        .map(|(&received, expected)| {
            if expected == 1 {
                received as i32
            } else {
                -(received as i32)
            }
        })
        .sum()
}

fn parity(value: u8) -> u8 {
    (value.count_ones() & 1) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_tail_biting(bits: &[u8]) -> Vec<i8> {
        let table = transition_table();
        let initial = (0..STATE_COUNT)
            .find(|&candidate| {
                bits.iter()
                    .fold(candidate, |state, &bit| next_state(state, bit))
                    == candidate
            })
            .expect("every bit sequence has one tail-biting state");
        let mut state = initial;
        let mut output = Vec::with_capacity(bits.len() * 3);
        for bit in bits {
            output.extend(table[state][*bit as usize].map(|bit| if bit == 1 { 100 } else { -100 }));
            state = next_state(state, *bit);
        }
        assert_eq!(state, initial);
        output
    }

    #[test]
    fn decodes_tail_biting_reference_sequence() {
        let input: Vec<u8> = (0..80).map(|index| ((index * 37 + 11) & 1) as u8).collect();
        let encoded = encode_tail_biting(&input);
        assert_eq!(decode_tail_biting(&encoded).unwrap().0, input);
    }

    #[test]
    fn corrects_soft_decision_errors() {
        let input: Vec<u8> = (0..80).map(|index| ((index * 19 + 7) & 1) as u8).collect();
        let mut encoded = encode_tail_biting(&input);
        for index in (5..encoded.len()).step_by(47) {
            encoded[index] = -encoded[index] / 4;
        }
        assert_eq!(decode_tail_biting(&encoded).unwrap().0, input);
    }
}
