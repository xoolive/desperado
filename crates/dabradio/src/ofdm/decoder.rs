//! OFDM decoder: DQPSK differential decoding and frequency de-interleaving.

use crate::constants::*;
use num_complex::Complex;
use tracing::trace;

/// Build the frequency de-interleaving permutation table for Mode I.
///
/// Uses the linear congruential generator from ETSI EN 300 401 §14.6:
///   tmp[0] = 0
///   tmp[i] = (13 * tmp[i-1] + V1) mod T_u
///
/// For Mode I: T_u=2048, V1=511, lwb=256, upb=1792.
/// The result maps logical carrier index i (0..K-1) to a physical FFT bin
/// offset from center (range approx [-K/2..+K/2], excluding 0/DC).
///
/// Reference: welle.io src/backend/ofdm/freq-interleaver.cpp
fn freq_deinterleave_table() -> Vec<i16> {
    let v1: i32 = 511;
    let lwb: i32 = 256;
    let upb: i32 = 1792; // lwb + K
    let tu = T_U as i32;
    let half = tu / 2;

    // Generate LCG sequence
    let mut tmp = vec![0i32; T_U];
    for i in 1..T_U {
        tmp[i] = (13 * tmp[i - 1] + v1) % tu;
    }

    // Filter: keep only carriers in the active range [lwb..upb],
    // skip DC (T_u/2), and shift to center-relative indices
    let mut perm_table = Vec::with_capacity(K);
    for t in tmp.iter().take(T_U) {
        let val = *t;
        if val == half {
            continue; // skip DC
        }
        if val < lwb || val > upb {
            continue; // skip out-of-band
        }
        perm_table.push((val - half) as i16); // center-relative
    }

    // Should have exactly K entries
    assert_eq!(perm_table.len(), K);
    perm_table
}

/// Perform DQPSK differential decoding on a sequence of frequency-domain symbols.
///
/// Combines frequency de-interleaving and DQPSK in a single pass, matching
/// the welle.io approach.
///
/// `coarse_offset` is the estimated carrier offset from the PRS analysis.
/// It shifts all FFT bin lookups by this amount.
///
/// Output layout (planar): for each symbol, produces 2*K soft bits:
///   `soft_bits[0..K]`   = I (real) soft bits for logical carriers 0..K-1
///   `soft_bits[K..2*K]` = Q (imaginary) soft bits for logical carriers 0..K-1
///
/// `symbols[0]` is the PRS (reference), `symbols[1..]` are data symbols.
///
/// Note: coarse frequency correction is already applied in the time domain by
/// `OfdmProcessor`, so no bin-index shifting is needed here (matching welle.io).
pub fn dqpsk_decode(symbols: &[Vec<Complex<f32>>]) -> Vec<Vec<i8>> {
    if symbols.len() < 2 {
        return Vec::new();
    }

    let perm_table = freq_deinterleave_table();
    let mut all_soft_bits = Vec::with_capacity(symbols.len() - 1);

    for sym_idx in 1..symbols.len() {
        let prev = &symbols[sym_idx - 1];
        let curr = &symbols[sym_idx];

        let mut soft_bits = vec![0i8; K * 2];

        for i in 0..K {
            // Look up physical FFT bin for logical carrier i
            // Note: coarse freq correction is already applied in time domain by processor,
            // so we do NOT add coarse_offset here (welle.io doesn't either).
            let mut index = perm_table[i] as i32;
            if index < 0 {
                index += T_U as i32;
            }
            let bin = index as usize;

            if bin >= prev.len() || bin >= curr.len() {
                continue;
            }

            // DQPSK: phase difference between same carrier in consecutive symbols
            let r1 = curr[bin] * prev[bin].conj();

            // Scale by L1 norm: 127 / (|re| + |im|)
            let l1 = r1.re.abs() + r1.im.abs();
            let scale = 127.0 / l1.max(1e-10);

            // Our Viterbi convention: negative = likely '1', positive = likely '0'.
            // D-QPSK bit mapping: transmitted bit=1 → positive r1.re (or .im).
            // Negate to match Viterbi: -r1.re gives negative for bit=1 ✓
            // This matches welle.io: ibits[i] = -real(r1) * ab1
            soft_bits[i] = (-r1.re * scale).clamp(-127.0, 127.0) as i8;
            soft_bits[K + i] = (-r1.im * scale).clamp(-127.0, 127.0) as i8;
        }

        // Log soft bit statistics for the first 3 symbols (FIC)
        if sym_idx <= 3 {
            let sum_abs: i64 = soft_bits.iter().map(|&x| (x as i64).abs()).sum();
            let avg_abs = sum_abs as f64 / (K * 2) as f64;
            let count_high: usize = soft_bits.iter().filter(|&&x| x.abs() > 50).count();
            // Print first 20 soft bits for symbol 1
            if sym_idx == 1 {
                let first_20: Vec<i8> = soft_bits[..20].to_vec();
                trace!(
                    sym_idx,
                    avg_abs = format!("{:.1}", avg_abs),
                    count_high,
                    first_20 = ?first_20,
                    "DQPSK soft bit stats"
                );
            } else {
                trace!(
                    sym_idx,
                    avg_abs = format!("{:.1}", avg_abs),
                    count_high,
                    "DQPSK soft bit stats"
                );
            }
        }

        all_soft_bits.push(soft_bits);
    }

    all_soft_bits
}
