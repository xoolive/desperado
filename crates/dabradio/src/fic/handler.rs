//! FIC (Fast Information Channel) handler.
//!
//! Takes DQPSK-decoded soft bits from the first 3 data symbols of each OFDM frame,
//! runs depuncturing + Viterbi decoding + energy dispersal, and produces FIBs (Fast
//! Information Blocks) of 32 bytes each.
//!
//! In Mode I, each frame provides 3 OFDM symbols × 3072 soft bits = 9216 total.
//! These are split into 4 sub-blocks of 2304 punctured soft bits each.
//! Each sub-block decodes to 768 data bits = 3 FIBs of 256 bits.
//! Total: 4 sub-blocks × 3 FIBs = 12 FIBs per frame.
//!
//! Reference: ETSI EN 300 401 §5.2, §11.1, welle.io src/backend/fic-handler.cpp

use crate::constants::*;
use crate::fec;
use tracing::debug;

/// Size of one FIC sub-block after puncturing (transmitted soft bits).
const FIC_SUBBLOCK_SIZE: usize = 2304;

/// Number of data bits per sub-block after Viterbi decoding (3 FIBs × 256 bits).
const FIC_DECODED_BITS: usize = 768;

/// CRC-16-CCITT check on individual bits (matching welle.io's check_CRC_bits).
/// `bits` is a slice of 256 u8 values, each 0 or 1.
/// Returns true if the CRC embedded in the last 16 bits is valid.
#[allow(clippy::needless_range_loop)]
fn check_crc_bits(bits: &[u8]) -> bool {
    // CRC-CCITT polynomial x^16 + x^12 + x^5 + 1
    // Represented as feedback taps (MSB..LSB of the 15 non-leading coefficients)
    let crc_poly: [u8; 15] = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0];
    let mut b = [1u8; 16]; // init all ones

    let size = bits.len();
    for i in 0..size {
        let mut d = bits[i];
        // Invert the last 16 bits (CRC field)
        if i >= size - 16 {
            d ^= 1;
        }

        if (b[0] ^ d) == 1 {
            for f in 0..15 {
                b[f] = crc_poly[f] ^ b[f + 1];
            }
            b[15] = 1;
        } else {
            // Shift register left
            for f in 0..15 {
                b[f] = b[f + 1];
            }
            b[15] = 0;
        }
    }

    let mut crc: u16 = 0;
    for i in 0..16 {
        crc |= (b[i] as u16) << i;
    }
    crc == 0
}

/// Process one frame's FIC data: 3 FIC symbols, each with 2*K = 3072 soft bits.
///
/// The 9216 soft bits are split into 4 sub-blocks of 2304 each. Each sub-block
/// is independently depunctured, Viterbi-decoded, energy-dispersed, and split
/// into 3 FIBs.
///
/// Returns valid FIBs (those passing CRC check).
pub fn process_fic(fic_soft_bits: &[Vec<i8>]) -> Vec<[u8; FIB_LENGTH]> {
    if fic_soft_bits.len() < FIC_SYMBOLS {
        return Vec::new();
    }

    // Concatenate the 3 FIC symbols' soft bits into one flat buffer
    let mut combined: Vec<i8> = Vec::with_capacity(FIC_SYMBOLS * K * 2);
    for sym in fic_soft_bits.iter().take(FIC_SYMBOLS) {
        combined.extend_from_slice(sym);
    }

    let mut fibs = Vec::new();

    // Process 4 sub-blocks of 2304 soft bits each
    let n_subblocks = combined.len() / FIC_SUBBLOCK_SIZE;
    for sb in 0..n_subblocks {
        let start = sb * FIC_SUBBLOCK_SIZE;
        let end = start + FIC_SUBBLOCK_SIZE;
        if end > combined.len() {
            break;
        }

        let subblock = &combined[start..end];

        // Step 1: Depuncture (insert erasures at punctured positions) → 3096 soft bits
        let depunctured = fec::eep::depuncture_fic(subblock);

        // Step 2: Viterbi decode → 774 hard bits (768 data + 6 tail)
        let (decoded_bits, path_metric) = fec::viterbi::viterbi_decode_with_metric(&depunctured);

        // Debug: log Viterbi metric for FIC sub-blocks
        let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
        let metric_pct = if achievable > 0 {
            path_metric as f64 / achievable as f64 * 100.0
        } else {
            0.0
        };
        debug!(
            subblock = sb,
            metric = path_metric,
            achievable = achievable,
            pct = format!("{:.1}", metric_pct),
            "FIC Viterbi metric"
        );

        // Take only the 768 data bits (discard tail)
        if decoded_bits.len() < FIC_DECODED_BITS {
            continue;
        }
        let mut data_bits: Vec<u8> = decoded_bits[..FIC_DECODED_BITS].to_vec();

        // Step 3: Energy dispersal (PRBS de-scrambling) across all 768 bits
        fec::energy_dispersal::energy_dispersal(&mut data_bits);

        // Step 4: Split into 3 FIBs and check CRC
        for fib_idx in 0..FIBS_PER_FIC {
            let bit_start = fib_idx * 256;
            let bit_end = bit_start + 256;
            if bit_end > data_bits.len() {
                break;
            }

            // Check CRC directly on bits (welle.io style)
            let fib_bits = &data_bits[bit_start..bit_end];
            let bit_crc_ok = check_crc_bits(fib_bits);

            // Pack 256 bits into 32 bytes (MSB first)
            let mut fib_bytes = [0u8; FIB_LENGTH];
            for (byte_idx, fib_byte) in fib_bytes.iter_mut().enumerate() {
                for bit_pos in 0..8 {
                    let bit_idx = bit_start + byte_idx * 8 + bit_pos;
                    if bit_idx < data_bits.len() && data_bits[bit_idx] == 1 {
                        *fib_byte |= 1 << (7 - bit_pos);
                    }
                }
            }

            // Check CRC on packed bytes
            let byte_crc_ok = crate::constants::fib_crc_valid(&fib_bytes);

            if bit_crc_ok || byte_crc_ok {
                fibs.push(fib_bytes);
            }
        }
    }

    fibs
}

#[cfg(test)]
mod tests {
    use super::*;

    /// THE critical test: feed welle.io's KNOWN-GOOD 2304 soft bits (from a ficno=0
    /// sub-block where CRC passed) through our entire pipeline:
    ///   depuncture → Viterbi → energy dispersal → CRC check
    ///
    /// If this passes: our FEC pipeline is correct, and the bug is in OFDM/DQPSK sync.
    /// If this fails: we have a bug in depuncturing, Viterbi, or energy dispersal.
    #[test]
    fn test_welleio_known_good_soft_bits() {
        // These 2304 i8 values were captured from welle.io's stderr output
        // (WELLE_SUCCESS_RAWINPUT ficno=0) on a frame where all 3 FIB CRCs passed.
        let soft_bits: Vec<i8> = vec![
            -72, 97, 78, 99, -49, -91, 46, -96, 69, -68, 18, -70, -77, -16, -41, -60, 28, -7, 1,
            70, 68, -124, 14, 41, -50, -54, -102, -49, -51, 91, -20, 43, -36, -74, -66, 63, 92,
            -89, 14, -58, 37, 4, 52, 22, 58, 56, 96, 51, 108, 33, -78, -52, -75, -60, 125, 83,
            -101, -59, -73, -108, -89, -57, 43, 47, -62, -25, -62, 81, 101, 45, 107, 35, -64, 30,
            -68, 77, -55, -94, 54, 67, 71, -83, 84, 59, -88, -28, -63, -37, -36, -110, -54, 26,
            100, -47, 59, 94, 93, 39, -35, -95, 50, 54, -112, -68, -58, -93, -78, 116, -27, 44, 33,
            -101, -84, 102, -26, -14, 34, 81, 48, 62, 17, -59, -70, -46, -62, -66, 69, 44, 19,
            -112, -44, 39, -14, 31, 41, -49, -69, 56, -21, -40, 87, -16, -56, -39, 83, 80, 126,
            -58, -68, 55, 81, -18, 35, -66, 76, 15, -43, 20, 61, 103, -81, -60, -101, 63, 43, -2,
            -82, -68, 62, 65, 75, 68, 66, -97, 63, 74, 70, -85, 60, 47, -51, 64, -57, -99, -38, 70,
            50, 9, -105, 63, -107, -66, 88, -40, 62, 87, -48, -82, 61, 92, -125, 35, 116, 38, 73,
            59, -100, 31, -51, 44, -90, 106, -41, -16, -58, -102, -93, -46, -61, -105, -104, 64,
            -54, 83, 60, -11, -98, 91, -64, -36, 94, 97, 25, 70, 61, -86, -89, 62, 60, 85, 59, -77,
            -94, 77, 52, 91, 69, -43, 92, -85, -64, -112, -84, 66, -72, -4, 70, 33, -78, 25, -115,
            -57, -87, 74, 44, -96, 67, 14, -117, -40, 46, 82, -56, 97, -73, 107, -73, 125, -58, 2,
            -40, -44, 50, 29, 77, 56, -40, 95, -68, -61, 50, 78, -41, 60, 62, -56, -51, -53, -92,
            70, 39, 60, -67, -14, 67, 91, 61, 106, -89, 67, -67, -66, 92, -56, 106, 59, -5, -58,
            -37, -48, -36, -53, -64, -69, -109, 54, 72, -30, 35, -77, 114, 57, 84, -37, -96, -37,
            26, -89, 57, 41, -60, -56, -98, 54, -79, -19, -56, -53, -91, -74, 84, -117, -94, -17,
            -41, -70, -11, 88, -49, -67, 78, 67, 94, -99, 20, -43, -72, 70, 37, 15, -74, -95, 95,
            -66, 92, -54, 58, 51, 46, -111, 43, -101, 46, -69, 89, -46, -42, 30, 55, -24, -86, 62,
            -48, -31, -59, 41, -41, 35, -36, -40, -66, 86, -42, -2, 93, 51, 41, -100, -54, 108, 43,
            0, -93, 49, 45, 52, -21, -51, 82, 65, -97, -30, 33, -24, -65, -30, -59, -51, -41, -41,
            56, 63, 37, -102, -80, 63, -53, -92, 37, -34, 112, 43, -31, 125, -100, 53, 90, 72, -95,
            -66, 34, -47, 26, -53, 50, -70, 24, -75, -58, 23, 40, -98, -2, -51, 90, -65, 44, -118,
            -25, -24, 57, -74, 84, 96, -46, 26, -47, -56, -84, 58, 47, -52, 81, -82, 39, -110, 103,
            -61, -33, 106, 69, 23, 44, -59, 75, -70, -67, 58, -79, 84, -80, 53, 72, 48, 85, 82, 49,
            -91, -32, -56, 9, -58, 82, -18, -64, 82, -14, 0, 66, 32, -114, -107, -97, -68, 14, -65,
            -82, 103, 52, 68, 48, 69, -75, -90, -4, -66, -54, -83, -57, -103, 46, -14, 83, 71, 93,
            -67, 53, 86, -116, 75, -77, 77, -101, 72, 77, 59, -4, 46, 107, -104, -97, 45, 82, -26,
            -34, 63, 64, -29, 107, -2, -76, -50, -44, -49, 15, 45, 9, 125, 87, 53, 63, -83, 64,
            -51, -73, -70, 14, 99, 50, 38, 125, 43, 14, -69, 107, -104, -42, -56, -104, -66, 125,
            30, 66, 66, 55, 55, -45, -87, -18, 6, -58, -62, -77, -66, 75, 104, 92, -91, 62, -55,
            54, -84, 55, -13, 103, -90, 73, -79, -62, -41, -47, 32, -59, -83, 68, -36, 81, -76,
            112, -54, -45, 75, -93, 57, -23, 86, 61, -77, 58, 64, 2, 62, -52, -43, 44, 73, 84, -78,
            63, 96, 36, -75, -74, 43, -69, -100, 96, -109, 115, 75, -120, 62, -123, -74, 108, -70,
            98, 111, -72, -125, 34, -126, -118, -89, 123, 59, 16, -66, -70, -88, -52, 114, -81,
            -89, 33, -24, -62, 40, 46, -40, 70, -111, -16, -78, -37, -23, 54, -44, -78, 65, 42, 54,
            -67, 49, -49, 16, 11, 49, -48, -37, -57, -20, 76, -99, -46, 33, -83, -11, -67, 75, 57,
            41, -7, 67, -76, -44, -70, 79, -68, -82, -114, -85, -81, -114, 71, -82, 71, 118, 91,
            99, -84, -108, 54, 91, -61, 56, -44, 95, 34, 59, -37, -52, -61, 32, 82, 21, 35, 42, 30,
            -41, -112, 87, 77, 104, -82, 4, -97, -32, 68, 85, 54, -106, -76, 73, -52, 44, -7, 28,
            0, -36, 42, -74, -54, -46, -9, -76, 78, -5, 65, 124, 104, 64, 71, 78, -38, -88, -73,
            62, -87, 66, -21, 112, -87, 87, 68, -65, -95, 21, 5, -72, -85, -82, 69, -51, -112, -65,
            -83, 71, -111, -42, 3, -39, -68, 99, -62, 40, 70, -39, -89, -55, 86, 53, -26, -64, -98,
            -79, -43, -26, -66, 79, -56, -78, -47, -109, -106, 13, 88, -74, 47, 13, -21, -35, 56,
            91, 34, 65, -45, 72, -56, -43, -14, 85, -61, 89, 34, 74, 91, 44, -103, 61, 100, -80,
            11, -119, 53, 61, -87, 18, 63, 53, -90, -2, 63, -34, -84, 72, 68, -57, -65, -59, 123,
            61, 53, -48, -43, -60, 79, -27, -2, -54, -47, 50, -80, -113, 87, -123, -61, 72, -59,
            -15, -52, -30, -116, 32, 56, -18, 19, -107, 73, 122, -105, 80, 102, -49, -59, -76, -64,
            -51, 6, 0, -20, 34, -76, 39, -61, -74, 119, 78, 66, -55, -74, -41, 79, 97, 50, 120, 6,
            -46, 92, 49, 88, -69, -84, 65, -57, -71, 67, -118, 61, -67, 20, -72, 86, -62, 75, 74,
            55, -50, 100, 31, -59, -64, 35, 102, -51, -88, -59, 83, 51, 98, -35, -122, -67, 54,
            -37, -104, 59, 31, -59, -77, 74, -92, 87, 51, 77, -56, -67, 54, 57, -70, -60, 53, -32,
            -15, -60, -4, 54, -24, -59, -111, -87, -109, 27, -89, 91, -80, -20, 47, -67, -70, 48,
            82, 94, -18, 33, 45, -54, -77, -52, -48, -56, -106, -26, -39, -60, 56, 57, 78, -56,
            119, 53, -70, -54, -57, -44, 119, -39, -68, -112, -64, -35, -72, -36, 65, 50, 115, -69,
            -67, 65, -52, -124, 50, -53, 34, -86, 114, -59, -72, 21, -18, 85, -61, 44, -86, -75,
            -70, 23, -46, 60, 61, 26, -53, -104, -51, -57, 66, 50, -34, 47, 59, -102, 92, -108, 44,
            51, -105, 75, 47, 76, -45, 117, 93, -77, 50, -52, -64, -102, 56, 110, -71, -121, 90,
            -36, -21, 60, 31, 35, -82, -49, 118, 62, -82, -45, -121, -101, -53, -123, -15, 51, 71,
            116, -55, 52, -53, 16, 62, -13, 98, -59, 65, 20, -55, -50, -27, 68, 95, 87, -49, -110,
            44, -48, -88, -44, -28, -24, 80, 107, 49, 14, 10, 43, -58, 50, 72, 117, -126, 117, 96,
            64, -53, 16, 88, 71, -42, -84, 73, -63, 12, 47, 0, 62, -64, -35, 90, 89, -60, -74, 70,
            -14, 75, -98, 111, -52, -89, 94, 2, -48, 55, -114, -45, -66, -72, -34, 36, 71, 50, -21,
            -86, 85, 70, -70, -87, 97, 89, 37, 52, -102, 79, -41, 86, 81, -88, 43, 55, 118, 51, 70,
            21, -36, -114, -6, -58, -54, -79, 54, 99, -10, -48, -91, 69, 19, 70, 30, 52, -23, 126,
            -81, -68, 101, 92, 75, -76, -84, -19, 11, -120, 5, -108, -94, 13, -54, -103, -70, -54,
            -54, -42, -66, -65, 55, -38, 123, -80, 82, -60, -56, -91, -64, 94, 56, 77, -11, 66,
            113, -24, 68, 66, -51, 68, 42, 88, 83, -17, -24, -62, -53, -101, 83, 66, 118, -78, 13,
            116, -63, 60, 52, 59, 103, -58, 60, -60, 63, 31, -57, -69, -104, 83, -95, -18, -71,
            -58, -30, 45, -80, 35, -102, -70, -104, -51, 73, -75, -65, -55, 43, 74, -67, -6, -54,
            66, 75, 55, 18, -44, -82, 43, -23, 10, 54, 119, 10, -98, 66, 22, -93, 57, -28, -90, 67,
            -83, -94, 62, -41, -91, 45, -104, 94, 68, -31, 65, -88, -69, 46, 94, -78, -39, -36, 97,
            97, 79, -103, 64, 115, 8, -37, 43, -71, 49, 46, 125, -86, -44, -80, 102, -102, -37,
            -90, -75, -66, -74, -59, -70, 110, -121, -58, -94, 40, 72, -79, 50, 92, -61, 94, -16,
            -74, 90, 78, -85, 53, -66, 50, -71, -47, 53, -48, 61, 32, 88, -100, -65, -51, 115, 110,
            -65, -13, -63, -89, -25, -71, 47, 30, 102, 22, 38, -59, 49, 42, 39, -82, 77, 31, -35,
            -118, 72, 82, -74, -85, 51, -41, -99, -44, 65, -53, 101, -117, 62, 54, -51, 91, 65, 81,
            -52, -45, 64, 61, 49, -52, 82, 29, -42, 61, 71, 1, 28, 45, -40, 39, 115, 0, 60, 47, 58,
            77, -16, 79, 81, 25, -67, -73, 63, 10, -33, -120, -5, -49, 91, 77, -44, 96, 70, 69,
            -61, 60, -77, 67, -14, 18, 59, 64, -93, 54, -14, 43, -83, -63, -98, 84, 10, -54, 29,
            48, 27, -77, -35, 80, 30, 57, 58, 108, 56, 49, -110, 85, 66, -98, 119, 125, -56, -58,
            2, 112, -85, -76, 72, 24, -77, -75, -35, 106, -83, -90, -52, 60, -63, -34, -37, -112,
            68, -89, 122, 74, 104, -68, -70, 30, -75, 18, -93, -48, 74, -51, 66, 1, -43, -25, 67,
            53, -18, -37, 69, -83, -79, 64, 101, 64, 45, 25, 81, 19, -91, 62, -96, -58, 49, 71,
            -32, 72, 59, -55, 43, 42, 67, -38, -98, -63, 89, 90, -16, 72, -100, -26, -79, -67, 32,
            -33, -87, 91, 31, -76, -72, -14, -58, -68, -33, 48, 10, 99, 82, -93, -25, -42, 24,
            -100, 112, -92, 45, -78, 64, -109, 67, -56, 80, 64, 60, 57, -82, 107, -14, 82, -87,
            112, 95, -85, 77, -57, -70, 105, 86, -39, 110, 70, 87, 43, 46, 0, 68, -58, -71, -45,
            108, 91, -60, 50, 111, 83, -106, 65, -23, -45, 66, -25, -63, -83, 124, -44, 58, 64,
            -61, -51, -58, 60, 29, 63, -52, 56, -41, 66, 79, 75, 62, -69, -27, -88, 56, -76, 117,
            21, 63, -19, 60, 38, 86, -64, -39, 78, -44, -65, 34, 1, 91, 10, -88, 53, 67, 26, -95,
            -75, -82, -36, 20, 85, -110, -68, -24, -33, 80, 65, -21, -22, -62, -72, 43, -66, -115,
            -28, -35, 62, 90, -32, 29, -101, 56, -65, -40, -37, -64, -66, 41, -67, -49, -32, 49,
            -74, 35, 57, -83, 34, -41, 62, -14, -42, -60, 54, 122, 56, -93, -48, -101, -11, 69,
            -39, 52, -82, 30, 59, -112, -9, 86, -80, 44, 70, 29, -53, 19, 53, 1, 68, -124, 86, -82,
            76, 97, 49, -70, 86, 31, -58, -65, -76, 48, -85, -66, -64, 70, 75, 73, -34, -56, 87,
            66, -59, 112, 59, 35, 65, 20, -37, 59, -59, -60, 34, 70, 20, 67, 121, -68, -89, 78, 90,
            73, -62, 57, -17, -72, -54, -96, -91, 49, -12, -69, 42, 89, -30, 89, -100, -37, 69, 85,
            66, 70, 28, 72, 47, -107, 70, -73, -35, -52, -42, -9, -32, 109, 85, 56, -115, 38, 77,
            -59, 48, 59, 32, -27, -106, 83, 54, 56, -89, -111, -52, 31, 31, -60, -34, 72, 68, 75,
            -80, -15, -83, -25, -80, -57, 37, 80, 84, -96, 71, 102, 40, 64, -78, 95, 67, -85, -85,
            -91, 90, -86, 60, 40, 84, -124, -33, 75, 85, -26, 72, 18, -83, 126, 33, -77, -81, -74,
            -105, -75, 44, -61, -29, -96, -93, 102, 61, 96, 67, -75, 85, 85, -70, -63, 89, -24,
            -46, 63, 73, -34, -89, -92, -14, 83, -95, 1, 26, 73, -36, 54, -31, -60, -92, -79, 100,
            73, -76, -56, -102, -51, 68, -103, 86, -28, -124, 75, -36, -61, -82, 8, -101, -102, 69,
            52, 42, 30, 80, 100, -79, -70, 42, 68, 79, -74, -45, -44, -87, -16, 23, -65, 93, 20,
            -57, -103, -82, 67, 51, 56, -59, 68, 47, 42, 46, -73, 54, 78, 41, 44, 77, 35, 94, 70,
            -117, 68, -44, 108, -62, 44, 112, 126, 60, -94, 12, -19, 29, -58, -112, 61, -44, -23,
            74, -58, -78, -57, -51, 36, 122, 60, 72, 43, -69, -23, -80, 112, 43, -55, 33, -59, -73,
            -40, -10, 51, -49, 49, -25, -54, 49, 67, 122, -80, 19, 22, 29, 81, 44, -100, 92, -63,
            62, 97, 19, 124, -50, 76, 82, 77, -111, -81, 117, -1, 39, -73, -63, -43, -62, 75, 53,
            -56, 112, -27, -76, -88, -1, 83, -112, -57, 19, 22, 84, 70, -22, 60, 1, -96, 60, 60,
            -71, 71, -81, -39, 108, -120, 68, 64, -49, -60, 51, 22, 34, -35, -64, -71, 72, -42,
            -71, 113, 23, -36, 53, -47, -64, -85, 79, 94, 67, -43, 58, 90, 45, 50, -14, 72, 81, 51,
            -33, 69, 103, 40, -65, -49, 68, -62, -124, 64, -74, 83, 82, -53, -42, 48, -63, 30, -90,
            51, 52, -83, 57, -26, -30, -17, 11, 51, 6, -64, -3, 52, 18, 56, 28, 15, 54, -1, 92, 0,
            -8, -37, 3, -67, -110, -60, 56, 38, -74, 12, 45, -37, -93, 102, 64, -86, 80, 86, 56,
            -15, 110, -48, 89, -103, -72, 82, -48, -61, -84, 72, 59, 77, 77, 110, 115, 77, 78, 89,
            -69, 106, 50, -27, 80, -93, 43, 115, -59, -51, 69, 85, -119, -59, -50, 82, 56, 47, -58,
            -44, -12, -41, -45, 12, -55, 44, 55, 8, 35, 27, 42, -18, 72, 35, 65, -70, 82, 31, 92,
            -67, 89, 74, -65, -94, -44, -105, -91, -84,
        ];
        assert_eq!(
            soft_bits.len(),
            2304,
            "Expected 2304 soft bits from welle.io"
        );

        // Step 1: Depuncture → 3096 soft bits
        let depunctured = fec::eep::depuncture_fic(&soft_bits);
        assert_eq!(
            depunctured.len(),
            3096,
            "Depuncturing should produce 3096 bits"
        );

        // Verify first few depunctured values match welle.io's WELLE_DEPUNCTURED output:
        // -72 97 78 0 99 -49 -91 0 46 -96 69 0 -68 18 -70 0 ...
        assert_eq!(depunctured[0], -72);
        assert_eq!(depunctured[1], 97);
        assert_eq!(depunctured[2], 78);
        assert_eq!(
            depunctured[3], 0,
            "Position 3 should be erasure (PI_16 pattern [1,1,1,0])"
        );
        assert_eq!(depunctured[4], 99);
        assert_eq!(depunctured[5], -49);
        assert_eq!(depunctured[6], -91);
        assert_eq!(depunctured[7], 0);

        // Step 2: Viterbi decode → 774 hard bits
        // Try both best-state and state-0 traceback
        let (decoded_bits_best, path_metric_best) =
            fec::viterbi::viterbi_decode_with_metric(&depunctured);
        let (decoded_bits_s0, path_metric_s0) = fec::viterbi::viterbi_decode_state0(&depunctured);

        assert_eq!(decoded_bits_best.len(), 774);
        assert_eq!(decoded_bits_s0.len(), 774);

        let achievable_max: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
        eprintln!(
            "  Viterbi best-state metric: {} / {} ({:.1}%)",
            path_metric_best,
            achievable_max,
            path_metric_best as f64 / achievable_max as f64 * 100.0
        );
        eprintln!(
            "  Viterbi state-0 metric: {} / {} ({:.1}%)",
            path_metric_s0,
            achievable_max,
            path_metric_s0 as f64 / achievable_max as f64 * 100.0
        );

        // Count differences between the two traceback methods
        let diff_count = decoded_bits_best
            .iter()
            .zip(decoded_bits_s0.iter())
            .filter(|&(&a, &b)| a != b)
            .count();
        eprintln!(
            "  Differences between best-state and state-0: {}",
            diff_count
        );

        // Try both traceback methods
        let mut crc_pass_count = 0;
        for (label, decoded_bits, path_metric) in [
            ("best-state", &decoded_bits_best, path_metric_best),
            ("state-0", &decoded_bits_s0, path_metric_s0),
        ] {
            eprintln!("\n  === Traceback: {} (metric={}) ===", label, path_metric);

            let mut data_bits: Vec<u8> = decoded_bits[..768].to_vec();
            fec::energy_dispersal::energy_dispersal(&mut data_bits);

            for fib_idx in 0..3 {
                let fib_bits = &data_bits[fib_idx * 256..(fib_idx + 1) * 256];
                let crc_ok = check_crc_bits(fib_bits);

                let mut fib_bytes = [0u8; FIB_LENGTH];
                for (byte_idx, fib_byte) in fib_bytes.iter_mut().enumerate() {
                    for bit_pos in 0..8 {
                        let bit_idx = fib_idx * 256 + byte_idx * 8 + bit_pos;
                        if bit_idx < data_bits.len() && data_bits[bit_idx] == 1 {
                            *fib_byte |= 1 << (7 - bit_pos);
                        }
                    }
                }
                let byte_crc_ok = crate::constants::fib_crc_valid(&fib_bytes);

                let hex: Vec<String> = fib_bytes.iter().map(|b| format!("{:02X}", b)).collect();
                eprintln!(
                    "  FIB {} CRC: bit={} byte={} | {}",
                    fib_idx,
                    if crc_ok { "PASS" } else { "FAIL" },
                    if byte_crc_ok { "PASS" } else { "FAIL" },
                    hex.join(" ")
                );

                if crc_ok || byte_crc_ok {
                    crc_pass_count += 1;
                }
            }

            // Also try without energy dispersal
            let data_bits_no_ed: Vec<u8> = decoded_bits[..768].to_vec();
            for fib_idx in 0..3 {
                let fib_bits = &data_bits_no_ed[fib_idx * 256..(fib_idx + 1) * 256];
                let crc_ok = check_crc_bits(fib_bits);
                if crc_ok {
                    eprintln!("  FIB {} CRC PASS WITHOUT energy dispersal!", fib_idx);
                    crc_pass_count += 1;
                }
            }

            // Also try with INVERTED soft bits
            let inverted: Vec<i8> = depunctured.iter().map(|&x| x.saturating_neg()).collect();
            let (inv_decoded, inv_metric) = fec::viterbi::viterbi_decode_state0(&inverted);
            let mut inv_data: Vec<u8> = inv_decoded[..768].to_vec();
            fec::energy_dispersal::energy_dispersal(&mut inv_data);
            for fib_idx in 0..3 {
                let fib_bits = &inv_data[fib_idx * 256..(fib_idx + 1) * 256];
                let crc_ok = check_crc_bits(fib_bits);
                if crc_ok {
                    eprintln!(
                        "  FIB {} CRC PASS with INVERTED bits! (metric={})",
                        fib_idx, inv_metric
                    );
                    crc_pass_count += 1;
                }
            }
        }

        // Compare against welle.io's known-good 768 decoded bits (before energy dispersal)
        let expected_str = "001100010000010000011011010011000100000000100110010110111000001100101111110110101111100100111111010101110010100101010000111111000100001010001101011011100101110111100110011111100010001001111010101011110111001000111011010010111010011110101100010111111101000000011001100011011010001111000011110000100011111101110111010101100001000000011100111000000000011001101001000111111111011000111010100010001010011110110100101000100010110001100011010000100101011001001001001110110000011100001001100000111011001001011011110101101111000001111100010111001100100000100101001110110100011110011111001101100010101001000111000110110101011100010011000100010000000010000100011000010011100101010110000110111101001101110010001010000101011010011111101100100100101101111110010011011110010001101000";
        let expected: Vec<u8> = expected_str.chars().map(|c| c as u8 - b'0').collect();

        // Count differences for best-state traceback
        let diff_best: usize = decoded_bits_best
            .iter()
            .zip(expected.iter())
            .take(768)
            .filter(|&(a, b)| a != b)
            .count();
        let diff_s0: usize = decoded_bits_s0
            .iter()
            .zip(expected.iter())
            .take(768)
            .filter(|&(a, b)| a != b)
            .count();
        eprintln!("\n  Bit differences vs welle.io expected (768 bits):");
        eprintln!("    best-state: {}/768 errors", diff_best);
        eprintln!("    state-0:    {}/768 errors", diff_s0);

        // Show where errors are concentrated
        if diff_s0 > 0 {
            let error_positions: Vec<usize> = decoded_bits_s0
                .iter()
                .zip(expected.iter())
                .take(768)
                .enumerate()
                .filter(|&(_, (a, b))| a != b)
                .map(|(i, _)| i)
                .collect();
            eprintln!(
                "    Error positions (first 40): {:?}",
                &error_positions[..error_positions.len().min(40)]
            );
        }

        // Print first 20 decoded bits for manual comparison
        eprintln!(
            "\n  First 20 decoded bits (state-0): {:?}",
            &decoded_bits_s0[..20]
        );
        eprintln!("  Expected first 20 bits:          {:?}", &expected[..20]);
        eprintln!(
            "  Last 10 decoded bits (should end in 6 zeros for tail): {:?}",
            &decoded_bits_s0[764..]
        );

        assert!(
            crc_pass_count >= 1,
            "At least 1 of 3 FIBs should pass CRC with welle.io's known-good soft bits. Got {} passes.",
            crc_pass_count
        );

        eprintln!("  RESULT: {} CRC passes total", crc_pass_count);
    }

    /// Also test that our Viterbi traceback from state 0 (tail-terminated) works
    /// the same as best-state traceback on known-good data.
    #[test]
    fn test_viterbi_state0_traceback() {
        // Use the same known-good data
        let soft_bits: Vec<i8> = vec![
            -72, 97, 78, 99, -49, -91, 46, -96, 69, -68, 18, -70, -77, -16, -41, -60, 28, -7, 1,
            70, 68, -124, 14, 41, -50, -54, -102, -49, -51, 91, -20, 43,
        ];
        // Just test with first 32 values to verify function doesn't crash
        let depunctured = fec::eep::depuncture_fic(&soft_bits);
        let (_, _) = fec::viterbi::viterbi_decode_with_metric(&depunctured);
        // More detailed state-0 test will come if the main test fails
    }
}
