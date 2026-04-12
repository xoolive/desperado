//! MSC (Main Service Channel) decoder.
//!
//! The MSC carries the actual audio/data content. In DAB Mode I:
//! - OFDM symbols 4-76 (indices 3..75 after DQPSK, i.e. soft_bits[3..75]) carry MSC data
//! - 72 MSC symbols are grouped into 4 CIFs of 18 symbols each
//! - Each CIF contains 55,296 soft bits (864 CUs × 64 bits/CU)
//! - Each OFDM symbol carries 3072 soft bits (2 × 1536 carriers)
//!
//! Pipeline per subchannel:
//! 1. CIF assembly: collect 18 symbols → 55,296 soft bits
//! 2. Subchannel extraction: start_addr * 64 .. (start_addr + sub_size) * 64
//! 3. Time de-interleaving: 16-deep buffer with fixed permutation
//! 4. FEC: depuncture → Viterbi → energy dispersal → decoded bytes
//!
//! Reference: ETSI EN 300 401 §11, §12, welle.io src/backend/msc-handler.cpp

use crate::constants;
use crate::fec::{eep, energy_dispersal, viterbi};
use crate::fic::fib::SubchannelInfo;
use tracing::debug;

/// Number of MSC OFDM symbols per CIF.
const MSC_SYMBOLS_PER_CIF: usize = 18;

/// Soft bits per OFDM symbol (2 × K carriers, interleaved I/Q).
const SOFT_BITS_PER_SYMBOL: usize = 2 * constants::K; // 3072

/// Soft bits per CIF (18 × 3072 = 55,296).
const SOFT_BITS_PER_CIF: usize = MSC_SYMBOLS_PER_CIF * SOFT_BITS_PER_SYMBOL;

/// Time de-interleaving depth (16 CIFs).
const DEINTERLEAVE_DEPTH: usize = 16;

/// Time de-interleaving permutation map.
/// For soft bit index i, the delay is DEINTERLEAVE_MAP[i % 16] CIFs.
/// Reference: ETSI EN 300 401 §12, welle.io src/backend/msc-handler.cpp
const DEINTERLEAVE_MAP: [usize; 16] = [0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15];

/// MSC handler for a single subchannel.
///
/// Accumulates OFDM symbols into CIFs, extracts the target subchannel,
/// applies time de-interleaving, and runs the FEC pipeline.
pub struct MscHandler {
    /// Subchannel parameters.
    subch: SubchannelInfo,

    /// EEP depuncturing parameters.
    eep_params: eep::EepParams,

    /// Current CIF accumulation buffer: soft bits for one CIF.
    cif_buf: Vec<i8>,

    /// How many MSC symbols have been fed into the current CIF (0..17).
    symbols_in_cif: usize,

    /// CIF counter within the current frame (0..3).
    cif_in_frame: usize,

    /// Time de-interleaving buffer: circular buffer of subchannel extractions.
    /// Matches welle.io's interleaveData[16][fragmentSize] ring buffer.
    deinterleave_buf: Vec<Vec<i8>>,

    /// Ring buffer write index (0..15), matching welle.io's interleaverIndex.
    interleaver_index: usize,

    /// Total CIFs received (used for warmup counting).
    cif_count: usize,

    /// Number of decoded frames output so far.
    pub frames_decoded: usize,

    /// Debug flag: bypass time de-interleaving.
    bypass_deinterleave: bool,
}

impl MscHandler {
    /// Create a new MSC handler for the given subchannel.
    ///
    /// Returns None if EEP parameters cannot be determined.
    pub fn new(subch: &SubchannelInfo) -> Option<Self> {
        if !subch.is_eep {
            debug!("UEP subchannels not yet supported");
            return None;
        }

        let params = eep::eep_params(subch.bitrate, subch.protection_level, subch.eep_option)?;

        // Verify that the punctured size matches the subchannel size
        let expected_punctured = eep::msc_punctured_size(&params);
        let actual_size = subch.sub_size as usize * constants::CU_SIZE;
        if expected_punctured != actual_size {
            debug!(
                "Punctured size mismatch: expected {} (from EEP params), got {} (from sub_size {}×{})",
                expected_punctured,
                actual_size,
                subch.sub_size,
                constants::CU_SIZE
            );
            return None;
        }

        let subch_size = subch.sub_size as usize * constants::CU_SIZE;

        Some(Self {
            subch: subch.clone(),
            eep_params: params,
            cif_buf: vec![0i8; SOFT_BITS_PER_CIF],
            symbols_in_cif: 0,
            cif_in_frame: 0,
            deinterleave_buf: vec![vec![0i8; subch_size]; DEINTERLEAVE_DEPTH],
            interleaver_index: 0,
            cif_count: 0,
            frames_decoded: 0,
            bypass_deinterleave: false,
        })
    }

    /// Feed one MSC OFDM symbol (soft_bits[3..75] from dqpsk_decode).
    ///
    /// `blkno` is the 0-based index within the MSC symbols (0..71).
    /// The interleaved soft bit layout from DQPSK is: [I_0..I_{K-1}, Q_0..Q_{K-1}].
    /// For CIF assembly, we need them in the sequential order matching welle.io:
    ///   for carrier i: soft_bits[i], soft_bits[K + i] (I then Q).
    ///
    /// Returns decoded bytes whenever a complete CIF is processed and time
    /// de-interleaving produces valid output.
    pub fn feed_symbol(&mut self, soft_bits: &[i8]) -> Option<Vec<u8>> {
        if soft_bits.len() < SOFT_BITS_PER_SYMBOL {
            return None;
        }

        // Place this symbol's soft bits into the CIF buffer.
        // welle.io interleaves I and Q: for each carrier i, bits go at
        // offset = symbols_in_cif * 3072 + i (for I) and + K + i (for Q).
        // But the CIF buffer is filled sequentially per symbol, so we just copy.
        let offset = self.symbols_in_cif * SOFT_BITS_PER_SYMBOL;
        self.cif_buf[offset..offset + SOFT_BITS_PER_SYMBOL]
            .copy_from_slice(&soft_bits[..SOFT_BITS_PER_SYMBOL]);
        self.symbols_in_cif += 1;

        if self.symbols_in_cif < MSC_SYMBOLS_PER_CIF {
            return None; // CIF not complete yet
        }

        // CIF is complete. Extract subchannel and process.
        self.symbols_in_cif = 0;
        self.cif_in_frame += 1;
        if self.cif_in_frame >= constants::CIFS_PER_FRAME {
            self.cif_in_frame = 0;
        }

        self.process_cif()
    }

    /// Process a complete CIF: extract subchannel, time de-interleave, FEC decode.
    fn process_cif(&mut self) -> Option<Vec<u8>> {
        // Extract subchannel soft bits from the CIF.
        let start = self.subch.start_addr as usize * constants::CU_SIZE;
        let size = self.subch.sub_size as usize * constants::CU_SIZE;
        let end = start + size;

        if end > SOFT_BITS_PER_CIF {
            debug!(
                "Subchannel extends beyond CIF: {} > {}",
                end, SOFT_BITS_PER_CIF
            );
            return None;
        }

        let subch_bits = &self.cif_buf[start..end];
        let idx = self.interleaver_index;

        // Time de-interleaving (matching welle.io exactly)
        let deinterleaved;
        if self.bypass_deinterleave {
            // Bypass: just use raw subchannel data directly
            deinterleaved = subch_bits.to_vec();
            self.interleaver_index = (idx + 1) % DEINTERLEAVE_DEPTH;
            self.cif_count += 1;
        } else {
            let mut deint = vec![0i8; size];
            // Each element reads from a different source slot based on i%16,
            // so this cannot be replaced with copy_from_slice.
            #[allow(clippy::manual_memcpy)]
            for i in 0..size {
                let src_slot = (idx + DEINTERLEAVE_MAP[i % 16]) % DEINTERLEAVE_DEPTH;
                deint[i] = self.deinterleave_buf[src_slot][i];
                self.deinterleave_buf[idx][i] = subch_bits[i];
            }
            self.interleaver_index = (idx + 1) % DEINTERLEAVE_DEPTH;
            self.cif_count += 1;

            // Need DEINTERLEAVE_DEPTH (16) CIFs of warmup before producing valid output.
            // welle.io uses `counter <= 15` (skips 16 CIFs), which is equivalent since
            // our cif_count is incremented after processing (count=16 after 16th CIF → skip).
            if self.cif_count <= DEINTERLEAVE_DEPTH {
                return None;
            }
            deinterleaved = deint;
        }

        // FEC pipeline: depuncture → Viterbi → energy dispersal
        let depunctured = eep::depuncture_msc(&deinterleaved, &self.eep_params);

        let (decoded_bits, metric) = viterbi::viterbi_decode_with_metric(&depunctured);

        // Log Viterbi metric for signal quality monitoring
        if self.frames_decoded < 3 {
            let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
            debug!(
                metric = metric,
                achievable = achievable,
                pct = if achievable > 0 {
                    metric as f64 / achievable as f64 * 100.0
                } else {
                    0.0
                },
                "MSC Viterbi metric"
            );
        }

        // Truncate to 24 * bitrate bits, discarding the 6 tail bits.
        // welle.io: outSize = 24 * bitRate, chainback only traces back outSize bits.
        let out_size = 24 * self.subch.bitrate as usize;

        if decoded_bits.len() < out_size {
            debug!(
                "Viterbi output too short: {} < {}",
                decoded_bits.len(),
                out_size
            );
            return None;
        }

        // Energy dispersal (only on the data bits, not tail)
        let mut bits = decoded_bits[..out_size].to_vec();
        energy_dispersal::energy_dispersal(&mut bits);

        // Pack bits into bytes
        let n_bytes = bits.len() / 8;
        let mut bytes = vec![0u8; n_bytes];
        for (byte_idx, byte) in bytes.iter_mut().enumerate() {
            for bit_idx in 0..8 {
                if bits[byte_idx * 8 + bit_idx] != 0 {
                    *byte |= 1 << (7 - bit_idx);
                }
            }
        }

        self.frames_decoded += 1;

        // Debug: log first few bytes of decoded frame to verify data integrity
        if self.frames_decoded <= 3 {
            debug!(
                "MSC decoded frame {}: {} bytes, first 16: {:02X?}",
                self.frames_decoded,
                bytes.len(),
                &bytes[..16.min(bytes.len())]
            );
        }

        Some(bytes)
    }

    /// Reset the handler for a new stream.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.symbols_in_cif = 0;
        self.cif_in_frame = 0;
        self.interleaver_index = 0;
        self.cif_count = 0;
        self.frames_decoded = 0;
        for buf in &mut self.deinterleave_buf {
            buf.fill(0);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::fec::{eep, energy_dispersal, viterbi};

    /// Test: feed welle.io's de-interleaved subchannel dump through our FEC pipeline.
    ///
    /// If the Viterbi metric is high (~80%+), our depuncturing + Viterbi is correct
    /// and the MSC bug must be in our time de-interleaving or data routing.
    /// If the metric is low (~2-3%), the dump might be bad or our FEC is wrong.
    #[test]
    #[ignore = "requires /tmp/welleio_deinterleaved_subch.bin from a welle.io debug build"]
    fn test_welleio_deinterleaved_through_our_fec() {
        let path = "/tmp/welleio_deinterleaved_subch.bin";
        let data = match std::fs::read(path) {
            Ok(d) => d,
            Err(_) => {
                eprintln!("Skipping test: {} not found (run welle.io first)", path);
                return;
            }
        };
        assert_eq!(
            data.len(),
            4224,
            "Expected 4224 bytes for FRANCE CULTURE subchannel"
        );

        // Convert u8 bytes to i8 soft bits (the file was written as int8_t)
        let soft_bits: Vec<i8> = data.iter().map(|&b| b as i8).collect();

        // EEP 3-A, 88 kbps: L1=63, L2=3, PI1=P_CODES[7], PI2=P_CODES[6]
        let params = eep::eep_params(88, 2, 0).expect("EEP 3-A 88kbps should be valid");
        assert_eq!(params.l1, 63);
        assert_eq!(params.l2, 3);
        assert_eq!(params.pi1_index, 7); // PI 8
        assert_eq!(params.pi2_index, 6); // PI 7

        // Depuncture
        let depunctured = eep::depuncture_msc(&soft_bits, &params);
        let expected_depunctured = eep::msc_depunctured_size(&params);
        assert_eq!(depunctured.len(), expected_depunctured);

        eprintln!("Depunctured: {} soft bits", depunctured.len());
        eprintln!("First 20 depunctured: {:?}", &depunctured[..20]);

        // Check depuncturing pattern matches welle.io
        // welle.io shows: -46 -10 0 0 76 51 0 0 66 -93 0 0 68 -67 0 0 15 57 0 0
        // The pattern is: keep 2, zero 2, keep 2, zero 2... (PI 8 = P_CODES[7])
        let nonzero: usize = depunctured.iter().filter(|&&x| x != 0).count();
        let mean_abs: f64 =
            depunctured.iter().map(|&x| (x as f64).abs()).sum::<f64>() / depunctured.len() as f64;
        eprintln!(
            "Nonzero: {}/{}, mean_abs: {:.1}",
            nonzero,
            depunctured.len(),
            mean_abs
        );

        // Viterbi decode
        let (decoded_bits, metric) = viterbi::viterbi_decode_with_metric(&depunctured);
        let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
        let metric_pct = if achievable > 0 {
            metric as f64 / achievable as f64 * 100.0
        } else {
            0.0
        };

        eprintln!(
            "Viterbi metric: {} / {} ({:.1}%)",
            metric, achievable, metric_pct
        );
        eprintln!(
            "First 20 decoded bits: {:?}",
            &decoded_bits[..20.min(decoded_bits.len())]
        );

        // Truncate to output size
        let out_size = 24 * 88; // 2112 bits
        assert!(decoded_bits.len() >= out_size);
        let mut bits = decoded_bits[..out_size].to_vec();

        // Energy dispersal
        energy_dispersal::energy_dispersal(&mut bits);

        // Pack into bytes
        let n_bytes = bits.len() / 8;
        let mut bytes = vec![0u8; n_bytes];
        for (byte_idx, byte) in bytes.iter_mut().enumerate() {
            for bit_idx in 0..8 {
                if bits[byte_idx * 8 + bit_idx] != 0 {
                    *byte |= 1 << (7 - bit_idx);
                }
            }
        }

        eprintln!(
            "Packed bytes (first 16): {:02X?}",
            &bytes[..16.min(bytes.len())]
        );

        // The critical check: the output bytes should match welle.io exactly.
        // welle.io output: 0x6e 0x59 0xab 0xaa 0x6c 0x04 0x63 0x0e ...
        let expected_first_bytes: &[u8] = &[
            0x6e, 0x59, 0xab, 0xaa, 0x6c, 0x04, 0x63, 0x0e, 0xea, 0x99, 0x58, 0xb0, 0xa6, 0x03,
            0x16, 0x69,
        ];
        assert_eq!(
            &bytes[..16],
            expected_first_bytes,
            "Output bytes should match welle.io's decoder output"
        );
        eprintln!("SUCCESS: Our FEC pipeline output matches welle.io byte-for-byte!");
        eprintln!(
            "Viterbi metric: {:.1}% (output is correct despite low metric)",
            metric_pct
        );
    }

    /// Test: feed welle.io's 25 raw subchannel CIF dumps through our complete
    /// MSC pipeline (time de-interleaving + FEC) and check Viterbi metrics.
    ///
    /// This tests the entire MSC pipeline end-to-end with known-good CIF data.
    /// If Viterbi metrics are high (~80%+), the pipeline is correct.
    #[test]
    #[ignore = "requires /tmp/welleio_subch_cif*.bin files from a welle.io debug build"]
    fn test_welleio_cifs_through_our_deinterleave_and_fec() {
        let params = eep::eep_params(88, 2, 0).expect("EEP 3-A 88kbps should be valid");
        let subch_size: usize = 66 * 64; // 4224

        // Load welle.io's raw subchannel CIF dumps
        let mut cif_data: Vec<Vec<i8>> = Vec::new();
        for i in 0..25 {
            let path = format!("/tmp/welleio_subch_cif{}.bin", i);
            let data = match std::fs::read(&path) {
                Ok(d) => d,
                Err(_) => {
                    eprintln!("Skipping test: {} not found (run welle.io first)", path);
                    return;
                }
            };
            assert_eq!(data.len(), subch_size);
            let soft_bits: Vec<i8> = data.iter().map(|&b| b as i8).collect();
            cif_data.push(soft_bits);
        }

        eprintln!(
            "Loaded {} CIF subchannel dumps from welle.io",
            cif_data.len()
        );

        // Run our time de-interleaving algorithm on these CIFs
        let mut deint_buf: Vec<Vec<i8>> = vec![vec![0i8; subch_size]; super::DEINTERLEAVE_DEPTH];
        let mut interleaver_index: usize = 0;
        let mut cif_count: usize = 0;
        let mut viterbi_metrics: Vec<(usize, f64)> = Vec::new();

        for (cif_idx, cif) in cif_data.iter().enumerate() {
            let idx = interleaver_index;
            let mut deint = vec![0i8; subch_size];

            #[allow(clippy::manual_memcpy)]
            for i in 0..subch_size {
                let src_slot = (idx + super::DEINTERLEAVE_MAP[i % 16]) % super::DEINTERLEAVE_DEPTH;
                deint[i] = deint_buf[src_slot][i];
                deint_buf[idx][i] = cif[i];
            }
            interleaver_index = (idx + 1) % super::DEINTERLEAVE_DEPTH;
            cif_count += 1;

            if cif_count <= super::DEINTERLEAVE_DEPTH {
                eprintln!(
                    "CIF {}: warmup ({}/{})",
                    cif_idx,
                    cif_count,
                    super::DEINTERLEAVE_DEPTH
                );
                continue;
            }

            // FEC pipeline
            let depunctured = eep::depuncture_msc(&deint, &params);
            let (decoded_bits, metric) = viterbi::viterbi_decode_with_metric(&depunctured);
            let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
            let metric_pct = if achievable > 0 {
                metric as f64 / achievable as f64 * 100.0
            } else {
                0.0
            };

            let out_size = 24 * 88usize;
            if decoded_bits.len() >= out_size {
                let mut bits = decoded_bits[..out_size].to_vec();
                energy_dispersal::energy_dispersal(&mut bits);
                let n_bytes = bits.len() / 8;
                let mut bytes = vec![0u8; n_bytes];
                for (byte_idx, byte) in bytes.iter_mut().enumerate() {
                    for bit_idx in 0..8 {
                        if bits[byte_idx * 8 + bit_idx] != 0 {
                            *byte |= 1 << (7 - bit_idx);
                        }
                    }
                }
                eprintln!(
                    "CIF {}: Viterbi metric {:.1}%, first 8 bytes: {:02X?}",
                    cif_idx,
                    metric_pct,
                    &bytes[..8.min(bytes.len())]
                );
            } else {
                eprintln!(
                    "CIF {}: Viterbi metric {:.1}%, output too short",
                    cif_idx, metric_pct
                );
            }

            viterbi_metrics.push((cif_idx, metric_pct));
        }

        // Check that at least some CIFs have good Viterbi metrics
        let good_count = viterbi_metrics.iter().filter(|(_, m)| *m > 50.0).count();
        let avg_metric: f64 = if !viterbi_metrics.is_empty() {
            viterbi_metrics.iter().map(|(_, m)| m).sum::<f64>() / viterbi_metrics.len() as f64
        } else {
            0.0
        };
        eprintln!(
            "\nSummary: {}/{} CIFs with >50% Viterbi metric, average: {:.1}%",
            good_count,
            viterbi_metrics.len(),
            avg_metric
        );

        assert!(
            good_count > 0,
            "At least some CIFs should have good Viterbi metrics when using welle.io's data"
        );
    }

    /// Diagnostic: run OUR OWN CIF subchannel dumps through our de-interleave + FEC.
    /// Compare with the same test using welle.io's dumps to find where data diverges.
    #[test]
    #[ignore = "requires /tmp/dabradio_subch_cif*.bin files from a dabradio debug run"]
    fn test_our_own_cifs_through_deinterleave_and_fec() {
        let params = eep::eep_params(88, 2, 0).expect("EEP 3-A 88kbps should be valid");
        let subch_size: usize = 66 * 64; // 4224

        // Load our own raw subchannel CIF dumps
        let mut cif_data: Vec<Vec<i8>> = Vec::new();
        for i in 0..25 {
            let path = format!("/tmp/dabradio_subch_cif{}.bin", i);
            let data = match std::fs::read(&path) {
                Ok(d) => d,
                Err(_) => {
                    eprintln!("Skipping test: {} not found (run dabradio first)", path);
                    return;
                }
            };
            assert_eq!(data.len(), subch_size);
            let soft_bits: Vec<i8> = data.iter().map(|&b| b as i8).collect();
            cif_data.push(soft_bits);
        }

        eprintln!("Loaded {} OWN CIF subchannel dumps", cif_data.len());

        // First: test raw (no de-interleaving) Viterbi on individual CIFs
        eprintln!("\n=== Raw CIFs (no de-interleaving) ===");
        for (cif_idx, cif) in cif_data.iter().enumerate().take(5) {
            let depunctured = eep::depuncture_msc(cif, &params);
            let (_decoded_bits, metric) = viterbi::viterbi_decode_with_metric(&depunctured);
            let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
            let metric_pct = if achievable > 0 {
                metric as f64 / achievable as f64 * 100.0
            } else {
                0.0
            };
            eprintln!("OUR raw CIF {}: Viterbi metric {:.1}%", cif_idx, metric_pct);
        }

        // Same for welle.io CIFs
        eprintln!("\n=== welle.io Raw CIFs (no de-interleaving) ===");
        for i in 0..5 {
            let path = format!("/tmp/welleio_subch_cif{}.bin", i);
            if let Ok(data) = std::fs::read(&path) {
                let soft_bits: Vec<i8> = data.iter().map(|&b| b as i8).collect();
                let depunctured = eep::depuncture_msc(&soft_bits, &params);
                let (_decoded_bits, metric) = viterbi::viterbi_decode_with_metric(&depunctured);
                let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
                let metric_pct = if achievable > 0 {
                    metric as f64 / achievable as f64 * 100.0
                } else {
                    0.0
                };
                eprintln!("welle.io raw CIF {}: Viterbi metric {:.1}%", i, metric_pct);
            }
        }

        // Run our time de-interleaving algorithm on our own CIFs
        eprintln!("\n=== OUR CIFs after de-interleaving ===");
        let mut deint_buf: Vec<Vec<i8>> = vec![vec![0i8; subch_size]; super::DEINTERLEAVE_DEPTH];
        let mut interleaver_index: usize = 0;
        let mut cif_count: usize = 0;

        for (cif_idx, cif) in cif_data.iter().enumerate() {
            let idx = interleaver_index;
            let mut deint = vec![0i8; subch_size];

            #[allow(clippy::manual_memcpy)]
            for i in 0..subch_size {
                let src_slot = (idx + super::DEINTERLEAVE_MAP[i % 16]) % super::DEINTERLEAVE_DEPTH;
                deint[i] = deint_buf[src_slot][i];
                deint_buf[idx][i] = cif[i];
            }
            interleaver_index = (idx + 1) % super::DEINTERLEAVE_DEPTH;
            cif_count += 1;

            if cif_count <= super::DEINTERLEAVE_DEPTH {
                continue;
            }

            // FEC pipeline
            let depunctured = eep::depuncture_msc(&deint, &params);
            let (_decoded_bits, metric) = viterbi::viterbi_decode_with_metric(&depunctured);
            let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
            let metric_pct = if achievable > 0 {
                metric as f64 / achievable as f64 * 100.0
            } else {
                0.0
            };

            eprintln!("OUR CIF {}: Viterbi metric {:.1}%", cif_idx, metric_pct);
        }
    }

    /// Diagnostic: run a single MSC symbol's raw soft bits through Viterbi
    /// to check if they have valid convolutional structure.
    ///
    /// FIC sub-blocks are 2304 soft bits → 3096 depunctured → 774 Viterbi bits.
    /// An MSC symbol is 3072 soft bits. If we treat the first 2304 as if they were
    /// an FIC sub-block and run them through FIC depuncture + Viterbi, a metric of
    /// ~30% means the soft bits have valid convolutional structure (good OFDM/DQPSK).
    /// A metric of ~2-3% means they don't (corrupted bits).
    #[test]
    #[ignore = "requires /tmp/dabradio_dqpsk_sym*.bin files from a dabradio debug run"]
    fn test_msc_symbol_convolutional_structure() {
        use crate::fec::{eep, viterbi};

        // Load MSC symbol 4 (first MSC symbol) from our dumps
        for frame in 0..3 {
            let path = format!("/tmp/dabradio_dqpsk_sym4_frame{}.bin", frame);
            let data = match std::fs::read(&path) {
                Ok(d) => d,
                Err(_) => {
                    eprintln!("Skipping: {} not found (run dabradio first)", path);
                    continue;
                }
            };
            assert_eq!(data.len(), 3072);
            let soft_bits: Vec<i8> = data.iter().map(|&b| b as i8).collect();

            // Take first 2304 soft bits and run through FIC depuncturing
            let subblock = &soft_bits[..2304];
            let depunctured = eep::depuncture_fic(subblock);
            let (_decoded, metric) = viterbi::viterbi_decode_with_metric(&depunctured);
            let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
            let pct = if achievable > 0 {
                metric as f64 / achievable as f64 * 100.0
            } else {
                0.0
            };
            eprintln!(
                "OUR sym4 frame {}: FIC-style Viterbi metric {:.1}% (30% = good, 2% = bad)",
                frame, pct
            );
        }

        // Also test on FIC symbol 1 as a sanity check (should be ~31%)
        for frame in 0..3 {
            let path = format!("/tmp/dabradio_dqpsk_sym1_frame{}.bin", frame);
            let data = match std::fs::read(&path) {
                Ok(d) => d,
                Err(_) => continue,
            };
            let soft_bits: Vec<i8> = data.iter().map(|&b| b as i8).collect();
            let subblock = &soft_bits[..2304];
            let depunctured = eep::depuncture_fic(subblock);
            let (_decoded, metric) = viterbi::viterbi_decode_with_metric(&depunctured);
            let achievable: i32 = depunctured.iter().map(|&x| (x as i32).abs()).sum();
            let pct = if achievable > 0 {
                metric as f64 / achievable as f64 * 100.0
            } else {
                0.0
            };
            eprintln!(
                "OUR sym1 frame {} (FIC, control): FIC-style Viterbi metric {:.1}%",
                frame, pct
            );
        }

        // Also test later MSC symbols (e.g., sym 20, 40, 60) if available
        // We need to add dumps for these - skip for now
    }

    /// Verify our de-interleaving algorithm matches welle.io's by running
    /// both side-by-side on the same input data and comparing outputs.
    #[test]
    #[allow(clippy::needless_range_loop, clippy::manual_memcpy)]
    fn test_deinterleave_matches_welleio() {
        const FRAG_SIZE: usize = 64;
        const DEPTH: usize = 16;
        const INTERLEAVE_MAP: [usize; 16] = [0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15];

        // Generate random-ish input data for 40 CIFs
        let num_cifs = 40;
        let mut input_data: Vec<Vec<i8>> = Vec::new();
        for cif_idx in 0..num_cifs {
            let mut data = vec![0i8; FRAG_SIZE];
            for i in 0..FRAG_SIZE {
                data[i] = ((cif_idx * 13 + i * 7 + 29) % 250) as i8;
                if (cif_idx + i) % 2 == 0 {
                    data[i] = data[i].wrapping_neg();
                }
            }
            input_data.push(data);
        }

        // --- welle.io algorithm (verbatim translation) ---
        let mut welleio_interleave_data: Vec<Vec<i8>> = vec![vec![0i8; FRAG_SIZE]; DEPTH];
        let mut welleio_interleaver_index: i16 = 0;
        let mut welleio_counter: i16 = 0;
        let mut welleio_outputs: Vec<Vec<i8>> = Vec::new();

        for cif_idx in 0..num_cifs {
            let data = &input_data[cif_idx];
            let mut temp_x = vec![0i8; FRAG_SIZE];

            for i in 0..FRAG_SIZE {
                let src = (welleio_interleaver_index as usize + INTERLEAVE_MAP[i & 0xF]) & 0xF;
                temp_x[i] = welleio_interleave_data[src][i];
                welleio_interleave_data[welleio_interleaver_index as usize][i] = data[i];
            }
            welleio_interleaver_index = (welleio_interleaver_index + 1) & 0x0F;

            if welleio_counter <= 15 {
                welleio_counter += 1;
                welleio_outputs.push(vec![]); // placeholder (skipped)
                continue;
            }
            welleio_outputs.push(temp_x);
        }

        // --- Our algorithm ---
        let mut our_deint_buf: Vec<Vec<i8>> = vec![vec![0i8; FRAG_SIZE]; DEPTH];
        let mut our_interleaver_index: usize = 0;
        let mut our_cif_count: usize = 0;
        let mut our_outputs: Vec<Vec<i8>> = Vec::new();

        for cif_idx in 0..num_cifs {
            let data = &input_data[cif_idx];
            let idx = our_interleaver_index;
            let mut deint = vec![0i8; FRAG_SIZE];

            for i in 0..FRAG_SIZE {
                let src_slot = (idx + super::DEINTERLEAVE_MAP[i % 16]) % super::DEINTERLEAVE_DEPTH;
                deint[i] = our_deint_buf[src_slot][i];
                our_deint_buf[idx][i] = data[i];
            }
            our_interleaver_index = (idx + 1) % super::DEINTERLEAVE_DEPTH;
            our_cif_count += 1;

            if our_cif_count <= super::DEINTERLEAVE_DEPTH {
                our_outputs.push(vec![]); // placeholder (skipped)
                continue;
            }
            our_outputs.push(deint);
        }

        // Compare outputs
        let mut matches = 0;
        let mut mismatches = 0;
        for cif_idx in 0..num_cifs {
            let w = &welleio_outputs[cif_idx];
            let o = &our_outputs[cif_idx];
            if w.is_empty() && o.is_empty() {
                continue; // both skipped (warmup)
            }
            if w.is_empty() || o.is_empty() {
                mismatches += 1;
                eprintln!(
                    "CIF {}: one is warmup and other isn't (welle.io empty={}, ours empty={})",
                    cif_idx,
                    w.is_empty(),
                    o.is_empty()
                );
                continue;
            }
            if w == o {
                matches += 1;
            } else {
                mismatches += 1;
                if mismatches <= 3 {
                    eprintln!(
                        "CIF {}: output differs. welle.io[0..8]={:?}, ours[0..8]={:?}",
                        cif_idx,
                        &w[..8],
                        &o[..8]
                    );
                }
            }
        }

        eprintln!(
            "welle.io vs our de-interleaver: {} matches, {} mismatches",
            matches, mismatches
        );
        assert_eq!(mismatches, 0, "Algorithms should produce identical output");
    }
}
