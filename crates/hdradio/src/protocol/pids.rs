//! PIDS deinterleaving, FEC, and CRC validation.

use crate::fec::decode_tail_biting;
use crate::ofdm::carriers::PrimaryMainSoftBits;

const BLOCKS_PER_P1_FRAME: usize = 16;
const PRIMARY_MAIN_BLOCK_BITS: usize = 23_040;
const P1_ENCODED_BITS: usize = 365_440;
const PIDS_ENCODED_BITS: usize = 200;
const PIDS_DECODED_BITS: usize = 80;
const PERMUTATION: [usize; 20] = [
    10, 2, 18, 6, 14, 8, 16, 0, 12, 4, 11, 3, 19, 7, 15, 9, 17, 1, 13, 5,
];

/// A CRC-valid, bit-order-normalized PIDS frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PidsFrame {
    /// PIDS frame type: zero for SIS, one for LLDS.
    pub frame_type: u8,
    /// 79-bit payload after the frame-type bit.
    pub payload: Vec<u8>,
}

/// Collects the 16 primary-main blocks needed for PIDS deinterleaving.
#[derive(Debug, Default)]
pub struct PidsDecoder {
    blocks: [Option<PrimaryMainSoftBits>; BLOCKS_PER_P1_FRAME],
    last_decoded: Option<Vec<u8>>,
}

impl PidsDecoder {
    /// Insert one primary-main block at its recovered NRSC-5 block-counter
    /// position. Returns a CRC-valid PIDS frame, when one is present.
    pub fn push(&mut self, block_counter: u8, bits: PrimaryMainSoftBits) -> Option<PidsFrame> {
        if bits.bits.len() != PRIMARY_MAIN_BLOCK_BITS {
            return None;
        }
        let slot = usize::from(block_counter) % BLOCKS_PER_P1_FRAME;
        self.blocks[slot] = Some(bits);
        let descrambled = self.decode_slot(slot)?;
        self.last_decoded = Some(descrambled.clone());
        let normalized = reverse_bits_in_bytes(&descrambled);
        if !crc12_valid(&normalized) {
            return None;
        }
        Some(PidsFrame {
            frame_type: normalized[0],
            payload: normalized[1..].to_vec(),
        })
    }

    /// Most recent decoded, descrambled, bit-normalized PIDS candidate.
    pub fn last_decoded(&self) -> Option<&[u8]> {
        self.last_decoded.as_deref()
    }

    fn decode_slot(&self, block_counter: usize) -> Option<Vec<u8>> {
        let block = self.blocks[block_counter].as_ref()?;
        let mut depunctured = Vec::with_capacity(PIDS_DECODED_BITS * 3);
        for i in block_counter * PIDS_ENCODED_BITS..(block_counter + 1) * PIDS_ENCODED_BITS {
            let partition = PERMUTATION[i % PERMUTATION.len()];
            let k = ((i / 20) % (PIDS_ENCODED_BITS / 20)) + (P1_ENCODED_BITS / (20 * 16));
            let row = (k * 11) % 32;
            let column = (k * 11 + k / (32 * 9)) % 36;
            let index = (row * (20 * 36)) + partition * 36 + column;
            depunctured.push(*block.bits.get(index)?);
            if depunctured.len() % 6 == 5 {
                depunctured.push(0);
            }
        }
        debug_assert_eq!(depunctured.len(), PIDS_DECODED_BITS * 3);
        let (decoded, _) = decode_tail_biting(&depunctured)?;
        Some(descramble(&decoded))
    }
}

/// Reverse the self-synchronizing 11-bit PIDS scrambler.
fn descramble(bits: &[u8]) -> Vec<u8> {
    let mut state = 0x3ffu16;
    bits.iter()
        .map(|&value| {
            let feedback = ((state >> 9) ^ state) & 1;
            state |= feedback << 11;
            state >>= 1;
            value ^ feedback as u8
        })
        .collect()
}

/// Reverse the transmission bit order within each byte.
fn reverse_bits_in_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .flat_map(|chunk| chunk.iter().rev().copied())
        .collect()
}

/// Validate the NRSC-5 PIDS CRC-12 over one normalized 80-bit frame.
fn crc12_valid(bits: &[u8]) -> bool {
    if bits.len() != PIDS_DECODED_BITS {
        return false;
    }
    let received = bits[68..80]
        .iter()
        .fold(0u16, |value, &bit| (value << 1) | u16::from(bit));
    let mut register = 0u16;
    for &bit in bits[..68].iter().rev() {
        let low = register & 1;
        register >>= 1;
        register ^= u16::from(bit) << 15;
        if low != 0 {
            register ^= 0xd010;
        }
    }
    for _ in 0..16 {
        let low = register & 1;
        register >>= 1;
        if low != 0 {
            register ^= 0xd010;
        }
    }
    received == (register ^ 0x955) & 0x0fff
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_wrong_length_crc_input() {
        assert!(!crc12_valid(&[0; 79]));
    }

    #[test]
    fn descrambler_matches_reference_lfsr_sequence() {
        assert_eq!(
            descramble(&[0; 16]),
            [0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0]
        );
    }
}
