//! Primary logical-channel deinterleaving and frame-lock validation.

use crate::fec::decode_tail_biting;
use crate::ofdm::carriers::PrimaryMainSoftBits;

const BLOCK_COUNT: usize = 16;
const BLOCK_BITS: usize = 23_040;
const ENCODED_BITS: usize = 365_440;
const FRAME_BITS: usize = 146_176;
const PARTITION_COUNT: usize = 20;
const COLUMNS: usize = 36;
const PERMUTATION: [usize; PARTITION_COUNT] = [
    10, 2, 18, 6, 14, 8, 16, 0, 12, 4, 11, 3, 19, 7, 15, 9, 17, 1, 13, 5,
];
const PCI_VALUES: [u32; 8] = [
    0x38d8d3, 0xce3634, 0xe3634c, 0x8d8d33, 0x3634ce, 0x8d338d, 0xd8d338, 0x634ce3,
];

/// Buffers primary-main blocks and validates the P1 program-control indicator.
#[derive(Debug)]
pub struct P1LockMonitor {
    blocks: Vec<Vec<i8>>,
    started: bool,
    last_frame: Option<Vec<u8>>,
}

impl Default for P1LockMonitor {
    fn default() -> Self {
        Self {
            blocks: vec![vec![0; BLOCK_BITS]; BLOCK_COUNT],
            started: false,
            last_frame: None,
        }
    }
}

impl P1LockMonitor {
    /// Insert a primary-main block. Returns `Some(false)` when a completed P1
    /// frame has no recognizable program-control indicator and lock must reset.
    pub fn push(&mut self, block_counter: u8, block: &PrimaryMainSoftBits) -> Option<bool> {
        self.last_frame = None;
        if block.bits.len() != BLOCK_BITS {
            return None;
        }
        let counter = usize::from(block_counter);
        self.blocks[counter].copy_from_slice(&block.bits);
        if counter == 0 {
            self.started = true;
        }
        if !self.started || counter != BLOCK_COUNT - 1 {
            return None;
        }
        self.last_frame = self.decode_frame();
        Some(self.last_frame.is_some())
    }

    /// Most recently decoded PCI-valid P1 frame bytes.
    pub fn last_frame(&self) -> Option<&[u8]> {
        self.last_frame.as_deref()
    }

    fn decode_frame(&self) -> Option<Vec<u8>> {
        let mut depunctured = Vec::with_capacity(FRAME_BITS * 3);
        for i in 0..ENCODED_BITS {
            let partition = PERMUTATION[i % PARTITION_COUNT];
            let block = ((i / PARTITION_COUNT) + partition * 7) % BLOCK_COUNT;
            let k = i / (PARTITION_COUNT * BLOCK_COUNT);
            let row = (k * 11) % 32;
            let column = (k * 11 + k / (32 * 9)) % COLUMNS;
            let index = row * (PARTITION_COUNT * COLUMNS) + partition * COLUMNS + column;
            depunctured.push(self.blocks[block][index]);
            if depunctured.len() % 6 == 5 {
                depunctured.push(0);
            }
        }
        let (decoded, _) = decode_tail_biting(&depunctured)?;
        let descrambled = descramble(&decoded);
        unpack_frame(&descrambled)
    }
}

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

fn unpack_frame(bits: &[u8]) -> Option<Vec<u8>> {
    const START: usize = FRAME_BITS - 30_000;
    const SPACING: usize = 1_248;
    let mut header = 0u32;
    let mut output = Vec::with_capacity((FRAME_BITS - 24) / 8);
    let mut byte = 0u8;
    let mut byte_bits = 0;
    let mut header_bits = 0;
    for index in 0..FRAME_BITS {
        let byte_start = (index >> 3) << 3;
        let byte_len = (FRAME_BITS - byte_start).min(8);
        let bit = bits[byte_start + byte_len - 1 - (index & 7)];
        if index >= START && (index - START).is_multiple_of(SPACING) && header_bits < 24 {
            header |= u32::from(bit) << (23 - header_bits);
            header_bits += 1;
        } else {
            byte |= bit << (7 - byte_bits);
            byte_bits += 1;
            if byte_bits == 8 {
                output.push(byte);
                byte = 0;
                byte_bits = 0;
            }
        }
    }
    PCI_VALUES
        .iter()
        .any(|&candidate| (header ^ candidate).count_ones() <= 4)
        .then_some(output)
}
