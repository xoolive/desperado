use std::collections::VecDeque;
use std::fmt;

/// RDS CRC polynomial (g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1)
/// Hex shown in many references: 0x5B9. See pysdr / RDS standard discussion.
const RDS_POLY: u32 = 0x5B9; // 10-bit polynomial (bits 9..0)
const RDS_POLY_DEGREE: usize = 10;

/// Offset words (d9..d0) for A, B, C, C', D (as 10-bit values).
/// Values from the RDS standard (IEC / EN documents).
const OFFSET_A: u16 = 0b0011111100; // A
const OFFSET_B: u16 = 0b0110011000; // B
const OFFSET_C: u16 = 0b0101101000; // C
const OFFSET_C_PRIME: u16 = 0b1101010000; // C'
const OFFSET_D: u16 = 0b0110110100; // D

/// Convert a 26-bit MSB-first word into a 10-bit syndrome (remainder).
/// Input: 26-bit value where bit 25 is MSB (first transmitted bit) and bit 0 is LSB (last).
fn rds_syndrome(word26: u32) -> u16 {
    // Copy into mutable variable for division
    // We'll do polynomial long division: for i from 25 down to 10, if bit i set then xor poly << (i-10)
    let mut v = word26;
    for i in (RDS_POLY_DEGREE..26).rev() {
        if ((v >> i) & 1) != 0 {
            let shift = i - RDS_POLY_DEGREE;
            v ^= RDS_POLY << shift;
        }
    }
    (v & ((1u32 << RDS_POLY_DEGREE) - 1)) as u16
}

/// Try to identify which offset (if any) matches this 26-bit block.
/// Returns Some(offset_id_char) where offset_id_char is 'A', 'B', 'C', 'c' (C'), or 'D'.
fn rds_offset_for_syndrome(s: u16) -> Option<char> {
    if s == OFFSET_A {
        Some('A')
    } else if s == OFFSET_B {
        Some('B')
    } else if s == OFFSET_C {
        Some('C')
    } else if s == OFFSET_C_PRIME {
        Some('c')
    }
    // we use lowercase c for C'
    else if s == OFFSET_D {
        Some('D')
    } else {
        None
    }
}

/// Convert 26 bits (msb-first) into the 16-bit data word (upper 16 bits).
fn rds_data_from_word(word26: u32) -> u16 {
    ((word26 >> 10) & 0xFFFF) as u16
}

/// RDS parser state: assemble validated blocks into groups and parse group types
pub struct RdsParser {
    /// Shift register of bits (most recently received bit is LSB)
    shift: u32,
    /// How many bits currently valid in shift (max 26)
    shift_len: usize,
    /// Buffer for in-progress group blocks (index by offset: A/B/C/C'/D)
    /// We'll push blocks as (offset_char, data_u16)
    pending_blocks: VecDeque<(char, u16)>,
    /// Program Service name buffer (8 chars)
    ps: [u8; 8],
    ps_received_mask: u8, // bit mask of which 2-char segments received (4 segments)
    /// Radiotext buffer (64 chars for 2A), track received segments
    rt: Vec<u8>,
    rt_received_mask: Vec<bool>,
}

impl fmt::Debug for RdsParser {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ps = String::from_utf8_lossy(&self.ps);
        write!(f, "RdsParser {{ PS='{}' }}", ps)
    }
}

impl RdsParser {
    pub fn new() -> Self {
        Self {
            shift: 0,
            shift_len: 0,
            pending_blocks: VecDeque::new(),
            ps: [b' '; 8],
            ps_received_mask: 0,
            rt: vec![b' '; 64],
            rt_received_mask: vec![false; 16], // 16 segments of 4 chars for RT 2A
        }
    }

    /// Get the current station name (PS) if at least one segment has been received
    pub fn station_name(&self) -> Option<String> {
        if self.ps_received_mask != 0 {
            let ps_str = String::from_utf8_lossy(&self.ps).trim_end().to_string();
            if !ps_str.is_empty() && ps_str != "        " {
                Some(ps_str)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Get the current radio text (RT) if at least some segments have been received
    pub fn radio_text(&self) -> Option<String> {
        let have = self.rt_received_mask.iter().filter(|&&x| x).count();
        if have >= 2 {
            let rt_text = String::from_utf8_lossy(&self.rt).trim_end().to_string();
            if !rt_text.is_empty() {
                Some(rt_text)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Feed raw RDS bits (0/1) into the parser. Bits must be in the transmitted order (MSB-first).
    /// Typically you call this with the bit sequence produced by your BPSK symbol slicer.
    pub fn push_bits(&mut self, bits: &[u8]) {
        for &b in bits {
            self.shift = ((self.shift << 1) & ((1u32 << 26) - 1)) | (b as u32 & 1);
            if self.shift_len < 26 {
                self.shift_len += 1;
            }
            if self.shift_len == 26 {
                // check syndrome
                let synd = rds_syndrome(self.shift);
                if let Some(off) = rds_offset_for_syndrome(synd) {
                    // We have a valid block with offset 'off'
                    let data = rds_data_from_word(self.shift);
                    self.push_block(off, data);
                } else {
                    // Not recognized as a valid block -> keep sliding (no sync)
                }
                // Note: we don't clear shift; next incoming bit will slide it
            }
        }
    }

    /// Called when a validated 26-bit block is found: offset char (A,B,C,c,D) and its 16-bit data.
    fn push_block(&mut self, offset: char, data: u16) {
        // Append to pending queue; store only small recent history to prevent runaway memory
        self.pending_blocks.push_back((offset, data));
        if self.pending_blocks.len() > 8 {
            self.pending_blocks.pop_front();
        }

        // Try to detect a group: we need a sequence A,B,C(or c),D in order
        if self.pending_blocks.len() >= 4 {
            // try from back
            for start in (0..=self.pending_blocks.len() - 4).rev() {
                let mut ok = true;
                let mut offsets = ['\0'; 4];
                let mut datas = [0u16; 4];

                // Safely extract the 4 blocks
                for i in 0..4 {
                    if start + i < self.pending_blocks.len() {
                        let (o, d) = self.pending_blocks[start + i];
                        offsets[i] = o;
                        datas[i] = d;
                    } else {
                        ok = false;
                        break;
                    }
                }

                if !ok {
                    continue;
                }

                ok &= offsets[0] == 'A';
                ok &= offsets[1] == 'B';
                ok &= (offsets[2] == 'C' || offsets[2] == 'c');
                ok &= offsets[3] == 'D';

                if ok {
                    // We extracted a candidate group
                    self.handle_group(datas);
                    // Remove the blocks up through start+3 (we consumed them)
                    for _ in 0..=start + 3 {
                        if !self.pending_blocks.is_empty() {
                            self.pending_blocks.pop_front();
                        }
                    }
                    break;
                }
            }
        }
    }

    /// Process a full 4-block group (datas array: block1..block4 data words).
    fn handle_group(&mut self, datas: [u16; 4]) {
        // block 1 is PI (program identification)
        let pi = datas[0];
        let block2 = datas[1];
        let block3 = datas[2];
        let block4 = datas[3];

        // group type: bits 15..12 of block2 (MSB-first)
        let group_type = ((block2 >> 12) & 0x0F) as u8;
        // version (A=0, B=1) is bit 11
        let version = ((block2 >> 11) & 0x01) as u8;
        // for PS and RT addressing we need the low bits:
        // for PS: C1 C0 usually located in block2 bits b1..b0 (we use block2 & 0x03)
        let c_bits = (block2 & 0x03) as u8;
        // for RT version A segment address is low 4 bits
        let rt_seg_addr = (block2 & 0x0F) as usize;

        // Debug print minimal info
        // e.g. Group 0A, PI:
        // println!("RDS group {}{} PI=0x{:04X}", group_type, if version==0 {'A'} else {'B'}, pi);

        match (group_type, version) {
            (0, 0) => {
                // 0A - PS name (2 ASCII chars in block3)
                let seg = c_bits as usize; // 0..3
                let ch1 = ((block3 >> 8) & 0xFF) as u8;
                let ch2 = (block3 & 0xFF) as u8;
                let pos = seg * 2;
                if pos + 1 < 8 {
                    self.ps[pos] = ch1;
                    self.ps[pos + 1] = ch2;
                    self.ps_received_mask |= 1 << seg;
                }
                // Remove direct printing here, let caller handle it
                // if self.ps_received_mask == 0x0F {
                //     let ps_str = String::from_utf8_lossy(&self.ps);
                //     println!("[RDS] PS = {}", ps_str);
                // }
            }
            (0, 1) => {
                // 0B - PS variant (block3 contains PI and block4 has chars)
                let ch1 = ((block4 >> 8) & 0xFF) as u8;
                let ch2 = (block4 & 0xFF) as u8;
                let seg = c_bits as usize;
                let pos = seg * 2;
                if pos + 1 < 8 {
                    self.ps[pos] = ch1;
                    self.ps[pos + 1] = ch2;
                    self.ps_received_mask |= 1 << seg;
                }
                // Remove direct printing
            }
            (2, 0) => {
                // 2A - Radiotext
                let seg = rt_seg_addr & 0x0F;
                let a = ((block3 >> 8) & 0xFF) as u8;
                let b = (block3 & 0xFF) as u8;
                let c = ((block4 >> 8) & 0xFF) as u8;
                let d = (block4 & 0xFF) as u8;
                let pos = (seg as usize) * 4;
                if pos + 3 < self.rt.len() {
                    self.rt[pos] = a;
                    self.rt[pos + 1] = b;
                    self.rt[pos + 2] = c;
                    self.rt[pos + 3] = d;
                    self.rt_received_mask[seg as usize] = true;
                }
                // Remove direct printing
            }
            (2, 1) => {
                // 2B - Radiotext version B
                let seg = rt_seg_addr & 0x0F;
                let c = ((block4 >> 8) & 0xFF) as u8;
                let d = (block4 & 0xFF) as u8;
                let pos = (seg as usize) * 4;
                if pos + 1 < self.rt.len() {
                    self.rt[pos] = c;
                    self.rt[pos + 1] = d;
                    self.rt_received_mask[seg as usize] = true;
                }
                // Remove direct printing
            }
            _ => {
                // For other group types you may add parsing here
            }
        }
    }
}

impl Default for RdsParser {
    fn default() -> Self {
        Self::new()
    }
}
