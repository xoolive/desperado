//! Spectral Huffman decode — ISO/IEC 14496-3 §4.6.3 + Tables 4.A.2–4.A.12.

use super::bits::BitReader;
use super::error::{Error, Result};
use super::{huff_esc, huff_pair, huff_quad};
use std::sync::LazyLock;

/// Binary prefix tree. `0` in `left`/`right` means no child (root is node 0).
struct HuffTree {
    left: Vec<u16>,
    right: Vec<u16>,
    leaf: Vec<i16>,
    /// 12-bit prefix → (symbol, nbits). `nbits == 0` ⇒ code longer than 12.
    lut_sym: [i16; 4096],
    lut_len: [u8; 4096],
}

impl HuffTree {
    fn from_tables(len: &[u8], code: &[u16]) -> Self {
        let mut left = vec![0u16];
        let mut right = vec![0u16];
        let mut leaf = vec![-1i16];
        let mut lut_sym = [-1i16; 4096];
        let mut lut_len = [0u8; 4096];
        for (idx, (&l, &c)) in len.iter().zip(code.iter()).enumerate() {
            let mut node = 0u16;
            for b in (0..l).rev() {
                let bit = (c >> b) & 1;
                let existing = if bit == 0 {
                    left[node as usize]
                } else {
                    right[node as usize]
                };
                if existing == 0 {
                    let id = left.len() as u16;
                    left.push(0);
                    right.push(0);
                    leaf.push(-1);
                    if bit == 0 {
                        left[node as usize] = id;
                    } else {
                        right[node as usize] = id;
                    }
                    node = id;
                } else {
                    node = existing;
                }
            }
            leaf[node as usize] = idx as i16;
            if l > 0 && l <= 12 {
                let shift = 12 - l;
                let base = (c as usize) << shift;
                for extra in 0..(1usize << shift) {
                    lut_sym[base + extra] = idx as i16;
                    lut_len[base + extra] = l;
                }
            }
        }
        Self {
            left,
            right,
            leaf,
            lut_sym,
            lut_len,
        }
    }

    fn decode(&self, br: &mut BitReader<'_>) -> Result<usize> {
        if let Some(p) = br.try_peek12() {
            let p = p as usize;
            let n = self.lut_len[p];
            if n != 0 {
                br.eat(u32::from(n));
                return Ok(self.lut_sym[p] as usize);
            }
        }
        let mut node = 0u16;
        loop {
            let bit = br.read_bit()?;
            let next = if bit {
                self.right[node as usize]
            } else {
                self.left[node as usize]
            };
            if next == 0 {
                return Err(Error::HuffmanInvalid);
            }
            node = next;
            let sym = self.leaf[node as usize];
            if sym >= 0 {
                return Ok(sym as usize);
            }
        }
    }
}

fn book(n: u8) -> Result<&'static HuffTree> {
    match n {
        1 => Ok(&TREE1),
        2 => Ok(&TREE2),
        3 => Ok(&TREE3),
        4 => Ok(&TREE4),
        5 => Ok(&TREE5),
        6 => Ok(&TREE6),
        7 => Ok(&TREE7),
        8 => Ok(&TREE8),
        9 => Ok(&TREE9),
        10 => Ok(&TREE10),
        11 => Ok(&TREE11),
        _ => Err(Error::InvalidCodebook(n)),
    }
}

macro_rules! tree {
    ($name:ident, $len:expr, $code:expr) => {
        static $name: LazyLock<HuffTree> = LazyLock::new(|| HuffTree::from_tables($len, $code));
    };
}

tree!(TREE1, &huff_quad::H1_LEN, &huff_quad::H1_CODE);
tree!(TREE2, &huff_quad::H2_LEN, &huff_quad::H2_CODE);
tree!(TREE3, &huff_quad::H3_LEN, &huff_quad::H3_CODE);
tree!(TREE4, &huff_quad::H4_LEN, &huff_quad::H4_CODE);
tree!(TREE5, &huff_pair::H5_LEN, &huff_pair::H5_CODE);
tree!(TREE6, &huff_pair::H6_LEN, &huff_pair::H6_CODE);
tree!(TREE7, &huff_pair::H7_LEN, &huff_pair::H7_CODE);
tree!(TREE8, &huff_pair::H8_LEN, &huff_pair::H8_CODE);
tree!(TREE9, &huff_pair::H9_LEN, &huff_pair::H9_CODE);
tree!(TREE10, &huff_pair::H10_LEN, &huff_pair::H10_CODE);
tree!(TREE11, &huff_esc::H11_LEN, &huff_esc::H11_CODE);

/// Decode one n-tuple from codebook `cb` (1..=11) into `out` (length 2 or 4).
/// Returns the number of coefficients written.
pub fn decode_tuple(br: &mut BitReader<'_>, cb: u8, out: &mut [i32]) -> Result<usize> {
    let idx = book(cb)?.decode(br)?;
    match cb {
        1 | 2 => {
            signed_quad(idx, 1, out);
            Ok(4)
        }
        3 | 4 => {
            unsigned_quad(idx, 2, out);
            apply_signs(br, out, 4)?;
            Ok(4)
        }
        5 | 6 => {
            signed_pair(idx, 4, out);
            Ok(2)
        }
        7 | 8 => {
            unsigned_pair(idx, 8, out);
            apply_signs(br, out, 2)?;
            Ok(2)
        }
        9 | 10 => {
            unsigned_pair(idx, 13, out);
            apply_signs(br, out, 2)?;
            Ok(2)
        }
        11 => {
            unsigned_pair(idx, 17, out);
            // §4.6.3.3: signs for each non-zero, then escape_sequence
            // for each magnitude-16. lavc writes the same order.
            apply_signs(br, out, 2)?;
            decode_esc(br, &mut out[0])?;
            decode_esc(br, &mut out[1])?;
            Ok(2)
        }
        _ => Err(Error::InvalidCodebook(cb)),
    }
}

fn signed_quad(idx: usize, _lav: i32, out: &mut [i32]) {
    // (2*lav+1)^4 = 81; offset lav=1 → values −1..=1
    out[0] = (idx / 27) as i32 - 1;
    out[1] = ((idx / 9) % 3) as i32 - 1;
    out[2] = ((idx / 3) % 3) as i32 - 1;
    out[3] = (idx % 3) as i32 - 1;
}

fn unsigned_quad(idx: usize, _lav: i32, out: &mut [i32]) {
    out[0] = (idx / 27) as i32;
    out[1] = ((idx / 9) % 3) as i32;
    out[2] = ((idx / 3) % 3) as i32;
    out[3] = (idx % 3) as i32;
}

fn signed_pair(idx: usize, lav: i32, out: &mut [i32]) {
    let dim = (2 * lav + 1) as usize;
    out[0] = (idx / dim) as i32 - lav;
    out[1] = (idx % dim) as i32 - lav;
}

fn unsigned_pair(idx: usize, dim: usize, out: &mut [i32]) {
    out[0] = (idx / dim) as i32;
    out[1] = (idx % dim) as i32;
}

fn apply_signs(br: &mut BitReader<'_>, out: &mut [i32], n: usize) -> Result<()> {
    for v in out.iter_mut().take(n) {
        if *v != 0 && br.read_bit()? {
            *v = -*v;
        }
    }
    Ok(())
}

/// §4.6.3.3 escape_sequence: book 11 magnitude 16 is a flag.
fn decode_esc(br: &mut BitReader<'_>, v: &mut i32) -> Result<()> {
    if v.abs() != 16 {
        return Ok(());
    }
    let neg = *v < 0;
    let mut n = 4u32;
    while br.read_bit()? {
        n += 1;
        if n > 21 {
            return Err(Error::HuffmanInvalid);
        }
    }
    let off = br.read(n)?;
    let mag = (1i32 << n) + off as i32;
    *v = if neg { -mag } else { mag };
    Ok(())
}
