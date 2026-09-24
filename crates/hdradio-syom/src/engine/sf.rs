//! `scale_factor_data()` — ISO/IEC 14496-3 Table 4.53 + Table 4.A.1.

use super::bits::BitReader;
use super::error::{Error, Result};
use super::ics::IcsInfo;
use super::section::{ZERO_HCB, is_intensity, is_noise};
use super::sf_tab::{SF_CODE, SF_LEN};
use std::sync::LazyLock;

const NOISE_OFFSET: i32 = 90;
const NOISE_PCM_BITS: u32 = 9;

struct SfTree {
    left: Vec<u16>,
    right: Vec<u16>,
    leaf: Vec<i16>,
    lut_sym: [i16; 4096],
    lut_len: [u8; 4096],
}

impl SfTree {
    fn build() -> Self {
        let mut left = vec![0u16];
        let mut right = vec![0u16];
        let mut leaf = vec![-1i16];
        let mut lut_sym = [-1i16; 4096];
        let mut lut_len = [0u8; 4096];
        for (idx, (&l, &c)) in SF_LEN.iter().zip(SF_CODE.iter()).enumerate() {
            let mut node = 0u16;
            for b in (0..l).rev() {
                let bit = ((c >> b) & 1) as u16;
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

    fn decode(&self, br: &mut BitReader<'_>) -> Result<i8> {
        if let Some(p) = br.try_peek12() {
            let p = p as usize;
            let n = self.lut_len[p];
            if n != 0 {
                br.eat(u32::from(n));
                return Ok(self.lut_sym[p] as i8 - 60);
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
                return Ok(sym as i8 - 60);
            }
        }
    }
}

static SF_TREE: LazyLock<SfTree> = LazyLock::new(SfTree::build);

/// Absolute scalefactors / IS positions / PNS energies per (group, sfb).
#[derive(Debug, Clone, Default)]
pub struct ScaleFactors {
    /// Spectrum scalefactor (valid on Huffman bands).
    pub sf: Vec<Vec<i32>>,
    /// Intensity stereo position (right channel IS bands).
    pub is_pos: Vec<Vec<i32>>,
    /// PNS `noise_nrg` (NOISE_HCB bands).
    pub noise_nrg: Vec<Vec<i32>>,
}

fn fit_i32(rows: &mut Vec<Vec<i32>>, groups: usize, n: usize) {
    if rows.len() < groups {
        rows.resize(groups, Vec::new());
    }
    rows.truncate(groups);
    for row in rows.iter_mut() {
        row.clear();
        row.resize(n, 0);
    }
}

/// Fill `out`, reusing inner row capacity.
pub fn parse_into(
    br: &mut BitReader<'_>,
    ics: &IcsInfo,
    sfb_cb: &[Vec<u8>],
    global_gain: u8,
    out: &mut ScaleFactors,
) -> Result<()> {
    let groups = ics.num_window_groups as usize;
    let max_sfb = ics.max_sfb as usize;
    fit_i32(&mut out.sf, groups, max_sfb);
    fit_i32(&mut out.is_pos, groups, max_sfb);
    fit_i32(&mut out.noise_nrg, groups, max_sfb);
    let mut last_sf = i32::from(global_gain);
    let mut last_is = 0i32;
    let mut last_nrg = i32::from(global_gain) - NOISE_OFFSET - 256;
    let mut noise_pcm_flag = true;
    for g in 0..groups {
        for sfb in 0..max_sfb {
            let cb = *sfb_cb.get(g).and_then(|v| v.get(sfb)).unwrap_or(&ZERO_HCB);
            if cb == ZERO_HCB {
                continue;
            }
            if is_intensity(cb) {
                last_is += i32::from(SF_TREE.decode(br)?);
                out.is_pos[g][sfb] = last_is;
            } else if is_noise(cb) {
                if noise_pcm_flag {
                    noise_pcm_flag = false;
                    last_nrg += br.read(NOISE_PCM_BITS)? as i32;
                } else {
                    last_nrg += i32::from(SF_TREE.decode(br)?);
                }
                out.noise_nrg[g][sfb] = last_nrg;
            } else {
                last_sf += i32::from(SF_TREE.decode(br)?);
                out.sf[g][sfb] = last_sf;
            }
        }
    }
    Ok(())
}

/// Table 4.A.1 decode of one DPCM delta (tests / encoder helpers).
#[cfg(test)]
pub fn decode_dpcm(br: &mut BitReader<'_>) -> Result<i8> {
    SF_TREE.decode(br)
}
