//! `section_data()` — ISO/IEC 14496-3 Table 4.5 / 13818-7 Table 17.

use super::bits::BitReader;
use super::error::{Error, Result};
use super::ics::IcsInfo;

/// Silent band: no scalefactor, no spectrum.
pub const ZERO_HCB: u8 = 0;
/// Escape book (LAV 16 + escape_sequence).
pub const ESC_HCB: u8 = 11;
/// Perceptual noise substitution.
pub const NOISE_HCB: u8 = 13;
/// Out-of-phase intensity stereo.
pub const INTENSITY_HCB2: u8 = 14;
/// In-phase intensity stereo.
pub const INTENSITY_HCB: u8 = 15;

/// Per-group codebook map `sfb_cb[g][sfb]`.
#[derive(Debug, Clone, Default)]
pub struct SectionData {
    /// `sfb_cb[group][sfb]` for `sfb in 0..max_sfb`.
    pub sfb_cb: Vec<Vec<u8>>,
}

fn fit_u8(rows: &mut Vec<Vec<u8>>, groups: usize, n: usize) {
    if rows.len() < groups {
        rows.resize(groups, Vec::new());
    }
    rows.truncate(groups);
    for row in rows.iter_mut() {
        row.clear();
        row.resize(n, 0);
    }
}

impl SectionData {
    /// Parse Table 17 run-length codebook assignment.
    #[cfg(test)]
    pub fn parse(br: &mut BitReader<'_>, ics: &IcsInfo) -> Result<Self> {
        let mut out = Self::default();
        out.parse_into(br, ics)?;
        Ok(out)
    }

    /// Fill `self`, reusing inner row capacity.
    pub fn parse_into(&mut self, br: &mut BitReader<'_>, ics: &IcsInfo) -> Result<()> {
        let esc = if ics.window_sequence.is_eight_short() {
            7u32
        } else {
            31
        };
        let incr_bits = if ics.window_sequence.is_eight_short() {
            3
        } else {
            5
        };
        let groups = ics.num_window_groups as usize;
        let max_sfb = ics.max_sfb as usize;
        fit_u8(&mut self.sfb_cb, groups, max_sfb);
        for g in 0..groups {
            let cb = &mut self.sfb_cb[g];
            let mut k = 0u32;
            while k < u32::from(ics.max_sfb) {
                let sect_cb = br.read(4)? as u8;
                if sect_cb == 12 {
                    return Err(Error::InvalidCodebook(sect_cb));
                }
                let mut sect_len = 0u32;
                loop {
                    let incr = br.read(incr_bits)?;
                    sect_len += incr;
                    if incr != esc {
                        break;
                    }
                }
                if k + sect_len > u32::from(ics.max_sfb) {
                    return Err(Error::SectionDataOverrun);
                }
                for sfb in k..k + sect_len {
                    cb[sfb as usize] = sect_cb;
                }
                k += sect_len;
            }
        }
        Ok(())
    }
}

/// Intensity codebook on the right channel.
#[must_use]
pub fn is_intensity(cb: u8) -> bool {
    cb == INTENSITY_HCB || cb == INTENSITY_HCB2
}

/// PNS codebook.
#[must_use]
pub fn is_noise(cb: u8) -> bool {
    cb == NOISE_HCB
}

/// Huffman spectrum is transmitted for this codebook.
#[must_use]
pub fn has_spectral(cb: u8) -> bool {
    (1..=ESC_HCB).contains(&cb)
}
