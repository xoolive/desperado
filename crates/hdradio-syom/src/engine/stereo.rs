//! M/S and intensity stereo — ISO/IEC 14496-3 §4.6.8.

use super::bits::BitReader;
use super::error::{Error, Result};
use super::ics::IcsInfo;
use super::section::{INTENSITY_HCB, INTENSITY_HCB2, SectionData, is_intensity, is_noise};
use super::sf::ScaleFactors;
use super::swb::{long_offsets, short_offsets};

/// `ms_mask_present` (2 bits): 0 off, 1 per-band, 2 all-1, 3 reserved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MsMask {
    /// No M/S.
    Off,
    /// Per-band `ms_used` bits.
    PerBand,
    /// All bands used.
    All,
}

/// CPE joint-stereo side info.
#[derive(Debug, Clone)]
pub struct MsInfo {
    /// Mask mode.
    pub mask: MsMask,
    /// `ms_used[g][sfb]` when `mask == PerBand`; empty otherwise.
    pub used: Vec<Vec<bool>>,
}

impl MsInfo {
    /// Parse `ms_mask_present` and optional `ms_used` bits (Table 4.4 CPE).
    pub fn parse(br: &mut BitReader<'_>, ics: &IcsInfo) -> Result<Self> {
        let present = br.read(2)?;
        let mask = match present {
            0 => MsMask::Off,
            1 => MsMask::PerBand,
            2 => MsMask::All,
            _ => return Err(Error::Format("reserved ms_mask_present")),
        };
        let mut used = Vec::new();
        if mask == MsMask::PerBand {
            for _ in 0..ics.num_window_groups {
                let mut row = Vec::with_capacity(ics.max_sfb as usize);
                for _ in 0..ics.max_sfb {
                    row.push(br.read_bit()?);
                }
                used.push(row);
            }
        }
        Ok(MsInfo { mask, used })
    }

    /// Whether this (group, sfb) is M/S coded.
    #[must_use]
    pub fn used(&self, g: usize, sfb: usize) -> bool {
        match self.mask {
            MsMask::Off => false,
            MsMask::All => true,
            MsMask::PerBand => self
                .used
                .get(g)
                .and_then(|v| v.get(sfb))
                .copied()
                .unwrap_or(false),
        }
    }
}

/// Inverse M/S: `l' = m+s`, `r' = m-s`, skipping IS and PNS bands.
pub fn apply_ms(
    left: &mut [f32],
    right: &mut [f32],
    ics: &IcsInfo,
    left_sec: &SectionData,
    right_sec: &SectionData,
    ms: &MsInfo,
    fs_index: u8,
) -> Result<()> {
    if ms.mask == MsMask::Off {
        return Ok(());
    }
    let win_len = ics.window_len();
    let offsets = if ics.window_sequence.is_eight_short() {
        short_offsets(fs_index)?
    } else {
        long_offsets(fs_index)?
    };
    let mut wbase = 0usize;
    for g in 0..ics.num_window_groups as usize {
        let glen = ics.window_group_length[g] as usize;
        for sfb in 0..ics.max_sfb as usize {
            let rcb = *right_sec
                .sfb_cb
                .get(g)
                .and_then(|v| v.get(sfb))
                .unwrap_or(&0);
            let lcb = *left_sec
                .sfb_cb
                .get(g)
                .and_then(|v| v.get(sfb))
                .unwrap_or(&0);
            if is_intensity(rcb) || is_noise(rcb) || is_noise(lcb) {
                continue;
            }
            if !ms.used(g, sfb) {
                continue;
            }
            let start = *offsets.get(sfb).ok_or(Error::SpectrumInvalid)? as usize;
            let end = *offsets.get(sfb + 1).ok_or(Error::SpectrumInvalid)? as usize;
            for b in 0..glen {
                let w = wbase + b;
                for i in start..end {
                    let idx = w * win_len + i;
                    let m = *left.get(idx).ok_or(Error::SpectrumInvalid)?;
                    let s = *right.get(idx).ok_or(Error::SpectrumInvalid)?;
                    left[idx] = m + s;
                    right[idx] = m - s;
                }
            }
        }
        wbase += glen;
    }
    Ok(())
}

/// Derive the right channel from the left on intensity-coded bands.
pub fn apply_intensity(
    left: &[f32],
    right: &mut [f32],
    ics: &IcsInfo,
    right_sec: &SectionData,
    sf: &ScaleFactors,
    ms: &MsInfo,
    fs_index: u8,
) -> Result<()> {
    let win_len = ics.window_len();
    let offsets = if ics.window_sequence.is_eight_short() {
        short_offsets(fs_index)?
    } else {
        long_offsets(fs_index)?
    };
    let mut wbase = 0usize;
    for g in 0..ics.num_window_groups as usize {
        let glen = ics.window_group_length[g] as usize;
        for sfb in 0..ics.max_sfb as usize {
            let cb = *right_sec
                .sfb_cb
                .get(g)
                .and_then(|v| v.get(sfb))
                .unwrap_or(&0);
            let sign = match cb {
                INTENSITY_HCB => 1.0,
                INTENSITY_HCB2 => -1.0,
                _ => continue,
            };
            let invert = if ms.mask == MsMask::PerBand && ms.used(g, sfb) {
                -1.0
            } else {
                1.0
            };
            let pos = *sf.is_pos.get(g).and_then(|v| v.get(sfb)).unwrap_or(&0);
            let scale = (sign * invert * (0.5f64).powf(0.25 * f64::from(pos))) as f32;
            let start = *offsets.get(sfb).ok_or(Error::SpectrumInvalid)? as usize;
            let end = *offsets.get(sfb + 1).ok_or(Error::SpectrumInvalid)? as usize;
            for b in 0..glen {
                let w = wbase + b;
                for i in start..end {
                    let idx = w * win_len + i;
                    right[idx] = scale * left.get(idx).copied().unwrap_or(0.0);
                }
            }
        }
        wbase += glen;
    }
    Ok(())
}
