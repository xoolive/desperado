//! `ics_info()` — ISO/IEC 14496-3 §4.4.6 Table 4.6.

use super::bits::BitReader;
use super::error::{Error, Result};
use super::swb::{long_offsets, short_offsets};

/// `window_sequence` — Table 4.128.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WindowSequence {
    /// One 2048-point transform.
    OnlyLong = 0,
    /// Long transform, start window on the right half.
    LongStart = 1,
    /// Eight 256-point transforms.
    EightShort = 2,
    /// Long transform, stop window on the left half.
    LongStop = 3,
}

impl WindowSequence {
    fn from_bits(bits: u8) -> Self {
        match bits & 3 {
            0 => Self::OnlyLong,
            1 => Self::LongStart,
            2 => Self::EightShort,
            _ => Self::LongStop,
        }
    }

    /// `true` ⇔ eight short windows.
    #[must_use]
    pub fn is_eight_short(self) -> bool {
        matches!(self, Self::EightShort)
    }
}

/// `window_shape` — 0 sine, 1 KBD (§4.5.2.3.1.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WindowShape {
    /// Sine window.
    Sine = 0,
    /// Kaiser-Bessel-derived window.
    Kbd = 1,
}

impl WindowShape {
    fn from_bit(bit: bool) -> Self {
        if bit { Self::Kbd } else { Self::Sine }
    }
}

/// Parsed `ics_info()` plus derived grouping.
#[derive(Debug, Clone, Copy)]
pub struct IcsInfo {
    /// Table 4.128 window sequence.
    pub window_sequence: WindowSequence,
    /// This block's right-half shape (left half comes from the previous block).
    pub window_shape: WindowShape,
    /// Number of coded scalefactor bands.
    pub max_sfb: u8,
    /// 1 for long, 8 for short.
    pub num_windows: u8,
    /// Groups implied by `scale_factor_grouping`.
    pub num_window_groups: u8,
    /// Windows per group (sums to `num_windows`).
    pub window_group_length: [u8; 8],
    /// Number of SWB in the table for this rate / sequence.
    pub num_swb: u8,
}

impl IcsInfo {
    /// Parse Table 4.6. LC rejects predictor / LTP bodies.
    pub fn parse(br: &mut BitReader<'_>, fs_index: u8, common_window: bool) -> Result<Self> {
        let _reserved = br.read_bit()?;
        let window_sequence = WindowSequence::from_bits(br.read(2)? as u8);
        let window_shape = WindowShape::from_bit(br.read_bit()?);
        let (max_sfb, grouping) = if window_sequence.is_eight_short() {
            (br.read(4)? as u8, Some(br.read(7)? as u8))
        } else {
            let max_sfb = br.read(6)? as u8;
            let predictor_data_present = br.read_bit()?;
            if predictor_data_present {
                // AAC-LC shall not carry predictor / LTP (§1.5.1.1).
                return Err(Error::Format("predictor/LTP on LC stream"));
            }
            let _ = common_window;
            (max_sfb, None)
        };

        let (num_windows, num_window_groups, window_group_length) =
            grouping_of(window_sequence, grouping);
        let offsets = if window_sequence.is_eight_short() {
            short_offsets(fs_index)?
        } else {
            long_offsets(fs_index)?
        };
        let num_swb = offsets.len().saturating_sub(1) as u8;
        if max_sfb as usize > num_swb as usize {
            return Err(Error::IcsInfoInvalid);
        }
        Ok(IcsInfo {
            window_sequence,
            window_shape,
            max_sfb,
            num_windows,
            num_window_groups,
            window_group_length,
            num_swb,
        })
    }

    /// Parse the NRSC-5 HDC ICS header. HDC swaps window shape/sequence,
    /// omits the AAC predictor bit, and carries its TNS flags separately.
    pub(crate) fn parse_hdc(br: &mut BitReader<'_>, fs_index: u8) -> Result<Self> {
        if br.read_bit()? {
            return Err(Error::Format("HDC ICS reserved bit"));
        }
        let window_shape = WindowShape::from_bit(br.read_bit()?);
        let window_sequence = WindowSequence::from_bits(br.read(2)? as u8);
        let (max_sfb, grouping) = if window_sequence.is_eight_short() {
            (br.read(4)? as u8, Some(br.read(7)? as u8))
        } else {
            (br.read(6)? as u8, None)
        };
        let (num_windows, num_window_groups, window_group_length) =
            grouping_of(window_sequence, grouping);
        let offsets = if window_sequence.is_eight_short() {
            short_offsets(fs_index)?
        } else {
            long_offsets(fs_index)?
        };
        let num_swb = offsets.len().saturating_sub(1) as u8;
        if max_sfb > num_swb {
            return Err(Error::IcsInfoInvalid);
        }
        Ok(Self {
            window_sequence,
            window_shape,
            max_sfb,
            num_windows,
            num_window_groups,
            window_group_length,
            num_swb,
        })
    }

    /// Window length in coefficients (1024 or 128).
    #[must_use]
    pub fn window_len(&self) -> usize {
        if self.window_sequence.is_eight_short() {
            super::swb::SHORT_WINDOW_LEN
        } else {
            super::swb::LONG_WINDOW_LEN
        }
    }
}

fn grouping_of(seq: WindowSequence, grouping: Option<u8>) -> (u8, u8, [u8; 8]) {
    if !seq.is_eight_short() {
        let mut lens = [0u8; 8];
        lens[0] = 1;
        return (1, 1, lens);
    }
    let gbits = grouping.unwrap_or(0);
    let mut lens = [0u8; 8];
    lens[0] = 1;
    let mut groups = 1u8;
    for i in 1..8 {
        // bit 6 of the 7-bit field groups window 1 with window 0, …
        if (gbits >> (6 - (i - 1))) & 1 == 1 {
            lens[(groups - 1) as usize] += 1;
        } else {
            lens[groups as usize] = 1;
            groups += 1;
        }
    }
    (8, groups, lens)
}
