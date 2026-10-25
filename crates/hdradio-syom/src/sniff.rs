//! Transport sniffers (ADTS, LOAS/LATM, ISO-BMFF).

use crate::isomp4::sniff_is_isobmff;

/// Sniff ADTS: 12-bit sync, layer 0, plausible frame length.
#[must_use]
pub fn sniff_is_adts(data: &[u8]) -> bool {
    if data.len() < 6 {
        return false;
    }
    let h = [data[0], data[1], data[2], data[3], data[4], data[5]];
    if h[0] != 0xFF || (h[1] & 0xF0) != 0xF0 {
        return false;
    }
    if (h[1] >> 1) & 3 != 0 {
        return false;
    }
    let sf_index = (h[2] >> 2) & 0x0F;
    if sf_index == 0x0F {
        return false;
    }
    let frame_length =
        (usize::from(h[3] & 3) << 11) | (usize::from(h[4]) << 3) | (usize::from(h[5]) >> 5);
    frame_length >= 7
}

/// LOAS syncword `0x2B7` in the first 11 bits.
#[must_use]
pub fn sniff_is_latm(data: &[u8]) -> bool {
    if data.len() < 2 {
        return false;
    }
    let w = (u16::from(data[0]) << 8) | u16::from(data[1]);
    w >> 5 == 0x2B7
}

/// True if the buffer looks like ADTS, LATM/LOAS, or ISO-BMFF M4A.
#[must_use]
pub fn sniff_aac(data: &[u8]) -> bool {
    sniff_is_adts(data) || sniff_is_latm(data) || sniff_is_isobmff(data)
}
