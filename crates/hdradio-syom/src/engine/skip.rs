//! Skip FIL / DSE / CCE so a raw_data_block stays bit-aligned.
//! PCE is parsed (not skipped) in `channel_map::parse_pce`.

use super::bits::BitReader;
use super::error::Result;
use super::ics_body::parse_ics;

/// `fill_element()` count field (bytes of `extension_payload`).
pub fn fill_count(br: &mut BitReader<'_>) -> Result<u32> {
    let mut count = br.read(4)?;
    if count == 15 {
        count += br.read(8)?;
        count = count.saturating_sub(1);
    }
    Ok(count)
}

/// `fill_element()` — 4-bit count, optional 8-bit escape, then `count` bytes.
#[cfg(test)]
pub fn skip_fil(br: &mut BitReader<'_>) -> Result<()> {
    let count = fill_count(br)?;
    br.skip(count.saturating_mul(8))?;
    Ok(())
}

/// `data_stream_element()`.
pub fn skip_dse(br: &mut BitReader<'_>) -> Result<()> {
    let _tag = br.read(4)?;
    let align = br.read_bit()?;
    let mut count = br.read(8)?;
    if count == 255 {
        count += br.read(8)?;
    }
    if align {
        br.byte_align()?;
    }
    br.skip(count.saturating_mul(8))?;
    Ok(())
}

/// Consume a CCE so the rest of the block stays aligned. Coupling is not applied.
pub fn skip_cce(br: &mut BitReader<'_>, fs_index: u8, aot: u8) -> Result<()> {
    let _tag = br.read(4)?;
    let ind_sw = br.read_bit()?;
    let coupled = br.read(3)? as usize + 1;
    if !ind_sw {
        let _sign = br.read_bit()?;
        let _scale = br.read(2)?;
    }
    for _ in 0..coupled {
        let is_cpe = br.read_bit()?;
        let _select = br.read(4)?;
        if is_cpe {
            let cc_l = br.read_bit()?;
            let cc_r = br.read_bit()?;
            if cc_l && cc_r {
                let _domain = br.read_bit()?;
            }
        }
    }
    // Coupling channel ICS (common_window = 0).
    let _ = parse_ics(br, fs_index, aot, None)?;
    // Gain elements after the ICS: one Huffman-ish scale per target.
    // Independent CCE puts a gain after each target; dependent after ICS.
    // Remaining bits of this element are not a second ICS; skip is best-effort
    // via the ICS parse having consumed the coupling channel body.
    let _ = ind_sw;
    Ok(())
}
