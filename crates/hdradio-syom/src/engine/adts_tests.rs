//! ADTS header write ↔ parse roundtrip.

use super::super::error::Result;
use super::{ADTS_HEADER_BYTES_NO_CRC, AdtsHeader};

fn hdr(frame_len: u16, fs_index: u8, ch: u8) -> AdtsHeader {
    AdtsHeader {
        mpeg_version_mpeg2: false,
        protection_absent: true,
        profile: 1, // LC
        sampling_frequency_index: fs_index,
        channel_configuration: ch,
        aac_frame_length: frame_len,
        adts_buffer_fullness: 0x7FF,
        number_of_raw_data_blocks_in_frame: 1,
    }
}

#[test]
fn write_parse_roundtrip() -> Result<()> {
    for (fs_index, ch) in [(0u8, 1u8), (3, 2), (4, 1), (7, 2), (11, 1)] {
        let want = hdr(7 + 100, fs_index, ch);
        let bytes = want.write();
        assert_eq!(bytes.len(), ADTS_HEADER_BYTES_NO_CRC);
        let (got, off) = AdtsHeader::parse(&bytes)?;
        assert_eq!(got, want, "fs_index {fs_index} ch {ch}");
        assert_eq!(off, ADTS_HEADER_BYTES_NO_CRC);
    }
    Ok(())
}

#[test]
fn write_field_extremes() -> Result<()> {
    let want = hdr(0x1FFF, 12, 7);
    let (got, _) = AdtsHeader::parse(&want.write())?;
    assert_eq!(got, want);
    Ok(())
}
