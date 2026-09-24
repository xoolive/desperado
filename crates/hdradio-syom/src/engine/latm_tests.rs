//! LATM mux config / payload helper branches.
//!
//! LOAS stream-level behavior (resync, CRC propagation over the public
//! streaming path) is covered by `crate::stream_latm_tests`.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{MuxCfg, latm_value, read_frame_len, skip_mux_tail};
use crate::engine::bits::{BitReader, BitWriter};
use crate::engine::crc::stream_mux_config_crc;
use crate::engine::error::Error;

fn push_bits(bits: &mut Vec<bool>, value: u32, n: u32) {
    for i in (0..n).rev() {
        bits.push((value >> i) & 1 != 0);
    }
}

fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let mut w = BitWriter::new();
    for &b in bits {
        w.write_bit(b);
    }
    w.finish()
}

/// A minimal `StreamMuxConfig()` body, up to but excluding
/// `crcCheckPresent`: LC, 44.1 kHz, stereo, `frameLengthType = 1`.
fn mux_config_bits() -> Vec<bool> {
    let mut b = Vec::new();
    push_bits(&mut b, 0, 1); // audioMuxVersion
    push_bits(&mut b, 1, 1); // allStreamsSameTimeFraming
    push_bits(&mut b, 0, 6); // numSubFrames
    push_bits(&mut b, 0, 4); // numProgram
    push_bits(&mut b, 0, 3); // numLayer
    push_bits(&mut b, 2, 5); // audioObjectType = AAC LC
    push_bits(&mut b, 4, 4); // samplingFrequencyIndex = 44.1 kHz
    push_bits(&mut b, 2, 4); // channelConfiguration = stereo
    push_bits(&mut b, 0, 3); // GASpecificConfig flags
    push_bits(&mut b, 1, 3); // frameLengthType = 1
    push_bits(&mut b, 0, 9); // frameLength
    push_bits(&mut b, 0, 1); // otherDataPresent = 0
    b
}

#[test]
fn mux_version_a_reserved_and_program_layer() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write_bit(true);
    w.write_bit(true);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert!(matches!(
        MuxCfg::parse(&mut br),
        Err(Error::LatmAudioMuxVersionAReserved)
    ));

    let mut w = BitWriter::new();
    w.write_bit(false);
    w.write_bit(false);
    w.write(0, 6);
    w.write(1, 4);
    w.write(0, 3);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert!(matches!(
        MuxCfg::parse(&mut br),
        Err(Error::LatmConfigOutOfRange)
    ));
    Ok(())
}

#[test]
fn frame_len_types_and_mux_tail_and_latm_value() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(0, 8);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert_eq!(read_frame_len(&mut br, 0)?, 0);

    let mut w = BitWriter::new();
    w.write(8, 9);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert_eq!(read_frame_len(&mut br, 1)?, 8);
    let mut br = BitReader::new(&[0xFF]);
    assert!(read_frame_len(&mut br, 2).is_err());

    // otherDataPresent (bit-length form) + crcCheckPresent, valid CRC.
    let mut b = Vec::new();
    push_bits(&mut b, 1, 1); // otherDataPresent
    push_bits(&mut b, 0, 1); // otherDataLenBits form (bits, not bytes)
    push_bits(&mut b, 0, 8); // otherDataLenBits = 0 → one bit follows
    push_bits(&mut b, 0, 1); // the otherData bit
    let crc = u32::from(stream_mux_config_crc(&b));
    b.push(true); // crcCheckPresent
    push_bits(&mut b, crc, 8);
    let bytes = bits_to_bytes(&b);
    let mut br = BitReader::new(&bytes);
    skip_mux_tail(&mut br, 0)?;

    // otherDataPresent (byte-escaped form), crcCheckPresent = 0.
    let mut b = Vec::new();
    push_bits(&mut b, 1, 1);
    push_bits(&mut b, 1, 1); // escaped
    b.push(false); // more = 0
    push_bits(&mut b, 1, 8);
    b.push(false); // crcCheckPresent = 0
    let bytes = bits_to_bytes(&b);
    let mut br = BitReader::new(&bytes);
    skip_mux_tail(&mut br, 0)?;

    let mut w = BitWriter::new();
    w.write(1, 8);
    w.write_bit(true);
    w.write(2, 8);
    w.write_bit(false);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert_eq!(latm_value(&mut br)?, (1 << 8) | 2);
    Ok(())
}

#[test]
fn mux_cfg_crc_absent_parses() -> Result<(), Error> {
    let mut b = mux_config_bits();
    b.push(false); // crcCheckPresent = 0
    let bytes = bits_to_bytes(&b);
    let mut br = BitReader::new(&bytes);
    let cfg = MuxCfg::parse(&mut br)?;
    assert_eq!(cfg.frame_length_type, 1);
    Ok(())
}

#[test]
fn mux_cfg_crc_present_valid_parses() -> Result<(), Error> {
    let mut b = mux_config_bits();
    let crc = u32::from(stream_mux_config_crc(&b));
    b.push(true); // crcCheckPresent
    push_bits(&mut b, crc, 8);
    let bytes = bits_to_bytes(&b);
    let mut br = BitReader::new(&bytes);
    let cfg = MuxCfg::parse(&mut br)?;
    assert_eq!(cfg.frame_length_type, 1);
    Ok(())
}

#[test]
fn mux_cfg_crc_present_corrupt_rejected() {
    let mut b = mux_config_bits();
    let crc = u32::from(stream_mux_config_crc(&b)) ^ 0xFF;
    b.push(true); // crcCheckPresent
    push_bits(&mut b, crc, 8);
    let bytes = bits_to_bytes(&b);
    let mut br = BitReader::new(&bytes);
    assert!(matches!(
        MuxCfg::parse(&mut br),
        Err(Error::LatmCrcMismatch)
    ));
}
