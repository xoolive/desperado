//! BitReader helpers used only from tests.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::error::Error;
use super::BitReader;

#[test]
fn wide_and_signed_helpers() -> Result<(), Error> {
    let bytes = [0xA5, 0x5A, 0xFF, 0x00, 0x80, 0x11];
    let mut br = BitReader::with_position(&bytes, 0);
    assert_eq!(br.read_u1()?, 1);
    assert!(br.peek_u32(7)? > 0);
    let _ = br.read_u32(7)?;
    let mut br = BitReader::new(&bytes);
    let _ = br.read_i32(16)?;
    let mut br = BitReader::new(&bytes);
    let _ = br.read_u64(40)?;
    let mut br = BitReader::new(&bytes);
    br.consume(3)?;
    br.align_to_byte()?;
    assert!(br.is_byte_aligned());
    assert_eq!(br.byte_position(), 1);
    Ok(())
}
