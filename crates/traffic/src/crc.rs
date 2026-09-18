//! CRC-16 CCITT (ITU-T X.25), used by DAB packets/FIBs and TPEG frames.
//!
//! Polynomial G(x) = x^16 + x^12 + x^5 + 1 (0x1021). Init 0xFFFF, result
//! complemented before comparison. Matches dab-cmdline `check_CRC_bits` when
//! applied to packed bytes.

/// Compute the CRC-16 of `data` (the CRC field itself is not included).
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc ^ 0xFFFF
}

/// True when the last two bytes of `buf` match the CRC of the preceding bytes.
pub fn crc16_check(buf: &[u8]) -> bool {
    if buf.len() < 2 {
        return false;
    }
    let calc = crc16_ccitt(&buf[..buf.len() - 2]);
    let recv = u16::from_be_bytes([buf[buf.len() - 2], buf[buf.len() - 1]]);
    calc == recv
}

/// Append a CRC-16 to `data` (for tests and synthetic frames).
pub fn crc16_append(data: &mut Vec<u8>) {
    let crc = crc16_ccitt(data);
    data.push((crc >> 8) as u8);
    data.push((crc & 0xFF) as u8);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_appended_crc() {
        let mut buf = b"DAB packet test".to_vec();
        crc16_append(&mut buf);
        assert!(crc16_check(&buf));
        buf[0] ^= 0x01;
        assert!(!crc16_check(&buf));
    }

    #[test]
    fn empty_and_short_buffers_fail() {
        assert!(!crc16_check(&[]));
        assert!(!crc16_check(&[0x00]));
    }
}
