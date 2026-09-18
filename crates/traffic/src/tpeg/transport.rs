//! TPEG transport-frame parsing (ISO/TS 21219-5 / ISO 18234-2 SSF).
//!
//! DAB carries a concatenated byte stream of TPEG transport frames inside
//! reassembled MSC data groups. Frames are delimited by sync word 0xFF0F,
//! protected by a header CRC over the 4-byte preamble and a frame CRC over
//! the remainder of the frame excluding the CRC itself.

use crate::crc::{crc16_ccitt, crc16_check};

/// TPEG sync word (ISO 18234-2 / ISO 21219-5).
pub const SYNC_WORD: [u8; 2] = [0xFF, 0x0F];

/// A validated TPEG transport frame.
#[derive(Debug, Clone)]
pub struct TransportFrame {
    /// Bytes after the 6-byte preamble (sync + length + header CRC) and
    /// before the 2-byte frame CRC.
    pub payload: Vec<u8>,
    /// Byte offset of the sync word in the input buffer.
    pub offset: usize,
}

/// Scan `data` for TPEG frames whose CRCs both pass.
pub fn find_frames(data: &[u8]) -> Vec<TransportFrame> {
    let mut frames = Vec::new();
    let mut i = 0;
    while i + 8 <= data.len() {
        if data[i] == SYNC_WORD[0] && data[i + 1] == SYNC_WORD[1] {
            let field_length = u16::from_be_bytes([data[i + 2], data[i + 3]]) as usize;
            // fieldLength counts bytes after that field, including header CRC (2)
            // and frame CRC (2), so the full frame is 4 + field_length.
            if field_length < 4 || i + 4 + field_length > data.len() {
                i += 1;
                continue;
            }
            let header = &data[i..i + 4];
            let header_crc = u16::from_be_bytes([data[i + 4], data[i + 5]]);
            if crc16_ccitt(header) != header_crc {
                i += 1;
                continue;
            }
            let frame_end = i + 4 + field_length;
            let frame_bytes = &data[i..frame_end];
            if !crc16_check(frame_bytes) {
                i += 1;
                continue;
            }
            let payload = data[i + 6..frame_end - 2].to_vec();
            frames.push(TransportFrame { payload, offset: i });
            i = frame_end;
            continue;
        }
        i += 1;
    }
    frames
}

/// Build a synthetic transport frame (tests / reference vectors).
pub fn encode_frame(payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(payload.len() + 8);
    frame.extend_from_slice(&SYNC_WORD);
    // fieldLength = headerCRC (2) + payload + frameCRC (2)
    let field_length = (2 + payload.len() + 2) as u16;
    frame.extend_from_slice(&field_length.to_be_bytes());
    let header_crc = crc16_ccitt(&frame);
    frame.extend_from_slice(&header_crc.to_be_bytes());
    frame.extend_from_slice(payload);
    let frame_crc = crc16_ccitt(&frame);
    frame.extend_from_slice(&frame_crc.to_be_bytes());
    frame
}

/// TPEG1-style service-frame header extracted from a transport payload.
#[derive(Debug, Clone)]
pub struct ServiceFrame {
    /// Encryption / CA indicator. Zero means clear (unencrypted).
    pub enc_id: u8,
    /// 24-bit TPEG service identifier.
    pub sid: u32,
    /// Service-component payloads keyed by SCID.
    pub components: Vec<ServiceComponent>,
}

#[derive(Debug, Clone)]
pub struct ServiceComponent {
    pub scid: u8,
    pub data: Vec<u8>,
}

/// Parse a TPEG1 SSF service frame (ISO 18234-2).
///
/// Layout after the transport header CRC:
/// `encId (8) | SID (24) | { SCID (8) | length (16) | data }*`
pub fn parse_tpeg1_service(payload: &[u8]) -> Option<ServiceFrame> {
    if payload.len() < 4 {
        return None;
    }
    let enc_id = payload[0];
    let sid = ((payload[1] as u32) << 16) | ((payload[2] as u32) << 8) | payload[3] as u32;
    let mut pos = 4;
    let mut components = Vec::new();
    while pos + 3 <= payload.len() {
        let scid = payload[pos];
        let len = u16::from_be_bytes([payload[pos + 1], payload[pos + 2]]) as usize;
        pos += 3;
        if pos + len > payload.len() {
            break;
        }
        components.push(ServiceComponent {
            scid,
            data: payload[pos..pos + len].to_vec(),
        });
        pos += len;
    }
    Some(ServiceFrame {
        enc_id,
        sid,
        components,
    })
}

/// A TPEG2 UBCR component: `id (IntUnLoMB) | length (IntUnLoMB) | data`.
#[derive(Debug, Clone)]
pub struct Tpeg2Component {
    pub id: u64,
    pub data: Vec<u8>,
    pub children: Vec<Tpeg2Component>,
}

/// Read an unsigned integer with MSB continuation (ISO 21219-3 IntUnLoMB).
///
/// Rejects values that cannot be represented in `u64` *before* shifting, so a
/// corrupted broadcast stream cannot trip release-profile overflow checks.
pub fn read_int_unlomb(data: &[u8], pos: &mut usize) -> Option<u64> {
    if *pos >= data.len() {
        return None;
    }
    let mut value: u64 = 0;
    loop {
        if *pos >= data.len() {
            return None;
        }
        let b = data[*pos];
        *pos += 1;
        // Guard before the shift: `(value << 7)` must not overflow.
        if value > (u64::MAX >> 7) {
            return None;
        }
        value = (value << 7) | u64::from(b & 0x7F);
        if b & 0x80 == 0 {
            return Some(value);
        }
    }
}

/// Parse a sequence of TPEG2 components occupying `data`.
pub fn parse_tpeg2_components(data: &[u8]) -> Vec<Tpeg2Component> {
    let mut pos = 0;
    let mut out = Vec::new();
    while pos < data.len() {
        let start = pos;
        let Some(id) = read_int_unlomb(data, &mut pos) else {
            break;
        };
        let Some(len) = read_int_unlomb(data, &mut pos) else {
            break;
        };
        let Ok(len) = usize::try_from(len) else {
            if start == 0 {
                return Vec::new();
            }
            break;
        };
        let Some(end) = pos.checked_add(len) else {
            if start == 0 {
                return Vec::new();
            }
            break;
        };
        if end > data.len() {
            // Not a valid component stream; abort rather than mis-parse.
            if start == 0 {
                return Vec::new();
            }
            break;
        }
        let inner = data[pos..end].to_vec();
        pos = end;
        let children = parse_tpeg2_components(&inner);
        // Nested parse is accepted only when it consumed the inner buffer
        // as a well-formed component list; otherwise treat as a leaf.
        let children = if component_stream_fits(&inner, &children) {
            children
        } else {
            Vec::new()
        };
        out.push(Tpeg2Component {
            id,
            data: inner,
            children,
        });
    }
    out
}

fn component_stream_fits(data: &[u8], children: &[Tpeg2Component]) -> bool {
    if children.is_empty() {
        return false;
    }
    let mut pos = 0;
    for _child in children {
        if read_int_unlomb(data, &mut pos).is_none() {
            return false;
        }
        let Some(len) = read_int_unlomb(data, &mut pos) else {
            return false;
        };
        let Ok(len) = usize::try_from(len) else {
            return false;
        };
        let Some(next) = pos.checked_add(len) else {
            return false;
        };
        pos = next;
    }
    pos == data.len()
}

/// Write IntUnLoMB (tests).
pub fn write_int_unlomb(value: u64) -> Vec<u8> {
    if value < 128 {
        return vec![value as u8];
    }
    let mut chunks = Vec::new();
    let mut v = value;
    while v > 0 {
        chunks.push((v & 0x7F) as u8);
        v >>= 7;
    }
    chunks.reverse();
    let last = chunks.len() - 1;
    for b in chunks.iter_mut().take(last) {
        *b |= 0x80;
    }
    chunks
}

/// Encode a TPEG2 component (tests).
pub fn encode_tpeg2_component(id: u64, data: &[u8]) -> Vec<u8> {
    let mut out = write_int_unlomb(id);
    out.extend_from_slice(&write_int_unlomb(data.len() as u64));
    out.extend_from_slice(data);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_empty_payload_frame() {
        let frame = encode_frame(b"");
        let found = find_frames(&frame);
        assert_eq!(found.len(), 1);
        assert!(found[0].payload.is_empty());
    }

    #[test]
    fn finds_frame_among_junk() {
        let mut buf = vec![0x00, 0x11, 0x22];
        buf.extend_from_slice(&encode_frame(b"hello"));
        buf.extend_from_slice(&[0xAA, 0xBB]);
        let found = find_frames(&buf);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].payload, b"hello");
    }

    #[test]
    fn rejects_corrupt_crc() {
        let mut frame = encode_frame(b"hello");
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        assert!(find_frames(&frame).is_empty());
    }

    #[test]
    fn tpeg1_service_walk() {
        let mut payload = vec![0x00, 0x00, 0x00, 0x05];
        payload.push(0x01); // SCID
        payload.extend_from_slice(&3u16.to_be_bytes());
        payload.extend_from_slice(b"abc");
        let svc = parse_tpeg1_service(&payload).unwrap();
        assert_eq!(svc.enc_id, 0);
        assert_eq!(svc.sid, 5);
        assert_eq!(svc.components.len(), 1);
        assert_eq!(svc.components[0].data, b"abc");
    }

    #[test]
    fn int_unlomb_round_trip() {
        for v in [0u64, 1, 127, 128, 255, 16383, 16384] {
            let bytes = write_int_unlomb(v);
            let mut pos = 0;
            assert_eq!(read_int_unlomb(&bytes, &mut pos), Some(v), "value {v}");
            assert_eq!(pos, bytes.len());
        }
    }

    #[test]
    fn tpeg2_nested_components() {
        let inner = encode_tpeg2_component(2, b"\x2A");
        let outer = encode_tpeg2_component(1, &inner);
        let parsed = parse_tpeg2_components(&outer);
        assert_eq!(parsed.len(), 1);
        assert_eq!(parsed[0].id, 1);
        assert_eq!(parsed[0].children.len(), 1);
        assert_eq!(parsed[0].children[0].id, 2);
        assert_eq!(parsed[0].children[0].data, b"\x2A");
    }

    #[test]
    fn overflowing_int_unlomb_is_rejected_without_panic() {
        // Ten continuation bytes (0xFF) force a shift that cannot fit in u64.
        let mut bytes = vec![0xFF; 10];
        bytes.push(0x7F);
        let mut pos = 0;
        assert_eq!(read_int_unlomb(&bytes, &mut pos), None);
    }

    #[test]
    fn crc_valid_frame_with_overflowing_component_length_does_not_panic() {
        // id=1, then an IntUnLoMB length that overflows on decode.
        let mut payload = vec![0x01];
        payload.extend(std::iter::repeat_n(0xFF, 10));
        payload.push(0x7F);
        let frame = encode_frame(&payload);
        assert_eq!(find_frames(&frame).len(), 1);
        assert!(parse_tpeg2_components(&payload).is_empty());
    }

    #[test]
    fn component_length_exceeding_usize_add_is_rejected() {
        // id=1 (single byte), length = a large but representable IntUnLoMB that
        // cannot be added to `pos` without overflowing usize bounds checks.
        let mut payload = write_int_unlomb(1);
        // Encode length = usize::MAX (fits u64 on 64-bit; checked_add with pos fails).
        payload.extend_from_slice(&write_int_unlomb(usize::MAX as u64));
        assert!(parse_tpeg2_components(&payload).is_empty());
    }
}
