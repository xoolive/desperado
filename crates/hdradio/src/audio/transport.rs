//! P1 audio-packet extraction and HDC PDU reassembly.

use std::collections::HashMap;

const HEADER_LEN: usize = 14;

/// One CRC-valid, complete HDC packet data unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HdcPdu {
    /// Zero-based HD program number.
    pub program: u8,
    /// Main or enhanced stream identifier.
    pub stream_id: u8,
    /// Six-bit elastic-buffer sequence number.
    pub sequence: u8,
    /// HDC payload bytes without packet CRC.
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
struct Header {
    codec_mode: u8,
    stream_id: u8,
    pfirst: bool,
    plast: bool,
    sequence: u8,
    packet_count: usize,
    has_expansion: bool,
    last_audio_location: usize,
}

#[derive(Debug, Default)]
struct PartialPacket {
    front: Vec<u8>,
    valid: bool,
}

/// Extracts HDC packets from PCI-valid P1 frame bytes.
#[derive(Debug, Default)]
pub struct AudioTransport {
    partial: HashMap<(u8, u8, u8), PartialPacket>,
}

impl AudioTransport {
    /// Parse one decoded P1 frame and return newly completed, CRC-valid HDC PDUs.
    pub fn push_p1(&mut self, frame: &[u8]) -> Vec<HdcPdu> {
        let mut output = Vec::new();
        let mut frame_offset = 0;
        while frame_offset + HEADER_LEN < frame.len() {
            let Some(header) = parse_header(&frame[frame_offset..]) else {
                break;
            };
            let location_bits = location_bits(header);
            let location_bytes = (location_bits * header.packet_count + 4) / 8;
            let mut offset = frame_offset + HEADER_LEN;
            if header.packet_count == 0
                || offset + location_bytes > frame.len()
                || frame_offset + header.last_audio_location >= frame.len()
            {
                break;
            }
            let locations: Vec<usize> = (0..header.packet_count)
                .map(|index| parse_location(&frame[offset..], location_bits, index))
                .collect();
            if locations
                .first()
                .is_none_or(|location| *location <= header.last_audio_location)
                || locations.windows(2).any(|pair| pair[1] <= pair[0])
                || locations
                    .last()
                    .is_none_or(|location| frame_offset + *location >= frame.len())
            {
                break;
            }
            offset += location_bytes;
            let mut program = 0;
            if header.has_expansion {
                let (length, decoded_program) = parse_expansion(&frame[offset..]);
                if length == 0 || offset + length > frame.len() {
                    break;
                }
                program = decoded_program;
            }
            // PSD/AAS occupies bytes through the last-audio marker.
            offset = frame_offset + header.last_audio_location + 1;
            for (packet_index, &location) in locations.iter().enumerate() {
                let packet_end = frame_offset + location;
                if packet_end < offset || packet_end >= frame.len() {
                    return output;
                }
                let packet = &frame[offset..=packet_end];
                let valid = crc8(packet) == 0;
                let payload = &packet[..packet.len() - 1];
                let sequence = header.sequence.wrapping_add(packet_index as u8) & 0x3f;
                let key = (program, header.stream_id, sequence);
                if packet_index == 0 && header.pfirst {
                    if let Some(partial) = self.partial.remove(&key)
                        && valid
                        && partial.valid
                    {
                        let mut complete = partial.front;
                        complete.extend_from_slice(payload);
                        output.push(HdcPdu {
                            program,
                            stream_id: header.stream_id,
                            sequence,
                            payload: complete,
                        });
                    }
                } else if packet_index + 1 == header.packet_count && header.plast {
                    self.partial.insert(
                        key,
                        PartialPacket {
                            front: payload.to_vec(),
                            valid,
                        },
                    );
                } else if valid {
                    output.push(HdcPdu {
                        program,
                        stream_id: header.stream_id,
                        sequence,
                        payload: payload.to_vec(),
                    });
                }
                offset = packet_end + 1;
            }
            frame_offset = locations
                .last()
                .map_or(frame.len(), |location| frame_offset + location + 1);
        }
        output
    }
}

fn parse_header(bytes: &[u8]) -> Option<Header> {
    if bytes.len() < HEADER_LEN {
        return None;
    }
    let header = Header {
        codec_mode: bytes[8] & 0x0f,
        stream_id: (bytes[8] >> 4) & 3,
        pfirst: bytes[11] & 2 != 0,
        plast: bytes[11] & 4 != 0,
        sequence: (bytes[11] >> 3) | ((bytes[12] & 1) << 5),
        packet_count: usize::from((bytes[12] >> 1) & 0x3f),
        has_expansion: bytes[12] & 0x80 != 0,
        last_audio_location: usize::from(bytes[13]),
    };
    matches!(header.codec_mode, 0..=3 | 10 | 13).then_some(header)
}

fn location_bits(header: Header) -> usize {
    match header.codec_mode {
        1..=3 if header.stream_id == 0 => 12,
        10 | 13 => 12,
        _ => 16,
    }
}

fn parse_location(bytes: &[u8], bits: usize, index: usize) -> usize {
    if bits == 16 {
        usize::from(bytes[2 * index]) | (usize::from(bytes[2 * index + 1]) << 8)
    } else if index.is_multiple_of(2) {
        usize::from(bytes[index / 2 * 3]) | ((usize::from(bytes[index / 2 * 3 + 1]) & 0x0f) << 8)
    } else {
        (usize::from(bytes[index / 2 * 3 + 1]) >> 4) | (usize::from(bytes[index / 2 * 3 + 2]) << 4)
    }
}

fn parse_expansion(bytes: &[u8]) -> (usize, u8) {
    let mut offset = 0;
    let mut program = 0;
    loop {
        let Some(&byte) = bytes.get(offset) else {
            return (0, program);
        };
        let continuation = byte & 0x80 != 0;
        match (byte >> 4) & 7 {
            1 => {
                program = (byte >> 1) & 7;
                if byte & 1 != 0 {
                    offset += 2;
                }
            }
            2 => offset += 1,
            3 => offset += if byte & 8 != 0 { 4 } else { 3 },
            4 => offset += if byte & 8 != 0 { 3 } else { 1 },
            _ => {}
        }
        offset += 1;
        if !continuation {
            return (offset, program);
        }
    }
}

fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0xffu8;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}
