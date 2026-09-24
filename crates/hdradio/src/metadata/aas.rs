//! Advanced Application Services (AAS), SIG, and ID3 metadata decoding.

use std::collections::BTreeMap;

/// Program-associated ID3 metadata.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ProgramMetadata {
    /// Track or program title.
    pub title: Option<String>,
    /// Artist or presenter.
    pub artist: Option<String>,
    /// Genre or station/program label.
    pub genre: Option<String>,
}

/// Streaming HDLC/AAS decoder over successive P1 frame payloads.
#[derive(Debug, Default)]
pub struct AasDecoder {
    hdlc: Vec<u8>,
    in_frame: bool,
    program_names: BTreeMap<u8, String>,
    metadata: BTreeMap<u8, ProgramMetadata>,
}

impl AasDecoder {
    /// Consume one PCI-valid P1 frame byte stream.
    pub fn push_p1(&mut self, frame: &[u8]) {
        // SIG HDLC streams share a P1 frame with multiple audio streams. Scan
        // the self-describing audio-service/name TLVs in the PCI-valid bytes;
        // intervening fixed-channel blocks prevent treating the whole physical
        // region as one contiguous HDLC stream.
        for offset in 0..frame.len().saturating_sub(7) {
            if frame[offset] != 0x40 || frame[offset + 4] != 0x69 {
                continue;
            }
            let service = u16::from_le_bytes([frame[offset + 1], frame[offset + 2]]);
            let length = usize::from(frame[offset + 5]);
            if service == 0 || length < 2 || offset + 5 + length > frame.len() {
                continue;
            }
            let name = latin1(&frame[offset + 7..offset + 5 + length])
                .trim_end()
                .to_owned();
            self.program_names.insert((service - 1) as u8, name);
        }
        for &byte in frame {
            if byte == 0x7e {
                if self.in_frame && !self.hdlc.is_empty() {
                    let encoded = std::mem::take(&mut self.hdlc);
                    self.process_hdlc(&encoded);
                }
                self.in_frame = true;
            } else if self.in_frame {
                self.hdlc.push(byte);
                if self.hdlc.len() > 65_536 {
                    self.hdlc.clear();
                    self.in_frame = false;
                }
            }
        }
    }

    /// Clear partial HDLC framing after PHY reacquisition while preserving
    /// previously validated metadata.
    pub fn reset_stream(&mut self) {
        self.hdlc.clear();
        self.in_frame = false;
    }

    /// Program labels recovered from the Station Information Guide.
    pub fn program_names(&self) -> &BTreeMap<u8, String> {
        &self.program_names
    }

    /// Latest ID3 metadata by program number.
    pub fn metadata(&self) -> &BTreeMap<u8, ProgramMetadata> {
        &self.metadata
    }

    fn process_hdlc(&mut self, encoded: &[u8]) {
        let mut decoded = Vec::with_capacity(encoded.len());
        let mut index = 0;
        while index < encoded.len() {
            if encoded[index] == 0x7d {
                index += 1;
                if index == encoded.len() {
                    return;
                }
                decoded.push(encoded[index] | 0x20);
            } else {
                decoded.push(encoded[index]);
            }
            index += 1;
        }
        if decoded.len() < 7 || fcs16(&decoded) != 0xf0b8 || decoded[0] != 0x21 {
            return;
        }
        let aas = &decoded[1..decoded.len() - 2];
        if aas.len() < 4 {
            return;
        }
        let port = u16::from_le_bytes([aas[0], aas[1]]);
        let payload = &aas[4..];
        if port == 0x5100 || (0x5201..=0x5207).contains(&port) {
            self.decode_id3((port & 7) as u8, payload);
        } else if port == 0x20 {
            self.decode_sig(payload);
        }
    }

    fn decode_id3(&mut self, program: u8, bytes: &[u8]) {
        if bytes.len() < 10 || &bytes[..5] != b"ID3\x03\x00" || bytes[5] != 0 {
            return;
        }
        let tag_length = synchsafe(&bytes[6..10]) + 10;
        if tag_length > bytes.len() {
            return;
        }
        let metadata = self.metadata.entry(program).or_default();
        let mut offset = 10;
        while offset + 10 <= tag_length {
            let header = &bytes[offset..offset + 10];
            let length = u32::from_be_bytes([header[4], header[5], header[6], header[7]]) as usize;
            if offset + 10 + length > tag_length {
                break;
            }
            let value = decode_text(&bytes[offset + 10..offset + 10 + length]);
            match &header[..4] {
                b"TIT2" => metadata.title = value,
                b"TPE1" => metadata.artist = value,
                b"TCON" => metadata.genre = value,
                _ => {}
            }
            offset += 10 + length;
        }
    }

    fn decode_sig(&mut self, bytes: &[u8]) {
        let mut offset = 0;
        let mut current_program = None;
        while offset < bytes.len() {
            let kind = bytes[offset];
            offset += 1;
            match kind & 0xf0 {
                0x40 if offset + 3 <= bytes.len() => {
                    let service = u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
                    current_program = (kind == 0x40 && service > 0).then_some((service - 1) as u8);
                    offset += 3;
                }
                0x60 if offset < bytes.len() => {
                    let length = usize::from(bytes[offset]);
                    offset += 1;
                    if length == 0 || offset + length - 1 > bytes.len() {
                        break;
                    }
                    let value = &bytes[offset..offset + length - 1];
                    if kind == 0x69
                        && value.len() >= 2
                        && let Some(program) = current_program
                    {
                        let name = latin1(&value[1..]);
                        self.program_names.insert(program, name);
                    }
                    offset += length - 1;
                }
                _ => break,
            }
        }
    }
}

fn synchsafe(bytes: &[u8]) -> usize {
    bytes.iter().fold(0usize, |value, &byte| {
        (value << 7) | usize::from(byte & 0x7f)
    })
}

fn decode_text(bytes: &[u8]) -> Option<String> {
    let (&encoding, text) = bytes.split_first()?;
    let value = match encoding {
        0 => latin1(text),
        1 => decode_utf16(text),
        _ => return None,
    };
    Some(value.trim_end_matches('\0').to_owned())
}

fn latin1(bytes: &[u8]) -> String {
    bytes.iter().map(|&byte| char::from(byte)).collect()
}

fn decode_utf16(bytes: &[u8]) -> String {
    if bytes.len() < 2 {
        return String::new();
    }
    let (little_endian, payload) = match &bytes[..2] {
        [0xff, 0xfe] => (true, &bytes[2..]),
        [0xfe, 0xff] => (false, &bytes[2..]),
        _ => (false, bytes),
    };
    let units = payload.chunks_exact(2).map(|pair| {
        if little_endian {
            u16::from_le_bytes([pair[0], pair[1]])
        } else {
            u16::from_be_bytes([pair[0], pair[1]])
        }
    });
    char::decode_utf16(units)
        .map(|value| value.unwrap_or(char::REPLACEMENT_CHARACTER))
        .collect()
}

fn fcs16(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &byte in bytes {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8408
            } else {
                crc >> 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_id3_text_frames() {
        let mut tag = b"ID3\x03\x00\x00\x00\x00\x00\x00".to_vec();
        for (name, value) in [(b"TIT2", "Choosin' Texas"), (b"TPE1", "Ella Langley")] {
            tag.extend_from_slice(name);
            tag.extend_from_slice(&((value.len() + 1) as u32).to_be_bytes());
            tag.extend_from_slice(&[0, 0, 0]);
            tag.extend_from_slice(value.as_bytes());
        }
        let length = tag.len() - 10;
        tag[9] = length as u8;
        let mut decoder = AasDecoder::default();
        decoder.decode_id3(0, &tag);
        let metadata = &decoder.metadata()[&0];
        assert_eq!(metadata.title.as_deref(), Some("Choosin' Texas"));
        assert_eq!(metadata.artist.as_deref(), Some("Ella Langley"));
    }

    #[test]
    fn scans_sig_audio_program_names() {
        let mut frame = vec![0; 12];
        frame.extend_from_slice(&[
            0x40, 1, 0, 0, 0x69, 9, 0, b'X', b'L', b'1', b'0', b'6', b'.', b'7',
        ]);
        let mut decoder = AasDecoder::default();
        decoder.push_p1(&frame);
        assert_eq!(
            decoder.program_names().get(&0).map(String::as_str),
            Some("XL106.7")
        );
    }
}
