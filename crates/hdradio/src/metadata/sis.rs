//! CRC-validated Station Information Service (SIS) control messages.

use std::collections::BTreeMap;

use crate::protocol::pids::PidsFrame;

const SIS_FRAME_BITS: usize = 80;
const SIS_PAYLOAD_BITS: usize = 63;
const MESSAGE_LENGTHS: [Option<usize>; 16] = [
    Some(32),
    Some(22),
    Some(58),
    Some(32),
    Some(27),
    Some(58),
    Some(27),
    Some(22),
    Some(58),
    Some(58),
    Some(27),
    None,
    None,
    None,
    None,
    None,
];

/// Incremental SIS state recovered from PIDS frames.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SisDecoder {
    country: Option<String>,
    fcc_facility_id: Option<u32>,
    short_name: Option<String>,
    universal_name: UniversalName,
    audio_services: BTreeMap<u8, AudioService>,
    data_services: Vec<DataService>,
}

/// Fields that changed while processing one SIS PIDS frame.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SisUpdate {
    /// The broadcast country code, when present.
    pub country: Option<String>,
    /// The FCC facility ID, when present.
    pub fcc_facility_id: Option<u32>,
    /// The short or universal station name, when present.
    pub short_name: Option<String>,
    /// Audio service descriptions changed by this frame.
    pub audio_services: Vec<AudioService>,
    /// Data service descriptions changed by this frame.
    pub data_services: Vec<DataService>,
}

/// SIS audio service description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AudioService {
    /// Zero-based HD program number.
    pub program: u8,
    /// Whether conditional access is required.
    pub access: bool,
    /// NRSC-5 program type.
    pub service_type: u8,
    /// Five-bit sound-experience value.
    pub sound_experience: u8,
}

/// SIS data service description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataService {
    /// Whether conditional access is required.
    pub access: bool,
    /// Nine-bit service type.
    pub service_type: u16,
    /// Twelve-bit MIME type identifier.
    pub mime_type: u16,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct UniversalName {
    encoding: u8,
    append_fm: bool,
    frame_count: usize,
    bytes: [u8; 12],
    have: [bool; 2],
}

impl SisDecoder {
    /// Consume one CRC-valid PIDS frame. LLDS frames are not SIS and are
    /// intentionally ignored here.
    pub fn push(&mut self, frame: &PidsFrame) -> SisUpdate {
        if frame.frame_type != 0 || frame.payload.len() != SIS_FRAME_BITS - 1 {
            return SisUpdate::default();
        }

        let payload_count = usize::from(frame.payload[0]) + 1;
        let mut offset = 1;
        let mut update = SisUpdate::default();
        for _ in 0..payload_count {
            if offset + 4 > SIS_PAYLOAD_BITS {
                break;
            }
            let message_id = read_unsigned(&frame.payload, &mut offset, 4) as usize;
            let Some(length) = MESSAGE_LENGTHS[message_id] else {
                break;
            };
            if offset + length > SIS_PAYLOAD_BITS {
                break;
            }
            let message = &frame.payload[offset..offset + length];
            match message_id {
                0 => self.decode_station_id(message, &mut update),
                1 => self.decode_short_name(message, &mut update),
                6 | 10 => self.decode_service(message, &mut update),
                8 => self.decode_universal_name(message, &mut update),
                _ => {}
            }
            offset += length;
        }
        update
    }

    /// Country code, if a station-ID message was received.
    pub fn country(&self) -> Option<&str> {
        self.country.as_deref()
    }

    /// FCC facility ID, if a station-ID message was received.
    pub fn fcc_facility_id(&self) -> Option<u32> {
        self.fcc_facility_id
    }

    /// Station short name, if received.
    pub fn short_name(&self) -> Option<&str> {
        self.short_name.as_deref()
    }

    /// Currently announced audio services, ordered by program number.
    pub fn audio_services(&self) -> impl Iterator<Item = &AudioService> {
        self.audio_services.values()
    }

    /// Currently announced data services.
    pub fn data_services(&self) -> &[DataService] {
        &self.data_services
    }

    fn decode_station_id(&mut self, bits: &[u8], update: &mut SisUpdate) {
        let mut offset = 0;
        let country = (0..2)
            .map(|_| char5(read_unsigned(bits, &mut offset, 5)))
            .collect::<String>();
        offset += 3;
        let facility = read_unsigned(bits, &mut offset, 19);
        if self.country.as_deref() != Some(country.as_str()) {
            self.country = Some(country.clone());
            update.country = Some(country);
        }
        if self.fcc_facility_id != Some(facility) {
            self.fcc_facility_id = Some(facility);
            update.fcc_facility_id = Some(facility);
        }
    }

    fn decode_short_name(&mut self, bits: &[u8], update: &mut SisUpdate) {
        let mut offset = 0;
        let mut name = (0..4)
            .map(|_| char5(read_unsigned(bits, &mut offset, 5)))
            .collect::<String>();
        if bits[offset] == 0 && bits[offset + 1] == 1 {
            name.push_str("-FM");
        }
        self.set_name(name, update);
    }

    fn decode_universal_name(&mut self, bits: &[u8], update: &mut SisUpdate) {
        let mut offset = 0;
        let frame = read_unsigned(bits, &mut offset, 4) as usize;
        let is_slogan = bits[offset] != 0;
        offset += 1;
        if is_slogan || frame >= self.universal_name.have.len() {
            return;
        }
        if frame == 0 {
            self.universal_name.encoding = read_unsigned(bits, &mut offset, 3) as u8;
            self.universal_name.append_fm = bits[offset] != 0;
            offset += 1;
            self.universal_name.frame_count = usize::from(bits[offset]) + 1;
            offset += 1;
        } else {
            offset += 5;
        }
        for byte in 0..6 {
            self.universal_name.bytes[frame * 6 + byte] = read_unsigned(bits, &mut offset, 8) as u8;
        }
        self.universal_name.have[frame] = true;
        let complete = self.universal_name.frame_count > 0
            && self.universal_name.have[..self.universal_name.frame_count]
                .iter()
                .all(|have| *have);
        if complete {
            let length = self
                .universal_name
                .bytes
                .iter()
                .position(|byte| *byte == 0)
                .unwrap_or(self.universal_name.frame_count * 6);
            let mut name =
                String::from_utf8_lossy(&self.universal_name.bytes[..length]).into_owned();
            if self.universal_name.append_fm {
                name.push_str("-FM");
            }
            self.set_name(name, update);
        }
    }

    fn decode_service(&mut self, bits: &[u8], update: &mut SisUpdate) {
        let mut offset = 0;
        match read_unsigned(bits, &mut offset, 2) {
            0 => {
                let service = AudioService {
                    access: read_unsigned(bits, &mut offset, 1) != 0,
                    program: read_unsigned(bits, &mut offset, 6) as u8,
                    service_type: read_unsigned(bits, &mut offset, 8) as u8,
                    sound_experience: {
                        offset += 5;
                        read_unsigned(bits, &mut offset, 5) as u8
                    },
                };
                if self.audio_services.get(&service.program) != Some(&service) {
                    self.audio_services.insert(service.program, service);
                    update.audio_services.push(service);
                }
            }
            1 => {
                let service = DataService {
                    access: read_unsigned(bits, &mut offset, 1) != 0,
                    service_type: read_unsigned(bits, &mut offset, 9) as u16,
                    mime_type: {
                        offset += 3;
                        read_unsigned(bits, &mut offset, 12) as u16
                    },
                };
                if !self.data_services.contains(&service) {
                    self.data_services.push(service);
                    update.data_services.push(service);
                }
            }
            _ => {}
        }
    }

    fn set_name(&mut self, name: String, update: &mut SisUpdate) {
        if self.short_name.as_deref() != Some(name.as_str()) {
            self.short_name = Some(name.clone());
            update.short_name = Some(name);
        }
    }
}

fn read_unsigned(bits: &[u8], offset: &mut usize, length: usize) -> u32 {
    let mut value = 0;
    for _ in 0..length {
        value = (value << 1) | u32::from(bits[*offset]);
        *offset += 1;
    }
    value
}

fn char5(value: u32) -> char {
    const TABLE: &[u8; 32] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ ?-*$ ";
    TABLE[value as usize] as char
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_unsigned(bits: &mut [u8], offset: &mut usize, value: u32, length: usize) {
        for shift in (0..length).rev() {
            bits[*offset] = ((value >> shift) & 1) as u8;
            *offset += 1;
        }
    }

    #[test]
    fn decodes_station_id_and_short_name() {
        let mut payload = vec![0; 79];
        payload[0] = 1; // Two SIS messages.
        let mut offset = 1;
        write_unsigned(&mut payload, &mut offset, 0, 4);
        write_unsigned(&mut payload, &mut offset, 20, 5); // U
        write_unsigned(&mut payload, &mut offset, 18, 5); // S
        offset += 3;
        write_unsigned(&mut payload, &mut offset, 29_569, 19);
        write_unsigned(&mut payload, &mut offset, 1, 4);
        for character in [23, 11, 27, 27] {
            // XL??
            write_unsigned(&mut payload, &mut offset, character, 5);
        }
        payload[offset] = 0;
        payload[offset + 1] = 1;

        let frame = PidsFrame {
            frame_type: 0,
            payload,
        };
        let mut decoder = SisDecoder::default();
        let update = decoder.push(&frame);
        assert_eq!(update.country.as_deref(), Some("US"));
        assert_eq!(update.fcc_facility_id, Some(29_569));
        assert_eq!(decoder.short_name(), Some("XL??-FM"));
    }
}
