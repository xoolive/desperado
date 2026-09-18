//! RDS-TMC ALERT-C decoding (ISO 14819-1).
//!
//! Group 3A (ODA, AID `CD46` / `CD47`) registers the TMC service and its
//! allocated group. Group 8A (or whatever group 3A allocated) carries user
//! messages (single-group and multi-group, up to 5 groups) and system /
//! encryption-administration groups.
//!
//! Encrypted services (location-table number 0, or an encryption-administration
//! group) are reported as encrypted. Location codes are not decrypted.

use super::events;
use crate::geojson::{Bearer, TrafficFeature, TrafficProperties};
use crate::location::LocationTable;

/// RDS Open Data Application IDs for TMC ALERT-C.
pub const TMC_AID_CD46: u16 = 0xCD46;
pub const TMC_AID_CD47: u16 = 0xCD47;

pub fn is_tmc_aid(aid: u16) -> bool {
    aid == TMC_AID_CD46 || aid == TMC_AID_CD47
}

/// Decoder state for one RDS-TMC service.
#[derive(Debug, Clone)]
pub struct TmcDecoder {
    initialized: bool,
    encrypted: bool,
    has_encid: bool,
    ltn: u8,
    sid: u8,
    encid: u8,
    continuity_index: u8,
    parts: [Option<(u16, u16)>; 5],
    locations: Option<LocationTable>,
}

impl Default for TmcDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl TmcDecoder {
    pub fn new() -> Self {
        Self {
            initialized: false,
            encrypted: false,
            has_encid: false,
            ltn: 0,
            sid: 0,
            encid: 0,
            continuity_index: 0,
            parts: [None; 5],
            locations: None,
        }
    }

    pub fn set_location_table(&mut self, table: LocationTable) {
        self.locations = Some(table);
    }

    pub fn is_encrypted(&self) -> bool {
        self.encrypted
    }

    pub fn location_table_number(&self) -> Option<u8> {
        if (self.initialized && !self.encrypted) || self.has_encid {
            Some(self.ltn)
        } else {
            None
        }
    }

    /// Group 3A application-info word (block 3) for a TMC AID.
    ///
    /// Variant 0 carries LTN; LTN = 0 means the service is encrypted
    /// (ISO 14819-1 clause 8.11).
    pub fn handle_system_group(&mut self, block3: u16) -> Option<TrafficFeature> {
        let variant = (block3 >> 14) & 0x03;
        if variant == 0 {
            self.initialized = true;
            let ltn = ((block3 >> 6) & 0x3F) as u8;
            self.encrypted = ltn == 0;
            if !self.encrypted {
                self.ltn = ltn;
            }
            if self.encrypted {
                return Some(self.encrypted_notice());
            }
        } else if variant == 1 {
            self.sid = ((block3 >> 6) & 0x3F) as u8;
        }
        None
    }

    /// User / system / encryption group: `x` is the 5 TMC bits of block 2,
    /// `y` is block 3, `z` is block 4.
    pub fn handle_user_group(&mut self, x: u16, y: u16, z: u16) -> Option<TrafficFeature> {
        if !self.initialized {
            return None;
        }

        // Encryption administration group: the five TMC bits are all zero.
        if x & 0x1F == 0 {
            self.sid = ((y >> 5) & 0x3F) as u8;
            self.encid = (y & 0x1F) as u8;
            self.ltn = ((z >> 10) & 0x3F) as u8;
            self.has_encid = true;
            self.encrypted = true;
            return Some(self.encrypted_notice());
        }

        let t = (x & 0x10) != 0;
        if t {
            // Tuning / system information: ignored for event output.
            return None;
        }

        if self.encrypted {
            // Do not decode encrypted location codes.
            return Some(self.encrypted_notice());
        }

        let f = (x & 0x08) != 0;
        if f {
            Some(self.decode_single(x, y, z))
        } else {
            self.push_multi(x, y, z)
        }
    }

    fn encrypted_notice(&self) -> TrafficFeature {
        let mut props = TrafficProperties::for_bearer(Bearer::FmRdsTmc);
        props.encrypted = Some(true);
        props.unsupported_ca = Some(true);
        props.description =
            Some("RDS-TMC service is encrypted (ISO 14819-6); decryption is not supported".into());
        if self.has_encid {
            props.extra = Some(serde_json::json!({
                "encryption_id": self.encid,
                "service_id": self.sid,
                "location_table": self.ltn,
            }));
        }
        TrafficFeature::new(props, None)
    }

    fn decode_single(&self, x: u16, y: u16, z: u16) -> TrafficFeature {
        let duration = (x & 0x07) as u8;
        let diversion = (y & 0x8000) != 0;
        let negative = (y & 0x4000) != 0;
        let extent = ((y >> 11) & 0x07) as u8;
        let event = y & 0x07FF;
        let location = z;
        self.feature(AlertCFields {
            events: &[event],
            location,
            extent,
            negative,
            diversion,
            duration: Some(duration),
            extra: &[],
        })
    }

    fn push_multi(&mut self, x: u16, y: u16, z: u16) -> Option<TrafficFeature> {
        let continuity = (x & 0x07) as u8;
        if continuity != self.continuity_index && self.continuity_index != 0 {
            self.clear_parts();
        }
        self.continuity_index = continuity;

        let is_first = (y & 0x8000) != 0;
        let current = if is_first {
            0
        } else if (y & 0x4000) != 0 {
            1
        } else {
            let gsi = ((y >> 12) & 0x03) as usize;
            4 - gsi
        };
        if current >= self.parts.len() {
            return None;
        }
        self.parts[current] = Some((y, z));

        let is_last = !is_first && ((y >> 12) & 0x03) == 0;
        if is_last {
            let feature = self.decode_multi();
            self.clear_parts();
            feature
        } else {
            None
        }
    }

    fn decode_multi(&self) -> Option<TrafficFeature> {
        let (y0, z0) = self.parts[0]?;
        let negative = (y0 & 0x4000) != 0;
        let mut extent = ((y0 >> 11) & 0x07) as u8;
        let mut events = vec![y0 & 0x07FF];
        let location = z0;
        let mut diversion = false;
        let mut duration = None;
        let mut extra_labels: Vec<(u8, u16)> = Vec::new();

        if let Some(fields) = self.freeform_fields() {
            for (label, data) in fields {
                match label {
                    0 => duration = Some(data as u8),
                    1 => match data {
                        4 => diversion = true,
                        5 => extent = extent.saturating_add(8),
                        6 => extent = extent.saturating_add(16),
                        _ => extra_labels.push((label, data)),
                    },
                    9 => events.push(data),
                    11 => extra_labels.push((label, data)), // diversion locations
                    _ => extra_labels.push((label, data)),
                }
            }
        }

        Some(self.feature(AlertCFields {
            events: &events,
            location,
            extent,
            negative,
            diversion,
            duration,
            extra: &extra_labels,
        }))
    }

    fn freeform_fields(&self) -> Option<Vec<(u8, u16)>> {
        const FIELD_SIZE: [usize; 16] = [3, 3, 5, 5, 5, 8, 8, 8, 8, 11, 16, 16, 16, 16, 0, 0];
        let second = self.parts[1]?;
        let second_gsi = ((second.0 >> 12) & 0x03) as usize;

        let mut bits: Vec<u8> = Vec::new();
        for i in 1..self.parts.len() {
            let Some((y, z)) = self.parts[i] else {
                break;
            };
            if i == 1 || i >= self.parts.len().saturating_sub(second_gsi) {
                for b in (0..12).rev() {
                    bits.push(((y >> b) & 1) as u8);
                }
                for b in (0..16).rev() {
                    bits.push(((z >> b) & 1) as u8);
                }
            }
        }

        let mut fields = Vec::new();
        let mut idx = 0;
        while bits.len() - idx > 4 {
            let label = pop_bits(&bits, &mut idx, 4) as u8;
            if (label as usize) >= FIELD_SIZE.len() {
                break;
            }
            let size = FIELD_SIZE[label as usize];
            if size == 0 || bits.len() - idx < size {
                break;
            }
            let data = pop_bits(&bits, &mut idx, size);
            if label == 0 && data == 0 {
                break;
            }
            if label <= 14 {
                fields.push((label, data));
            }
        }
        Some(fields)
    }

    fn feature(&self, f: AlertCFields<'_>) -> TrafficFeature {
        let mut props = TrafficProperties::for_bearer(Bearer::FmRdsTmc);
        props.event_code = f.events.first().copied();
        if f.events.len() > 1 {
            props.event_codes = Some(f.events.to_vec());
        }
        props.description = f
            .events
            .iter()
            .filter_map(|c| events::event_description(*c).map(str::to_string))
            .reduce(|a, b| format!("{a}. {b}"));
        props.location_code = Some(f.location);
        props.location_table = self.location_table_number();
        let signed_extent = if f.negative {
            -(f.extent as i8)
        } else {
            f.extent as i8
        };
        props.extent = Some(signed_extent);
        props.direction = Some(if f.negative { "negative" } else { "positive" }.to_string());
        props.diversion_advised = Some(f.diversion);
        props.duration = f.duration;
        if !f.extra.is_empty() {
            props.extra = Some(serde_json::json!({ "freeform": f.extra }));
        }
        let geometry = self.locations.as_ref().and_then(|t| t.geometry(f.location));
        TrafficFeature::new(props, geometry)
    }

    fn clear_parts(&mut self) {
        self.parts = [None; 5];
        self.continuity_index = 0;
    }
}

struct AlertCFields<'a> {
    events: &'a [u16],
    location: u16,
    extent: u8,
    negative: bool,
    diversion: bool,
    duration: Option<u8>,
    extra: &'a [(u8, u16)],
}

fn pop_bits(bits: &[u8], idx: &mut usize, n: usize) -> u16 {
    let mut v = 0u16;
    for _ in 0..n {
        if *idx < bits.len() {
            v = (v << 1) | bits[*idx] as u16;
            *idx += 1;
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmc_x_single(duration: u8) -> u16 {
        // T=0, F=1 (single), duration in low 3 bits
        0x08 | (duration as u16 & 0x07)
    }

    #[test]
    fn ltn_zero_marks_encrypted() {
        let mut dec = TmcDecoder::new();
        let feature = dec.handle_system_group(0x0000).unwrap();
        assert!(dec.is_encrypted());
        assert_eq!(feature.properties.encrypted, Some(true));
        assert_eq!(feature.properties.unsupported_ca, Some(true));
    }

    #[test]
    fn free_service_decodes_single_group() {
        let mut dec = TmcDecoder::new();
        // Variant 0, LTN=17, AFI=0, MGS=0
        assert!(dec.handle_system_group((17u16) << 6).is_none());
        assert!(!dec.is_encrypted());

        let x = tmc_x_single(3);
        // diversion=1, direction=+, extent=2, event=101
        let y = (1u16 << 15) | (2 << 11) | 101;
        let z = 0x1234;
        let feature = dec.handle_user_group(x, y, z).unwrap();
        assert_eq!(feature.properties.event_code, Some(101));
        assert_eq!(feature.properties.location_code, Some(0x1234));
        assert_eq!(feature.properties.extent, Some(2));
        assert_eq!(feature.properties.diversion_advised, Some(true));
        assert_eq!(
            feature.properties.description.as_deref(),
            Some("stationary traffic")
        );
        assert_eq!(feature.properties.bearer, "fm-rds-tmc");
        assert!(feature.geometry.is_none());
    }

    #[test]
    fn encrypted_user_group_is_not_decoded() {
        let mut dec = TmcDecoder::new();
        dec.handle_system_group(0); // LTN=0
        let feature = dec.handle_user_group(tmc_x_single(0), 101, 0x1234).unwrap();
        assert_eq!(feature.properties.encrypted, Some(true));
        assert!(feature.properties.location_code.is_none());
        assert!(feature.properties.event_code.is_none());
    }

    #[test]
    fn encryption_admin_group_is_reported() {
        let mut dec = TmcDecoder::new();
        dec.handle_system_group(17 << 6);
        // Force encrypted via admin group (x=0)
        let y = (0x12u16 << 5) | 0x05; // SID, ENCID
        let z = 17u16 << 10; // LTNBE
        let feature = dec.handle_user_group(0, y, z).unwrap();
        assert_eq!(feature.properties.encrypted, Some(true));
        assert!(dec.is_encrypted());
    }

    #[test]
    fn multi_group_assembles_additional_event() {
        let mut dec = TmcDecoder::new();
        dec.handle_system_group(17 << 6);

        // First group of a multi-group message: F=0, first-group bit in Y.
        let x0 = 0x01; // T=0, F=0, continuity 1
        let y0 = 0x8000 | (3 << 11) | 201; // first, extent 3, accident
        let z0 = 0x2222;
        assert!(dec.handle_user_group(x0, y0, z0).is_none());

        // Subsequent / last group (SG=1, GSI=0): freeform additional event (label 9, 11 bits)
        // Y bits 11..0 plus Z (16) = 28 freeform bits.
        // label 9 (01001) + event 701 (01010111101) = 9 bits label-size + 11 = 16, rest padding.
        let mut freeform: u32 = 0;
        freeform |= 0x9 << (28 - 4);
        freeform |= 701u32 << (28 - 4 - 11);
        let y1 = ((freeform >> 16) as u16) & 0x0FFF;
        let y1 = y1 | 0x4000; // SG
        let z1 = (freeform & 0xFFFF) as u16;
        let feature = dec.handle_user_group(x0, y1, z1).unwrap();
        assert_eq!(feature.properties.event_code, Some(201));
        assert_eq!(feature.properties.location_code, Some(0x2222));
    }
}
