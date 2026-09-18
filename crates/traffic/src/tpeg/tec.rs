//! TPEG2-TEC (ISO 21219-15) application decoding.
//!
//! Component identifiers follow the TPEG2 UBCR convention used by open
//! implementations of the TEC UML model:
//!
//! - 0: Message Management Container (MMC)
//! - 1: TEC event
//! - 2: Location Referencing Container (LRC)
//!
//! Nested TEC/LRC fields are interpreted when they match the compact
//! integer encoding; unknown structure is retained in `extra` so a
//! consumer can inspect the raw tree. Encrypted / CA-protected service
//! frames are reported and not decoded.

use super::transport::{
    ServiceFrame, Tpeg2Component, TransportFrame, parse_tpeg1_service, parse_tpeg2_components,
};
use crate::geojson::{Bearer, TrafficFeature, TrafficProperties};
use crate::location::LocationTable;

/// TPEG2 application ID for TEC (ISO 21219-1).
pub const TEC_AID: u64 = 0x0005;

/// Conventional TPEG2 component IDs inside a TEC message.
pub const COMP_MMC: u64 = 0;
pub const COMP_TEC: u64 = 1;
pub const COMP_LRC: u64 = 2;

/// Decode every TPEG transport frame in `payload` into GeoJSON features.
pub fn decode_payload(
    payload: &[u8],
    locations: Option<&LocationTable>,
    service_id: Option<&str>,
) -> Vec<TrafficFeature> {
    let mut features = Vec::new();
    for frame in super::transport::find_frames(payload) {
        features.extend(decode_frame(&frame, locations, service_id));
    }
    features
}

/// Decode one validated transport frame.
pub fn decode_frame(
    frame: &TransportFrame,
    locations: Option<&LocationTable>,
    service_id: Option<&str>,
) -> Vec<TrafficFeature> {
    if frame.payload.is_empty() {
        return Vec::new();
    }

    if let Some(svc) = parse_tpeg1_service(&frame.payload) {
        if svc.enc_id != 0 {
            return vec![ca_unsupported_feature(svc.sid, service_id)];
        }
        let mut features = Vec::new();
        for comp in &svc.components {
            features.extend(decode_application_data(
                &comp.data, svc.sid, locations, service_id,
            ));
        }
        if !features.is_empty() {
            return features;
        }
    }

    decode_application_data(&frame.payload, 0, locations, service_id)
}

fn ca_unsupported_feature(sid: u32, service_id: Option<&str>) -> TrafficFeature {
    let mut props = TrafficProperties::for_bearer(Bearer::DabTpeg);
    props.unsupported_ca = Some(true);
    props.encrypted = Some(true);
    props.tpeg_sid = Some(format!("0x{sid:06X}"));
    props.service_id = service_id.map(str::to_string);
    props.description =
        Some("TPEG component is CA-protected; descrambling is not supported".into());
    TrafficFeature::new(props, None)
}

fn decode_application_data(
    data: &[u8],
    sid: u32,
    locations: Option<&LocationTable>,
    service_id: Option<&str>,
) -> Vec<TrafficFeature> {
    let tree = parse_tpeg2_components(data);
    if tree.is_empty() {
        return Vec::new();
    }
    let mut features = Vec::new();
    collect_tec_messages(&tree, sid, locations, service_id, &mut features);
    features
}

fn collect_tec_messages(
    nodes: &[Tpeg2Component],
    sid: u32,
    locations: Option<&LocationTable>,
    service_id: Option<&str>,
    out: &mut Vec<TrafficFeature>,
) {
    // A TEC message is a sibling group of MMC + TEC + LRC, or a node
    // whose children contain those IDs.
    if has_tec_siblings(nodes)
        && let Some(feature) = feature_from_group(nodes, sid, locations, service_id)
    {
        out.push(feature);
    }
    for node in nodes {
        if !node.children.is_empty() {
            collect_tec_messages(&node.children, sid, locations, service_id, out);
        }
    }
}

fn has_tec_siblings(nodes: &[Tpeg2Component]) -> bool {
    nodes.iter().any(|n| n.id == COMP_TEC || n.id == TEC_AID)
}

fn feature_from_group(
    nodes: &[Tpeg2Component],
    sid: u32,
    locations: Option<&LocationTable>,
    service_id: Option<&str>,
) -> Option<TrafficFeature> {
    let tec = nodes.iter().find(|n| n.id == COMP_TEC || n.id == TEC_AID)?;
    let lrc = nodes.iter().find(|n| n.id == COMP_LRC);

    let mut props = TrafficProperties::for_bearer(Bearer::DabTpeg);
    props.tpeg_sid = (sid != 0).then(|| format!("0x{sid:06X}"));
    props.service_id = service_id.map(str::to_string);

    if let Some((effect, cause, extra)) = parse_tec_fields(tec) {
        props.effect_code = effect;
        props.cause_code = cause;
        props.description = effect.and_then(effect_description);
        props.extra = extra;
    }

    let mut geometry = None;
    if let Some(lrc) = lrc
        && let Some(fields) = parse_lrc_fields(lrc)
    {
        props.location_code = fields.location;
        props.location_table = fields.table;
        props.direction = fields.direction;
        props.extent = fields.extent;
        if let Some(code) = fields.location {
            geometry = locations.and_then(|t| t.geometry(code));
        }
    }

    Some(TrafficFeature::new(props, geometry))
}

fn parse_tec_fields(
    tec: &Tpeg2Component,
) -> Option<(Option<u64>, Option<u64>, Option<serde_json::Value>)> {
    // Compact encoding: first IntUnLoMB is effectCode, optional second is causeCode.
    // Nested children override the compact form.
    let mut effect = None;
    let mut cause = None;
    if tec.children.is_empty() {
        let mut pos = 0;
        effect = super::transport::read_int_unlomb(&tec.data, &mut pos);
        if pos < tec.data.len() {
            cause = super::transport::read_int_unlomb(&tec.data, &mut pos);
        }
    } else {
        for child in &tec.children {
            match child.id {
                0 | 1 => {
                    let mut pos = 0;
                    effect = super::transport::read_int_unlomb(&child.data, &mut pos);
                }
                2 => {
                    let mut pos = 0;
                    cause = super::transport::read_int_unlomb(&child.data, &mut pos);
                }
                _ => {}
            }
        }
    }
    let extra = serde_json::to_value(component_summary(tec)).ok();
    Some((effect, cause, extra))
}

struct LrcFields {
    location: Option<u16>,
    table: Option<u8>,
    direction: Option<String>,
    extent: Option<i8>,
}

fn parse_lrc_fields(lrc: &Tpeg2Component) -> Option<LrcFields> {
    // TMC location reference, compact: loc16 | (table, direction, extent) optional.
    let data = if lrc.children.is_empty() {
        &lrc.data
    } else {
        lrc.children.first().map(|c| &c.data).unwrap_or(&lrc.data)
    };
    if data.len() < 2 {
        return None;
    }
    let loc = u16::from_be_bytes([data[0], data[1]]);
    let table = data.get(2).copied();
    let direction = data.get(3).map(|d| {
        if d & 1 == 0 {
            "positive".to_string()
        } else {
            "negative".to_string()
        }
    });
    let extent = data.get(4).map(|&e| e as i8);
    Some(LrcFields {
        location: Some(loc),
        table,
        direction,
        extent,
    })
}

fn component_summary(node: &Tpeg2Component) -> serde_json::Value {
    serde_json::json!({
        "id": node.id,
        "data_len": node.data.len(),
        "children": node.children.iter().map(component_summary).collect::<Vec<_>>(),
    })
}

/// Human-readable TPEG2-TEC effect codes (ISO 21219-15 subset).
fn effect_description(code: u64) -> Option<String> {
    Some(
        match code {
            1 => "traffic congestion",
            2 => "accident",
            3 => "roadworks",
            4 => "narrow lanes",
            5 => "impassability",
            6 => "slippery road",
            7 => "fire",
            8 => "hazardous driving conditions",
            9 => "objects on the road",
            10 => "animals on roadway",
            11 => "people on roadway",
            12 => "broken down vehicles",
            13 => "vehicle on wrong carriageway",
            14 => "rescue and recovery work",
            15 => "regulatory measure",
            16 => "extreme weather",
            17 => "visibility reduced",
            18 => "precipitation",
            19 => "thunderstorm",
            99 => "this message is cancelled",
            _ => return None,
        }
        .to_string(),
    )
}

/// True when a TPEG1 service frame signals encryption / CA.
pub fn service_is_encrypted(svc: &ServiceFrame) -> bool {
    svc.enc_id != 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::location::{Location, LocationTable};
    use crate::tpeg::transport::{encode_frame, encode_tpeg2_component, write_int_unlomb};

    fn tec_payload(effect: u64, loc: u16) -> Vec<u8> {
        let tec_data = write_int_unlomb(effect);
        let tec = encode_tpeg2_component(COMP_TEC, &tec_data);
        let mut lrc_data = loc.to_be_bytes().to_vec();
        lrc_data.extend_from_slice(&[1, 0, 2]); // table 1, positive, extent 2
        let lrc = encode_tpeg2_component(COMP_LRC, &lrc_data);
        let mut inner = tec;
        inner.extend_from_slice(&lrc);
        encode_tpeg2_component(10, &inner)
    }

    #[test]
    fn decodes_tec_event_from_transport_frame() {
        let app = tec_payload(2, 0x1234);
        let mut tpeg1 = vec![0x00, 0x00, 0x00, 0x42];
        tpeg1.push(0x05);
        tpeg1.extend_from_slice(&(app.len() as u16).to_be_bytes());
        tpeg1.extend_from_slice(&app);
        let frame = encode_frame(&tpeg1);
        let features = decode_payload(&frame, None, Some("0xF201"));
        assert_eq!(features.len(), 1);
        assert_eq!(features[0].properties.effect_code, Some(2));
        assert_eq!(features[0].properties.location_code, Some(0x1234));
        assert_eq!(
            features[0].properties.description.as_deref(),
            Some("accident")
        );
        assert!(features[0].geometry.is_none());
        assert_eq!(features[0].properties.bearer, "dab-tpeg");
    }

    #[test]
    fn ca_protected_frame_is_flagged_not_decoded() {
        let payload = vec![0x01, 0x00, 0x00, 0x01]; // enc_id = 1
        let frame = encode_frame(&payload);
        let features = decode_payload(&frame, None, None);
        assert_eq!(features.len(), 1);
        assert_eq!(features[0].properties.unsupported_ca, Some(true));
        assert_eq!(features[0].properties.encrypted, Some(true));
        assert!(features[0].properties.effect_code.is_none());
    }

    #[test]
    fn location_table_fills_geometry() {
        let mut table = LocationTable::new();
        table.insert(Location {
            code: 0x1234,
            lat: 59.9,
            lon: 10.7,
            name: None,
        });
        let app = tec_payload(1, 0x1234);
        let mut tpeg1 = vec![0x00, 0x00, 0x00, 0x01];
        tpeg1.push(0x05);
        tpeg1.extend_from_slice(&(app.len() as u16).to_be_bytes());
        tpeg1.extend_from_slice(&app);
        let frame = encode_frame(&tpeg1);
        let features = decode_payload(&frame, Some(&table), None);
        assert!(features[0].geometry.is_some());
    }
}
