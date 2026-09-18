//! Shared GeoJSON traffic-event output.
//!
//! Both DAB/TPEG and FM/RDS-TMC emit the same Feature shape so a downstream
//! consumer does not need to special-case the bearer.

use serde::Serialize;

/// Bearer that produced a traffic event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum Bearer {
    DabTpeg,
    FmRdsTmc,
}

/// GeoJSON geometry. Coordinates are `[lon, lat]` per RFC 7946.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type")]
pub enum Geometry {
    Point { coordinates: [f64; 2] },
    LineString { coordinates: Vec<[f64; 2]> },
}

/// Traffic-event properties shared across bearers.
#[derive(Debug, Clone, Serialize, Default)]
pub struct TrafficProperties {
    pub bearer: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub service_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event_code: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event_codes: Option<Vec<u16>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location_code: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location_table: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extent: Option<i8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub direction: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diversion_advised: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub urgency: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub effect_code: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cause_code: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub encrypted: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unsupported_ca: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tpeg_sid: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extra: Option<serde_json::Value>,
}

impl TrafficProperties {
    pub fn for_bearer(bearer: Bearer) -> Self {
        Self {
            bearer: match bearer {
                Bearer::DabTpeg => "dab-tpeg",
                Bearer::FmRdsTmc => "fm-rds-tmc",
            },
            ..Self::default()
        }
    }
}

/// A GeoJSON Feature representing one traffic event.
#[derive(Debug, Clone, Serialize)]
pub struct TrafficFeature {
    #[serde(rename = "type")]
    pub feature_type: &'static str,
    pub geometry: Option<Geometry>,
    pub properties: TrafficProperties,
}

impl TrafficFeature {
    pub fn new(properties: TrafficProperties, geometry: Option<Geometry>) -> Self {
        Self {
            feature_type: "Feature",
            geometry,
            properties,
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_else(|_| "{}".to_string())
    }
}

/// GeoJSON FeatureCollection wrapper.
#[derive(Debug, Clone, Serialize)]
pub struct FeatureCollection {
    #[serde(rename = "type")]
    pub collection_type: &'static str,
    pub features: Vec<TrafficFeature>,
}

impl FeatureCollection {
    pub fn new(features: Vec<TrafficFeature>) -> Self {
        Self {
            collection_type: "FeatureCollection",
            features,
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self)
            .unwrap_or_else(|_| "{\"type\":\"FeatureCollection\",\"features\":[]}".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feature_serializes_null_geometry() {
        let mut props = TrafficProperties::for_bearer(Bearer::FmRdsTmc);
        props.event_code = Some(101);
        props.location_code = Some(1234);
        let json = TrafficFeature::new(props, None).to_json();
        assert!(json.contains("\"geometry\":null"));
        assert!(json.contains("fm-rds-tmc"));
        assert!(json.contains("\"event_code\":101"));
    }
}
