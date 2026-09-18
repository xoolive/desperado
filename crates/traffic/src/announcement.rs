//! DAB Traffic Announcement events (FIG 0/18 support + FIG 0/19 switching).
//!
//! Unlike TPEG/TEC or RDS-TMC, announcements carry no structured event code
//! or location reference — only that an NRK (or other) audio announcement
//! started or ended on a given cluster/subchannel. Keep this shape separate
//! from the GeoJSON TEC schema.

use serde::Serialize;

/// Bearer tag for DAB announcement switching (FIG 0/19).
pub const BEARER_DAB_ANNOUNCEMENT: &str = "dab-announcement";

/// Announcement-type bit flags (ETSI EN 300 401 Table 15 / TS 101 756).
///
/// Bits are numbered b0..b15 with b0 = LSB of the 16-bit ASu/ASw field when
/// that field is read as a big-endian `u16`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum AnnouncementTypeBit {
    Alarm = 1 << 0,
    RoadTrafficFlash = 1 << 1,
    TransportFlash = 1 << 2,
    WarningService = 1 << 3,
    NewsFlash = 1 << 4,
    AreaWeatherFlash = 1 << 5,
    EventAnnouncement = 1 << 6,
    SpecialEvent = 1 << 7,
    ProgrammeInformation = 1 << 8,
    SportReport = 1 << 9,
    FinancialReport = 1 << 10,
}

impl AnnouncementTypeBit {
    pub fn name(self) -> &'static str {
        match self {
            Self::Alarm => "Alarm",
            Self::RoadTrafficFlash => "Road Traffic flash",
            Self::TransportFlash => "Transport flash",
            Self::WarningService => "Warning/Service",
            Self::NewsFlash => "News flash",
            Self::AreaWeatherFlash => "Area weather flash",
            Self::EventAnnouncement => "Event announcement",
            Self::SpecialEvent => "Special event",
            Self::ProgrammeInformation => "Programme Information",
            Self::SportReport => "Sport report",
            Self::FinancialReport => "Financial report",
        }
    }

    pub fn all() -> &'static [Self] {
        &[
            Self::Alarm,
            Self::RoadTrafficFlash,
            Self::TransportFlash,
            Self::WarningService,
            Self::NewsFlash,
            Self::AreaWeatherFlash,
            Self::EventAnnouncement,
            Self::SpecialEvent,
            Self::ProgrammeInformation,
            Self::SportReport,
            Self::FinancialReport,
        ]
    }
}

/// Decode set announcement-type names from a 16-bit ASu/ASw bitmap.
pub fn announcement_type_names(flags: u16) -> Vec<&'static str> {
    AnnouncementTypeBit::all()
        .iter()
        .filter(|b| flags & (**b as u16) != 0)
        .map(|b| b.name())
        .collect()
}

/// True when the bitmap includes Road Traffic and/or Transport flash.
pub fn is_traffic_relevant(flags: u16) -> bool {
    flags
        & (AnnouncementTypeBit::RoadTrafficFlash as u16
            | AnnouncementTypeBit::TransportFlash as u16)
        != 0
}

/// Lifecycle of one announcement switching event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AnnouncementPhase {
    Started,
    Continuing,
    Ended,
}

/// Location-less announcement event (not a TEC/TMC GeoJSON feature).
#[derive(Debug, Clone, Serialize)]
pub struct AnnouncementEvent {
    #[serde(rename = "type")]
    pub event_type: &'static str,
    pub bearer: &'static str,
    pub phase: AnnouncementPhase,
    pub cluster_id: u8,
    pub subchannel_id: u8,
    pub asw_flags: u16,
    pub asw_flags_hex: String,
    pub announcement_types: Vec<String>,
    pub traffic_relevant: bool,
    pub new_flag: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub region_id: Option<u8>,
    /// Services that advertise membership in this cluster via FIG 0/18.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub supporting_service_ids: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ensemble_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp_unix_ms: Option<u64>,
}

impl AnnouncementEvent {
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_else(|_| "{}".to_string())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn started(
        cluster_id: u8,
        subchannel_id: u8,
        asw_flags: u16,
        new_flag: bool,
        region_id: Option<u8>,
        supporting_service_ids: Vec<String>,
        ensemble_id: Option<String>,
        timestamp_unix_ms: Option<u64>,
    ) -> Self {
        Self {
            event_type: "AnnouncementEvent",
            bearer: BEARER_DAB_ANNOUNCEMENT,
            phase: AnnouncementPhase::Started,
            cluster_id,
            subchannel_id,
            asw_flags,
            asw_flags_hex: format!("0x{asw_flags:04X}"),
            announcement_types: announcement_type_names(asw_flags)
                .into_iter()
                .map(str::to_string)
                .collect(),
            traffic_relevant: is_traffic_relevant(asw_flags),
            new_flag,
            region_id,
            supporting_service_ids,
            ensemble_id,
            timestamp_unix_ms,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn continuing(
        cluster_id: u8,
        subchannel_id: u8,
        asw_flags: u16,
        new_flag: bool,
        region_id: Option<u8>,
        supporting_service_ids: Vec<String>,
        ensemble_id: Option<String>,
        timestamp_unix_ms: Option<u64>,
    ) -> Self {
        let mut e = Self::started(
            cluster_id,
            subchannel_id,
            asw_flags,
            new_flag,
            region_id,
            supporting_service_ids,
            ensemble_id,
            timestamp_unix_ms,
        );
        e.phase = AnnouncementPhase::Continuing;
        e
    }

    #[allow(clippy::too_many_arguments)]
    pub fn ended(
        cluster_id: u8,
        subchannel_id: u8,
        asw_flags: u16,
        supporting_service_ids: Vec<String>,
        ensemble_id: Option<String>,
        timestamp_unix_ms: Option<u64>,
    ) -> Self {
        let mut e = Self::started(
            cluster_id,
            subchannel_id,
            asw_flags,
            false,
            None,
            supporting_service_ids,
            ensemble_id,
            timestamp_unix_ms,
        );
        e.phase = AnnouncementPhase::Ended;
        e
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn traffic_bits_match_table_15() {
        assert_eq!(AnnouncementTypeBit::RoadTrafficFlash as u16, 0x0002);
        assert_eq!(AnnouncementTypeBit::TransportFlash as u16, 0x0004);
        assert!(is_traffic_relevant(0x0002));
        assert!(is_traffic_relevant(0x0004));
        assert!(is_traffic_relevant(0x0006));
        assert!(!is_traffic_relevant(0x0001)); // Alarm only
        assert!(!is_traffic_relevant(0x0010)); // News only
    }

    #[test]
    fn names_decode_bitmap() {
        let names = announcement_type_names(0x0006);
        assert_eq!(names, vec!["Road Traffic flash", "Transport flash"]);
    }

    #[test]
    fn event_serializes_with_bearer() {
        let e = AnnouncementEvent::started(
            1,
            50,
            0x0002,
            true,
            None,
            vec!["0xF801".into()],
            Some("0xF501".into()),
            Some(1),
        );
        let j = e.to_json();
        assert!(j.contains("dab-announcement"));
        assert!(j.contains("Road Traffic flash"));
        assert!(j.contains("\"phase\":\"started\""));
    }
}
