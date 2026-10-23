//! Typed, deterministic JSON output model.

use serde::Serialize;

/// Front-end state emitted before protocol decoding is available.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct FrontEndReport {
    /// JSON schema version.
    pub schema_version: u8,
    /// RF/capture settings used by the front end.
    pub rf: FrontEndRfReport,
    /// Number of samples read from the capture.
    pub input_samples: u64,
    /// Number of station-centred samples emitted at the NRSC-5 rate.
    pub output_samples: u64,
    /// Native OFDM acquisition measurements; protocol metadata is added only
    /// after the FEC/transport decoder validates it.
    pub phy: PhyReport,
    /// Reference-carrier synchronization after integer-CFO search.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_sync: Option<FrameSyncReport>,
    /// CRC-validated Station Information Service snapshot.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sis: Option<SisReport>,
    /// Program labels combined from SIS and the Station Information Guide.
    pub programs: Vec<Program>,
    /// Latest program-associated ID3 metadata for HD1.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub now_playing: Option<NowPlaying>,
}

/// Station Information Service snapshot.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct SisReport {
    /// Broadcast country code.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub country: Option<String>,
    /// FCC facility identifier.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fcc_facility_id: Option<u32>,
    /// Universal or legacy station name.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub station_name: Option<String>,
    /// Announced audio programs.
    pub audio_services: Vec<SisAudioServiceReport>,
    /// Announced data services.
    pub data_services: Vec<SisDataServiceReport>,
}

/// SIS audio service descriptor.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct SisAudioServiceReport {
    /// Zero-based HD program number.
    pub program: u8,
    /// Conditional-access flag.
    pub access: bool,
    /// NRSC-5 program type.
    pub service_type: u8,
    /// Sound-experience value.
    pub sound_experience: u8,
}

/// SIS data service descriptor.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct SisDataServiceReport {
    /// Conditional-access flag.
    pub access: bool,
    /// Service type.
    pub service_type: u16,
    /// MIME type identifier.
    pub mime_type: u16,
}

/// NRSC-5 32-symbol frame synchronization report.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct FrameSyncReport {
    /// Integer carrier displacement from nominal bins.
    pub coarse_frequency_bins: i32,
    /// Recovered block position.
    pub block_offset: u8,
    /// Primary service mode identifier.
    pub primary_service_mode: u8,
    /// Logical-channel block counter.
    pub block_counter: u8,
    /// Number of matching independent reference carriers.
    pub reference_matches: u8,
}

/// Native OFDM-acquisition measurements.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct PhyReport {
    /// Number of complete 32-symbol OFDM blocks extracted.
    pub locked_blocks: u64,
    /// Latest CP-derived carrier-frequency offset in Hz.
    pub frequency_offset_hz: f32,
    /// Latest timing peak-to-mean correlation ratio.
    pub timing_metric: f32,
    /// CRC-valid PIDS control frames recovered so far.
    pub valid_pids_frames: u64,
    /// Complete CRC-valid HDC packet data units extracted from P1.
    pub hdc_pdus: u64,
    /// AAC-core HDC frames decoded to PCM for the selected program.
    pub hdc_core_frames: u64,
    /// Structurally invalid HDC frames rendered as silence.
    pub hdc_core_rejected_frames: u64,
}

/// Frequency settings for [`FrontEndReport`].
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct FrontEndRfReport {
    /// FM hybrid mode label.
    pub mode: &'static str,
    /// Station centre selected by the user.
    pub station_frequency_hz: u32,
    /// I/Q recording centre.
    pub capture_center_hz: u32,
    /// I/Q recording rate.
    pub input_sample_rate_hz: u32,
    /// Fixed NRSC-5 FM PHY rate.
    pub ofdm_sample_rate_hz: u32,
    /// Applied baseband shift: station minus capture centre.
    pub station_offset_hz: i64,
}

/// Final protocol snapshot. Fields remain optional until their corresponding
/// CRC-validated NRSC-5 messages have been decoded.
#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct ReceiverSnapshot {
    /// JSON schema version.
    pub schema_version: u8,
    /// Protocol lock status.
    pub sync: &'static str,
    /// Station identity, when available.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub station: Option<Station>,
    /// Audio/data service descriptors.
    pub programs: Vec<Program>,
    /// Current PSD/ID3 information for the selected program.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub now_playing: Option<NowPlaying>,
}

/// Station Identity Service data.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Station {
    /// ISO country code.
    pub country: String,
    /// FCC facility number.
    pub fcc_facility_id: u32,
    /// Long station name.
    pub name: String,
    /// Station slogan.
    pub slogan: String,
    /// Geographic station location.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<Location>,
}

/// Station geographic location.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Location {
    /// North/south latitude in degrees.
    pub latitude: f64,
    /// East/west longitude in degrees.
    pub longitude: f64,
    /// Antenna altitude in metres.
    pub altitude_m: i32,
}

/// One NRSC-5 audio program.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct Program {
    /// Zero-based HD program index.
    pub index: u8,
    /// Program/service name.
    pub name: String,
    /// Program format/type.
    pub program_type: String,
    /// Codec identifier, e.g. `hdc`.
    pub codec: String,
}

/// Program-associated now-playing metadata.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct NowPlaying {
    /// Zero-based HD program index.
    pub program: u8,
    /// Track/program title.
    pub title: String,
    /// Artist name.
    pub artist: String,
    /// Genre/program format.
    pub genre: String,
}
