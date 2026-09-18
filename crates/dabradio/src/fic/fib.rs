//! FIB (Fast Information Block) parser.
//!
//! Parses FIG (Fast Information Group) types from FIBs to extract:
//! - Ensemble info (FIG 0/0): ensemble ID, country, change flags
//! - Subchannel config (FIG 0/1): subchannel ID, start address, size, protection
//! - Service-component mapping (FIG 0/2): service ID → component → subchannel / SCId
//! - Packet-mode components (FIG 0/3): SCId → SubChId + packet address
//! - Service-component global definition (FIG 0/8): SId+SCIdS → SubChId / SCId
//! - User application information (FIG 0/13): UAtype (0x004 = TPEG)
//! - Announcement support (FIG 0/18): ASu bitmap + cluster membership
//! - Announcement switching (FIG 0/19): live cluster → SubChId trigger
//! - Service labels (FIG 1/1, FIG 1/5): 16-char labels for audio and data services
//! - Ensemble label (FIG 1/0 with OE=0, ext=0): ensemble name
//!
//! Reference: ETSI EN 300 401 §5.2, §6, §8

use crate::charsets;
use serde::Serialize;
use std::collections::HashMap;

/// Accumulated ensemble information from FIBs.
#[derive(Debug, Default, Clone)]
pub struct EnsembleInfo {
    pub ensemble_id: Option<u16>,
    pub ensemble_label: Option<String>,
    pub services: HashMap<u32, ServiceInfo>,
    pub subchannels: HashMap<u8, SubchannelInfo>,
    /// FIG 0/3 packet-mode components keyed by 12-bit SCId.
    pub packet_components: HashMap<u16, PacketComponent>,
    /// FIG 0/8 bindings: (SId, SCIdS) → subchannel or packet SCId.
    pub scids_bindings: HashMap<(u32, u8), ScidsBinding>,
    /// FIG 0/13 user applications keyed by (SId, SCIdS).
    pub user_applications: HashMap<(u32, u8), Vec<UserApplication>>,
    /// FIG 0/18 announcement support keyed by SId.
    pub announcement_support: HashMap<u32, AnnouncementSupport>,
    /// Latest FIG 0/19 switching state keyed by Cluster Id.
    pub announcement_switching: HashMap<u8, AnnouncementSwitch>,
}

/// Audio coding signalled by FIG 0/2 ASCTy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum AudioCoding {
    /// ASCTy 0: legacy DAB MPEG-1/2 Audio Layer II.
    MpegLayer2,
    /// ASCTy 63: DAB+ HE-AAC.
    DabPlus,
    /// A stream-audio component type not yet supported.
    Other(u8),
}

impl AudioCoding {
    fn from_ascty(ascty: u8) -> Self {
        match ascty {
            0 => Self::MpegLayer2,
            63 => Self::DabPlus,
            other => Self::Other(other),
        }
    }
}

/// Information about a single DAB service.
#[derive(Debug, Default, Clone, Serialize)]
pub struct ServiceInfo {
    pub service_id: u32,
    pub label: Option<String>,
    pub subchannel_id: Option<u8>,
    pub is_audio: bool,
    pub audio_coding: Option<AudioCoding>,
    pub bitrate: Option<u16>,
    pub protection: Option<String>,
    /// All MSC components signalled in FIG 0/2 for this service.
    #[serde(default)]
    pub components: Vec<ServiceComponent>,
}

/// FIG 0/2 service component (stream audio/data or packet mode).
#[derive(Debug, Clone, Serialize, Default)]
pub struct ServiceComponent {
    /// Transport Mechanism Identifier: 0 audio, 1 stream data, 3 packet data.
    pub tmid: u8,
    pub subchannel_id: Option<u8>,
    /// 12-bit SCId from FIG 0/2 when `tmid == 3`; resolved via FIG 0/3.
    pub scid: Option<u16>,
    pub packet_address: Option<u16>,
    pub dscty: Option<u8>,
    pub ascty: Option<u8>,
    pub ps_flag: bool,
    pub ca_flag: bool,
    pub scids: Option<u8>,
    pub user_applications: Vec<UserApplication>,
}

/// FIG 0/3 packet-mode component description.
///
/// Bit packing follows ETSI EN 300 401 §6.3.2: the optional CAOrg field is
/// present only when the CAOrg flag is set, so the entry is 5 or 7 bytes.
#[derive(Debug, Clone, Serialize, Default)]
pub struct PacketComponent {
    pub scid: u16,
    pub subchannel_id: u8,
    pub packet_address: u16,
    pub dscty: u8,
    /// `true` when data groups are **not** used (DG flag = 1).
    pub no_data_groups: bool,
    pub ca_org: Option<u16>,
}

/// FIG 0/8 mapping of (SId, SCIdS) onto a subchannel or packet SCId.
#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
pub enum ScidsBinding {
    Subchannel(u8),
    Packet(u16),
}

/// FIG 0/13 user application entry. UAtype 0x004 is TPEG (ETSI TS 101 756).
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct UserApplication {
    pub ua_type: u16,
    pub data: Vec<u8>,
}

impl UserApplication {
    pub const TPEG: u16 = 0x004;
    pub const TMC: u16 = 0x006;

    pub fn name(&self) -> &'static str {
        match self.ua_type {
            0x002 => "MOT slideshow",
            0x003 => "MOT website",
            Self::TPEG => "TPEG",
            0x005 => "DGPS",
            Self::TMC => "TMC",
            0x007 => "EPG",
            0x44A => "Journaline",
            _ => "other",
        }
    }
}

/// FIG 0/18 announcement support for one service (EN 300 401 §8.1.6.1).
#[derive(Debug, Clone, Serialize, PartialEq, Eq, Default)]
pub struct AnnouncementSupport {
    pub service_id: u32,
    /// 16-bit ASu flags (Table 15): bit1 = Road Traffic, bit2 = Transport, …
    pub asu_flags: u16,
    pub cluster_ids: Vec<u8>,
}

/// FIG 0/19 announcement switching entry (EN 300 401 §8.1.6.2).
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct AnnouncementSwitch {
    pub cluster_id: u8,
    /// 16-bit ASw flags: currently active announcement types.
    pub asw_flags: u16,
    pub new_flag: bool,
    pub region_flag: bool,
    pub subchannel_id: u8,
    pub region_id: Option<u8>,
}

/// Subchannel configuration.
#[derive(Debug, Default, Clone, Serialize)]
pub struct SubchannelInfo {
    pub id: u8,
    pub start_addr: u16,
    pub sub_size: u16, // in CUs
    pub protection_level: u8,
    pub is_eep: bool,
    pub eep_option: u8, // 0 = EEP-A, 1 = EEP-B (only meaningful when is_eep=true)
    /// FIG 0/1 short-form UEP table index; absent for EEP long form.
    pub uep_table_index: Option<u8>,
    pub bitrate: u16,
}

/// A resolved packet-mode component (FIG 0/2 tmid=3 + FIG 0/3 + optional 0/13).
#[derive(Debug, Clone)]
pub struct TpegTarget {
    pub service_id: u32,
    pub label: Option<String>,
    pub subchannel: SubchannelInfo,
    pub packet_address: u16,
    pub ca_flag: bool,
    pub ca_org: Option<u16>,
    pub dscty: Option<u8>,
    pub no_data_groups: bool,
    pub ua_types: Vec<u16>,
}

impl TpegTarget {
    pub fn is_tpeg(&self) -> bool {
        self.ua_types.contains(&UserApplication::TPEG)
    }
}

/// JSON output format for ensemble listing.
#[derive(Debug, Serialize)]
pub struct EnsembleOutput {
    pub ensemble_id: Option<u16>,
    pub ensemble_label: Option<String>,
    pub services: Vec<ServiceOutput>,
}

#[derive(Debug, Serialize)]
pub struct ServiceOutput {
    pub service_id: String,
    pub label: Option<String>,
    pub subchannel_id: Option<u8>,
    pub bitrate: Option<u16>,
    pub protection: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<ComponentOutput>,
}

#[derive(Debug, Serialize)]
pub struct ComponentOutput {
    pub tmid: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subchannel_id: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scid: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub packet_address: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dscty: Option<u8>,
    pub ca: bool,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub user_applications: Vec<String>,
}

impl EnsembleInfo {
    pub fn new() -> Self {
        Self::default()
    }

    /// Parse a FIB (30 data bytes, CRC already validated).
    pub fn parse_fib(&mut self, fib: &[u8; 32]) {
        let data = &fib[..30]; // 30 data bytes (CRC in bytes 30-31)
        let mut pos = 0;

        while pos < 30 {
            // Each FIG starts with a type byte
            let fig_type = (data[pos] >> 5) & 0x07;
            let fig_length = (data[pos] & 0x1F) as usize;

            if fig_type == 7 || fig_length == 0 {
                break; // End marker or padding
            }

            if pos + 1 + fig_length > 30 {
                break; // Malformed
            }

            let fig_data = &data[pos + 1..pos + 1 + fig_length];

            match fig_type {
                0 => self.parse_fig_type0(fig_data),
                1 => self.parse_fig_type1(fig_data),
                _ => {} // Other FIG types not needed for service listing
            }

            pos += 1 + fig_length;
        }
    }

    /// Parse FIG type 0 (Multiplex Configuration Information).
    fn parse_fig_type0(&mut self, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        let cn = (data[0] >> 7) & 1; // C/N flag
        let _oe = (data[0] >> 6) & 1; // Other Ensemble
        let _pd = (data[0] >> 5) & 1; // P/D flag (0=16-bit SId, 1=32-bit)
        let extension = data[0] & 0x1F;
        let _ = cn; // suppress unused warning

        let fig_data = &data[1..];

        match extension {
            0 => self.parse_fig0_ext0(fig_data),
            1 => self.parse_fig0_ext1(fig_data),
            2 => self.parse_fig0_ext2(fig_data, (data[0] >> 5) & 1),
            3 => self.parse_fig0_ext3(fig_data),
            8 => self.parse_fig0_ext8(fig_data, (data[0] >> 5) & 1),
            13 => self.parse_fig0_ext13(fig_data, (data[0] >> 5) & 1),
            18 => self.parse_fig0_ext18(fig_data, (data[0] >> 5) & 1),
            19 => self.parse_fig0_ext19(fig_data),
            _ => {}
        }
    }

    /// FIG 0/0: Ensemble information.
    /// Contains Ensemble ID (EId), change flags, AL flag, CIF count.
    fn parse_fig0_ext0(&mut self, data: &[u8]) {
        if data.len() < 4 {
            return;
        }
        let eid = ((data[0] as u16) << 8) | data[1] as u16;
        self.ensemble_id = Some(eid);
    }

    /// FIG 0/1: Sub-channel organization (basic subchannel info).
    /// Each entry: SubChId (6 bits), Start Address (10 bits), form flag (1 bit), then details.
    fn parse_fig0_ext1(&mut self, data: &[u8]) {
        let mut pos = 0;
        while pos + 3 <= data.len() {
            let subch_id = (data[pos] >> 2) & 0x3F;
            let start_addr = ((data[pos] as u16 & 0x03) << 8) | data[pos + 1] as u16;
            let form = (data[pos + 2] >> 7) & 1;

            if form == 0 {
                // Short form: table-driven (UEP). Table Switch=1 is
                // reserved for future use and its low six bits must not be
                // interpreted using the current table.
                let table_switch = (data[pos + 2] >> 6) & 1;
                if table_switch != 0 {
                    pos += 3;
                    continue;
                }
                let table_index = data[pos + 2] & 0x3F;
                let (sub_size, bitrate, prot_level) = uep_table(table_index);
                self.subchannels.insert(
                    subch_id,
                    SubchannelInfo {
                        id: subch_id,
                        start_addr,
                        sub_size,
                        protection_level: prot_level,
                        is_eep: false,
                        eep_option: 0,
                        uep_table_index: Some(table_index),
                        bitrate,
                    },
                );
                pos += 3;
            } else {
                // Long form: EEP
                if pos + 4 > data.len() {
                    break;
                }
                let option = (data[pos + 2] >> 4) & 0x07;
                let prot_level = (data[pos + 2] >> 2) & 0x03;
                let sub_size = ((data[pos + 2] as u16 & 0x03) << 8) | data[pos + 3] as u16;
                let bitrate = eep_bitrate(sub_size, option, prot_level);
                self.subchannels.insert(
                    subch_id,
                    SubchannelInfo {
                        id: subch_id,
                        start_addr,
                        sub_size,
                        protection_level: prot_level,
                        is_eep: true,
                        eep_option: option,
                        uep_table_index: None,
                        bitrate,
                    },
                );
                pos += 4;
            }
        }
    }

    /// FIG 0/2: Service organization.
    /// Maps service IDs to their components (subchannel references).
    fn parse_fig0_ext2(&mut self, data: &[u8], pd: u8) {
        let mut pos = 0;
        while pos < data.len() {
            let (sid, sid_len) = if pd == 0 {
                // 16-bit service ID (audio)
                if pos + 2 > data.len() {
                    break;
                }
                let sid = ((data[pos] as u32) << 8) | data[pos + 1] as u32;
                (sid, 2)
            } else {
                // 32-bit service ID (data)
                if pos + 4 > data.len() {
                    break;
                }
                let sid = ((data[pos] as u32) << 24)
                    | ((data[pos + 1] as u32) << 16)
                    | ((data[pos + 2] as u32) << 8)
                    | data[pos + 3] as u32;
                (sid, 4)
            };
            pos += sid_len;

            if pos >= data.len() {
                break;
            }

            let _local_flag = (data[pos] >> 7) & 1;
            let num_components = data[pos] & 0x0F;
            pos += 1;

            for _ in 0..num_components {
                if pos + 2 > data.len() {
                    break;
                }
                let tmid = (data[pos] >> 6) & 0x03;
                let is_audio = tmid == 0; // 0 = MSC stream audio
                let ascty = data[pos] & 0x3F;

                // FIG 0/2 component descriptor (ETSI EN 300 401 §6.3.1):
                //   TMId 00/01: Byte0 TMId|ASCTy/DSCTy, Byte1 SubChId|PS|CA
                //   TMId 11:    TMId (2) | SCId (12) | PS (1) | CA (1)
                let (subch_id, scid) = if tmid == 0 || tmid == 1 {
                    (Some((data[pos + 1] >> 2) & 0x3F), None)
                } else if tmid == 3 {
                    let scid = ((data[pos] as u16 & 0x3F) << 6) | (data[pos + 1] as u16 >> 2);
                    (None, Some(scid))
                } else {
                    (None, None)
                };

                let primary = (data[pos + 1] >> 1) & 1 != 0;
                let ca_flag = data[pos + 1] & 1 != 0;

                let service = self.services.entry(sid).or_insert_with(|| ServiceInfo {
                    service_id: sid,
                    ..Default::default()
                });
                // A service can have audio plus data/secondary components. Never
                // let a later data descriptor clear the selected audio codec;
                // prefer the primary audio component when one is signalled.
                if is_audio && (service.audio_coding.is_none() || primary) {
                    service.is_audio = true;
                    service.audio_coding = Some(AudioCoding::from_ascty(ascty));
                    if let Some(sc_id) = subch_id {
                        service.subchannel_id = Some(sc_id);
                    }
                }

                let mut component = ServiceComponent {
                    tmid,
                    subchannel_id: subch_id,
                    scid,
                    packet_address: None,
                    dscty: if tmid == 1 { Some(ascty) } else { None },
                    ascty: if is_audio { Some(ascty) } else { None },
                    ps_flag: primary,
                    ca_flag,
                    scids: None,
                    user_applications: Vec::new(),
                };
                if tmid == 3
                    && let Some(pkt) = scid.and_then(|id| self.packet_components.get(&id))
                {
                    component.subchannel_id = Some(pkt.subchannel_id);
                    component.packet_address = Some(pkt.packet_address);
                    component.dscty = Some(pkt.dscty);
                }
                // Deduplicate by stable identity. For packet-mode (TMId=3) that is
                // SCId — subchannel_id is resolved later via FIG 0/3 and must not
                // create a second entry when it flips from None to Some.
                if tmid == 3 {
                    if let Some(existing) = service
                        .components
                        .iter_mut()
                        .find(|c| c.tmid == 3 && c.scid == component.scid)
                    {
                        if component.subchannel_id.is_some() {
                            existing.subchannel_id = component.subchannel_id;
                        }
                        if component.packet_address.is_some() {
                            existing.packet_address = component.packet_address;
                        }
                        if component.dscty.is_some() {
                            existing.dscty = component.dscty;
                        }
                        existing.ps_flag = component.ps_flag;
                        existing.ca_flag = component.ca_flag;
                    } else {
                        service.components.push(component);
                    }
                } else {
                    let dup = service.components.iter().any(|c| {
                        c.tmid == component.tmid && c.subchannel_id == component.subchannel_id
                    });
                    if !dup {
                        service.components.push(component);
                    }
                }

                pos += 2;
            }
        }
    }

    /// FIG 0/3: Service component in packet mode (ETSI EN 300 401 §6.3.2).
    ///
    /// Each entry is 40 bits (5 bytes) plus an optional 16-bit CAOrg field
    /// when the CAOrg flag is set. welle.io always advances 7 bytes; that is
    /// only correct when CAOrg is present.
    fn parse_fig0_ext3(&mut self, data: &[u8]) {
        let mut pos = 0;
        while pos + 5 <= data.len() {
            let scid = ((data[pos] as u16) << 4) | (data[pos + 1] as u16 >> 4);
            let ca_org_flag = data[pos + 1] & 0x01 != 0;
            let dg_flag = data[pos + 2] & 0x80 != 0;
            let dscty = data[pos + 2] & 0x3F;
            let subch_id = data[pos + 3] >> 2;
            let packet_address = (((data[pos + 3] as u16) & 0x03) << 8) | data[pos + 4] as u16;

            let ca_org = if ca_org_flag {
                if pos + 7 > data.len() {
                    break;
                }
                Some(((data[pos + 5] as u16) << 8) | data[pos + 6] as u16)
            } else {
                None
            };

            self.packet_components.insert(
                scid,
                PacketComponent {
                    scid,
                    subchannel_id: subch_id,
                    packet_address,
                    dscty,
                    no_data_groups: dg_flag,
                    ca_org,
                },
            );

            pos += if ca_org_flag { 7 } else { 5 };
        }
    }

    /// FIG 0/8: Service component global definition (ETSI EN 300 401 §6.3.5).
    ///
    /// After SId: Ext(1) | Rfa(3) | SCIdS(4), then L/S flag. Short form binds
    /// a SubChId; long form binds a 12-bit packet SCId. An extra Rfa byte is
    /// present when Ext=1.
    fn parse_fig0_ext8(&mut self, data: &[u8], pd: u8) {
        let mut pos = 0;
        while pos < data.len() {
            let sid_len = if pd == 0 { 2 } else { 4 };
            if pos + sid_len + 2 > data.len() {
                break;
            }
            let sid = if pd == 0 {
                ((data[pos] as u32) << 8) | data[pos + 1] as u32
            } else {
                ((data[pos] as u32) << 24)
                    | ((data[pos + 1] as u32) << 16)
                    | ((data[pos + 2] as u32) << 8)
                    | data[pos + 3] as u32
            };
            pos += sid_len;

            let ext_flag = data[pos] & 0x80 != 0;
            let scids = data[pos] & 0x0F;
            pos += 1;
            if pos >= data.len() {
                break;
            }

            let ls_flag = data[pos] & 0x80 != 0;
            let binding = if ls_flag {
                // Long form: Rfa(3) | SCId(12) spanning this byte and the next.
                if pos + 2 > data.len() {
                    break;
                }
                let scid = (((data[pos] as u16) & 0x0F) << 8) | data[pos + 1] as u16;
                pos += 2;
                ScidsBinding::Packet(scid)
            } else {
                let subch = data[pos] & 0x3F;
                pos += 1;
                ScidsBinding::Subchannel(subch)
            };
            if ext_flag {
                if pos >= data.len() {
                    break;
                }
                pos += 1;
            }
            self.scids_bindings.insert((sid, scids), binding);
        }
    }

    /// FIG 0/13: User Application Information (ETSI EN 300 401 §6.3.6).
    ///
    /// SId (16 or 32 bit per P/D) + SCIdS (4) + count (4), then `count` entries
    /// of `{UAtype: 11 bits, length: 5 bits, data: N bytes}`.
    fn parse_fig0_ext13(&mut self, data: &[u8], pd: u8) {
        let mut pos = 0;
        while pos < data.len() {
            let sid_len = if pd == 0 { 2 } else { 4 };
            if pos + sid_len + 1 > data.len() {
                break;
            }
            let sid = if pd == 0 {
                ((data[pos] as u32) << 8) | data[pos + 1] as u32
            } else {
                ((data[pos] as u32) << 24)
                    | ((data[pos + 1] as u32) << 16)
                    | ((data[pos + 2] as u32) << 8)
                    | data[pos + 3] as u32
            };
            pos += sid_len;
            let scids = data[pos] >> 4;
            let count = data[pos] & 0x0F;
            pos += 1;

            let mut apps = Vec::new();
            for _ in 0..count {
                if pos + 2 > data.len() {
                    break;
                }
                let ua_type = ((data[pos] as u16) << 3) | (data[pos + 1] as u16 >> 5);
                let ua_len = (data[pos + 1] & 0x1F) as usize;
                pos += 2;
                if pos + ua_len > data.len() {
                    break;
                }
                apps.push(UserApplication {
                    ua_type,
                    data: data[pos..pos + ua_len].to_vec(),
                });
                pos += ua_len;
            }
            self.user_applications.insert((sid, scids), apps);
        }
    }

    /// FIG 0/18: Announcement support (ETSI EN 300 401 §8.1.6.1).
    ///
    /// Per service: SId (16/32 via P/D) + ASu flags (16) + Rfa (3) +
    /// Number of Clusters (5) + that many Cluster Id bytes.
    fn parse_fig0_ext18(&mut self, data: &[u8], pd: u8) {
        let mut pos = 0;
        while pos < data.len() {
            let sid_len = if pd == 0 { 2 } else { 4 };
            if pos + sid_len + 3 > data.len() {
                break;
            }
            let sid = if pd == 0 {
                ((data[pos] as u32) << 8) | data[pos + 1] as u32
            } else {
                ((data[pos] as u32) << 24)
                    | ((data[pos + 1] as u32) << 16)
                    | ((data[pos + 2] as u32) << 8)
                    | data[pos + 3] as u32
            };
            pos += sid_len;
            let asu_flags = ((data[pos] as u16) << 8) | data[pos + 1] as u16;
            pos += 2;
            // Rfa (3) | Number of Clusters (5) — matches EN 300 401 / welle.io.
            let n_clusters = (data[pos] & 0x1F) as usize;
            pos += 1;
            if pos + n_clusters > data.len() {
                break;
            }
            let mut cluster_ids = Vec::with_capacity(n_clusters);
            for &cid in &data[pos..pos + n_clusters] {
                if cid != 0 {
                    cluster_ids.push(cid);
                }
            }
            pos += n_clusters;
            self.announcement_support.insert(
                sid,
                AnnouncementSupport {
                    service_id: sid,
                    asu_flags,
                    cluster_ids,
                },
            );
        }
    }

    /// FIG 0/19: Announcement switching (ETSI EN 300 401 §8.1.6.2).
    ///
    /// Cluster Id (8) + ASw flags (16) + New (1) + Region flag (1) +
    /// SubChId (6), plus an optional RegionId byte when Region flag is set.
    fn parse_fig0_ext19(&mut self, data: &[u8]) {
        let mut pos = 0;
        while pos + 4 <= data.len() {
            let cluster_id = data[pos];
            let asw_flags = ((data[pos + 1] as u16) << 8) | data[pos + 2] as u16;
            let new_flag = data[pos + 3] & 0x80 != 0;
            let region_flag = data[pos + 3] & 0x40 != 0;
            let subchannel_id = data[pos + 3] & 0x3F;
            pos += 4;
            let region_id = if region_flag {
                if pos >= data.len() {
                    break;
                }
                // EN 300 401 / welle.io: Rfa(2) then RegionId(6) in the extension byte.
                let rid = data[pos] & 0x3F;
                pos += 1;
                Some(rid)
            } else {
                None
            };
            self.announcement_switching.insert(
                cluster_id,
                AnnouncementSwitch {
                    cluster_id,
                    asw_flags,
                    new_flag,
                    region_flag,
                    subchannel_id,
                    region_id,
                },
            );
        }
    }

    /// Services that FIG 0/18 lists as members of `cluster_id`.
    pub fn services_in_announcement_cluster(&self, cluster_id: u8) -> Vec<u32> {
        let mut out: Vec<u32> = self
            .announcement_support
            .values()
            .filter(|s| s.cluster_ids.contains(&cluster_id))
            .map(|s| s.service_id)
            .collect();
        out.sort_unstable();
        out
    }

    /// Parse FIG type 1 (Labels).
    fn parse_fig_type1(&mut self, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        let charset = (data[0] >> 4) & 0x0F;
        let _oe = (data[0] >> 3) & 1;
        let extension = data[0] & 0x07;

        let fig_data = &data[1..];

        match extension {
            0 => self.parse_fig1_ext0(fig_data, charset),
            1 => self.parse_fig1_ext1(fig_data, charset),
            5 => self.parse_fig1_ext5(fig_data, charset),
            _ => {}
        }
    }

    /// FIG 1/0: Ensemble label.
    fn parse_fig1_ext0(&mut self, data: &[u8], charset: u8) {
        if data.len() < 18 {
            return; // 2 bytes EId + 16 chars + ... (may have char flag)
        }
        // Skip 2-byte Ensemble ID
        let label_bytes = &data[2..18];
        self.ensemble_label = Some(decode_label(label_bytes, charset));
    }

    /// FIG 1/1: Service label (16-bit SId).
    fn parse_fig1_ext1(&mut self, data: &[u8], charset: u8) {
        if data.len() < 18 {
            return; // 2 bytes SId + 16 chars
        }
        let sid = ((data[0] as u32) << 8) | data[1] as u32;
        let label_bytes = &data[2..18];
        let label = decode_label(label_bytes, charset);

        let service = self.services.entry(sid).or_insert_with(|| ServiceInfo {
            service_id: sid,
            ..Default::default()
        });
        service.label = Some(label);
    }

    /// FIG 1/5: Data service label (32-bit SId).
    fn parse_fig1_ext5(&mut self, data: &[u8], charset: u8) {
        if data.len() < 20 {
            return;
        }
        let sid = ((data[0] as u32) << 24)
            | ((data[1] as u32) << 16)
            | ((data[2] as u32) << 8)
            | data[3] as u32;
        let label_bytes = &data[4..20];
        let label = decode_label(label_bytes, charset);

        let service = self.services.entry(sid).or_insert_with(|| ServiceInfo {
            service_id: sid,
            ..Default::default()
        });
        service.label = Some(label);
    }

    /// Resolve subchannel info (bitrate, protection) into services, and
    /// attach FIG 0/3 / 0/8 / 0/13 packet-mode metadata onto components.
    pub fn resolve_services(&mut self) {
        let subchannels = self.subchannels.clone();
        let packet_components = self.packet_components.clone();
        let scids_bindings = self.scids_bindings.clone();
        let user_applications = self.user_applications.clone();

        for service in self.services.values_mut() {
            if let Some(sc_id) = service.subchannel_id
                && let Some(subch) = subchannels.get(&sc_id)
            {
                service.bitrate = Some(subch.bitrate);
                service.protection = Some(if subch.is_eep {
                    let option_letter = if subch.eep_option == 0 { "A" } else { "B" };
                    format!("EEP {}-{}", subch.protection_level + 1, option_letter)
                } else {
                    format!("UEP {}", subch.protection_level)
                });
            }

            for component in &mut service.components {
                if component.tmid == 3
                    && let Some(scid) = component.scid
                    && let Some(pkt) = packet_components.get(&scid)
                {
                    component.subchannel_id = Some(pkt.subchannel_id);
                    component.packet_address = Some(pkt.packet_address);
                    component.dscty = Some(pkt.dscty);
                }
            }

            for ((sid, scids), binding) in &scids_bindings {
                if *sid != service.service_id {
                    continue;
                }
                match binding {
                    ScidsBinding::Packet(scid) => {
                        if let Some(comp) = service
                            .components
                            .iter_mut()
                            .find(|c| c.tmid == 3 && c.scid == Some(*scid))
                        {
                            comp.scids = Some(*scids);
                            if let Some(apps) = user_applications.get(&(*sid, *scids)) {
                                comp.user_applications = apps.clone();
                            }
                        }
                    }
                    ScidsBinding::Subchannel(subch) => {
                        if let Some(comp) = service
                            .components
                            .iter_mut()
                            .find(|c| c.subchannel_id == Some(*subch))
                        {
                            comp.scids = Some(*scids);
                            if let Some(apps) = user_applications.get(&(*sid, *scids)) {
                                comp.user_applications = apps.clone();
                            }
                        }
                    }
                }
            }

            // Primary packet component of a data service uses SCIdS 0 when
            // FIG 0/8 is absent; still attach FIG 0/13 UAtypes keyed by SCIdS 0.
            if let Some(apps) = user_applications.get(&(service.service_id, 0)) {
                for component in &mut service.components {
                    if component.user_applications.is_empty()
                        && (component.scids.is_none() || component.scids == Some(0))
                    {
                        component.scids = Some(0);
                        component.user_applications = apps.clone();
                    }
                }
            }
        }
    }

    /// Packet-mode components with a resolved SubChId and packet address.
    pub fn packet_mode_targets(&self) -> Vec<TpegTarget> {
        let mut out = Vec::new();
        for service in self.services.values() {
            for component in &service.components {
                if component.tmid != 3 {
                    continue;
                }
                let Some(subch_id) = component.subchannel_id else {
                    continue;
                };
                let Some(subch) = self.subchannels.get(&subch_id) else {
                    continue;
                };
                let Some(address) = component.packet_address else {
                    continue;
                };
                out.push(TpegTarget {
                    service_id: service.service_id,
                    label: service.label.clone(),
                    subchannel: subch.clone(),
                    packet_address: address,
                    ca_flag: component.ca_flag,
                    ca_org: component
                        .scid
                        .and_then(|id| self.packet_components.get(&id))
                        .and_then(|p| p.ca_org),
                    dscty: component.dscty,
                    no_data_groups: component
                        .scid
                        .and_then(|id| self.packet_components.get(&id))
                        .map(|p| p.no_data_groups)
                        .unwrap_or(false),
                    ua_types: component
                        .user_applications
                        .iter()
                        .map(|ua| ua.ua_type)
                        .collect(),
                });
            }
        }
        out
    }

    /// Packet-mode components that FIG 0/13 identifies as TPEG (UAtype 0x004).
    pub fn tpeg_targets(&self) -> Vec<TpegTarget> {
        self.packet_mode_targets()
            .into_iter()
            .filter(|t| t.is_tpeg())
            .collect()
    }

    /// Convert to JSON output format.
    pub fn to_output(&self) -> EnsembleOutput {
        let mut services: Vec<ServiceOutput> = self
            .services
            .values()
            .map(|s| {
                let sid = if s.service_id > 0xFFFF {
                    format!("0x{:08X}", s.service_id)
                } else {
                    format!("0x{:04X}", s.service_id)
                };
                ServiceOutput {
                    service_id: sid,
                    label: s.label.clone(),
                    subchannel_id: s.subchannel_id,
                    bitrate: s.bitrate,
                    protection: s.protection.clone(),
                    components: s
                        .components
                        .iter()
                        .map(|c| ComponentOutput {
                            tmid: c.tmid,
                            subchannel_id: c.subchannel_id,
                            scid: c.scid,
                            packet_address: c.packet_address,
                            dscty: c.dscty,
                            ca: c.ca_flag,
                            user_applications: c
                                .user_applications
                                .iter()
                                .map(|ua| format!("{} (0x{:03X})", ua.name(), ua.ua_type))
                                .collect(),
                        })
                        .collect(),
                }
            })
            .collect();
        services.sort_by(|a, b| a.service_id.cmp(&b.service_id));

        EnsembleOutput {
            ensemble_id: self.ensemble_id,
            ensemble_label: self.ensemble_label.clone(),
            services,
        }
    }

    /// Check if we have at least one service with a known subchannel.
    pub fn has_services(&self) -> bool {
        !self.services.is_empty() && self.services.values().any(|s| s.subchannel_id.is_some())
    }

    /// Check if the ensemble info looks complete:
    /// - ensemble label is known
    /// - all services that have a subchannel also have a label and resolved bitrate
    pub fn is_complete(&self) -> bool {
        if self.ensemble_label.is_none() {
            return false;
        }
        if self.services.is_empty() {
            return false;
        }
        // Every service with a subchannel must have a label and bitrate
        self.services.values().all(|s| {
            if let Some(sid) = s.subchannel_id {
                s.label.is_some() && self.subchannels.contains_key(&sid)
            } else {
                // Services without subchannel (data services) — just need a label
                s.label.is_some()
            }
        })
    }
}

/// Decode a 16-byte label field to a UTF-8 string.
fn decode_label(bytes: &[u8], charset: u8) -> String {
    let bytes = trim_label_padding(bytes);
    match charset {
        0 => charsets::ebu_latin_to_utf8(bytes).trim_end().to_string(),
        // charset 6 = UTF-8
        6 => String::from_utf8_lossy(bytes).trim_end().to_string(),
        // Others: fallback to lossy UTF-8
        _ => String::from_utf8_lossy(bytes).trim_end().to_string(),
    }
}

fn trim_label_padding(bytes: &[u8]) -> &[u8] {
    let mut end = bytes.len();
    while end > 0 && matches!(bytes[end - 1], 0x00 | 0x20 | 0xFF) {
        end -= 1;
    }
    &bytes[..end]
}

/// UEP (Unequal Error Protection) table lookup.
/// Returns (sub_size in CUs, bitrate in kbps, protection_level).
/// Reference: ETSI EN 300 401 Table 6.
fn uep_table(index: u8) -> (u16, u16, u8) {
    // Complete ETSI EN 300 401 FIG 0/1 short-form table. UEP level 1
    // is strongest and level 5 weakest. Keep all 64 rows: omitting rows
    // shifts every later on-air index and selects the wrong bitrate/FEC profile.
    const TABLE: [(u16, u16, u8); 64] = [
        (16, 32, 5),
        (21, 32, 4),
        (24, 32, 3),
        (29, 32, 2),
        (35, 32, 1),
        (24, 48, 5),
        (29, 48, 4),
        (35, 48, 3),
        (42, 48, 2),
        (52, 48, 1),
        (29, 56, 5),
        (35, 56, 4),
        (42, 56, 3),
        (52, 56, 2),
        (32, 64, 5),
        (42, 64, 4),
        (48, 64, 3),
        (58, 64, 2),
        (70, 64, 1),
        (40, 80, 5),
        (52, 80, 4),
        (58, 80, 3),
        (70, 80, 2),
        (84, 80, 1),
        (48, 96, 5),
        (58, 96, 4),
        (70, 96, 3),
        (84, 96, 2),
        (104, 96, 1),
        (58, 112, 5),
        (70, 112, 4),
        (84, 112, 3),
        (104, 112, 2),
        (64, 128, 5),
        (84, 128, 4),
        (96, 128, 3),
        (116, 128, 2),
        (140, 128, 1),
        (80, 160, 5),
        (104, 160, 4),
        (116, 160, 3),
        (140, 160, 2),
        (168, 160, 1),
        (96, 192, 5),
        (116, 192, 4),
        (140, 192, 3),
        (168, 192, 2),
        (208, 192, 1),
        (116, 224, 5),
        (140, 224, 4),
        (168, 224, 3),
        (208, 224, 2),
        (232, 224, 1),
        (128, 256, 5),
        (168, 256, 4),
        (192, 256, 3),
        (232, 256, 2),
        (280, 256, 1),
        (160, 320, 5),
        (208, 320, 4),
        (280, 320, 2),
        (192, 384, 5),
        (280, 384, 3),
        (416, 384, 1),
    ];
    TABLE[index as usize]
}

/// Compute EEP bitrate from sub-channel size, option, and protection level.
fn eep_bitrate(sub_size: u16, option: u8, protection_level: u8) -> u16 {
    // EEP: bitrate depends on sub_size and code rate
    // Option A: code rates from table
    // Option B: different code rates
    let n = sub_size as u32;
    let bitrate = match option {
        0 => match protection_level {
            // EEP-A
            0 => n * 8 / 12, // 1-A
            1 => n * 8 / 8,  // 2-A
            2 => n * 8 / 6,  // 3-A
            3 => n * 8 / 4,  // 4-A
            _ => 0,
        },
        1 => match protection_level {
            // EEP-B
            0 => n * 32 / 27, // 1-B
            1 => n * 32 / 21, // 2-B
            2 => n * 32 / 18, // 3-B
            3 => n * 32 / 15, // 4-B
            _ => 0,
        },
        _ => 0,
    };
    bitrate as u16
}

#[cfg(test)]
mod tests {
    use super::{
        AnnouncementMonitor, AudioCoding, EnsembleInfo, ScidsBinding, SubchannelInfo,
        UserApplication, decode_label, uep_table,
    };
    use crate::fec::uep::{profile, punctured_size};

    #[test]
    fn decode_label_trims_nul_padding_before_ebu_conversion() {
        let label = decode_label(b"BORDEAUX 8C\0\0\0\0\0", 0);
        assert_eq!(label, "BORDEAUX 8C");
    }

    #[test]
    fn decode_label_trims_ff_padding_before_ebu_conversion() {
        let label = decode_label(b"NOVA\xFF\xFF\xFF\xFF", 0);
        assert_eq!(label, "NOVA");
    }

    #[test]
    fn decode_label_keeps_internal_reserved_bytes_visible() {
        let label = decode_label(b"A\0B\0\0", 0);
        assert_eq!(label, "A�B");
    }

    #[test]
    fn complete_uep_table_keeps_profiles_and_allocations_aligned() {
        assert_eq!(uep_table(35), (96, 128, 3));
        assert_eq!(uep_table(63), (416, 384, 1));
        for index in 0..64 {
            let (capacity_units, bitrate, level) = uep_table(index);
            let coding = profile(bitrate, level).unwrap();
            let padding = capacity_units as usize * 64 - punctured_size(&coding);
            assert!(matches!(padding, 0 | 4 | 8), "table index {index}");
        }
    }

    #[test]
    fn audio_component_type_selects_codec() {
        assert_eq!(AudioCoding::from_ascty(0), AudioCoding::MpegLayer2);
        assert_eq!(AudioCoding::from_ascty(63), AudioCoding::DabPlus);
        assert_eq!(AudioCoding::from_ascty(7), AudioCoding::Other(7));
    }

    #[test]
    fn short_form_rejects_reserved_table_switch() {
        let mut decoder = EnsembleInfo::new();
        decoder.parse_fig0_ext1(&[0, 0, 0x40 | 35]);
        assert!(decoder.subchannels.is_empty());

        decoder.parse_fig0_ext1(&[0, 0, 35]);
        let subchannel = decoder.subchannels.get(&0).unwrap();
        assert_eq!(subchannel.bitrate, 128);
        assert_eq!(subchannel.protection_level, 3);
    }

    #[test]
    fn data_component_does_not_clear_primary_audio_codec() {
        let mut decoder = EnsembleInfo::new();
        // Service 0x1234, two components: primary ASCTy 0 on subchannel 1,
        // followed by a packet-data component.
        decoder.parse_fig0_ext2(&[0x12, 0x34, 2, 0, (1 << 2) | 2, 3 << 6, 0], 0);
        let service = decoder.services.get(&0x1234).unwrap();
        assert!(service.is_audio);
        assert_eq!(service.subchannel_id, Some(1));
        assert_eq!(service.audio_coding, Some(AudioCoding::MpegLayer2));
    }

    #[test]
    fn primary_audio_component_supersedes_secondary() {
        let mut decoder = EnsembleInfo::new();
        // Secondary MP2 followed by primary DAB+.
        decoder.parse_fig0_ext2(&[0x12, 0x34, 2, 0, 1 << 2, 63, (2 << 2) | 2], 0);
        let service = decoder.services.get(&0x1234).unwrap();
        assert_eq!(service.subchannel_id, Some(2));
        assert_eq!(service.audio_coding, Some(AudioCoding::DabPlus));
    }

    fn pack_fig0_3(
        scid: u16,
        ca_org: Option<u16>,
        dg: bool,
        dscty: u8,
        subch: u8,
        addr: u16,
    ) -> Vec<u8> {
        let mut bytes = vec![0u8; if ca_org.is_some() { 7 } else { 5 }];
        bytes[0] = (scid >> 4) as u8;
        bytes[1] = ((scid & 0x0F) << 4) as u8;
        if ca_org.is_some() {
            bytes[1] |= 0x01;
        }
        bytes[2] = dscty & 0x3F;
        if dg {
            bytes[2] |= 0x80;
        }
        bytes[3] = (subch << 2) | ((addr >> 8) as u8 & 0x03);
        bytes[4] = (addr & 0xFF) as u8;
        if let Some(org) = ca_org {
            bytes[5] = (org >> 8) as u8;
            bytes[6] = (org & 0xFF) as u8;
        }
        bytes
    }

    #[test]
    fn fig0_3_without_caorg_is_five_bytes() {
        let mut decoder = EnsembleInfo::new();
        decoder.parse_fig0_ext3(&pack_fig0_3(0xABC, None, false, 5, 12, 852));
        let pkt = decoder.packet_components.get(&0xABC).unwrap();
        assert_eq!(pkt.subchannel_id, 12);
        assert_eq!(pkt.packet_address, 852);
        assert_eq!(pkt.dscty, 5);
        assert!(!pkt.no_data_groups);
        assert!(pkt.ca_org.is_none());
    }

    #[test]
    fn fig0_3_with_caorg_is_seven_bytes_and_shifts_next_entry() {
        let mut decoder = EnsembleInfo::new();
        let mut bytes = pack_fig0_3(0x001, Some(0x1234), false, 5, 3, 10);
        bytes.extend_from_slice(&pack_fig0_3(0x002, None, true, 60, 4, 20));
        decoder.parse_fig0_ext3(&bytes);
        let a = decoder.packet_components.get(&0x001).unwrap();
        assert_eq!(a.ca_org, Some(0x1234));
        assert_eq!(a.packet_address, 10);
        let b = decoder.packet_components.get(&0x002).unwrap();
        assert_eq!(b.packet_address, 20);
        assert!(b.no_data_groups);
        assert_eq!(b.dscty, 60);
    }

    #[test]
    fn fig0_2_packet_component_dedupes_by_scid_across_resolution() {
        let mut decoder = EnsembleInfo::new();
        let scid: u16 = 0xABC;
        let b0 = (3 << 6) | ((scid >> 6) as u8);
        let b1 = ((scid as u8) << 2) | 0x02;
        // First FIG 0/2 cycle: SCId known, SubChId not yet resolved.
        decoder.parse_fig0_ext2(&[0xF2, 0x01, 1, b0, b1], 0);
        assert_eq!(decoder.services.get(&0xF201).unwrap().components.len(), 1);
        assert!(
            decoder.services.get(&0xF201).unwrap().components[0]
                .subchannel_id
                .is_none()
        );

        // FIG 0/3 arrives and resolves the SCId.
        decoder.parse_fig0_ext3(&pack_fig0_3(scid, None, false, 5, 12, 852));
        // Second FIG 0/2 cycle now carries a resolved SubChId — must update the
        // existing component, not insert a duplicate keyed on subchannel_id.
        decoder.parse_fig0_ext2(&[0xF2, 0x01, 1, b0, b1], 0);
        let service = decoder.services.get(&0xF201).unwrap();
        assert_eq!(service.components.len(), 1);
        assert_eq!(service.components[0].scid, Some(scid));
        assert_eq!(service.components[0].subchannel_id, Some(12));
        assert_eq!(service.components[0].packet_address, Some(852));
    }

    #[test]
    fn late_fig0_13_surfaces_tpeg_target_after_earlier_completeness() {
        let mut decoder = EnsembleInfo::new();
        decoder.subchannels.insert(
            12,
            SubchannelInfo {
                id: 12,
                start_addr: 84,
                sub_size: 6,
                protection_level: 2,
                is_eep: true,
                eep_option: 0,
                uep_table_index: None,
                bitrate: 8,
            },
        );
        decoder.parse_fig0_ext3(&pack_fig0_3(0x001, None, false, 5, 12, 852));
        let b0 = (3 << 6) | ((0x001u16 >> 6) as u8);
        let b1 = ((0x001u16 as u8) << 2) | 0x02;
        decoder.parse_fig0_ext2(&[0xF2, 0x01, 1, b0, b1], 0);
        decoder.ensemble_id = Some(0x1234);
        decoder.ensemble_label = Some("TestEns".into());
        decoder.services.get_mut(&0xF201).unwrap().label = Some("Pkt".into());
        decoder.resolve_services();
        // Ensemble looks complete for audio/label purposes, but FIG 0/13 has not
        // arrived yet — so there is no TPEG target.
        assert!(decoder.is_complete() || decoder.has_services());
        assert!(decoder.tpeg_targets().is_empty());
        assert_eq!(decoder.packet_mode_targets().len(), 1);

        // Late FIG 0/13: must become visible without resetting prior state.
        let ua_type: u16 = 0x004;
        decoder.parse_fig0_ext13(
            &[
                0xF2,
                0x01,
                0x01,
                (ua_type >> 3) as u8,
                ((ua_type as u8) << 5) & 0xE0,
            ],
            0,
        );
        decoder.resolve_services();
        let targets = decoder.tpeg_targets();
        assert_eq!(targets.len(), 1);
        assert!(targets[0].is_tpeg());
        assert_eq!(targets[0].packet_address, 852);
    }

    #[test]
    fn fig0_2_tmid3_extracts_scid() {
        let mut decoder = EnsembleInfo::new();
        // SId 0xF201, 1 component, TMId=3, SCId=0xABC, PS=1, CA=0
        let scid: u16 = 0xABC;
        let b0 = (3 << 6) | ((scid >> 6) as u8);
        let b1 = ((scid as u8) << 2) | 0x02;
        decoder.parse_fig0_ext2(&[0xF2, 0x01, 1, b0, b1], 0);
        let service = decoder.services.get(&0xF201).unwrap();
        assert_eq!(service.components.len(), 1);
        assert_eq!(service.components[0].tmid, 3);
        assert_eq!(service.components[0].scid, Some(0xABC));
        assert!(service.components[0].ps_flag);
        assert!(!service.components[0].ca_flag);
    }

    #[test]
    fn fig0_13_parses_tpeg_uatype() {
        let mut decoder = EnsembleInfo::new();
        // SId 0xF201, SCIdS=0, count=1, UAtype=0x004, length=0
        let ua_type: u16 = 0x004;
        let b0 = (ua_type >> 3) as u8;
        let b1 = ((ua_type as u8) << 5) & 0xE0; // length 0 in low 5 bits
        decoder.parse_fig0_ext13(&[0xF2, 0x01, 0x01, b0, b1], 0);
        let apps = decoder.user_applications.get(&(0xF201, 0)).unwrap();
        assert_eq!(apps.len(), 1);
        assert_eq!(apps[0].ua_type, UserApplication::TPEG);
    }

    #[test]
    fn fig0_8_long_form_binds_scids_to_scid() {
        let mut decoder = EnsembleInfo::new();
        // 16-bit SId, Ext=0, SCIdS=1, L/S=1, SCId=0xABC
        decoder.parse_fig0_ext8(&[0xF2, 0x01, 0x01, 0x8A, 0xBC], 0);
        assert_eq!(
            decoder.scids_bindings.get(&(0xF201, 1)),
            Some(&ScidsBinding::Packet(0xABC))
        );
    }

    #[test]
    fn tpeg_target_resolves_via_fig0_2_3_13() {
        let mut decoder = EnsembleInfo::new();
        decoder.parse_fig0_ext1(&[0x30, 0x54, 0x80, 0x06]); // subch 12, start 0x054, EEP long, size 6
        // Force a known 8 kbps EEP-A-3 row: option=0, prot=2, size=6
        // Long form byte2: form=1, option=000, prot=10, size high=00 -> 0x80 | (2<<2) = 0x88, size=6
        decoder.subchannels.insert(
            12,
            SubchannelInfo {
                id: 12,
                start_addr: 84,
                sub_size: 6,
                protection_level: 2,
                is_eep: true,
                eep_option: 0,
                uep_table_index: None,
                bitrate: 8,
            },
        );
        decoder.parse_fig0_ext3(&pack_fig0_3(0x001, None, false, 5, 12, 852));
        let b0 = (3 << 6) | ((0x001u16 >> 6) as u8);
        let b1 = ((0x001u16 as u8) << 2) | 0x02;
        decoder.parse_fig0_ext2(&[0xF2, 0x01, 1, b0, b1], 0);
        let ua_type: u16 = 0x004;
        decoder.parse_fig0_ext13(
            &[
                0xF2,
                0x01,
                0x01,
                (ua_type >> 3) as u8,
                ((ua_type as u8) << 5) & 0xE0,
            ],
            0,
        );
        decoder.resolve_services();
        let targets = decoder.tpeg_targets();
        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].packet_address, 852);
        assert_eq!(targets[0].subchannel.id, 12);
        assert!(!targets[0].ca_flag);
        assert!(targets[0].is_tpeg());
        assert_eq!(decoder.packet_mode_targets().len(), 1);
    }

    #[test]
    fn epg_packet_component_is_not_a_tpeg_target() {
        let mut decoder = EnsembleInfo::new();
        decoder.subchannels.insert(
            50,
            SubchannelInfo {
                id: 50,
                start_addr: 852,
                sub_size: 6,
                protection_level: 2,
                is_eep: true,
                eep_option: 0,
                uep_table_index: None,
                bitrate: 8,
            },
        );
        decoder.parse_fig0_ext3(&pack_fig0_3(0, None, false, 60, 50, 1));
        let b0 = 3 << 6;
        let b1 = 0x02;
        decoder.parse_fig0_ext2(&[0xE2, 0xF2, 0xA2, 0x01, 1, b0, b1], 1);
        decoder.parse_fig0_ext13(&[0xE2, 0xF2, 0xA2, 0x01, 0x01, 0x00, 0xE0], 1);
        decoder.resolve_services();
        assert_eq!(decoder.packet_mode_targets().len(), 1);
        assert_eq!(decoder.packet_mode_targets()[0].packet_address, 1);
        assert!(decoder.tpeg_targets().is_empty());
    }

    #[test]
    fn fig0_18_parses_asu_and_clusters() {
        let mut decoder = EnsembleInfo::new();
        // SId 0xF801, ASu = Road Traffic|Transport = 0x0006, 2 clusters: 1, 2
        decoder.parse_fig0_ext18(&[0xF8, 0x01, 0x00, 0x06, 0x02, 0x01, 0x02], 0);
        let s = decoder.announcement_support.get(&0xF801).unwrap();
        assert_eq!(s.asu_flags, 0x0006);
        assert_eq!(s.cluster_ids, vec![1, 2]);
        assert!(traffic::is_traffic_relevant(s.asu_flags));
    }

    #[test]
    fn fig0_19_parses_switch_without_region() {
        let mut decoder = EnsembleInfo::new();
        // Cluster 1, ASw Road Traffic 0x0002, New=1, Region=0, SubCh=50
        decoder.parse_fig0_ext19(&[0x01, 0x00, 0x02, 0x80 | 50]);
        let sw = decoder.announcement_switching.get(&1).unwrap();
        assert_eq!(sw.asw_flags, 0x0002);
        assert!(sw.new_flag);
        assert!(!sw.region_flag);
        assert_eq!(sw.subchannel_id, 50);
        assert!(sw.region_id.is_none());
    }

    #[test]
    fn fig0_19_with_region_advances_five_bytes() {
        let mut decoder = EnsembleInfo::new();
        // Two entries: first with region, second without — packing must not slip.
        let mut bytes = vec![0x01, 0x00, 0x02, 0xC0 | 50, 0x0A]; // region_id=10
        bytes.extend_from_slice(&[0x02, 0x00, 0x04, 40]); // cluster 2, transport, subch 40
        decoder.parse_fig0_ext19(&bytes);
        assert_eq!(
            decoder.announcement_switching.get(&1).unwrap().region_id,
            Some(10)
        );
        assert_eq!(
            decoder
                .announcement_switching
                .get(&2)
                .unwrap()
                .subchannel_id,
            40
        );
        assert_eq!(
            decoder.announcement_switching.get(&2).unwrap().asw_flags,
            0x0004
        );
    }

    #[test]
    fn announcement_monitor_emits_start_and_end() {
        let mut enc = EnsembleInfo::new();
        enc.parse_fig0_ext18(&[0xF8, 0x01, 0x00, 0x02, 0x01, 0x01], 0);
        let mut mon = AnnouncementMonitor::new();
        let events = mon.observe(&enc, Some(1000));
        assert!(events.is_empty()); // no FIG 0/19 yet

        enc.parse_fig0_ext19(&[0x01, 0x00, 0x02, 0x80 | 50]);
        let events = mon.observe(&enc, Some(2000));
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].phase, traffic::AnnouncementPhase::Started);
        assert_eq!(events[0].cluster_id, 1);
        assert_eq!(events[0].subchannel_id, 50);
        assert!(events[0].traffic_relevant);

        // Same announcement still present, new_flag cleared → continuing (no emit by default)
        enc.announcement_switching.clear();
        enc.parse_fig0_ext19(&[0x01, 0x00, 0x02, 50]);
        let events = mon.observe(&enc, Some(3000));
        assert!(events.is_empty());

        // Cluster disappears → ended
        enc.announcement_switching.clear();
        let events = mon.end_missing(&enc, Some(4000));
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].phase, traffic::AnnouncementPhase::Ended);
    }
}

/// Tracks FIG 0/19 edges so callers can emit start/end announcement events.
#[derive(Debug, Default)]
pub struct AnnouncementMonitor {
    /// Clusters currently considered active: cluster → last switch state.
    active: HashMap<u8, AnnouncementSwitch>,
}

impl AnnouncementMonitor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Observe the latest ensemble switching table. Emits `Started` when a
    /// cluster newly appears or its `new_flag` rises while already tracked.
    pub fn observe(
        &mut self,
        ensemble: &EnsembleInfo,
        timestamp_unix_ms: Option<u64>,
    ) -> Vec<traffic::AnnouncementEvent> {
        let mut events = Vec::new();
        let eid = ensemble.ensemble_id.map(|e| format!("0x{e:04X}"));
        for (cluster, sw) in &ensemble.announcement_switching {
            if sw.asw_flags == 0 {
                continue;
            }
            let supporting: Vec<String> = ensemble
                .services_in_announcement_cluster(*cluster)
                .into_iter()
                .map(|sid| {
                    if sid > 0xFFFF {
                        format!("0x{sid:08X}")
                    } else {
                        format!("0x{sid:04X}")
                    }
                })
                .collect();
            match self.active.get(cluster) {
                None => {
                    events.push(traffic::AnnouncementEvent::started(
                        sw.cluster_id,
                        sw.subchannel_id,
                        sw.asw_flags,
                        sw.new_flag,
                        sw.region_id,
                        supporting,
                        eid.clone(),
                        timestamp_unix_ms,
                    ));
                    self.active.insert(*cluster, sw.clone());
                }
                Some(prev) if sw.new_flag && !prev.new_flag => {
                    events.push(traffic::AnnouncementEvent::started(
                        sw.cluster_id,
                        sw.subchannel_id,
                        sw.asw_flags,
                        true,
                        sw.region_id,
                        supporting,
                        eid.clone(),
                        timestamp_unix_ms,
                    ));
                    self.active.insert(*cluster, sw.clone());
                }
                Some(_) => {
                    self.active.insert(*cluster, sw.clone());
                }
            }
        }
        events
    }

    /// Emit `Ended` for any previously active cluster absent from the current
    /// switching table (or present with ASw == 0). Call once per OFDM frame
    /// after FIBs for that frame have been ingested.
    pub fn end_missing(
        &mut self,
        ensemble: &EnsembleInfo,
        timestamp_unix_ms: Option<u64>,
    ) -> Vec<traffic::AnnouncementEvent> {
        let eid = ensemble.ensemble_id.map(|e| format!("0x{e:04X}"));
        let mut ended = Vec::new();
        let still: std::collections::HashSet<u8> = ensemble
            .announcement_switching
            .iter()
            .filter(|(_, sw)| sw.asw_flags != 0)
            .map(|(c, _)| *c)
            .collect();
        let gone: Vec<u8> = self
            .active
            .keys()
            .copied()
            .filter(|c| !still.contains(c))
            .collect();
        for cluster in gone {
            if let Some(sw) = self.active.remove(&cluster) {
                let supporting: Vec<String> = ensemble
                    .services_in_announcement_cluster(cluster)
                    .into_iter()
                    .map(|sid| {
                        if sid > 0xFFFF {
                            format!("0x{sid:08X}")
                        } else {
                            format!("0x{sid:04X}")
                        }
                    })
                    .collect();
                ended.push(traffic::AnnouncementEvent::ended(
                    sw.cluster_id,
                    sw.subchannel_id,
                    sw.asw_flags,
                    supporting,
                    eid.clone(),
                    timestamp_unix_ms,
                ));
            }
        }
        ended
    }
}
