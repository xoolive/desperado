//! PAD (Programme Associated Data) extraction for DAB+.
//!
//! Implements ETSI EN 300 401 §7.4.5, ETSI TS 102 980 (Dynamic Label Segment),
//! and ETSI EN 301 234 (MOT - Multimedia Object Transfer).
//!
//! In DAB+, PAD is carried inside an AAC Data Stream Element (DSE) within each
//! Access Unit. The structure is:
//!
//! ```text
//! AU: [ DSE header | PAD data (X-PAD reversed + F-PAD) | ... other AAC data ... ]
//!      ^-- element_id=4 (3 bits), data_byte_align_flag, count, [esc_count], data
//! ```
//!
//! F-PAD (Fixed PAD, 2 bytes) is at the end of the PAD region.
//! X-PAD (Extended PAD) is before F-PAD, with bytes in reversed order.
//!
//! Reference: ETSI TS 102 563 §5.4, ISO/IEC 14496-3 (AAC syntax)

mod mot;

use mot::{MotFile, MotManager};
use serde::Serialize;
use std::collections::BTreeMap;
use tracing::{debug, trace};

/// Extracted DLS text.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct DlsMetadata {
    pub text: String,
}

/// Extracted MOT slideshow image.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct MotMetadata {
    /// Image data (JPEG or PNG).
    pub data: Vec<u8>,
    /// Content type (e.g., "image/jpeg", "image/png").
    pub content_type: String,
    /// Optional content name/filename.
    pub content_name: Option<String>,
    /// Optional category title.
    pub category_title: Option<String>,
    /// Optional click-through URL.
    pub click_through_url: Option<String>,
}

impl From<MotFile> for MotMetadata {
    fn from(file: MotFile) -> Self {
        let content_type = match (file.content_type, file.content_sub_type) {
            (0x02, 0x001) => "image/jpeg".to_string(),
            (0x02, 0x003) => "image/png".to_string(),
            (ct, cst) => format!("application/x-mot-{:02x}-{:03x}", ct, cst),
        };
        Self {
            data: file.data,
            content_type,
            content_name: file.content_name,
            category_title: file.category_title,
            click_through_url: file.click_through_url,
        }
    }
}

/// Combined PAD metadata output.
#[derive(Debug, Clone)]
pub enum PadData {
    /// Dynamic Label Segment (text metadata).
    Dls(DlsMetadata),
    /// MOT slideshow image.
    Mot(MotMetadata),
}

/// X-PAD Content Indicator.
#[derive(Debug, Clone, Copy)]
struct XpadCi {
    /// Application type (1-31, 0 = end marker).
    app_type: u8,
    /// Length of data subfield in bytes.
    len: usize,
}

/// X-PAD length table (ETSI EN 300 401, Table 10).
const XPAD_LEN_TABLE: [usize; 8] = [4, 6, 8, 12, 16, 24, 32, 48];

/// Application type for DLS (Dynamic Label Segment).
const XPAD_APP_DGLI: u8 = 1;
const XPAD_APP_DLS_START: u8 = 2;
const XPAD_APP_DLS_CONT: u8 = 3;
/// Application types for MOT (app_type 12 = start, 13 = continuation).
const XPAD_APP_MOT_START: u8 = 12;
const XPAD_APP_MOT_CONT: u8 = 13;

/// DLS segment structure (ETSI TS 102 980).
#[derive(Debug, Clone)]
struct DlsSegment {
    /// 2-byte prefix.
    prefix: [u8; 2],
    /// Character data.
    chars: Vec<u8>,
}

impl DlsSegment {
    /// Toggle flag (changes when label content changes).
    fn toggle(&self) -> bool {
        self.prefix[0] & 0x80 != 0
    }

    /// First segment flag (segment 0).
    fn first(&self) -> bool {
        self.prefix[0] & 0x40 != 0
    }

    /// Last segment flag.
    fn last(&self) -> bool {
        self.prefix[0] & 0x20 != 0
    }

    /// Segment number (0-7).
    fn seg_num(&self) -> u8 {
        if self.first() {
            0
        } else {
            (self.prefix[1] >> 4) & 0x07
        }
    }

    /// Character set (from first segment only).
    fn charset(&self) -> u8 {
        self.prefix[1] >> 4
    }
}

/// CRC-16-CCITT calculator for DLS validation.
struct Crc16Ccitt;

impl Crc16Ccitt {
    const LUT: [u16; 256] = Self::build_lut();

    const fn build_lut() -> [u16; 256] {
        let mut lut = [0u16; 256];
        let mut i = 0;
        while i < 256 {
            let mut crc = (i as u16) << 8;
            let mut j = 0;
            while j < 8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
                j += 1;
            }
            lut[i] = crc;
            i += 1;
        }
        lut
    }

    fn compute(data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &byte in data {
            crc = (crc << 8) ^ Self::LUT[((crc >> 8) as u8 ^ byte) as usize];
        }
        crc ^ 0xFFFF
    }

    fn check(data: &[u8], stored_crc: u16) -> bool {
        Self::compute(data) == stored_crc
    }
}

/// DLS segment reassembler.
///
/// Collects segments by segment number and emits complete labels
/// when all segments (0 through last) are received.
#[derive(Debug, Default)]
struct DlsReassembler {
    /// Segments collected by segment number.
    segments: BTreeMap<u8, DlsSegment>,
    /// Current toggle value (to detect label changes).
    current_toggle: Option<bool>,
}

impl DlsReassembler {
    #[allow(dead_code)]
    fn new() -> Self {
        Self::default()
    }

    /// Add a segment and return the complete label if all segments received.
    fn add_segment(&mut self, seg: DlsSegment) -> Option<String> {
        let toggle = seg.toggle();
        let seg_num = seg.seg_num();
        let is_last = seg.last();

        // If toggle changed, clear all cached segments
        if self.current_toggle != Some(toggle) {
            trace!(
                toggle,
                "DLS toggle changed, clearing {} cached segments",
                self.segments.len()
            );
            self.segments.clear();
            self.current_toggle = Some(toggle);
        }

        // Don't overwrite existing segment
        if self.segments.contains_key(&seg_num) {
            return None;
        }

        trace!(
            seg_num,
            is_last,
            chars_len = seg.chars.len(),
            "DLS segment received"
        );
        self.segments.insert(seg_num, seg);

        // Check if label is complete
        self.try_assemble()
    }

    /// Try to assemble a complete label from cached segments.
    fn try_assemble(&self) -> Option<String> {
        // Must have segment 0
        if !self.segments.contains_key(&0) {
            return None;
        }

        // Find which segment is marked as last
        let mut last_seg_num: Option<u8> = None;
        for (&num, seg) in &self.segments {
            if seg.last() {
                last_seg_num = Some(num);
                break;
            }
        }
        let last_seg_num = last_seg_num?;

        // Check we have all segments from 0 to last
        for i in 0..=last_seg_num {
            if !self.segments.contains_key(&i) {
                return None;
            }
        }

        // Concatenate character data
        let charset = self.segments.get(&0).map(|s| s.charset()).unwrap_or(0);
        let mut raw_label = Vec::new();
        for i in 0..=last_seg_num {
            if let Some(seg) = self.segments.get(&i) {
                raw_label.extend_from_slice(&seg.chars);
            }
        }

        // Decode character set
        let text = decode_charset(&raw_label, charset);
        debug!(
            label = %text,
            segments = last_seg_num + 1,
            charset,
            "DLS label complete"
        );
        Some(text)
    }

    fn reset(&mut self) {
        self.segments.clear();
        self.current_toggle = None;
    }
}

/// Decode EBU Latin character set to UTF-8 (ETSI TS 101 756).
fn decode_charset(data: &[u8], charset: u8) -> String {
    match charset {
        0 => decode_ebu_latin(data),                     // EBU Latin (default)
        6 => String::from_utf8_lossy(data).into_owned(), // UTF-8
        15 => decode_ebu_latin(data),                    // EBU Latin (explicit)
        _ => {
            // For other charsets, try UTF-8 then fall back to Latin-1
            if let Ok(s) = std::str::from_utf8(data) {
                s.to_string()
            } else {
                data.iter().map(|&b| b as char).collect()
            }
        }
    }
}

/// Decode EBU Latin to UTF-8 (subset mapping).
fn decode_ebu_latin(data: &[u8]) -> String {
    let mut result = String::with_capacity(data.len());
    for &b in data {
        // EBU Latin is mostly compatible with ISO 8859-1/15
        // Map common special characters
        let c = match b {
            0x00..=0x1F => ' ', // Control chars -> space
            0x8D => 'ß',        // German sharp S
            0x8E => '¿',
            0x8F => '¡',
            0x91 => 'æ',
            0x92 => 'Æ',
            0x93 => 'ô',
            0x94 => 'ö',
            0x95 => 'ò',
            0x96 => 'û',
            0x97 => 'ù',
            0x98 => 'ÿ',
            0x99 => 'Ö',
            0x9A => 'Ü',
            0x9C => '£',
            0x9D => '¥',
            0x9F => 'ƒ',
            0xE0 => 'Ω', // Greek omega
            0xE1 => 'á',
            0xE2 => 'í',
            0xE3 => 'ó',
            0xE4 => 'ú',
            0xE5 => 'ñ',
            0xE6 => 'Ñ',
            0xE7 => 'ª',
            0xE8 => 'º',
            0xE9 => '¿',
            0xEA => '⌐',
            0xEB => '¬',
            0xEC => '½',
            0xED => '¼',
            0xEE => '¡',
            0xEF => '«',
            0xF0 => '»',
            0xF1 => 'À',
            0xF2 => 'Â',
            0xF3 => 'Ê',
            0xF4 => 'Ë',
            0xF5 => 'È',
            0xF6 => 'Î',
            0xF7 => 'Ï',
            0xF8 => 'Ì',
            0xF9 => 'Ô',
            0xFA => 'Ò',
            0xFB => 'Û',
            0xFC => 'Ù',
            0xFD => 'ÿ',
            0xFE => '¯',
            0xFF => '\u{00A0}', // Non-breaking space
            _ => b as char,     // Direct mapping for printable ASCII + Latin-1
        };
        result.push(c);
    }
    result.trim().to_string()
}

/// F-PAD length in bytes.
const FPAD_LEN: usize = 2;

/// Maximum DLS data group size (128 chars + 2 prefix + 2 CRC).
const MAX_DLS_DATA_GROUP: usize = 132;

/// DLS Data Group accumulator.
///
/// Accumulates bytes from multiple X-PAD data subfields until a complete
/// DLS data group is received.
#[derive(Debug, Default)]
struct DlsDataGroupAccumulator {
    /// Accumulated bytes.
    buffer: Vec<u8>,
    /// Whether we're currently accumulating (started with app_type 2).
    active: bool,
}

impl DlsDataGroupAccumulator {
    /// Start a new data group (app_type 2 = DLS start).
    fn start(&mut self, data: &[u8]) {
        self.buffer.clear();
        self.buffer.extend_from_slice(data);
        self.active = true;
    }

    /// Continue accumulating (app_type 3 = DLS continuation).
    fn accumulate(&mut self, data: &[u8]) {
        if self.active {
            self.buffer.extend_from_slice(data);
            // Prevent unbounded growth
            if self.buffer.len() > MAX_DLS_DATA_GROUP {
                self.buffer.clear();
                self.active = false;
            }
        }
    }

    /// Try to extract a complete DLS segment from the accumulated data.
    ///
    /// Returns the segment if complete, None otherwise.
    fn try_extract(&mut self) -> Option<DlsSegment> {
        if !self.active || self.buffer.len() < 4 {
            return None;
        }

        // Check if this is a command (not a segment)
        let is_command = self.buffer[0] & 0x10 != 0;
        if is_command {
            // Commands are 2 bytes + CRC
            if self.buffer.len() >= 4 {
                let cmd = self.buffer[0] & 0x0F;
                // Validate CRC
                let crc_stored = (self.buffer[2] as u16) << 8 | self.buffer[3] as u16;
                if Crc16Ccitt::check(&self.buffer[..2], crc_stored) {
                    trace!(cmd, "DLS command received");
                    self.buffer.clear();
                    self.active = false;
                    // Return None - commands handled separately
                }
            }
            return None;
        }

        // Field length from lower 4 bits of prefix byte 0
        let field_len = (self.buffer[0] & 0x0F) as usize + 1;
        let total_len = 2 + field_len + 2; // prefix + chars + CRC

        if self.buffer.len() < total_len {
            trace!(
                have = self.buffer.len(),
                need = total_len,
                "DLS data group incomplete"
            );
            return None;
        }

        // Verify CRC
        let crc_stored =
            (self.buffer[total_len - 2] as u16) << 8 | self.buffer[total_len - 1] as u16;
        let crc_data = &self.buffer[..total_len - 2];
        if !Crc16Ccitt::check(crc_data, crc_stored) {
            trace!(
                stored = format!("{:04X}", crc_stored),
                computed = format!("{:04X}", Crc16Ccitt::compute(crc_data)),
                "DLS data group CRC mismatch"
            );
            // Bad CRC - reset accumulator
            self.buffer.clear();
            self.active = false;
            return None;
        }

        // Build segment
        let seg = DlsSegment {
            prefix: [self.buffer[0], self.buffer[1]],
            chars: self.buffer[2..2 + field_len].to_vec(),
        };

        // Remove consumed bytes, keep any remainder for next segment
        self.buffer.drain(..total_len);
        if self.buffer.is_empty() {
            self.active = false;
        }

        Some(seg)
    }

    /// Check if this is a command and handle it.
    fn check_command(&mut self) -> Option<u8> {
        if !self.active || self.buffer.len() < 4 {
            return None;
        }

        let is_command = self.buffer[0] & 0x10 != 0;
        if !is_command {
            return None;
        }

        let cmd = self.buffer[0] & 0x0F;
        // Commands are just 2 bytes + CRC = 4 bytes
        let crc_stored = (self.buffer[2] as u16) << 8 | self.buffer[3] as u16;
        if Crc16Ccitt::check(&self.buffer[..2], crc_stored) {
            self.buffer.drain(..4);
            if self.buffer.is_empty() {
                self.active = false;
            }
            return Some(cmd);
        }

        None
    }

    fn reset(&mut self) {
        self.buffer.clear();
        self.active = false;
    }
}

/// MOT Data Group accumulator.
///
/// Similar to DLS but for MOT data groups which can be larger.
/// Uses DGLI (Data Group Length Indicator) to know expected size.
#[derive(Debug, Default)]
struct MotDataGroupAccumulator {
    /// Accumulated bytes.
    buffer: Vec<u8>,
    /// Whether we're currently accumulating.
    active: bool,
    /// Expected length from DGLI (0 = unknown).
    expected_len: usize,
}

/// Maximum MOT data group size (64KB should be plenty for slideshows).
const MAX_MOT_DATA_GROUP: usize = 65536;

impl MotDataGroupAccumulator {
    /// Set expected length from DGLI.
    fn set_len(&mut self, len: usize) {
        self.expected_len = len;
    }

    /// Start a new data group.
    fn start(&mut self, data: &[u8]) {
        self.buffer.clear();
        self.buffer.extend_from_slice(data);
        self.active = true;
    }

    /// Continue accumulating.
    fn accumulate(&mut self, data: &[u8]) {
        if self.active {
            self.buffer.extend_from_slice(data);
            // Prevent unbounded growth
            if self.buffer.len() > MAX_MOT_DATA_GROUP {
                self.buffer.clear();
                self.active = false;
            }
        }
    }

    /// Try to extract a complete MOT data group.
    fn try_extract(&mut self) -> Option<Vec<u8>> {
        if !self.active {
            return None;
        }

        // If we have a DGLI length, use it
        let needed = if self.expected_len > 0 {
            self.expected_len
        } else {
            // Without DGLI, we can't know the length
            // This shouldn't happen in practice
            return None;
        };

        if self.buffer.len() < needed {
            return None;
        }

        // Verify CRC (last 2 bytes)
        if needed < 2 {
            return None;
        }

        let crc_stored = (self.buffer[needed - 2] as u16) << 8 | self.buffer[needed - 1] as u16;
        let crc_data = &self.buffer[..needed - 2];
        if !Crc16Ccitt::check(crc_data, crc_stored) {
            trace!("MOT data group CRC mismatch");
            self.buffer.clear();
            self.active = false;
            self.expected_len = 0;
            return None;
        }

        // Extract the data group
        let dg: Vec<u8> = self.buffer[..needed].to_vec();
        self.buffer.drain(..needed);
        if self.buffer.is_empty() {
            self.active = false;
        }
        self.expected_len = 0;

        Some(dg)
    }

    fn reset(&mut self) {
        self.buffer.clear();
        self.active = false;
        self.expected_len = 0;
    }
}

/// PAD extractor for DAB+ Access Units.
#[derive(Debug, Default)]
pub struct PadExtractor {
    /// DLS data group accumulator.
    dls_accumulator: DlsDataGroupAccumulator,
    /// DLS segment reassembler.
    dls_reassembler: DlsReassembler,
    /// Last emitted label (for deduplication).
    last_label: Option<String>,
    /// Last X-PAD CI for continuation handling.
    last_ci: Option<XpadCi>,
    /// MOT data group accumulator.
    mot_accumulator: MotDataGroupAccumulator,
    /// MOT object manager.
    mot_manager: MotManager,
    /// Last DGLI value.
    dgli_len: usize,
}

impl PadExtractor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Find PAD data embedded in an AAC Data Stream Element.
    ///
    /// Returns (X-PAD data with bytes reversed, F-PAD 2 bytes) if found.
    fn find_pad_in_dse(&self, au: &[u8]) -> Option<(Vec<u8>, [u8; 2])> {
        // Minimum: 1 byte header + at least F-PAD (2 bytes)
        if au.len() < 3 {
            return None;
        }

        // Check for Data Stream Element: element_id = 4 (bits 7-5 = 0b100)
        let element_id = au[0] >> 5;
        if element_id != 4 {
            trace!(element_id, "No DSE found (element_id != 4)");
            return None;
        }

        // DSE structure (ISO/IEC 14496-3):
        // - element_instance_tag: 4 bits
        // - data_byte_align_flag: 1 bit
        // - count: 8 bits
        // - if count == 255: esc_count: 8 bits (total = count + esc_count)
        // - data_stream_byte[]: count bytes

        let mut pad_start = 2; // Skip first 2 bytes (element header + count)
        let mut pad_len = au[1] as usize;

        if pad_len == 255 {
            if au.len() < 4 {
                return None;
            }
            pad_len += au[2] as usize;
            pad_start = 3;
        }

        // Need at least F-PAD (2 bytes)
        if pad_len < FPAD_LEN {
            trace!(pad_len, "DSE too short for F-PAD");
            return None;
        }

        if au.len() < pad_start + pad_len {
            trace!(
                au_len = au.len(),
                pad_start, pad_len, "AU too short for announced PAD"
            );
            return None;
        }

        let pad_region = &au[pad_start..pad_start + pad_len];

        // F-PAD is the last 2 bytes of the PAD region
        let fpad_start = pad_len - FPAD_LEN;
        let fpad = [pad_region[fpad_start], pad_region[fpad_start + 1]];

        // X-PAD is before F-PAD, in reversed byte order
        let xpad_raw = &pad_region[..fpad_start];
        let xpad: Vec<u8> = xpad_raw.iter().rev().copied().collect();

        trace!(
            pad_len,
            xpad_len = xpad.len(),
            fpad = format!("{:02X} {:02X}", fpad[0], fpad[1]),
            xpad_start = format!("{:02X?}", &xpad[..xpad.len().min(6)]),
            "Found PAD in DSE"
        );

        Some((xpad, fpad))
    }

    /// Process extracted PAD data, returning both DLS and MOT.
    fn process_pad(&mut self, xpad: &[u8], fpad: &[u8; 2]) -> Vec<PadData> {
        let mut results = Vec::new();

        let fpad_type = fpad[0] >> 6;
        let xpad_ind = (fpad[0] >> 4) & 0x03;
        let ci_flag = fpad[1] & 0x02 != 0;

        // Only process F-PAD type 0 (data from X-PAD)
        if fpad_type != 0 {
            trace!(fpad_type, "Skipping non-data F-PAD");
            return results;
        }

        // Parse X-PAD based on indicator
        let xpad_cis = match xpad_ind {
            0b01 if ci_flag && !xpad.is_empty() => {
                // Short X-PAD with CI
                let ci_byte = xpad[0];
                let app_type = ci_byte & 0x1F;
                if app_type != 0 {
                    vec![XpadCi { app_type, len: 3 }] // Short X-PAD data is 3 bytes
                } else {
                    Vec::new()
                }
            }
            0b10 if ci_flag => {
                // Variable size X-PAD with CI list
                self.parse_variable_ci_list(xpad)
            }
            0b01 | 0b10 if !ci_flag => {
                // Continuation of previous X-PAD
                if let Some(ci) = self.last_ci {
                    vec![ci]
                } else {
                    Vec::new()
                }
            }
            _ => Vec::new(),
        };

        if xpad_cis.is_empty() {
            return results;
        }

        // Calculate where data starts (after CI bytes)
        let ci_bytes = if ci_flag {
            match xpad_ind {
                0b01 => 1, // Short X-PAD: 1 CI byte
                0b10 => {
                    // Variable: count CI bytes until end marker or max 4
                    let mut count = 0;
                    for &b in xpad.iter().take(4) {
                        count += 1;
                        if b & 0x1F == 0 {
                            break;
                        }
                    }
                    count
                }
                _ => 0,
            }
        } else {
            0
        };

        // Process each X-PAD data subfield
        let mut offset = ci_bytes;
        let mut last_continued_type = None;

        for ci in &xpad_cis {
            if offset + ci.len > xpad.len() {
                trace!(
                    offset,
                    ci_len = ci.len,
                    xpad_len = xpad.len(),
                    "X-PAD data truncated"
                );
                break;
            }

            let data = &xpad[offset..offset + ci.len];
            offset += ci.len;

            match ci.app_type {
                XPAD_APP_DGLI => {
                    // Data Group Length Indicator
                    self.process_dgli(data);
                }
                XPAD_APP_DLS_START => {
                    // DLS start - begin new data group
                    self.dls_accumulator.start(data);
                    self.process_accumulated_dls(&mut results);
                    last_continued_type = Some(XPAD_APP_DLS_CONT);
                }
                XPAD_APP_DLS_CONT => {
                    // DLS continuation - add to data group
                    self.dls_accumulator.accumulate(data);
                    self.process_accumulated_dls(&mut results);
                    last_continued_type = Some(XPAD_APP_DLS_CONT);
                }
                XPAD_APP_MOT_START => {
                    // MOT start - use DGLI length
                    self.mot_accumulator.set_len(self.dgli_len);
                    self.mot_accumulator.start(data);
                    self.process_accumulated_mot(&mut results);
                    last_continued_type = Some(XPAD_APP_MOT_CONT);
                }
                XPAD_APP_MOT_CONT => {
                    // MOT continuation
                    self.mot_accumulator.accumulate(data);
                    self.process_accumulated_mot(&mut results);
                    last_continued_type = Some(XPAD_APP_MOT_CONT);
                }
                _ => {
                    trace!(app_type = ci.app_type, "Ignoring unknown X-PAD app_type");
                }
            }
        }

        // Save last CI for continuation
        if let Some(ct) = last_continued_type {
            self.last_ci = Some(XpadCi {
                app_type: ct,
                len: xpad_cis.last().map(|c| c.len).unwrap_or(0),
            });
        }

        results
    }

    /// Parse variable-size X-PAD CI list.
    fn parse_variable_ci_list(&self, xpad: &[u8]) -> Vec<XpadCi> {
        let mut cis = Vec::new();

        for &ci_byte in xpad.iter().take(4) {
            let app_type = ci_byte & 0x1F;
            if app_type == 0 {
                break; // End marker
            }

            let len_index = (ci_byte >> 5) as usize;
            if len_index < XPAD_LEN_TABLE.len() {
                cis.push(XpadCi {
                    app_type,
                    len: XPAD_LEN_TABLE[len_index],
                });
            }
        }

        cis
    }

    /// Process DGLI (Data Group Length Indicator).
    fn process_dgli(&mut self, data: &[u8]) {
        // DGLI is 2 bytes + 2 byte CRC = 4 bytes minimum
        if data.len() < 4 {
            return;
        }

        // Verify CRC
        let crc_stored = (data[2] as u16) << 8 | data[3] as u16;
        if !Crc16Ccitt::check(&data[..2], crc_stored) {
            return;
        }

        // Extract length (14 bits)
        self.dgli_len = ((data[0] as usize & 0x3F) << 8) | (data[1] as usize);
        trace!(dgli_len = self.dgli_len, "DGLI received");
    }

    /// Process accumulated DLS data and extract any complete segments.
    fn process_accumulated_dls(&mut self, results: &mut Vec<PadData>) {
        // Check for commands first
        if let Some(cmd) = self.dls_accumulator.check_command()
            && cmd == 0x01
        {
            // Remove label command
            self.dls_reassembler.reset();
            self.last_label = None;
            debug!("DLS: remove label command");
        }

        // Try to extract complete segments
        while let Some(seg) = self.dls_accumulator.try_extract() {
            if let Some(label) = self.dls_reassembler.add_segment(seg)
                && self.last_label.as_ref() != Some(&label)
            {
                self.last_label = Some(label.clone());
                results.push(PadData::Dls(DlsMetadata { text: label }));
            }
        }
    }

    /// Process accumulated MOT data and extract any complete objects.
    fn process_accumulated_mot(&mut self, results: &mut Vec<PadData>) {
        // Try to extract complete data group
        if let Some(dg) = self.mot_accumulator.try_extract() {
            // Pass to MOT manager
            if let Some(file) = self.mot_manager.handle_data_group(&dg) {
                results.push(PadData::Mot(MotMetadata::from(file)));
            }
        }
    }

    /// Extract PAD data from a DAB+ Access Unit, returning both DLS and MOT.
    pub fn extract_all_from_au(&mut self, au: &[u8]) -> Vec<PadData> {
        if let Some((xpad_data, fpad)) = self.find_pad_in_dse(au) {
            self.process_pad(&xpad_data, &fpad)
        } else {
            Vec::new()
        }
    }

    /// Reset extractor state.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.dls_accumulator.reset();
        self.dls_reassembler.reset();
        self.mot_accumulator.reset();
        self.mot_manager.reset();
        self.last_label = None;
        self.last_ci = None;
        self.dgli_len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_ccitt() {
        // Known test vector
        let data = b"123456789";
        let crc = Crc16Ccitt::compute(data);
        // CRC-16-CCITT (init=0xFFFF, final XOR=0xFFFF) for "123456789" = 0xD64E
        assert_eq!(crc, 0xD64E);
    }

    #[test]
    fn test_dls_segment_flags() {
        // First segment (seg_num should be 0 regardless of prefix[1])
        let seg = DlsSegment {
            prefix: [0xC0, 0x45], // toggle=1, first=1, last=0
            chars: b"TEST".to_vec(),
        };
        assert!(seg.toggle());
        assert!(seg.first());
        assert!(!seg.last());
        assert_eq!(seg.seg_num(), 0);
        assert_eq!(seg.charset(), 4); // Upper nibble of prefix[1]

        // Middle segment
        let seg2 = DlsSegment {
            prefix: [0x80, 0x20], // toggle=1, first=0, last=0, seg_num=2
            chars: b"DATA".to_vec(),
        };
        assert!(seg2.toggle());
        assert!(!seg2.first());
        assert!(!seg2.last());
        assert_eq!(seg2.seg_num(), 2);

        // Last segment
        let seg3 = DlsSegment {
            prefix: [0xA0, 0x30], // toggle=1, first=0, last=1, seg_num=3
            chars: b"END".to_vec(),
        };
        assert!(seg3.toggle());
        assert!(!seg3.first());
        assert!(seg3.last());
        assert_eq!(seg3.seg_num(), 3);
    }

    #[test]
    fn test_dls_reassembler_single_segment() {
        let mut reassembler = DlsReassembler::new();

        // Single segment that is both first and last
        let seg = DlsSegment {
            prefix: [0xE0, 0x00], // toggle=1, first=1, last=1
            chars: b"HELLO".to_vec(),
        };

        let result = reassembler.add_segment(seg);
        assert_eq!(result, Some("HELLO".to_string()));
    }

    #[test]
    fn test_dls_reassembler_multiple_segments() {
        let mut reassembler = DlsReassembler::new();

        // Segment 0 (first, not last)
        let seg0 = DlsSegment {
            prefix: [0xC0, 0x00], // toggle=1, first=1, last=0
            chars: b"HELLO ".to_vec(),
        };
        assert!(reassembler.add_segment(seg0).is_none());

        // Segment 1 (not first, last)
        let seg1 = DlsSegment {
            prefix: [0xA0, 0x10], // toggle=1, first=0, last=1, seg_num=1
            chars: b"WORLD".to_vec(),
        };
        let result = reassembler.add_segment(seg1);
        assert_eq!(result, Some("HELLO WORLD".to_string()));
    }

    #[test]
    fn test_dls_reassembler_toggle_change() {
        let mut reassembler = DlsReassembler::new();

        // Add segment with toggle=0
        let seg0 = DlsSegment {
            prefix: [0x40, 0x00], // toggle=0, first=1, last=0
            chars: b"OLD".to_vec(),
        };
        reassembler.add_segment(seg0);

        // Add segment with toggle=1 (should clear old segments)
        let seg1 = DlsSegment {
            prefix: [0xE0, 0x00], // toggle=1, first=1, last=1
            chars: b"NEW".to_vec(),
        };
        let result = reassembler.add_segment(seg1);
        assert_eq!(result, Some("NEW".to_string()));
    }

    #[test]
    fn test_decode_ebu_latin() {
        let data = b"FRANCE CULTURE";
        let result = decode_ebu_latin(data);
        assert_eq!(result, "FRANCE CULTURE");

        // With special characters
        let data2 = [b'C', b'a', b'f', 0xE9]; // "Café" with é
        let result2 = decode_ebu_latin(&data2);
        assert!(result2.starts_with("Caf"));
    }

    #[test]
    fn test_xpad_len_table() {
        assert_eq!(XPAD_LEN_TABLE[0], 4);
        assert_eq!(XPAD_LEN_TABLE[1], 6);
        assert_eq!(XPAD_LEN_TABLE[2], 8);
        assert_eq!(XPAD_LEN_TABLE[3], 12);
        assert_eq!(XPAD_LEN_TABLE[4], 16);
        assert_eq!(XPAD_LEN_TABLE[5], 24);
        assert_eq!(XPAD_LEN_TABLE[6], 32);
        assert_eq!(XPAD_LEN_TABLE[7], 48);
    }

    #[test]
    fn test_pad_extractor_minimal_au() {
        let mut extractor = PadExtractor::new();

        // Too short
        let result = extractor.extract_all_from_au(&[0x00, 0x00]);
        assert!(result.is_empty());

        // Minimum valid (but no X-PAD indicator)
        let result = extractor.extract_all_from_au(&[0x00, 0x00, 0x00]);
        assert!(result.is_empty());
    }
}
