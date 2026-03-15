//! MOT (Multimedia Object Transfer) decoder for DAB+ slideshows.
//!
//! Implements ETSI EN 301 234 for decoding MOT objects (images) from X-PAD.

use std::collections::BTreeMap;
use tracing::{debug, trace};

/// Completed MOT file.
#[derive(Debug, Clone, Default)]
pub struct MotFile {
    /// Image data.
    pub data: Vec<u8>,
    /// Body size from header.
    pub body_size: usize,
    /// Content type (0x02 = image).
    pub content_type: u8,
    /// Content sub-type (0x001 = JPEG, 0x003 = PNG).
    pub content_sub_type: u16,
    /// Content name/filename.
    pub content_name: Option<String>,
    /// Category title.
    pub category_title: Option<String>,
    /// Click-through URL.
    pub click_through_url: Option<String>,
    /// Trigger time is "now".
    pub trigger_time_now: bool,
}

impl MotFile {
    /// Content type for images.
    #[allow(dead_code)]
    pub const CONTENT_TYPE_IMAGE: u8 = 0x02;
    /// Content sub-type for JPEG.
    #[allow(dead_code)]
    pub const CONTENT_SUB_TYPE_JFIF: u16 = 0x001;
    /// Content sub-type for PNG.
    #[allow(dead_code)]
    pub const CONTENT_SUB_TYPE_PNG: u16 = 0x003;

    /// Check if this is a displayable image.
    #[allow(dead_code)]
    pub fn is_image(&self) -> bool {
        self.content_type == Self::CONTENT_TYPE_IMAGE
            && (self.content_sub_type == Self::CONTENT_SUB_TYPE_JFIF
                || self.content_sub_type == Self::CONTENT_SUB_TYPE_PNG)
    }

    /// Get file extension based on content type.
    #[allow(dead_code)]
    pub fn extension(&self) -> &'static str {
        match (self.content_type, self.content_sub_type) {
            (0x02, 0x001) => "jpg",
            (0x02, 0x003) => "png",
            _ => "bin",
        }
    }
}

/// MOT entity (header or body) segment collector.
#[derive(Debug)]
struct MotEntity {
    /// Segments by segment number.
    segments: BTreeMap<u16, Vec<u8>>,
    /// Last segment number (-1 if unknown).
    last_seg_number: i32,
    /// Total accumulated size.
    size: usize,
}

impl Default for MotEntity {
    fn default() -> Self {
        Self {
            segments: BTreeMap::new(),
            last_seg_number: -1,
            size: 0,
        }
    }
}

impl MotEntity {
    fn reset(&mut self) {
        self.segments.clear();
        self.last_seg_number = -1;
        self.size = 0;
    }

    fn add_segment(&mut self, seg_number: u16, last_seg: bool, data: &[u8]) {
        if last_seg {
            self.last_seg_number = seg_number as i32;
        }

        // Don't overwrite existing segment
        if self.segments.contains_key(&seg_number) {
            return;
        }

        self.segments.insert(seg_number, data.to_vec());
        self.size += data.len();
    }

    fn is_finished(&self) -> bool {
        if self.last_seg_number < 0 {
            return false;
        }

        // Check all segments from 0 to last are present
        for i in 0..=self.last_seg_number as u16 {
            if !self.segments.contains_key(&i) {
                return false;
            }
        }
        true
    }

    fn get_data(&self) -> Vec<u8> {
        let mut result = Vec::with_capacity(self.size);
        for i in 0..=self.last_seg_number as u16 {
            if let Some(seg) = self.segments.get(&i) {
                result.extend_from_slice(seg);
            }
        }
        result
    }

    fn get_size(&self) -> usize {
        self.size
    }
}

/// MOT object being assembled.
#[derive(Debug, Default)]
struct MotObject {
    header: MotEntity,
    body: MotEntity,
    header_received: bool,
    shown: bool,
    result_file: MotFile,
}

impl MotObject {
    fn add_segment(&mut self, is_header: bool, seg_number: u16, last_seg: bool, data: &[u8]) {
        if is_header {
            self.header.add_segment(seg_number, last_seg, data);
        } else {
            self.body.add_segment(seg_number, last_seg, data);
        }
    }

    /// Parse and validate the MOT header.
    fn parse_header(&mut self) -> bool {
        let data = self.header.get_data();

        // Parse header core (minimum 7 bytes)
        if data.len() < 7 {
            return false;
        }

        let body_size = ((data[0] as usize) << 20)
            | ((data[1] as usize) << 12)
            | ((data[2] as usize) << 4)
            | ((data[3] >> 4) as usize);
        let header_size = (((data[3] & 0x0F) as usize) << 9)
            | ((data[4] as usize) << 1)
            | ((data[5] >> 7) as usize);
        let content_type = (data[5] & 0x7F) >> 1;
        let content_sub_type = (((data[5] & 0x01) as u16) << 8) | (data[6] as u16);

        trace!(
            body_size,
            header_size, content_type, content_sub_type, "MOT header core"
        );

        // Verify header size matches
        if header_size != self.header.get_size() {
            trace!(
                expected = header_size,
                actual = self.header.get_size(),
                "MOT header size mismatch"
            );
            return false;
        }

        self.result_file.body_size = body_size;
        self.result_file.content_type = content_type;
        self.result_file.content_sub_type = content_sub_type;

        // Parse header extension parameters
        let mut offset = 7;
        while offset < data.len() {
            let pli = data[offset] >> 6;
            let param_id = data[offset] & 0x3F;
            offset += 1;

            // Get parameter length
            let data_len = match pli {
                0b00 => 0,
                0b01 => 1,
                0b10 => 4,
                0b11 => {
                    if offset >= data.len() {
                        return false;
                    }
                    let ext = data[offset] & 0x80 != 0;
                    let mut len = (data[offset] & 0x7F) as usize;
                    offset += 1;

                    if ext {
                        if offset >= data.len() {
                            return false;
                        }
                        len = (len << 8) + data[offset] as usize;
                        offset += 1;
                    }
                    len
                }
                _ => 0,
            };

            if offset + data_len > data.len() {
                return false;
            }

            // Process parameter
            match param_id {
                0x05 => {
                    // TriggerTime
                    if data_len >= 4 {
                        self.result_file.trigger_time_now = data[offset] & 0x80 == 0;
                    }
                }
                0x0C => {
                    // ContentName
                    if data_len > 0 {
                        let charset = data[offset] >> 4;
                        let name_bytes = &data[offset + 1..offset + data_len];
                        self.result_file.content_name =
                            Some(decode_mot_string(name_bytes, charset));
                    }
                }
                0x26 => {
                    // CategoryTitle (already UTF-8)
                    let title = String::from_utf8_lossy(&data[offset..offset + data_len]);
                    self.result_file.category_title = Some(title.into_owned());
                }
                0x27 => {
                    // ClickThroughURL (already UTF-8)
                    let url = String::from_utf8_lossy(&data[offset..offset + data_len]);
                    self.result_file.click_through_url = Some(url.into_owned());
                }
                _ => {}
            }

            offset += data_len;
        }

        self.header_received = true;
        true
    }

    /// Check if the object is ready to be displayed.
    fn is_ready_to_show(&mut self) -> bool {
        if self.shown {
            return false;
        }

        // Process finished header
        if self.header.is_finished() && !self.header_received {
            if !self.parse_header() {
                return false;
            }
            self.header.reset(); // Allow for header updates
        }

        // Check if complete
        if !self.header_received {
            return false;
        }
        if !self.body.is_finished() {
            return false;
        }
        if self.body.get_size() != self.result_file.body_size {
            return false;
        }
        if !self.result_file.trigger_time_now {
            return false;
        }

        // Add body data
        self.result_file.data = self.body.get_data();
        self.shown = true;
        true
    }

    fn get_file(&self) -> MotFile {
        self.result_file.clone()
    }

    fn get_current_body_size(&self) -> usize {
        self.body.get_size()
    }

    fn get_total_body_size(&self) -> usize {
        self.result_file.body_size
    }
}

/// Decode MOT string based on charset.
fn decode_mot_string(data: &[u8], charset: u8) -> String {
    match charset {
        0 | 15 => {
            // EBU Latin - similar to ISO 8859-15
            data.iter()
                .map(|&b| if b < 0x20 { ' ' } else { b as char })
                .collect::<String>()
                .trim()
                .to_string()
        }
        6 => {
            // UTF-8
            String::from_utf8_lossy(data).into_owned()
        }
        _ => {
            // Try UTF-8, fall back to Latin-1
            String::from_utf8(data.to_vec())
                .unwrap_or_else(|_| data.iter().map(|&b| b as char).collect())
        }
    }
}

/// MOT Manager - handles MOT data groups and assembles objects.
#[derive(Debug, Default)]
pub struct MotManager {
    object: MotObject,
    current_transport_id: i32,
}

impl MotManager {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.object = MotObject::default();
        self.current_transport_id = -1;
    }

    /// Handle a complete MOT data group.
    ///
    /// Returns a completed MOT file if one is ready.
    pub fn handle_data_group(&mut self, dg: &[u8]) -> Option<MotFile> {
        let mut offset = 0;

        // Parse Data Group header
        if dg.len() < offset + 2 {
            return None;
        }

        let extension_flag = dg[offset] & 0x80 != 0;
        let crc_flag = dg[offset] & 0x40 != 0;
        let segment_flag = dg[offset] & 0x20 != 0;
        let user_access_flag = dg[offset] & 0x10 != 0;
        let dg_type = dg[offset] & 0x0F;
        offset += 2;

        if extension_flag {
            offset += 2;
        }

        // Must have CRC, segment, and user access
        if !crc_flag || !segment_flag || !user_access_flag {
            return None;
        }

        // Only accept MOT header (3) or body (4)
        if dg_type != 3 && dg_type != 4 {
            return None;
        }

        // Parse session header
        if dg.len() < offset + 3 {
            return None;
        }

        let last_seg = dg[offset] & 0x80 != 0;
        let seg_number = (((dg[offset] & 0x7F) as u16) << 8) | (dg[offset + 1] as u16);
        let transport_id_flag = dg[offset + 2] & 0x10 != 0;
        let len_indicator = (dg[offset + 2] & 0x0F) as usize;
        offset += 3;

        if !transport_id_flag || len_indicator < 2 {
            return None;
        }

        if dg.len() < offset + len_indicator {
            return None;
        }

        let transport_id = ((dg[offset] as i32) << 8) | (dg[offset + 1] as i32);
        offset += len_indicator;

        // Parse segmentation header
        if dg.len() < offset + 2 {
            return None;
        }

        let seg_size = (((dg[offset] & 0x1F) as usize) << 8) | (dg[offset + 1] as usize);
        offset += 2;

        // Verify segment size (should match remaining data minus CRC)
        let crc_len = 2;
        if seg_size != dg.len() - offset - crc_len {
            trace!(
                announced = seg_size,
                actual = dg.len() - offset - crc_len,
                "MOT segment size mismatch"
            );
            return None;
        }

        // Reset object if transport ID changed
        if self.current_transport_id != transport_id {
            self.current_transport_id = transport_id;
            self.object = MotObject::default();
        }

        // Add segment
        let is_header = dg_type == 3;
        self.object.add_segment(
            is_header,
            seg_number,
            last_seg,
            &dg[offset..offset + seg_size],
        );

        trace!(
            dg_type,
            seg_number,
            last_seg,
            transport_id,
            seg_size,
            body_progress = format!(
                "{}/{}",
                self.object.get_current_body_size(),
                self.object.get_total_body_size()
            ),
            "MOT segment received"
        );

        // Check if object is ready to show
        if self.object.is_ready_to_show() {
            let file = self.object.get_file();
            debug!(
                content_type = file.content_type,
                content_sub_type = file.content_sub_type,
                size = file.data.len(),
                name = ?file.content_name,
                "MOT file completed"
            );
            return Some(file);
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mot_entity_segments() {
        let mut entity = MotEntity::default();

        entity.add_segment(0, false, b"HELLO");
        assert!(!entity.is_finished());

        entity.add_segment(1, true, b"WORLD");
        assert!(entity.is_finished());

        let data = entity.get_data();
        assert_eq!(data, b"HELLOWORLD");
    }

    #[test]
    fn test_mot_file_extension() {
        let file = MotFile {
            content_type: 0x02,
            content_sub_type: 0x001,
            ..Default::default()
        };
        assert_eq!(file.extension(), "jpg");

        let file = MotFile {
            content_type: 0x02,
            content_sub_type: 0x003,
            ..Default::default()
        };
        assert_eq!(file.extension(), "png");
    }
}
