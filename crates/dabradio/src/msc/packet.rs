//! DAB packet-mode reassembly (ETSI EN 300 401 §5.3.2 / §5.3.3 / §5.3.5).
//!
//! MSC logical frames from [`super::MscHandler`] are a concatenation of
//! 24/48/72/96-byte packets. Each packet has a 3-byte header, a useful-data
//! field, padding, and a 2-byte CRC-16 (ITU-T X.25). Packets sharing an
//! address are concatenated using First/Last flags into an MSC data group.
//!
//! When FIG 0/14 signals FEC (or address-1022 FEC packets are observed), an
//! outer Reed-Solomon RS(204,188) layer protects every packet in the
//! sub-channel. Nine 24-byte FEC packets (addr 1022) carry the parity for
//! each 2256-byte application packet set — see EN 300 401 §5.3.5 and
//! dab-cmdline `dataProcessor::handleRSPacket()`.
//!
//! Packet header layout matches dab-cmdline `dataProcessor::handlePacket()`:
//! length (2), continuity (2), first/last (2), address (10), command (1),
//! useful length (7).

use crate::constants;
use reed_solomon::Decoder as RsDecoder;
use tracing::debug;

/// Standard packet lengths in bytes (EN 300 401 Table 6).
const PACKET_LENGTHS: [usize; 4] = [24, 48, 72, 96];

/// FEC frame: 12 rows × 188 application columns (EN 300 401 §5.3.5.1).
const RS_ROWS: usize = 12;
const RS_DATA_COLS: usize = 188;
const RS_PARITY_COLS: usize = 16;
const RS_CODEWORD_LEN: usize = RS_DATA_COLS + RS_PARITY_COLS; // 204
const APP_DATA_LEN: usize = RS_ROWS * RS_DATA_COLS; // 2256
const FEC_PACKETS: usize = 9;
const FEC_DATA_PER_PACKET: usize = 22;
const FEC_ADDR: u16 = 1022;
const RS_PARITY: usize = RS_PARITY_COLS;

/// Statistics for CRC validation. A pass rate indistinguishable from
/// chance (1/65536 per check) means the MSC bits are still wrong.
#[derive(Debug, Default, Clone, Copy)]
pub struct PacketStats {
    pub packets_seen: u64,
    pub crc_ok: u64,
    pub crc_fail: u64,
    pub address_mismatch: u64,
    pub groups_complete: u64,
    /// Histogram of First/Last flags on CRC-ok, address-matched packets
    /// (index = fl value 0..=3).
    pub first_last_hist: [u64; 4],
    pub continuity_gaps: u64,
    pub useful_zero: u64,
    pub command_flag: u64,
    /// CRC-ok packets whose address matched the target.
    pub address_match: u64,
    /// Top observed addresses on the wire (incl. CRC-fail / FEC).
    pub seen_addresses: [u16; 8],
    pub seen_address_counts: [u64; 8],
    /// Addresses on CRC-ok packets after FEC (or legacy path).
    pub crc_ok_addresses: [u16; 8],
    pub crc_ok_address_counts: [u64; 8],
    /// Completed FEC frames (RS applied).
    pub fec_frames: u64,
    /// FEC frames skipped because the application buffer was not full.
    pub fec_desync: u64,
    /// Rows where RS reported uncorrectable errors.
    pub fec_rs_fail_rows: u64,
}

impl PacketStats {
    pub fn crc_pass_rate(&self) -> f64 {
        if self.packets_seen == 0 {
            0.0
        } else {
            self.crc_ok as f64 / self.packets_seen as f64
        }
    }

    /// True when the observed CRC pass rate is indistinguishable from a
    /// random bitstream scanned at known packet boundaries.
    pub fn is_chance_level(&self) -> bool {
        self.packets_seen >= 64 && self.crc_pass_rate() < 0.05
    }
}

/// One reassembled MSC data group (header + optional session header + data
/// + optional CRC), as raw bytes.
#[derive(Debug, Clone)]
pub struct DataGroup {
    pub address: u16,
    pub bytes: Vec<u8>,
}

impl DataGroup {
    /// Strip the MSC data-group header / CRC and return the application
    /// payload. Returns the full buffer if the header is truncated.
    pub fn application_payload(&self) -> &[u8] {
        strip_data_group_header(&self.bytes)
    }

    /// Select the bytes to feed an application decoder.
    ///
    /// When FIG 0/3 DG=1 (`no_data_groups`), the packet useful-data field *is*
    /// the application payload — there is no MSC data-group header to strip.
    pub fn payload_for_dg_flag(&self, no_data_groups: bool) -> &[u8] {
        if no_data_groups {
            &self.bytes
        } else {
            self.application_payload()
        }
    }
}

/// Assemble packets for a single packet address.
pub struct PacketAssembler {
    address: u16,
    assembling: bool,
    last_continuity: Option<u8>,
    buffer: Vec<u8>,
    pub stats: PacketStats,
    /// Auto-enabled when an address-1022 FEC packet is seen.
    fec_enabled: bool,
    app_buf: Vec<u8>,
    fec_buf: [u8; FEC_PACKETS * FEC_DATA_PER_PACKET],
    fec_present: [bool; FEC_PACKETS],
    rs: RsDecoder,
}

impl PacketAssembler {
    pub fn new(address: u16) -> Self {
        Self {
            address,
            assembling: false,
            last_continuity: None,
            buffer: Vec::new(),
            stats: PacketStats::default(),
            fec_enabled: false,
            app_buf: Vec::with_capacity(APP_DATA_LEN + 96),
            fec_buf: [0u8; FEC_PACKETS * FEC_DATA_PER_PACKET],
            fec_present: [false; FEC_PACKETS],
            rs: RsDecoder::new(RS_PARITY),
        }
    }

    /// Force FEC mode (FIG 0/14). Auto-detect also enables on addr 1022.
    #[allow(dead_code)]
    pub fn set_fec_scheme(&mut self, enabled: bool) {
        self.fec_enabled = enabled;
    }

    /// Consume one MSC logical frame (decoded subchannel bytes) and return
    /// any completed data groups.
    pub fn feed_bytes(&mut self, bytes: &[u8]) -> Vec<DataGroup> {
        let mut groups = Vec::new();
        let mut offset = 0;
        while offset + 5 <= bytes.len() {
            let length_id = (bytes[offset] >> 6) & 0x03;
            let packet_len = PACKET_LENGTHS[length_id as usize];
            if offset + packet_len > bytes.len() {
                break;
            }
            let packet = &bytes[offset..offset + packet_len];
            offset += packet_len;
            groups.extend(self.dispatch_packet(packet));
        }
        groups
    }

    fn dispatch_packet(&mut self, packet: &[u8]) -> Vec<DataGroup> {
        let address = (((packet[0] as u16) & 0x03) << 8) | packet[1] as u16;
        self.record_address(address);

        // FEC packets: length always 24, address 1022 (EN 300 401 §5.3.5.2).
        if packet.len() == 24 && address == FEC_ADDR {
            self.fec_enabled = true;
            return self.handle_fec_packet(packet);
        }

        if self.fec_enabled {
            if self.app_buf.len() + packet.len() > APP_DATA_LEN + 96 {
                debug!("packet FEC app buffer overflow — resync");
                self.clear_fec_state();
                self.app_buf.clear();
            }
            self.app_buf.extend_from_slice(packet);
            return Vec::new();
        }

        if let Some(g) = self.handle_packet(packet) {
            vec![g]
        } else {
            Vec::new()
        }
    }

    fn handle_fec_packet(&mut self, packet: &[u8]) -> Vec<DataGroup> {
        // Counter occupies the 4 bits that would be continuity+first/last.
        let counter = ((packet[0] >> 2) & 0x0F) as usize;
        if counter >= FEC_PACKETS {
            return Vec::new();
        }
        self.fec_buf[counter * FEC_DATA_PER_PACKET..(counter + 1) * FEC_DATA_PER_PACKET]
            .copy_from_slice(&packet[2..2 + FEC_DATA_PER_PACKET]);
        self.fec_present[counter] = true;

        if counter != FEC_PACKETS - 1 || !self.fec_complete() {
            return Vec::new();
        }

        let mut groups = Vec::new();
        if self.app_buf.len() >= APP_DATA_LEN {
            // Trim any overrun padding before RS.
            self.app_buf.truncate(APP_DATA_LEN);
            if self.process_rs() {
                self.stats.fec_frames += 1;
                groups = self.handle_rs_packets();
            }
        } else {
            self.stats.fec_desync += 1;
            debug!(
                filled = self.app_buf.len(),
                expected = APP_DATA_LEN,
                "packet FEC frame desync"
            );
        }
        self.clear_fec_state();
        self.app_buf.clear();
        groups
    }

    fn fec_complete(&self) -> bool {
        self.fec_present.iter().all(|&p| p)
    }

    fn clear_fec_state(&mut self) {
        self.fec_present = [false; FEC_PACKETS];
        self.fec_buf = [0u8; FEC_PACKETS * FEC_DATA_PER_PACKET];
    }

    /// Column-major deinterleave, RS(204,188) per row, write corrected data back.
    fn process_rs(&mut self) -> bool {
        let mut table = [[0u8; RS_CODEWORD_LEN]; RS_ROWS];
        for (i, &b) in self.app_buf[..APP_DATA_LEN].iter().enumerate() {
            table[i % RS_ROWS][i / RS_ROWS] = b;
        }
        // First 192 FEC bytes fill the 12×16 RS data table (column-major);
        // the last 6 bytes of FEC packet 8 are padding zeros.
        let rs_bytes = RS_ROWS * RS_PARITY_COLS;
        for i in 0..rs_bytes {
            table[i % RS_ROWS][RS_DATA_COLS + i / RS_ROWS] = self.fec_buf[i];
        }

        let mut ok = true;
        for row in table.iter_mut() {
            match self.rs.correct(row, None) {
                Ok(corrected) => {
                    let data = corrected.data();
                    row[..RS_DATA_COLS].copy_from_slice(&data[..RS_DATA_COLS]);
                }
                Err(_) => {
                    self.stats.fec_rs_fail_rows += 1;
                    ok = false;
                }
            }
        }

        for i in 0..APP_DATA_LEN {
            self.app_buf[i] = table[i % RS_ROWS][i / RS_ROWS];
        }
        // Still emit packets even if some rows failed — CRC will discard bad ones.
        let _ = ok;
        true
    }

    fn handle_rs_packets(&mut self) -> Vec<DataGroup> {
        let mut groups = Vec::new();
        let mut base = 0;
        while base + 5 <= APP_DATA_LEN {
            let length_id = (self.app_buf[base] >> 6) & 0x03;
            let packet_len = PACKET_LENGTHS[length_id as usize];
            if base + packet_len > APP_DATA_LEN {
                break;
            }
            // Collect packet bytes before calling handle_packet (borrows self).
            let packet: Vec<u8> = self.app_buf[base..base + packet_len].to_vec();
            base += packet_len;
            if let Some(g) = self.handle_packet(&packet) {
                groups.push(g);
            }
        }
        groups
    }

    fn record_address(&mut self, address: u16) {
        Self::record_into(
            &mut self.stats.seen_addresses,
            &mut self.stats.seen_address_counts,
            address,
        );
    }

    fn record_into(addrs: &mut [u16; 8], counts: &mut [u64; 8], address: u16) {
        for i in 0..addrs.len() {
            if counts[i] == 0 {
                addrs[i] = address;
                counts[i] = 1;
                return;
            }
            if addrs[i] == address {
                counts[i] += 1;
                return;
            }
        }
    }

    fn handle_packet(&mut self, packet: &[u8]) -> Option<DataGroup> {
        self.stats.packets_seen += 1;
        let continuity = (packet[0] >> 4) & 0x03;
        let first_last = (packet[0] >> 2) & 0x03;
        let address = (((packet[0] as u16) & 0x03) << 8) | packet[1] as u16;
        let command = (packet[2] >> 7) & 0x01;
        let useful = (packet[2] & 0x7F) as usize;

        if !constants::crc16_check(packet) {
            self.stats.crc_fail += 1;
            self.assembling = false;
            self.buffer.clear();
            return None;
        }
        self.stats.crc_ok += 1;
        Self::record_into(
            &mut self.stats.crc_ok_addresses,
            &mut self.stats.crc_ok_address_counts,
            address,
        );

        if address != self.address {
            self.stats.address_mismatch += 1;
            return None;
        }
        self.stats.address_match += 1;
        self.stats.first_last_hist[first_last as usize] += 1;
        if command != 0 {
            self.stats.command_flag += 1;
        }
        if useful == 0 {
            self.stats.useful_zero += 1;
            // Padding / empty — nothing to assemble.
            return None;
        }
        if useful + 5 > packet.len() {
            self.assembling = false;
            self.buffer.clear();
            return None;
        }
        let payload = &packet[3..3 + useful];

        // dab-cmdline skips exact continuity repeats (except First packets).
        if let Some(prev) = self.last_continuity
            && continuity == prev
            && first_last != 2
        {
            return None;
        }

        if let Some(prev) = self.last_continuity
            && continuity != (prev + 1) % 4
            && first_last != 2
            && first_last != 3
        {
            debug!(
                expected = (prev + 1) % 4,
                got = continuity,
                "packet continuity gap"
            );
            self.stats.continuity_gaps += 1;
            self.assembling = false;
            self.buffer.clear();
        }
        self.last_continuity = Some(continuity);

        match first_last {
            2 => {
                // First packet of a series
                self.buffer.clear();
                self.buffer.extend_from_slice(payload);
                self.assembling = true;
                None
            }
            0 => {
                if self.assembling {
                    self.buffer.extend_from_slice(payload);
                }
                None
            }
            1 => {
                if self.assembling {
                    self.buffer.extend_from_slice(payload);
                    self.assembling = false;
                    self.stats.groups_complete += 1;
                    Some(DataGroup {
                        address,
                        bytes: std::mem::take(&mut self.buffer),
                    })
                } else {
                    None
                }
            }
            3 => {
                // Single packet = complete data group
                self.assembling = false;
                self.stats.groups_complete += 1;
                Some(DataGroup {
                    address,
                    bytes: payload.to_vec(),
                })
            }
            _ => None,
        }
    }
}

/// Parse an MSC data-group header and return the data-field bytes.
///
/// EN 300 401 §5.3.3.1: extension / CRC / segment / user-access flags in
/// byte 0, continuity+repetition in byte 1, then optional fields, then the
/// data field, then an optional 2-byte CRC when the CRC flag is set.
fn strip_data_group_header(bytes: &[u8]) -> &[u8] {
    if bytes.len() < 2 {
        return bytes;
    }
    let extension = bytes[0] & 0x80 != 0;
    let crc_flag = bytes[0] & 0x40 != 0;
    let segment = bytes[0] & 0x20 != 0;
    let user_access = bytes[0] & 0x10 != 0;
    let mut pos = 2;
    if extension {
        if pos >= bytes.len() {
            return bytes;
        }
        pos += 1;
    }
    if segment {
        pos += 2;
    }
    if user_access {
        if pos >= bytes.len() {
            return bytes;
        }
        let length_indicator = (bytes[pos] & 0x0F) as usize;
        pos += 1 + length_indicator;
    }
    if pos > bytes.len() {
        return bytes;
    }
    let end = if crc_flag && bytes.len() >= pos + 2 {
        bytes.len() - 2
    } else {
        bytes.len()
    };
    if pos > end {
        &bytes[pos.min(bytes.len())..]
    } else {
        &bytes[pos..end]
    }
}

/// Build a well-formed packet for tests (header + payload + padding + CRC).
#[cfg(test)]
pub fn encode_packet(
    length_id: u8,
    continuity: u8,
    first_last: u8,
    address: u16,
    payload: &[u8],
) -> Vec<u8> {
    let packet_len = PACKET_LENGTHS[length_id as usize];
    let mut pkt = vec![0u8; packet_len];
    pkt[0] = (length_id << 6)
        | ((continuity & 0x03) << 4)
        | ((first_last & 0x03) << 2)
        | ((address >> 8) as u8 & 0x03);
    pkt[1] = (address & 0xFF) as u8;
    pkt[2] = payload.len() as u8 & 0x7F;
    pkt[3..3 + payload.len()].copy_from_slice(payload);
    let crc = constants::crc16_ccitt(&pkt[..packet_len - 2]);
    pkt[packet_len - 2] = (crc >> 8) as u8;
    pkt[packet_len - 1] = (crc & 0xFF) as u8;
    pkt
}

#[cfg(test)]
mod tests {
    use super::*;
    use reed_solomon::Encoder as RsEncoder;

    #[test]
    fn crc_rejects_corrupt_packet() {
        let mut pkt = encode_packet(0, 0, 3, 852, b"hello");
        let mut asm = PacketAssembler::new(852);
        assert_eq!(asm.feed_bytes(&pkt).len(), 1);
        pkt[3] ^= 0xFF;
        assert!(asm.feed_bytes(&pkt).is_empty());
        assert!(asm.stats.crc_fail >= 1);
    }

    #[test]
    fn single_packet_data_group() {
        let pkt = encode_packet(0, 0, 3, 852, b"TPEG");
        let mut asm = PacketAssembler::new(852);
        let groups = asm.feed_bytes(&pkt);
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].bytes, b"TPEG");
    }

    #[test]
    fn multi_packet_reassembly() {
        let first = encode_packet(0, 0, 2, 10, b"AAA");
        let mid = encode_packet(0, 1, 0, 10, b"BBB");
        let last = encode_packet(0, 2, 1, 10, b"CCC");
        let mut asm = PacketAssembler::new(10);
        let mut buf = first;
        buf.extend_from_slice(&mid);
        buf.extend_from_slice(&last);
        let groups = asm.feed_bytes(&buf);
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].bytes, b"AAABBBCCC");
    }

    #[test]
    fn wrong_address_is_ignored() {
        let pkt = encode_packet(0, 0, 3, 1, b"x");
        let mut asm = PacketAssembler::new(852);
        assert!(asm.feed_bytes(&pkt).is_empty());
        assert!(asm.stats.address_mismatch >= 1);
        assert!(asm.stats.crc_ok >= 1);
    }

    #[test]
    fn empty_useful_data_is_not_a_group() {
        let pkt = encode_packet(0, 0, 3, 852, b"");
        let mut asm = PacketAssembler::new(852);
        assert!(asm.feed_bytes(&pkt).is_empty());
        assert_eq!(asm.stats.useful_zero, 1);
        assert_eq!(asm.stats.groups_complete, 0);
    }

    #[test]
    fn chance_level_detection() {
        let mut stats = PacketStats {
            packets_seen: 100,
            crc_ok: 1,
            crc_fail: 99,
            ..Default::default()
        };
        assert!(stats.is_chance_level());
        stats.crc_ok = 80;
        stats.crc_fail = 20;
        assert!(!stats.is_chance_level());
    }

    #[test]
    fn dg_flag_selects_raw_bytes_when_no_data_groups() {
        // Bytes that look like a data-group header (extension+CRC flags) but are
        // actually raw application data when DG=1.
        let raw = vec![0xC0, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
        let group = DataGroup {
            address: 852,
            bytes: raw.clone(),
        };
        assert_eq!(group.payload_for_dg_flag(true), raw.as_slice());
        // With data groups in use, the same bytes would be stripped as a header.
        assert_ne!(group.payload_for_dg_flag(false), raw.as_slice());
        assert_eq!(
            group.payload_for_dg_flag(false),
            group.application_payload()
        );
    }

    /// Build one FEC-protected packet set: 94×24-byte packets (addr 1 data +
    /// padding) filling 2256 bytes, then 9 FEC packets with RS parity.
    #[test]
    fn fec_frame_recovers_corrupted_data_packet() {
        let enc = RsEncoder::new(RS_PARITY);
        let mut app = Vec::with_capacity(APP_DATA_LEN);
        // One single-packet data group with payload, then padding to fill the set.
        let data = encode_packet(0, 0, 3, 1, b"TPEG-OK");
        app.extend_from_slice(&data);
        while app.len() + 24 <= APP_DATA_LEN {
            app.extend_from_slice(&encode_packet(0, 0, 3, 0, b""));
        }
        assert_eq!(app.len(), APP_DATA_LEN);

        // Interleave into RS rows and encode.
        let mut table = [[0u8; RS_CODEWORD_LEN]; RS_ROWS];
        for (i, &b) in app.iter().enumerate() {
            table[i % RS_ROWS][i / RS_ROWS] = b;
        }
        for row in table.iter_mut() {
            let encoded = enc.encode(&row[..RS_DATA_COLS]);
            row[..RS_CODEWORD_LEN].copy_from_slice(&encoded[..RS_CODEWORD_LEN]);
        }
        let mut fec = [0u8; FEC_PACKETS * FEC_DATA_PER_PACKET];
        for i in 0..(RS_ROWS * RS_PARITY_COLS) {
            fec[i] = table[i % RS_ROWS][RS_DATA_COLS + i / RS_ROWS];
        }

        // Corrupt several bytes in the first data packet (within RS capability).
        let mut noisy = app.clone();
        for &idx in &[3usize, 5, 7, 9] {
            noisy[idx] ^= 0xFF;
        }
        assert!(!constants::crc16_check(&noisy[..24]));

        let mut stream = noisy;
        for c in 0..FEC_PACKETS {
            let mut fp = [0u8; 24];
            // length=0, counter=c, address=1022
            fp[0] = ((c as u8) << 2) | ((FEC_ADDR >> 8) as u8 & 0x03);
            fp[1] = (FEC_ADDR & 0xFF) as u8;
            fp[2..24].copy_from_slice(&fec[c * FEC_DATA_PER_PACKET..(c + 1) * FEC_DATA_PER_PACKET]);
            stream.extend_from_slice(&fp);
        }

        let mut asm = PacketAssembler::new(1);
        // FIG 0/14 / prior FEC packet would enable this before the packet set.
        asm.set_fec_scheme(true);
        let groups = asm.feed_bytes(&stream);
        assert!(
            asm.stats.fec_frames >= 1,
            "expected FEC frame, desync={}",
            asm.stats.fec_desync
        );
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].bytes, b"TPEG-OK");
    }

    #[test]
    fn fec_auto_detect_recovers_on_second_frame() {
        let enc = RsEncoder::new(RS_PARITY);

        let build_frame = |payload: &[u8], continuity: u8| {
            let mut app = Vec::with_capacity(APP_DATA_LEN);
            app.extend_from_slice(&encode_packet(0, continuity, 3, 1, payload));
            while app.len() + 24 <= APP_DATA_LEN {
                app.extend_from_slice(&encode_packet(0, 0, 3, 0, b""));
            }
            let mut table = [[0u8; RS_CODEWORD_LEN]; RS_ROWS];
            for (i, &b) in app.iter().enumerate() {
                table[i % RS_ROWS][i / RS_ROWS] = b;
            }
            for row in table.iter_mut() {
                let encoded = enc.encode(&row[..RS_DATA_COLS]);
                row[..RS_CODEWORD_LEN].copy_from_slice(&encoded[..RS_CODEWORD_LEN]);
            }
            let mut fec = [0u8; FEC_PACKETS * FEC_DATA_PER_PACKET];
            for i in 0..(RS_ROWS * RS_PARITY_COLS) {
                fec[i] = table[i % RS_ROWS][RS_DATA_COLS + i / RS_ROWS];
            }
            let mut stream = app;
            for c in 0..FEC_PACKETS {
                let mut fp = [0u8; 24];
                fp[0] = ((c as u8) << 2) | ((FEC_ADDR >> 8) as u8 & 0x03);
                fp[1] = (FEC_ADDR & 0xFF) as u8;
                fp[2..24]
                    .copy_from_slice(&fec[c * FEC_DATA_PER_PACKET..(c + 1) * FEC_DATA_PER_PACKET]);
                stream.extend_from_slice(&fp);
            }
            stream
        };

        let mut asm = PacketAssembler::new(1);
        // First frame: FEC not yet known, application packets take the legacy
        // path; the closing FEC set desyncs and enables FEC mode.
        let _ = asm.feed_bytes(&build_frame(b"FIRST", 0));
        assert!(asm.fec_enabled);
        // Second frame is fully FEC-protected.
        let groups = asm.feed_bytes(&build_frame(b"SECOND", 1));
        assert!(asm.stats.fec_frames >= 1);
        assert!(groups.iter().any(|g| g.bytes == b"SECOND"));
    }
}
