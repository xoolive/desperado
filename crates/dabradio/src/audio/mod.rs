//! DAB+ audio decoder: superframe assembly, Reed-Solomon, AAC decoding.
//!
//! Pipeline:
//! 1. Accumulate 5 MSC logical frames → 1 superframe
//! 2. Fire Code sync check (CRC-16 polynomial 0x782F on bytes 2..10, stored in bytes 0..1)
//! 3. Reed-Solomon RS(120,110) error correction with stride deinterleaving
//! 4. Parse format byte → AudioSpecificConfig
//! 5. Extract AAC Access Units with CRC-16-CCITT validation
//! 6. Decode HE-AAC v2 → PCM samples
//!
//! Reference: ETSI TS 102 563 (DAB+ audio), welle.io src/backend/dabplus_decoder.cpp

use reed_solomon::Decoder as RsDecoder;
use tracing::{debug, info, warn};

use crate::pad::{PadData, PadExtractor};

/// Number of logical frames per superframe.
const FRAMES_PER_SUPERFRAME: usize = 5;

/// RS codeword length (110 data + 10 parity).
const RS_BLOCK_LEN: usize = 120;

/// RS parity bytes per codeword.
const RS_PARITY: usize = 10;

/// RS data bytes per codeword.
const RS_DATA: usize = RS_BLOCK_LEN - RS_PARITY; // 110

// ---------------------------------------------------------------------------
// Fire Code CRC-16 (polynomial 0x782F)
// ---------------------------------------------------------------------------

/// CRC-16 lookup table for Fire Code polynomial 0x782F.
/// Generated at compile time.
const fn build_fire_code_lut() -> [u16; 256] {
    let mut lut = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut j = 0;
        while j < 8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x782F;
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

static FIRE_CODE_LUT: [u16; 256] = build_fire_code_lut();

/// Compute Fire Code CRC-16 over a byte slice.
fn fire_code_crc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc = (crc << 8) ^ FIRE_CODE_LUT[((crc >> 8) as u8 ^ byte) as usize];
    }
    crc
}

/// Check Fire Code sync: stored CRC in sf[0..2] must match CRC over sf[2..11].
fn fire_code_check(sf: &[u8]) -> bool {
    if sf.len() < 11 {
        return false;
    }
    // Reject all-zero header (prevents false sync on silence)
    if sf[3] == 0x00 && sf[4] == 0x00 {
        return false;
    }
    let stored = (sf[0] as u16) << 8 | sf[1] as u16;
    let calced = fire_code_crc(&sf[2..11]);
    stored == calced
}

// ---------------------------------------------------------------------------
// CRC-16-CCITT for AU validation
// ---------------------------------------------------------------------------

/// CRC-16-CCITT lookup table (polynomial 0x1021, init 0xFFFF, final XOR 0xFFFF).
const fn build_ccitt_lut() -> [u16; 256] {
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

static CCITT_LUT: [u16; 256] = build_ccitt_lut();

/// Compute CRC-16-CCITT (init=0xFFFF, final XOR=0xFFFF) over a byte slice.
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc = (crc << 8) ^ CCITT_LUT[((crc >> 8) as u8 ^ byte) as usize];
    }
    crc ^ 0xFFFF
}

// ---------------------------------------------------------------------------
// DAB+ superframe format
// ---------------------------------------------------------------------------

/// Parsed DAB+ superframe format flags from byte sf[2].
#[derive(Debug, Clone, Copy)]
pub struct SuperframeFormat {
    pub dac_rate: bool,
    pub sbr_flag: bool,
    pub aac_channel_mode: bool, // false=mono, true=stereo (parametric or discrete)
    pub ps_flag: bool,
    #[allow(dead_code)]
    pub mpeg_surround_config: u8,
}

impl SuperframeFormat {
    fn parse(byte: u8) -> Self {
        Self {
            dac_rate: byte & 0x40 != 0,
            sbr_flag: byte & 0x20 != 0,
            aac_channel_mode: byte & 0x10 != 0,
            ps_flag: byte & 0x08 != 0,
            mpeg_surround_config: byte & 0x07,
        }
    }

    /// Core AAC sample rate index.
    fn core_sr_index(&self) -> u8 {
        if self.dac_rate {
            if self.sbr_flag { 6 } else { 3 } // 24000 / 48000
        } else if self.sbr_flag {
            8
        } else {
            5 // 16000 / 32000
        }
    }

    /// Core channel configuration (1=mono, 2=stereo).
    fn core_ch_config(&self) -> u8 {
        if self.aac_channel_mode { 2 } else { 1 }
    }

    /// Extension (SBR output) sample rate index.
    fn ext_sr_index(&self) -> u8 {
        if self.dac_rate { 3 } else { 5 } // 48000 / 32000
    }

    /// Number of AAC Access Units per superframe.
    pub fn num_aus(&self) -> usize {
        if self.dac_rate {
            if self.sbr_flag { 3 } else { 6 }
        } else if self.sbr_flag {
            2
        } else {
            4
        }
    }

    /// Byte offset where AU #0 data starts (after the header).
    fn au0_start(&self) -> usize {
        if self.dac_rate {
            if self.sbr_flag { 6 } else { 11 }
        } else if self.sbr_flag {
            5
        } else {
            8
        }
    }

    /// Output sample rate in Hz.
    pub fn sample_rate(&self) -> u32 {
        if self.dac_rate { 48000 } else { 32000 }
    }

    /// Number of output channels.
    pub fn channels(&self) -> usize {
        // HE-AAC v2 with PS or stereo channel mode → stereo output
        if (self.sbr_flag && self.ps_flag) || self.aac_channel_mode {
            2
        } else {
            1
        }
    }

    /// Build the MPEG-4 AudioSpecificConfig bytes for fdk-aac.
    ///
    /// Structure:
    ///   - 5 bits: AudioObjectType (2 = AAC-LC)
    ///   - 4 bits: samplingFrequencyIndex (core)
    ///   - 4 bits: channelConfiguration
    ///   - 3 bits: GASpecificConfig (0b100 = 960 transform)
    ///
    /// If SBR:
    ///   - 11 bits: SBR sync extension (0x2B7)
    ///   - 5 bits: AudioObjectType 5 (SBR)
    ///   - 1 bit: SBR present (1)
    ///   - 4 bits: extensionSamplingFrequencyIndex
    ///
    /// If PS:
    ///   - 11 bits: PS sync extension (0x548)
    ///   - 1 bit: PS present (1)
    pub fn audio_specific_config(&self) -> Vec<u8> {
        let mut asc = Vec::with_capacity(7);
        let sr = self.core_sr_index();
        let ch = self.core_ch_config();

        // AAC-LC core: 00010 + sr(4) + ch(4) + 100
        asc.push(0b0001_0000 | (sr >> 1));
        asc.push((sr & 0x01) << 7 | ch << 3 | 0b100);

        if self.sbr_flag {
            // SBR extension: 01010110111 00101 1 xxxx
            let ext_sr = self.ext_sr_index();
            asc.push(0x56); // 01010110
            asc.push(0xE5); // 11100101
            asc.push(0x80 | (ext_sr << 3));

            if self.ps_flag {
                // PS extension: 10101001000 1
                let last = asc.last_mut().unwrap();
                *last |= 0x05; // ...00101
                asc.push(0x48); // 01001000
                asc.push(0x80); // 10000000
            }
        }

        asc
    }
}

// ---------------------------------------------------------------------------
// SuperframeDecoder: accumulates logical frames, does RS + AU extraction
// ---------------------------------------------------------------------------

/// DAB+ superframe decoder.
///
/// Accumulates 5 logical frames into a superframe, performs Reed-Solomon
/// error correction, and extracts AAC Access Units.
pub struct SuperframeDecoder {
    /// Logical frame size in bytes (bitrate_kbps * 24 / 8).
    frame_size: usize,

    /// Superframe buffer (5 × frame_size bytes).
    sf_buf: Vec<u8>,

    /// Number of logical frames accumulated so far (0..4).
    frame_count: usize,

    /// RS decoder (10 parity symbols).
    rs_decoder: RsDecoder,

    /// Number of superframes processed.
    pub superframes_decoded: usize,

    /// Whether we have achieved Fire Code sync.
    synced: bool,

    /// Parsed format (set after first successful sync).
    pub format: Option<SuperframeFormat>,

    /// Number of RS corrections applied.
    pub rs_corrections: usize,

    /// Number of RS uncorrectable errors.
    pub rs_errors: usize,

    /// Number of AU CRC failures.
    pub au_crc_errors: usize,
}

impl SuperframeDecoder {
    /// Create a new superframe decoder for the given bitrate.
    pub fn new(bitrate_kbps: u16) -> Self {
        let frame_size = (bitrate_kbps as usize) * 24 / 8;
        Self {
            frame_size,
            sf_buf: vec![0u8; frame_size * FRAMES_PER_SUPERFRAME],
            frame_count: 0,
            rs_decoder: RsDecoder::new(RS_PARITY),
            superframes_decoded: 0,
            synced: false,
            format: None,
            rs_corrections: 0,
            rs_errors: 0,
            au_crc_errors: 0,
        }
    }

    /// Feed one MSC logical frame (decoded bytes from MscHandler).
    ///
    /// Uses a sliding window approach to find the correct superframe boundary:
    /// - Accumulate the first 5 frames normally
    /// - After that, slide (shift frames 1-4 left, append new frame at position 4)
    ///   and try to decode on every incoming frame
    /// - Once fire code sync is found, switch to collecting fresh 5-frame groups
    ///
    /// Returns a Vec of AAC Access Units when a complete superframe is
    /// successfully decoded, or None if still accumulating / sync lost.
    pub fn feed_frame(&mut self, frame: &[u8]) -> Option<Vec<Vec<u8>>> {
        if frame.len() != self.frame_size {
            warn!(
                "Frame size mismatch: expected {}, got {}",
                self.frame_size,
                frame.len()
            );
            return None;
        }

        if self.synced {
            // Synced mode: collect exactly 5 frames, then process
            let offset = self.frame_count * self.frame_size;
            self.sf_buf[offset..offset + self.frame_size].copy_from_slice(frame);
            self.frame_count += 1;

            if self.frame_count < FRAMES_PER_SUPERFRAME {
                return None;
            }

            self.frame_count = 0;
            let result = self.process_superframe();
            if result.is_none() {
                // Fire code lost — fall back to sliding window search
                debug!("Fire code sync lost, reverting to sliding window search");
                self.synced = false;
                // Keep the buffer contents for sliding
                self.frame_count = FRAMES_PER_SUPERFRAME;
            }
            return result;
        }

        // Unsynced mode: sliding window search
        if self.frame_count < FRAMES_PER_SUPERFRAME {
            // Still filling up the initial 5 frames
            let offset = self.frame_count * self.frame_size;
            self.sf_buf[offset..offset + self.frame_size].copy_from_slice(frame);
            self.frame_count += 1;

            if self.frame_count < FRAMES_PER_SUPERFRAME {
                return None;
            }
        } else {
            // Slide: shift frames 1..4 left, place new frame at position 4
            self.sf_buf
                .copy_within(self.frame_size..FRAMES_PER_SUPERFRAME * self.frame_size, 0);
            let offset = (FRAMES_PER_SUPERFRAME - 1) * self.frame_size;
            self.sf_buf[offset..offset + self.frame_size].copy_from_slice(frame);
        }

        // Try to decode superframe on every incoming frame
        let result = self.process_superframe();
        if result.is_some() {
            // Sync found — switch to synced mode, collect next 5 fresh frames
            debug!("Fire code sync acquired");
            self.synced = true;
            self.frame_count = 0;
        }
        result
    }

    /// Process a complete superframe: fire code check, RS decode, AU extraction.
    fn process_superframe(&mut self) -> Option<Vec<Vec<u8>>> {
        let sf_len = self.sf_buf.len();
        let subch_index = sf_len / RS_BLOCK_LEN;

        // Fire Code sync check (before RS — cheap pre-filter).
        // On a clean signal the header bytes are usually error-free,
        // so fire code passes on the raw data. RS corrects the rest.
        if !fire_code_check(&self.sf_buf) {
            // Debug: log first few bytes of superframe header to understand why fire code fails
            if self.superframes_decoded == 0 && self.frame_count == 0 {
                debug!(
                    "Fire code check failed. Header bytes: {:02X?}, stored_crc=0x{:04X}, calc_crc=0x{:04X}",
                    &self.sf_buf[..16.min(self.sf_buf.len())],
                    (self.sf_buf[0] as u16) << 8 | self.sf_buf[1] as u16,
                    fire_code_crc(&self.sf_buf[2..11.min(self.sf_buf.len())])
                );
            }
            return None;
        }

        // Note: synced state is managed by the caller (feed_frame),
        // not here — because this function can return None after fire code
        // passes (e.g. AU plausibility failure), and the caller needs to
        // know whether to transition to synced mode.

        // Reed-Solomon error correction with stride deinterleaving.
        // The superframe is a 120-row × subch_index-column matrix.
        // Each column is one RS(120,110) codeword.
        let mut rs_packet = [0u8; RS_BLOCK_LEN];
        let mut total_corrections = 0usize;
        let mut uncorrectable_count = 0usize;

        for i in 0..subch_index {
            // Stride deinterleave: collect column i
            for (pos, slot) in rs_packet.iter_mut().enumerate() {
                *slot = self.sf_buf[pos * subch_index + i];
            }

            // Try RS correction
            match self.rs_decoder.correct(&rs_packet, None) {
                Ok(corrected) => {
                    let data = corrected.data();
                    let ecc = corrected.ecc();
                    // Count corrections by comparing with original
                    let mut corrections = 0;
                    for pos in 0..RS_DATA {
                        if data[pos] != rs_packet[pos] {
                            corrections += 1;
                            // Write correction back to superframe buffer
                            self.sf_buf[pos * subch_index + i] = data[pos];
                        }
                    }
                    for pos in 0..RS_PARITY {
                        if ecc[pos] != rs_packet[RS_DATA + pos] {
                            corrections += 1;
                            self.sf_buf[(RS_DATA + pos) * subch_index + i] = ecc[pos];
                        }
                    }
                    total_corrections += corrections;
                }
                Err(_) => {
                    uncorrectable_count += 1;
                }
            }
        }

        if total_corrections > 0 {
            debug!("RS corrected {} symbols", total_corrections);
            self.rs_corrections += total_corrections;
        }
        if uncorrectable_count > 0 {
            debug!("RS: {uncorrectable_count}/{subch_index} codewords uncorrectable");
            self.rs_errors += 1;
        }

        self.superframes_decoded += 1;

        // Parse format byte
        let fmt = SuperframeFormat::parse(self.sf_buf[2]);
        self.format = Some(fmt);

        // Parse AU start offsets
        let num_aus = fmt.num_aus();
        let mut au_start = vec![0usize; num_aus + 1];

        // AU #0 starts after the header
        au_start[0] = fmt.au0_start();

        // End of usable data: first 110 rows × subch_index columns
        au_start[num_aus] = subch_index * RS_DATA;

        // Parse 12-bit AU start offsets from header bytes
        au_start[1] = (self.sf_buf[3] as usize) << 4 | (self.sf_buf[4] as usize) >> 4;
        if num_aus >= 3 {
            au_start[2] = ((self.sf_buf[4] & 0x0F) as usize) << 8 | self.sf_buf[5] as usize;
        }
        if num_aus >= 4 {
            au_start[3] = (self.sf_buf[6] as usize) << 4 | (self.sf_buf[7] as usize) >> 4;
        }
        if num_aus == 6 {
            au_start[4] = ((self.sf_buf[7] & 0x0F) as usize) << 8 | self.sf_buf[8] as usize;
            au_start[5] = (self.sf_buf[9] as usize) << 4 | (self.sf_buf[10] as usize) >> 4;
        }

        // Plausibility check: offsets must be strictly increasing
        for i in 0..num_aus {
            if au_start[i] >= au_start[i + 1] {
                debug!(
                    "AU offset plausibility check failed: au_start[{}]={} >= au_start[{}]={}",
                    i,
                    au_start[i],
                    i + 1,
                    au_start[i + 1]
                );
                return None;
            }
        }

        // Extract AUs with CRC validation
        let mut aus = Vec::with_capacity(num_aus);
        for i in 0..num_aus {
            let start = au_start[i];
            let end = au_start[i + 1];
            if end > sf_len || end < start + 2 {
                debug!("AU #{}: invalid range {}..{}", i, start, end);
                continue;
            }

            let au_data = &self.sf_buf[start..end];
            let au_len = au_data.len();

            // Last 2 bytes are CRC-16-CCITT
            let stored_crc = (au_data[au_len - 2] as u16) << 8 | au_data[au_len - 1] as u16;
            let calced_crc = crc16_ccitt(&au_data[..au_len - 2]);

            if stored_crc != calced_crc {
                debug!(
                    "AU #{}: CRC mismatch (stored=0x{:04X}, calc=0x{:04X})",
                    i, stored_crc, calced_crc
                );
                self.au_crc_errors += 1;
                continue; // Skip this AU
            }

            // Strip CRC, push raw AU data
            aus.push(au_data[..au_len - 2].to_vec());
        }

        if aus.is_empty() { None } else { Some(aus) }
    }

    /// Reset sync state (e.g., after seek or channel change).
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.frame_count = 0;
        self.synced = false;
        self.format = None;
    }
}

// ---------------------------------------------------------------------------
// AacDecoder: wrapper around fdk-aac
// ---------------------------------------------------------------------------

/// HE-AAC v2 decoder for DAB+ audio.
pub struct AacDecoder {
    decoder: fdk_aac::dec::Decoder,
    /// PCM output buffer (sized for one decoded frame).
    pcm_buf: Vec<i16>,
    /// Whether the decoder has been configured.
    configured: bool,
    /// Output sample rate (from stream info).
    pub sample_rate: u32,
    /// Output channels.
    pub channels: usize,
}

impl AacDecoder {
    /// Create a new AAC decoder.
    pub fn new() -> Self {
        let decoder = fdk_aac::dec::Decoder::new(fdk_aac::dec::Transport::Raw);
        Self {
            decoder,
            // Large enough for one decoded frame: 2048 samples × 2 channels × 2 (SBR)
            pcm_buf: vec![0i16; 8192],
            configured: false,
            sample_rate: 0,
            channels: 0,
        }
    }

    /// Configure the decoder with the AudioSpecificConfig from the DAB+ format byte.
    pub fn configure(&mut self, asc: &[u8]) -> Result<(), String> {
        self.decoder
            .config_raw(asc)
            .map_err(|e| format!("fdk-aac config_raw failed: {:?}", e))?;
        self.configured = true;
        Ok(())
    }

    /// Decode one AAC Access Unit into PCM f32 samples.
    ///
    /// Returns interleaved f32 samples (L, R, L, R, ... for stereo).
    pub fn decode_au(&mut self, au: &[u8]) -> Option<Vec<f32>> {
        if !self.configured {
            return None;
        }

        // Fill the decoder's internal buffer
        match self.decoder.fill(au) {
            Ok(_) => {}
            Err(e) => {
                debug!("fdk-aac fill error: {:?}", e);
                return None;
            }
        }

        // Decode one frame
        match self.decoder.decode_frame(&mut self.pcm_buf) {
            Ok(()) => {}
            Err(e) => {
                debug!("fdk-aac decode error: {:?}", e);
                return None;
            }
        }

        // Get actual output size
        let frame_size = self.decoder.decoded_frame_size();
        if frame_size == 0 {
            return None;
        }

        // Update stream info
        let info = self.decoder.stream_info();
        self.sample_rate = info.sampleRate as u32;
        self.channels = info.numChannels as usize;

        // Convert i16 → f32
        let samples: Vec<f32> = self.pcm_buf[..frame_size]
            .iter()
            .map(|&s| s as f32 / 32768.0)
            .collect();

        Some(samples)
    }
}

// ---------------------------------------------------------------------------
// DabPlusDecoder: full pipeline combining SuperframeDecoder + AacDecoder
// ---------------------------------------------------------------------------

/// Complete DAB+ audio decoder.
///
/// Feed it MSC logical frames and get back PCM audio samples.
pub struct DabPlusDecodeOutput {
    pub pcm: Vec<f32>,
    pub metadata: Vec<PadData>,
}

pub struct DabPlusDecoder {
    pub superframe: SuperframeDecoder,
    pub aac: AacDecoder,
    /// Whether AAC decoder has been configured with ASC.
    aac_configured: bool,
    pad: PadExtractor,
}

impl DabPlusDecoder {
    /// Create a new DAB+ decoder for the given bitrate.
    pub fn new(bitrate_kbps: u16) -> Self {
        Self {
            superframe: SuperframeDecoder::new(bitrate_kbps),
            aac: AacDecoder::new(),
            aac_configured: false,
            pad: PadExtractor::new(),
        }
    }

    /// Feed one MSC logical frame.
    ///
    /// Returns decoded PCM f32 samples (interleaved stereo at 48 kHz typically)
    /// whenever a complete superframe's worth of audio is decoded.
    #[allow(dead_code)]
    pub fn feed_frame(&mut self, frame: &[u8]) -> Vec<f32> {
        self.feed_frame_with_metadata(frame).pcm
    }

    /// Feed one MSC logical frame and return audio + extracted metadata candidates.
    pub fn feed_frame_with_metadata(&mut self, frame: &[u8]) -> DabPlusDecodeOutput {
        let mut pcm_out = Vec::new();
        let mut metadata = Vec::new();

        let aus = match self.superframe.feed_frame(frame) {
            Some(aus) => aus,
            None => {
                return DabPlusDecodeOutput {
                    pcm: pcm_out,
                    metadata,
                };
            }
        };

        // Configure AAC decoder on first successful superframe
        if !self.aac_configured
            && let Some(fmt) = self.superframe.format
        {
            let asc = fmt.audio_specific_config();
            debug!(
                "DAB+ format: dac_rate={}, sbr={}, ch_mode={}, ps={}, {}Hz {}ch, ASC={:02X?}",
                fmt.dac_rate,
                fmt.sbr_flag,
                fmt.aac_channel_mode,
                fmt.ps_flag,
                fmt.sample_rate(),
                fmt.channels(),
                &asc
            );
            match self.aac.configure(&asc) {
                Ok(()) => {
                    self.aac_configured = true;
                    info!(
                        sample_rate = fmt.sample_rate(),
                        channels = fmt.channels(),
                        aus_per_superframe = fmt.num_aus(),
                        "DAB+ audio configured"
                    );
                }
                Err(e) => {
                    warn!("Failed to configure AAC decoder: {}", e);
                }
            }
        }

        // Decode each AU
        for au in &aus {
            metadata.extend(self.pad.extract_all_from_au(au));
            if let Some(samples) = self.aac.decode_au(au) {
                pcm_out.extend_from_slice(&samples);
            }
        }

        DabPlusDecodeOutput {
            pcm: pcm_out,
            metadata,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fire_code_lut() {
        // The LUT should have 256 entries, all computed from polynomial 0x782F
        assert_eq!(FIRE_CODE_LUT.len(), 256);
        // First entry (byte=0) should be 0
        assert_eq!(FIRE_CODE_LUT[0], 0);
        // Entry for byte=1 should be 0x782F (the polynomial itself)
        assert_eq!(FIRE_CODE_LUT[1], 0x782F);
    }

    #[test]
    fn test_ccitt_lut() {
        assert_eq!(CCITT_LUT.len(), 256);
        assert_eq!(CCITT_LUT[0], 0);
        assert_eq!(CCITT_LUT[1], 0x1021);
    }

    #[test]
    fn test_fire_code_rejects_zeros() {
        // All-zero superframe header should not pass fire code check
        let sf = vec![0u8; 120];
        assert!(!fire_code_check(&sf));
    }

    #[test]
    fn test_superframe_format_parse() {
        // 0x60 = dac_rate=1, sbr_flag=1, ch=0, ps=0
        let fmt = SuperframeFormat::parse(0x60);
        assert!(fmt.dac_rate);
        assert!(fmt.sbr_flag);
        assert!(!fmt.aac_channel_mode);
        assert!(!fmt.ps_flag);
        assert_eq!(fmt.num_aus(), 3);
        assert_eq!(fmt.au0_start(), 6);
        assert_eq!(fmt.sample_rate(), 48000);

        // 0x78 = dac_rate=1, sbr_flag=1, ch=1, ps=1
        let fmt2 = SuperframeFormat::parse(0x78);
        assert!(fmt2.dac_rate);
        assert!(fmt2.sbr_flag);
        assert!(fmt2.aac_channel_mode);
        assert!(fmt2.ps_flag);
        assert_eq!(fmt2.channels(), 2);

        // 0x00 = dac_rate=0, sbr=0, ch=0, ps=0 → 32kHz AAC-LC mono
        let fmt3 = SuperframeFormat::parse(0x00);
        assert_eq!(fmt3.num_aus(), 4);
        assert_eq!(fmt3.au0_start(), 8);
        assert_eq!(fmt3.sample_rate(), 32000);
        assert_eq!(fmt3.channels(), 1);
    }

    #[test]
    fn test_audio_specific_config_he_aac_v2() {
        // HE-AAC v2 stereo: dac_rate=1, sbr=1, ch=1, ps=1
        let fmt = SuperframeFormat::parse(0x78);
        let asc = fmt.audio_specific_config();
        // Should be 7 bytes: 2 (core) + 3 (SBR ext) + 2 (PS ext)
        assert_eq!(asc.len(), 7);
        // First byte: 00010 (AOT=2) + sr_index[3:1]
        // sr_index for dac_rate=1,sbr=1 = 6 (24000 Hz) = 0b0110
        // 00010_011 = 0x13
        assert_eq!(asc[0], 0x13);
    }

    #[test]
    fn test_audio_specific_config_aac_lc() {
        // AAC-LC mono: dac_rate=1, sbr=0, ch=0, ps=0
        let fmt = SuperframeFormat::parse(0x40);
        let asc = fmt.audio_specific_config();
        // Should be 2 bytes (no SBR/PS extensions)
        assert_eq!(asc.len(), 2);
        assert_eq!(fmt.num_aus(), 6);
        assert_eq!(fmt.sample_rate(), 48000);
    }

    #[test]
    fn test_superframe_decoder_frame_accumulation() {
        // 88 kbps → 264 bytes/frame → 1320 bytes/superframe → subch_index=11
        let mut dec = SuperframeDecoder::new(88);
        assert_eq!(dec.frame_size, 264);
        assert_eq!(dec.sf_buf.len(), 1320);

        // Feeding 4 frames should return None
        let dummy_frame = vec![0u8; 264];
        for _ in 0..4 {
            assert!(dec.feed_frame(&dummy_frame).is_none());
        }
        // 5th frame completes superframe but fire code won't pass on zeros
        assert!(dec.feed_frame(&dummy_frame).is_none());
    }

    #[test]
    fn test_rs_decoder_basic() {
        // Verify we can encode and decode with the reed-solomon crate
        let enc = reed_solomon::Encoder::new(RS_PARITY);
        let data = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        let encoded = enc.encode(&data);
        assert_eq!(encoded.data(), &data);

        // Corrupt one byte
        let mut corrupted = encoded.to_vec();
        corrupted[3] = 0xFF;

        let dec = RsDecoder::new(RS_PARITY);
        let corrected = dec.correct(&corrupted, None).unwrap();
        assert_eq!(corrected.data(), &data);
    }

    #[test]
    fn test_crc16_ccitt_known_value() {
        // CRC-16-CCITT (init=0xFFFF, final XOR=0xFFFF, poly=0x1021)
        // For "123456789": CRC-16/CCITT-FALSE gives 0x29B1, then XOR 0xFFFF = 0xD64E
        let data = b"123456789";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, 0xD64E);
    }

    #[test]
    fn test_fire_code_self_consistency() {
        // Build a header where we compute the fire code and store it
        let mut sf = vec![0u8; 120];
        // Set some non-zero content in the format/AU offset bytes
        sf[2] = 0x78; // format byte
        sf[3] = 0x00;
        sf[4] = 0x1A; // AU offset data
        sf[5] = 0x03;
        sf[6] = 0x5B;
        sf[7] = 0x00;
        sf[8] = 0x98;
        sf[9] = 0x01;
        sf[10] = 0x20;

        // Compute fire code over sf[2..11] and store in sf[0..2]
        let crc = fire_code_crc(&sf[2..11]);
        sf[0] = (crc >> 8) as u8;
        sf[1] = crc as u8;

        assert!(fire_code_check(&sf));

        // Corrupt one byte → should fail
        sf[5] ^= 0xFF;
        assert!(!fire_code_check(&sf));
    }
}
