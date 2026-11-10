//! ADTS fixed + variable header — ISO/IEC 13818-7 §1.A.2.2.

use super::bits::{BitReader, BitWriter};
use super::error::{Error, Result};

/// 12-bit ADTS syncword.
pub const ADTS_SYNCWORD: u16 = 0x0FFF;
/// Header bytes when `protection_absent == 1`.
pub const ADTS_HEADER_BYTES_NO_CRC: usize = 7;
/// Header bytes when a 16-bit CRC follows the variable header.
pub const ADTS_HEADER_BYTES_WITH_CRC: usize = 9;

/// ISO/IEC 14496-3 Table 1.18 — `samplingFrequencyIndex` 0..=12.
pub const ADTS_SAMPLE_RATES_HZ: [u32; 13] = [
    96_000, 88_200, 64_000, 48_000, 44_100, 32_000, 24_000, 22_050, 16_000, 12_000, 11_025, 8_000,
    7_350,
];

/// Parsed ADTS header. `profile_ObjectType` on the wire is AOT − 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdtsHeader {
    /// MPEG-2 (`true`) vs MPEG-4 (`false`) ID bit.
    pub mpeg_version_mpeg2: bool,
    /// `true` ⇒ no CRC; payload starts at byte 7.
    pub protection_absent: bool,
    /// Wire `profile_ObjectType` (0=Main, 1=LC, 2=SSR, 3=LTP).
    pub profile: u8,
    /// Table 1.18 index.
    pub sampling_frequency_index: u8,
    /// Table 1.19 channel configuration.
    pub channel_configuration: u8,
    /// Total frame length in bytes, including this header.
    pub aac_frame_length: u16,
    /// Buffer fullness; `0x7FF` means VBR / unknown.
    pub adts_buffer_fullness: u16,
    /// Number of `raw_data_block()` payloads (wire field is N−1).
    pub number_of_raw_data_blocks_in_frame: u8,
}

impl AdtsHeader {
    /// Parse the 56-bit header. Returns the header and payload byte offset.
    pub fn parse(data: &[u8]) -> Result<(Self, usize)> {
        if data.len() < ADTS_HEADER_BYTES_NO_CRC {
            return Err(Error::UnexpectedEnd);
        }
        let mut br = BitReader::new(data);
        let sync = br.read(12)? as u16;
        if sync != ADTS_SYNCWORD {
            return Err(Error::AdtsSyncNotFound);
        }
        let mpeg_version_mpeg2 = br.read_bit()?;
        let layer = br.read(2)?;
        if layer != 0 {
            return Err(Error::AdtsLayerNonZero);
        }
        let protection_absent = br.read_bit()?;
        let profile = br.read(2)? as u8;
        let sampling_frequency_index = br.read(4)? as u8;
        if sampling_frequency_index >= 13 {
            return Err(Error::AdtsReservedSampleRateIndex);
        }
        let _private_bit = br.read_bit()?;
        let channel_configuration = br.read(3)? as u8;
        let _original_copy = br.read_bit()?;
        let _home = br.read_bit()?;
        let _copyright_identification_bit = br.read_bit()?;
        let _copyright_identification_start = br.read_bit()?;
        let aac_frame_length = br.read(13)? as u16;
        let adts_buffer_fullness = br.read(11)? as u16;
        let number_of_raw_data_blocks_in_frame = br.read(2)? as u8 + 1;
        let payload_offset = if protection_absent {
            ADTS_HEADER_BYTES_NO_CRC
        } else {
            ADTS_HEADER_BYTES_WITH_CRC
        };
        if (aac_frame_length as usize) < payload_offset {
            return Err(Error::AdtsFrameLengthTooSmall);
        }
        if !protection_absent && data.len() < ADTS_HEADER_BYTES_WITH_CRC {
            return Err(Error::UnexpectedEnd);
        }
        Ok((
            AdtsHeader {
                mpeg_version_mpeg2,
                protection_absent,
                profile,
                sampling_frequency_index,
                channel_configuration,
                aac_frame_length,
                adts_buffer_fullness,
                number_of_raw_data_blocks_in_frame,
            },
            payload_offset,
        ))
    }

    /// Table 1.18 rate in Hz.
    #[must_use]
    pub fn sample_rate(&self) -> u32 {
        ADTS_SAMPLE_RATES_HZ
            .get(self.sampling_frequency_index as usize)
            .copied()
            .unwrap_or(0)
    }
    /// `audioObjectType` = wire profile + 1 (§1.A.2).
    #[must_use]
    pub fn audio_object_type(&self) -> u8 {
        self.profile + 1
    }

    /// Serialize as the 56-bit no-CRC header (`protection_absent` on the wire
    /// is always 1; the encoder never writes a CRC). Exactly 7 bytes.
    #[must_use]
    pub fn write(&self) -> [u8; ADTS_HEADER_BYTES_NO_CRC] {
        let mut w = BitWriter::new();
        w.write(u32::from(ADTS_SYNCWORD), 12);
        w.write_bit(self.mpeg_version_mpeg2);
        w.write(0, 2); // layer
        w.write_bit(true); // protection_absent
        w.write(u32::from(self.profile), 2);
        w.write(u32::from(self.sampling_frequency_index), 4);
        w.write_bit(false); // private_bit
        w.write(u32::from(self.channel_configuration), 3);
        w.write(0, 4); // original/copy, home, copyright bits
        w.write(u32::from(self.aac_frame_length), 13);
        w.write(u32::from(self.adts_buffer_fullness), 11);
        w.write(
            u32::from(self.number_of_raw_data_blocks_in_frame.saturating_sub(1)),
            2,
        );
        let bytes = w.finish();
        let mut out = [0u8; ADTS_HEADER_BYTES_NO_CRC];
        out.copy_from_slice(&bytes);
        out
    }
}

#[cfg(test)]
#[path = "adts_tests.rs"]
mod adts_tests;
