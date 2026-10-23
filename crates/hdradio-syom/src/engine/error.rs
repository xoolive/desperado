//! Engine errors. ISO/IEC 14496-3 / 13818-7 parse and tool failures.

use std::fmt;

/// Local engine result.
pub type Result<T> = core::result::Result<T, Error>;

/// Failures while parsing or reconstructing AAC-LC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Bitstream ended mid-field.
    UnexpectedEnd,
    /// ADTS syncword was not `0xFFF` (§1.A.2.2.1).
    AdtsSyncNotFound,
    /// ADTS `layer` was not zero.
    AdtsLayerNonZero,
    /// Reserved ADTS `sampling_frequency_index` (13/14) or illegal 15.
    AdtsReservedSampleRateIndex,
    /// `aac_frame_length` smaller than the header.
    AdtsFrameLengthTooSmall,
    /// `audioObjectType` is not LC (2) or HE-AAC (5/29).
    UnsupportedAot(u8),
    /// `frameLengthFlag == 1` (960-line) is out of v1.
    UnsupportedFrameLength,
    /// `samplingFrequencyIndex` has no SWB table.
    UnsupportedSampleRateIndex(u8),
    /// Malformed `ics_info` / grouping / `max_sfb`.
    IcsInfoInvalid,
    /// `section_data` ran past `max_sfb`.
    SectionDataOverrun,
    /// Huffman codebook index with no table (12, or >15).
    InvalidCodebook(u8),
    /// Huffman prefix did not match a complete codeword.
    HuffmanInvalid,
    /// Spectral / pulse / TNS geometry walked off the window.
    SpectrumInvalid,
    /// Filterbank `spec` length disagrees with `window_sequence`.
    FilterbankInvalid,
    /// Generic structural reject (PCE/LTP/predictor on the LC path).
    Format(&'static str),
    /// LOAS syncword was not `0x2B7`.
    LoasSyncInvalid,
    /// LATM `audioMuxVersionA == 1` is reserved.
    LatmAudioMuxVersionAReserved,
    /// LATM program/layer geometry is outside the AAC-single-stream subset.
    LatmConfigOutOfRange,
    /// LATM CRC mismatch.
    LatmCrcMismatch,
    /// LATM payload without a prior `StreamMuxConfig`.
    LatmNoPreviousMuxConfig,
    /// LATM `frameLengthType` is CELP/HVXC (not AAC).
    LatmUnsupportedFrameLengthType,
    /// SBR QMF geometry invalid.
    SbrQmfInvalid,
    /// SBR frequency tables invalid.
    SbrFreqBandInvalid,
    /// SBR grid invalid.
    SbrGridInvalid,
    /// SBR Huffman invalid.
    SbrHuffInvalid,
    /// Parametric stereo payload invalid.
    PsDataInvalid,
    /// FIL `extension_payload` SBR type without a body (legacy skip).
    #[allow(dead_code)]
    UnsupportedExtensionSbr(u8),
    /// FIL reserved `extension_type`.
    UnsupportedExtensionType(u8),
    /// FIL `extension_payload` structural error.
    ExtensionPayloadInvalid,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnexpectedEnd => write!(f, "aac: unexpected end of bitstream"),
            Self::AdtsSyncNotFound => write!(f, "aac: ADTS sync not found"),
            Self::AdtsLayerNonZero => write!(f, "aac: ADTS layer must be 0"),
            Self::AdtsReservedSampleRateIndex => write!(f, "aac: reserved ADTS sample rate"),
            Self::AdtsFrameLengthTooSmall => write!(f, "aac: ADTS frame length too small"),
            Self::UnsupportedAot(a) => write!(f, "aac: unsupported audioObjectType {a}"),
            Self::UnsupportedFrameLength => write!(f, "aac: 960-line frames are Media"),
            Self::UnsupportedSampleRateIndex(i) => {
                write!(f, "aac: no SWB table for samplingFrequencyIndex {i}")
            }
            Self::IcsInfoInvalid => write!(f, "aac: invalid ics_info"),
            Self::SectionDataOverrun => write!(f, "aac: section_data overrun"),
            Self::InvalidCodebook(c) => write!(f, "aac: invalid codebook {c}"),
            Self::HuffmanInvalid => write!(f, "aac: Huffman codeword invalid"),
            Self::SpectrumInvalid => write!(f, "aac: spectral geometry invalid"),
            Self::FilterbankInvalid => write!(f, "aac: filterbank length mismatch"),
            Self::Format(msg) => write!(f, "aac: {msg}"),
            Self::LoasSyncInvalid => write!(f, "aac: LOAS sync invalid"),
            Self::LatmAudioMuxVersionAReserved => write!(f, "aac: LATM audioMuxVersionA reserved"),
            Self::LatmConfigOutOfRange => write!(f, "aac: LATM config out of range"),
            Self::LatmCrcMismatch => write!(f, "aac: LATM CRC mismatch"),
            Self::LatmNoPreviousMuxConfig => write!(f, "aac: LATM missing StreamMuxConfig"),
            Self::LatmUnsupportedFrameLengthType => {
                write!(f, "aac: LATM frameLengthType not AAC")
            }
            Self::SbrQmfInvalid => write!(f, "aac: SBR QMF invalid"),
            Self::SbrFreqBandInvalid => write!(f, "aac: SBR frequency table invalid"),
            Self::SbrGridInvalid => write!(f, "aac: SBR grid invalid"),
            Self::SbrHuffInvalid => write!(f, "aac: SBR Huffman invalid"),
            Self::PsDataInvalid => write!(f, "aac: PS data invalid"),
            Self::UnsupportedExtensionSbr(t) => write!(f, "aac: SBR extension_type {t}"),
            Self::UnsupportedExtensionType(t) => write!(f, "aac: extension_type {t}"),
            Self::ExtensionPayloadInvalid => write!(f, "aac: extension_payload invalid"),
        }
    }
}

impl std::error::Error for Error {}
