//! `AacError` Display / source / format-class.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{AacError, Result};
use std::io;

#[test]
fn display_and_source_cover_variants() -> Result<()> {
    let io = AacError::from(io::Error::other("x"));
    assert!(io.to_string().contains('x'));
    assert!(std::error::Error::source(&io).is_some());
    assert_eq!(AacError::NotAac.to_string(), "Unsupported audio format");
    assert!(AacError::sample_rate(8, 4).to_string().contains("8Hz"));
    assert!(AacError::too_long(9.0, 1.0).to_string().contains("9s"));
    assert_eq!(AacError::format("f").to_string(), "f");
    assert_eq!(AacError::decode("d").to_string(), "d");
    assert!(AacError::NotAac.is_format_class());
    assert!(AacError::format("f").is_format_class());
    assert!(!AacError::decode("d").is_format_class());
    Ok(())
}

#[test]
fn engine_error_display_covers_variants() {
    use crate::engine::error::Error;
    let all = [
        Error::UnexpectedEnd,
        Error::AdtsSyncNotFound,
        Error::AdtsLayerNonZero,
        Error::AdtsReservedSampleRateIndex,
        Error::AdtsFrameLengthTooSmall,
        Error::UnsupportedAot(5),
        Error::UnsupportedFrameLength,
        Error::UnsupportedSampleRateIndex(15),
        Error::IcsInfoInvalid,
        Error::SectionDataOverrun,
        Error::InvalidCodebook(12),
        Error::HuffmanInvalid,
        Error::SpectrumInvalid,
        Error::FilterbankInvalid,
        Error::Format("x"),
        Error::LoasSyncInvalid,
        Error::LatmAudioMuxVersionAReserved,
        Error::LatmConfigOutOfRange,
        Error::LatmCrcMismatch,
        Error::LatmNoPreviousMuxConfig,
        Error::LatmUnsupportedFrameLengthType,
        Error::SbrQmfInvalid,
        Error::SbrFreqBandInvalid,
        Error::SbrGridInvalid,
        Error::SbrHuffInvalid,
        Error::PsDataInvalid,
        Error::UnsupportedExtensionSbr(0xd),
        Error::UnsupportedExtensionType(2),
        Error::ExtensionPayloadInvalid,
    ];
    for e in all {
        assert!(e.to_string().starts_with("aac:"));
        let _ = std::error::Error::source(&e);
    }
}
