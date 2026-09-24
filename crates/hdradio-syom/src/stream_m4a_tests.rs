//! Streaming API: M4A/ISOBMFF slice decode — elst parity with one-shot.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::stream_tests::{assert_eq_collected, collect_slice};
use super::{DecodeOptions, Result};

fn m4a_cases() -> [(&'static [u8], &'static str); 5] {
    [
        (&include_bytes!("goldens/he48.m4a")[..], "he48"),
        (&include_bytes!("goldens/ps48.m4a")[..], "ps48"),
        (&include_bytes!("goldens/sine441.m4a")[..], "sine441"),
        (&include_bytes!("goldens/lecture.m4a")[..], "lecture"),
        (&include_bytes!("goldens/mc51.m4a")[..], "mc51"),
    ]
}

#[test]
fn m4a_slice_streaming_matches_oneshot() -> Result<()> {
    for (data, label) in m4a_cases() {
        for opts in [DecodeOptions::speech(), DecodeOptions::unbounded()] {
            let one = crate::decode_with(data, &opts)?;
            let s = collect_slice(data, &opts)?;
            // Bit-identical planes: the elst skip-left walk replaces the
            // one-shot post-hoc shift and must land on the same samples.
            assert_eq_collected(&one, &s, label);
        }
    }
    Ok(())
}

#[test]
fn m4a_stream_info_matches_elst_trimmed_length() -> Result<()> {
    for (data, label) in m4a_cases() {
        let opts = DecodeOptions::unbounded();
        let one = crate::decode_with(data, &opts)?;
        let s = collect_slice(data, &opts)?;
        assert_eq!(
            s.info.samples as usize,
            one.channels[0].len(),
            "{label} samples after elst skip"
        );
        assert!(s.info.aac_frames > 0, "{label} frames");
    }
    Ok(())
}

#[test]
fn m4a_caps_match_oneshot() {
    let data = &include_bytes!("goldens/sine441.m4a")[..];
    let opts = DecodeOptions::speech().with_max_sample_rate(8_000);
    let one = crate::decode_with(data, &opts);
    let mut tracks: Vec<Vec<f32>> = Vec::new();
    let stream = crate::decode_streaming(data, &opts, |f| {
        for (d, s) in tracks.iter_mut().zip(f.planar.iter()) {
            d.extend_from_slice(s);
        }
        Ok(())
    });
    assert!(matches!(
        one,
        Err(crate::AacError::UnsupportedSampleRate { .. })
    ));
    assert!(matches!(
        stream,
        Err(crate::AacError::UnsupportedSampleRate { .. })
    ));
}
