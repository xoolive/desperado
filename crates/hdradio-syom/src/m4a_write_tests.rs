//! M4A writer structure: the shipped isomp4 reader is the oracle.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::mux_aac_lc;
use crate::engine::adts::ADTS_SAMPLE_RATES_HZ;
use crate::engine::asc::AudioSpecificConfig;
use crate::error::Result;

/// Three fake payloads; the muxer treats them as opaque sample bytes.
fn payloads() -> Vec<Vec<u8>> {
    vec![vec![0xAA; 100], vec![0xBB; 137], vec![0xCC; 90]]
}

#[test]
fn mux_parses_back_structurally() -> Result<()> {
    let p = payloads();
    let m4a = mux_aac_lc(&p, 3, 2, 48_000)?;
    assert!(crate::isomp4::sniff_is_isobmff(&m4a));
    let track = crate::isomp4::parse_aac_track(&m4a)?;
    let (asc, _) = AudioSpecificConfig::parse(&track.asc)?;
    assert_eq!(asc.aot, 2);
    assert_eq!(asc.sampling_frequency_index, 3);
    assert_eq!(asc.channel_configuration, 2);
    assert_eq!(asc.sample_rate, 48_000);
    assert_eq!(track.frames.len(), p.len());
    // `total_samples` is Σ stts.sample_count — the frame count for the
    // single-entry table we (and ffmpeg) write.
    assert_eq!(track.total_samples, 3);
    assert_eq!(track.edit_start, 1024, "priming via elst media_time");
    assert_eq!(track.media_timescale, 48_000);
    for (frame, want) in track.frames.iter().zip(&p) {
        let (off, len) = *frame;
        assert_eq!(len as usize, want.len());
        assert_eq!(
            &m4a[off as usize..off as usize + len as usize],
            want.as_slice(),
            "frame bytes must be the payload, in order"
        );
    }
    Ok(())
}

#[test]
fn muxed_frame_bytes_roundtrip_bit_exact() -> Result<()> {
    let p = payloads();
    let m4a = mux_aac_lc(&p, 7, 1, 22_050)?;
    let track = crate::isomp4::parse_aac_track(&m4a)?;
    let joined: Vec<u8> = track
        .frames
        .iter()
        .flat_map(|&(off, len)| m4a[off as usize..off as usize + len as usize].to_vec())
        .collect();
    let want: Vec<u8> = p.iter().flatten().copied().collect();
    assert_eq!(joined, want);
    assert_eq!(track.media_timescale, ADTS_SAMPLE_RATES_HZ[7]);
    Ok(())
}
