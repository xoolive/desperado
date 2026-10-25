//! Streaming API: ADTS equivalence with one-shot decode, chunking, caps.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{AacError, DecodeOptions, Decoder, Frame, Result, StreamInfo, decode_streaming};

/// PCM collected from frame callbacks plus the final tallies.
pub(crate) struct Collected {
    pub rate: u32,
    pub tracks: Vec<Vec<f32>>,
    pub info: StreamInfo,
}

fn push_frame_into(tracks: &mut Vec<Vec<f32>>, f: Frame<'_>) -> Result<()> {
    if tracks.is_empty() {
        tracks.resize_with(f.planar.len(), Vec::new);
    }
    assert_eq!(tracks.len(), f.planar.len(), "plane count changed");
    for (dst, src) in tracks.iter_mut().zip(f.planar.iter()) {
        assert_eq!(src.len(), f.samples, "frame.samples disagrees with plane");
        dst.extend_from_slice(src);
    }
    Ok(())
}

/// `decode_streaming` over the whole slice.
pub(crate) fn collect_slice(data: &[u8], opts: &DecodeOptions) -> Result<Collected> {
    let mut tracks: Vec<Vec<f32>> = Vec::new();
    let info = decode_streaming(data, opts, |f| push_frame_into(&mut tracks, f))?;
    Ok(Collected {
        rate: info.sample_rate,
        tracks,
        info,
    })
}

/// Push `Decoder` fed in `chunk`-byte pieces (0 = one whole feed).
pub(crate) fn collect_push(data: &[u8], opts: &DecodeOptions, chunk: usize) -> Result<Collected> {
    let mut dec = Decoder::new(opts.clone());
    let mut tracks: Vec<Vec<f32>> = Vec::new();
    let mut cb = |f: Frame<'_>| push_frame_into(&mut tracks, f);
    if chunk == 0 {
        dec.feed(data, &mut cb)?;
    } else {
        for piece in data.chunks(chunk) {
            dec.feed(piece, &mut cb)?;
        }
    }
    let info = dec.finish(&mut cb)?;
    Ok(Collected {
        rate: info.sample_rate,
        tracks,
        info,
    })
}

pub(crate) fn assert_eq_collected(one: &crate::DecodedAac, s: &Collected, label: &str) {
    assert_eq!(one.sample_rate, s.rate, "{label} sample rate");
    assert_eq!(one.channels.len(), s.tracks.len(), "{label} channels");
    assert_eq!(one.channels, s.tracks, "{label} pcm not bit-identical");
    assert_eq!(
        one.channels.first().map_or(0, Vec::len) as u64,
        s.info.samples,
        "{label} StreamInfo.samples"
    );
    assert!(s.info.aac_frames > 0, "{label} no frames counted");
}

pub(crate) fn adts_cases() -> [(&'static [u8], &'static str); 9] {
    [
        (&include_bytes!("goldens/sine48.adts")[..], "sine48"),
        (&include_bytes!("goldens/tns48.adts")[..], "tns48"),
        (&include_bytes!("goldens/pns48.adts")[..], "pns48"),
        (&include_bytes!("goldens/he48.adts")[..], "he48"),
        (&include_bytes!("goldens/ps48.adts")[..], "ps48"),
        (&include_bytes!("goldens/mc30.adts")[..], "mc30"),
        (&include_bytes!("goldens/mc40.adts")[..], "mc40"),
        (&include_bytes!("goldens/mc50.adts")[..], "mc50"),
        (&include_bytes!("goldens/mc51.adts")[..], "mc51"),
    ]
}

fn both_modes() -> [DecodeOptions; 2] {
    [DecodeOptions::speech(), DecodeOptions::unbounded()]
}

#[test]
fn adts_slice_streaming_matches_oneshot() -> Result<()> {
    for (data, label) in adts_cases() {
        for opts in both_modes() {
            let one = crate::decode_with(data, &opts)?;
            let s = collect_slice(data, &opts)?;
            assert_eq_collected(&one, &s, label);
        }
    }
    Ok(())
}

#[test]
fn adts_push_chunked_matches_oneshot() -> Result<()> {
    for (data, label) in adts_cases() {
        for opts in both_modes() {
            let one = crate::decode_with(data, &opts)?;
            for chunk in [0usize, 1, 7, 64, 4096] {
                let s = collect_push(data, &opts, chunk)?;
                assert_eq_collected(&one, &s, &format!("{label} chunk={chunk}"));
            }
        }
    }
    Ok(())
}

#[test]
fn adts_push_split_at_frame_boundaries() -> Result<()> {
    use crate::engine::adts::AdtsHeader;
    for (data, label) in adts_cases() {
        let opts = DecodeOptions::unbounded();
        let one = crate::decode_with(data, &opts)?;
        let mut dec = Decoder::new(opts.clone());
        let mut tracks: Vec<Vec<f32>> = Vec::new();
        let mut cb = |f: Frame<'_>| push_frame_into(&mut tracks, f);
        let mut pos = 0usize;
        let mut frames = 0u64;
        while pos < data.len() {
            let (hdr, _) = AdtsHeader::parse(&data[pos..])
                .map_err(|e| AacError::decode(format!("{label} hdr: {e:?}")))?;
            let fl = usize::from(hdr.aac_frame_length);
            dec.feed(&data[pos..pos + fl], &mut cb)?;
            frames += 1;
            pos += fl;
        }
        let info = dec.finish(&mut cb)?;
        assert_eq!(info.aac_frames, frames, "{label} aac_frames");
        assert_eq!(one.channels, tracks, "{label} frame-boundary pcm");
    }
    Ok(())
}

#[test]
fn adts_truncated_tail_parity() -> Result<()> {
    let data = &include_bytes!("goldens/sine48.adts")[..];
    for cut in [1usize, 5, 100] {
        let t = &data[..data.len() - cut];
        for opts in both_modes() {
            let one = crate::decode_with(t, &opts)?;
            let s = collect_push(t, &opts, 7)?;
            assert_eq_collected(&one, &s, &format!("cut={cut}"));
        }
    }
    Ok(())
}

#[test]
fn adts_garbage_between_frames_resyncs_like_oneshot() -> Result<()> {
    use crate::engine::adts::AdtsHeader;
    let data = &include_bytes!("goldens/sine48.adts")[..];
    // End of frame 2, walking real (VBR) frame lengths.
    let mut pos = 0usize;
    for _ in 0..2 {
        let (hdr, _) =
            AdtsHeader::parse(&data[pos..]).map_err(|e| AacError::decode(format!("hdr: {e:?}")))?;
        pos += usize::from(hdr.aac_frame_length);
    }
    let mut dirty = data[..pos].to_vec();
    dirty.extend_from_slice(&[0x00, 0x11, 0xFF, 0x00]); // garbage, incl. lone 0xFF
    dirty.extend_from_slice(&data[pos..]);
    let one = crate::decode_with(&dirty, &DecodeOptions::unbounded())?;
    let s = collect_push(&dirty, &DecodeOptions::unbounded(), 3)?;
    assert_eq_collected(&one, &s, "garbage-mid");
    Ok(())
}

#[test]
fn garbage_only_is_not_aac_at_finish() {
    let data = b"this is not aac at all, just junk bytes";
    assert!(matches!(crate::decode(data), Err(AacError::NotAac)));
    let mut dec = Decoder::new(DecodeOptions::speech());
    // Junk is never decidable mid-feed; `finish` delivers NotAac.
    assert!(dec.feed(&data[..4], |_| Ok(())).is_ok());
    assert!(dec.feed(&data[4..], |_| Ok(())).is_ok());
    assert!(matches!(dec.finish(|_| Ok(())), Err(AacError::NotAac)));
}

#[test]
fn adts_garbage_prefix_streams_recover() -> Result<()> {
    // Leading junk is skipped by byte resync (both one-shot and push, now
    // that one-shot layers on the streaming core; pre-streaming one-shot
    // answered NotAac here).
    let data = &include_bytes!("goldens/sine48.adts")[..];
    let mut dirty = vec![0x00, 0x42, 0x13];
    dirty.extend_from_slice(data);
    let one = crate::decode_with(data, &DecodeOptions::unbounded())?;
    let prefixed = crate::decode_with(&dirty, &DecodeOptions::unbounded())?;
    assert_eq!(one.channels, prefixed.channels, "one-shot prefix resync");
    let s = collect_push(&dirty, &DecodeOptions::unbounded(), 5)?;
    assert_eq_collected(&one, &s, "garbage-prefix");
    Ok(())
}

#[test]
fn caps_too_long_fires_mid_stream() {
    let data = &include_bytes!("goldens/sine48.adts")[..];
    let opts = DecodeOptions::speech().with_max_duration_secs(0.01);
    for chunk in [0usize, 64] {
        let mut dec = Decoder::new(opts.clone());
        let r = if chunk == 0 {
            dec.feed(data, |_| Ok(()))
        } else {
            let mut out = Ok(data.len());
            for piece in data.chunks(chunk) {
                out = dec.feed(piece, |_| Ok(()));
                if out.is_err() {
                    break;
                }
            }
            out
        };
        assert!(
            matches!(r, Err(AacError::TooLong { .. })),
            "chunk={chunk}: {r:?}"
        );
    }
}

#[test]
fn rate_ceiling_fires_on_first_frame() {
    let data = &include_bytes!("goldens/sine48.adts")[..];
    let opts = DecodeOptions::speech().with_max_sample_rate(8_000);
    let mut dec = Decoder::new(opts);
    match dec.feed(data, |_| Ok(())) {
        Err(AacError::UnsupportedSampleRate {
            rate: 48_000,
            max: 8_000,
        }) => {}
        other => panic!("expected rate reject, got {other:?}"),
    }
}

#[test]
fn channel_count_change_mid_stream_is_error() {
    let mut cat = include_bytes!("goldens/sine48.adts").to_vec();
    cat.extend_from_slice(&include_bytes!("goldens/mc51.adts")[..]);
    let mut dec = Decoder::new(DecodeOptions::unbounded());
    match dec.feed(&cat, |_| Ok(())) {
        Err(AacError::Decode(msg)) => {
            assert_eq!(msg, "aac: channel count changed mid-stream (1 → 6)");
        }
        other => panic!("expected channel-change error, got {other:?}"),
    }
    // Mono mode collapses both to one plane: no error.
    let s = collect_push(&cat, &DecodeOptions::speech(), 0);
    assert!(s.is_ok(), "mono mix should ride over channel change");
}

#[test]
fn sample_rate_change_mid_stream_is_error() {
    let a = &include_bytes!("goldens/sine48.adts")[..];
    let mut b = a.to_vec();
    let mut pos = 0usize;
    while pos + 7 <= b.len() {
        if b[pos] == 0xFF && (b[pos + 1] & 0xF0) == 0xF0 {
            b[pos + 2] = (b[pos + 2] & 0xC3) | (4 << 2); // 48 kHz -> 44.1 kHz
            let fl = (usize::from(b[pos + 3] & 3) << 11)
                | (usize::from(b[pos + 4]) << 3)
                | (usize::from(b[pos + 5]) >> 5);
            pos += fl.max(7);
        } else {
            pos += 1;
        }
    }
    let mut cat = a.to_vec();
    cat.extend_from_slice(&b);
    let mut dec = Decoder::new(DecodeOptions::unbounded());
    match dec.feed(&cat, |_| Ok(())) {
        Err(AacError::Decode(msg)) => {
            assert_eq!(
                msg,
                "aac: sample rate changed mid-stream (48000Hz → 44100Hz)"
            );
        }
        other => panic!("expected rate-change error, got {other:?}"),
    }
}

#[test]
fn callback_abort_propagates() {
    let data = &include_bytes!("goldens/sine48.adts")[..];
    let mut seen = 0u32;
    let r = decode_streaming(data, &DecodeOptions::speech(), |_| {
        seen += 1;
        if seen >= 2 {
            return Err(AacError::format("consumer stop"));
        }
        Ok(())
    });
    match r {
        Err(AacError::Format(msg)) => assert_eq!(msg, "consumer stop"),
        other => panic!("expected abort, got {other:?}"),
    }
    assert_eq!(seen, 2);
    let mut dec = Decoder::new(DecodeOptions::speech());
    let r = dec.feed(data, |_| Err(AacError::decode("boom")));
    assert!(matches!(r, Err(AacError::Decode(_))));
}

#[test]
fn m4a_push_is_rejected() {
    let m4a = &include_bytes!("goldens/sine441.m4a")[..];
    let mut dec = Decoder::new(DecodeOptions::speech());
    assert_eq!(dec.feed(&m4a[..4], |_| Ok(())).ok(), Some(4));
    match dec.feed(&m4a[4..], |_| Ok(())) {
        Err(AacError::Format(msg)) => assert!(msg.contains("random access"), "{msg}"),
        other => panic!("expected m4a reject, got {other:?}"),
    }
}

#[test]
fn stream_info_tallies_sine48() -> Result<()> {
    use crate::engine::adts::AdtsHeader;
    let data = &include_bytes!("goldens/sine48.adts")[..];
    let mut frames = 0u64;
    let mut pos = 0usize;
    while pos < data.len() {
        let (hdr, _) =
            AdtsHeader::parse(&data[pos..]).map_err(|e| AacError::decode(format!("hdr: {e:?}")))?;
        pos += usize::from(hdr.aac_frame_length);
        frames += 1;
    }
    let s = collect_slice(data, &DecodeOptions::speech())?;
    assert_eq!(s.info.aac_frames, frames);
    assert_eq!(s.info.channels, 1);
    assert_eq!(s.info.sample_rate, 48_000);
    assert_eq!(s.info.samples, frames * 1024);
    Ok(())
}

#[test]
fn streaming_does_not_accumulate_pcm() -> Result<()> {
    // The callback drops every frame; only a running peak survives. The
    // guarantee itself is structural (Frame borrows decoder scratch, the
    // decoder keeps no PCM history) — asserted here as: many frames stream
    // through while the callback never holds more than one frame.
    let data = &include_bytes!("goldens/lecture.m4a")[..];
    let mut total = 0u64;
    let mut peak_frame = 0usize;
    let info = decode_streaming(data, &DecodeOptions::unbounded(), |f| {
        total += f.samples as u64;
        peak_frame = peak_frame.max(f.samples * f.planar.len());
        Ok(())
    })?;
    assert_eq!(total, info.samples);
    assert!(total >= 12 * 1024, "lecture has many frames: {total}");
    assert!(
        peak_frame <= 8192,
        "one frame stays one frame: {peak_frame}"
    );
    Ok(())
}
