//! Streaming API: LATM/LOAS equivalence, resync, CRC propagation.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::stream_tests::{assert_eq_collected, collect_push, collect_slice};
use super::{AacError, DecodeOptions, Decoder, Result};
use crate::engine::bits::{BitReader, BitWriter};
use crate::engine::crc::stream_mux_config_crc;
use crate::engine::latm::{LOAS_SYNC, MuxCfg};

fn latm_cases() -> [(&'static [u8], &'static str); 2] {
    [
        (&include_bytes!("goldens/latm48.latm")[..], "latm48"),
        (&include_bytes!("goldens/he48.latm")[..], "he48"),
    ]
}

fn push_bits(bits: &mut Vec<bool>, value: u32, n: u32) {
    for i in (0..n).rev() {
        bits.push((value >> i) & 1 != 0);
    }
}

fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let mut w = BitWriter::new();
    for &b in bits {
        w.write_bit(b);
    }
    w.finish()
}

fn to_bits(data: &[u8]) -> Vec<bool> {
    let mut v = Vec::with_capacity(data.len() * 8);
    for &b in data {
        push_bits(&mut v, u32::from(b), 8);
    }
    v
}

/// he48.latm carries `crcCheckPresent == 0`; splice a `crcCheckSum`
/// into its first `StreamMuxConfig()` (corrupt on demand) and return
/// the rebuilt LOAS stream.
fn splice_he48_crc(corrupt: bool) -> Result<Vec<u8>> {
    let data = include_bytes!("goldens/he48.latm");
    let hdr = (u32::from(data[0]) << 16) | (u32::from(data[1]) << 8) | u32::from(data[2]);
    let mux_len = (hdr & 0x1FFF) as usize;
    let frame = &data[3..3 + mux_len];
    // Locate the protected region with the real parser.
    let mut br = BitReader::new(frame);
    let _use_same = br
        .read_bit()
        .map_err(|e| AacError::decode(format!("{e:?}")))?;
    let cfg_start = br.bit_position();
    let _cfg = MuxCfg::parse(&mut br).map_err(|e| AacError::decode(format!("{e:?}")))?;
    let crc_end = br.bit_position() - 1;
    let payload_start = br.bit_position();

    let frame_bits = to_bits(frame);
    let mut crc = u32::from(stream_mux_config_crc(
        &frame_bits[cfg_start as usize..crc_end as usize],
    ));
    if corrupt {
        crc ^= 0xFF;
    }
    let mut spliced = frame_bits[..payload_start as usize].to_vec();
    spliced[payload_start as usize - 1] = true; // crcCheckPresent = 1
    push_bits(&mut spliced, crc, 8);
    spliced.extend_from_slice(&frame_bits[payload_start as usize..]);

    let element = bits_to_bytes(&spliced);
    let hdr = (LOAS_SYNC << 13) | element.len() as u32;
    let mut out = vec![(hdr >> 16) as u8, (hdr >> 8) as u8, hdr as u8];
    out.extend_from_slice(&element);
    out.extend_from_slice(&data[3 + mux_len..]);
    Ok(out)
}

#[test]
fn latm_slice_streaming_matches_oneshot() -> Result<()> {
    for (data, label) in latm_cases() {
        for opts in [DecodeOptions::speech(), DecodeOptions::unbounded()] {
            let one = crate::decode_with(data, &opts)?;
            let s = collect_slice(data, &opts)?;
            assert_eq_collected(&one, &s, label);
        }
    }
    Ok(())
}

#[test]
fn latm_push_chunked_matches_oneshot() -> Result<()> {
    for (data, label) in latm_cases() {
        for opts in [DecodeOptions::speech(), DecodeOptions::unbounded()] {
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
fn latm_mux_config_split_across_feeds() -> Result<()> {
    // Byte-at-a-time feeding splits the LOAS header, StreamMuxConfig and
    // payload across feeds; the config must persist for later packets.
    let data = &include_bytes!("goldens/latm48.latm")[..];
    let opts = DecodeOptions::unbounded();
    let one = crate::decode_with(data, &opts)?;
    let mut dec = Decoder::new(opts);
    let mut frames = 0u32;
    for b in data {
        dec.feed(&[*b], |_| {
            frames += 1;
            Ok(())
        })?;
    }
    let info = dec.finish(|_| {
        frames += 1;
        Ok(())
    })?;
    assert!(frames > 0, "no frames decoded byte-by-byte");
    assert_eq!(frames as u64, info.aac_frames);
    assert_eq!(info.samples as usize, one.channels[0].len());
    Ok(())
}

#[test]
fn latm_garbage_between_packets_resyncs_like_oneshot() -> Result<()> {
    let data = &include_bytes!("goldens/latm48.latm")[..];
    // Split into LOAS packets, rejoin with junk between them.
    let mut packets: Vec<&[u8]> = Vec::new();
    let mut pos = 0usize;
    while pos + 3 <= data.len() {
        let v = (u32::from(data[pos]) << 16)
            | (u32::from(data[pos + 1]) << 8)
            | u32::from(data[pos + 2]);
        let len = 3 + (v & 0x1FFF) as usize;
        let end = (pos + len).min(data.len());
        packets.push(&data[pos..end]);
        pos = end;
    }
    assert!(packets.len() > 2, "fixture has several packets");
    let mut dirty = packets[0].to_vec();
    for p in &packets[1..] {
        dirty.extend_from_slice(&[0x00, 0xFF, 0x2B]); // junk incl. partial sync
        dirty.extend_from_slice(p);
    }
    let opts = DecodeOptions::unbounded();
    let one = crate::decode_with(&dirty, &opts)?;
    let s = collect_push(&dirty, &opts, 5)?;
    assert_eq_collected(&one, &s, "latm-garbage-mid");
    Ok(())
}

#[test]
fn latm_truncated_tail_parity() -> Result<()> {
    let data = &include_bytes!("goldens/latm48.latm")[..];
    for cut in [1usize, 2, 40] {
        let t = &data[..data.len() - cut];
        let opts = DecodeOptions::unbounded();
        let one = crate::decode_with(t, &opts)?;
        let s = collect_push(t, &opts, 3)?;
        assert_eq_collected(&one, &s, &format!("cut={cut}"));
    }
    Ok(())
}

#[test]
fn loas_corrupt_crc_propagates_mid_stream() -> Result<()> {
    let corrupt = splice_he48_crc(true)?;
    match crate::decode_with(&corrupt, &DecodeOptions::unbounded()) {
        Err(AacError::Decode(msg)) => assert!(msg.contains("CRC"), "{msg}"),
        other => panic!("expected CRC decode error, got {other:?}"),
    }
    // Same error surfaces from `feed`, not only at finish.
    let mut dec = Decoder::new(DecodeOptions::unbounded());
    match dec.feed(&corrupt, |_| Ok(())) {
        Err(AacError::Decode(msg)) => assert!(msg.contains("CRC"), "{msg}"),
        other => panic!("expected CRC error from feed, got {other:?}"),
    }
    Ok(())
}

#[test]
fn loas_valid_spliced_crc_decodes_identically() -> Result<()> {
    let spliced = splice_he48_crc(false)?;
    let opts = DecodeOptions::unbounded();
    let one = crate::decode_with(include_bytes!("goldens/he48.latm"), &opts)?;
    let s = collect_slice(&spliced, &opts)?;
    assert_eq_collected(&one, &s, "he48-crc-spliced");
    Ok(())
}

#[test]
fn latm_stream_with_no_frames_is_decode_error_like_oneshot() {
    // Sniffs as LOAS (sync + mux_len 0) but carries no payload: the one-shot
    // contract for LATM is a decode-class error, not NotAac.
    let data = [0x56, 0xE0, 0x00]; // (0x2B7 << 13) | mux_len 0
    assert!(matches!(
        crate::decode_with(&data, &DecodeOptions::speech()),
        Err(AacError::Decode(_))
    ));
    // 3 bytes sit under the sniff window during feed; the empty-payload
    // decode-class error surfaces at finish.
    let mut dec = Decoder::new(DecodeOptions::speech());
    assert!(dec.feed(&data, |_| Ok(())).is_ok());
    match dec.finish(|_| Ok(())) {
        Err(AacError::Decode(_)) => {}
        other => panic!("expected decode-class error, got {other:?}"),
    }
    // LOAS sync seen, but the announced body never arrives: decode-class too.
    let data = [0x56, 0xE7, 0xFF, 0x00, 0x00]; // mux_len 0x7FF, body absent
    let mut dec = Decoder::new(DecodeOptions::speech());
    assert!(dec.feed(&data, |_| Ok(())).is_ok());
    match dec.finish(|_| Ok(())) {
        Err(AacError::Decode(_)) => {}
        other => panic!("expected decode-class error, got {other:?}"),
    }
}

#[test]
fn latm_caps_too_long_fires_mid_stream() {
    let data = &include_bytes!("goldens/latm48.latm")[..];
    let opts = DecodeOptions::speech().with_max_duration_secs(0.005);
    let mut dec = Decoder::new(opts);
    assert!(
        matches!(dec.feed(data, |_| Ok(())), Err(AacError::TooLong { .. })),
        "latm duration cap must fire from feed"
    );
}
