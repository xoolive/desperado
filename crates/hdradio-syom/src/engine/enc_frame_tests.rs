//! enc_frame raw_data_block structure, decoded by the shipped decoder.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::decode::StreamDecoder;
use super::super::error::Result;
use super::super::swb::LONG_WINDOW_LEN;
use super::{LcEncoder, MAX_PAYLOAD_BYTES};

fn sine_frame(t: usize, amp: f32) -> Vec<f32> {
    (0..LONG_WINDOW_LEN)
        .map(|i| {
            amp * (2.0 * std::f32::consts::PI * 440.0 * (t * LONG_WINDOW_LEN + i) as f32 / 48_000.0)
                .sin()
        })
        .collect()
}

#[test]
fn mono_frames_decode_and_fit() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
    let mut dec = StreamDecoder::new();
    for f in 0..4 {
        let pcm = sine_frame(f, 0.5);
        let payload = enc.encode_frame(&[&pcm])?;
        assert!(payload.len() <= MAX_PAYLOAD_BYTES);
        let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 1, 1, &payload)?;
        assert_eq!(frame.planar.len(), 1);
        assert_eq!(frame.planar[0].len(), LONG_WINDOW_LEN);
        if f >= 1 {
            let energy: f64 = frame.planar[0]
                .iter()
                .map(|&x| f64::from(x) * f64::from(x))
                .sum();
            assert!(energy > 1e6, "frame {f} lost the sine (energy {energy})");
        }
    }
    Ok(())
}

#[test]
fn stereo_frames_decode_as_cpe() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    let mut dec = StreamDecoder::new();
    let l = sine_frame(0, 0.5);
    let r = sine_frame(0, 0.25);
    let payload = enc.encode_frame(&[&l, &r])?;
    assert!(payload.len() <= MAX_PAYLOAD_BYTES);
    let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 2, 1, &payload)?;
    assert_eq!(frame.planar.len(), 2, "CPE must yield two planes");
    assert_eq!(frame.planar[0].len(), LONG_WINDOW_LEN);
    assert_eq!(frame.planar[1].len(), LONG_WINDOW_LEN);
    Ok(())
}

#[test]
fn silence_encodes_small() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
    let pcm = [0.0f32; LONG_WINDOW_LEN];
    let payload = enc.encode_frame(&[&pcm])?;
    assert!(
        payload.len() < 64,
        "silent frame is {} bytes",
        payload.len()
    );
    let mut dec = StreamDecoder::new();
    let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 1, 1, &payload)?;
    let peak = frame.planar[0]
        .iter()
        .map(|x| x.abs())
        .fold(0.0f32, f32::max);
    assert!(peak < 1.0, "silent frame decoded with peak {peak}");
    Ok(())
}

#[test]
fn rejects_bad_shape() {
    let mut enc = LcEncoder::new(48_000, 1, 128_000).expect("encoder");
    let pcm = [0.0f32; 512];
    assert!(enc.encode_frame(&[&pcm]).is_err());
    assert!(LcEncoder::new(47_000, 1, 128_000).is_err());
    assert!(LcEncoder::new(48_000, 3, 128_000).is_err());
    assert!(LcEncoder::new(48_000, 1, 0).is_err());
}
