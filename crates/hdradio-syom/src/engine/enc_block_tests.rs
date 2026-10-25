//! Block switching: sequence selection on steady/transient content,
//! short-frame decodability through the shipped decoder, and the pre-echo
//! improvement the switcher exists for.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::bits::BitReader;
use super::super::decode::StreamDecoder;
use super::super::error::Result;
use super::super::ics::{IcsInfo, WindowSequence};
use super::super::swb::LONG_WINDOW_LEN;
use super::{LcEncoder, MAX_PAYLOAD_BYTES};

pub(super) fn sine_frame(t: usize, amp: f32) -> Vec<f32> {
    (0..LONG_WINDOW_LEN)
        .map(|i| {
            amp * (2.0 * std::f32::consts::PI * 440.0 * (t * LONG_WINDOW_LEN + i) as f32 / 48_000.0)
                .sin()
        })
        .collect()
}

/// Add a 32-sample castanet-like burst at `pos` (deterministic LCG).
pub(super) fn add_click(frame: &mut [f32], pos: usize, amp: f32) {
    let mut lcg = 0x1234_5678u32;
    for v in &mut frame[pos..pos + 32] {
        lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let u = (lcg >> 9) as f32 / (1u32 << 23) as f32;
        *v += amp * (2.0 * u - 1.0);
    }
}

/// SCE payload → its `ics_info` window sequence.
fn payload_seq(payload: &[u8]) -> WindowSequence {
    let mut br = BitReader::new(payload);
    assert_eq!(br.read(3).expect("element id"), 0, "expected SCE");
    br.read(4).expect("tag");
    br.read(8).expect("global_gain");
    IcsInfo::parse(&mut br, 3, false)
        .expect("ics_info")
        .window_sequence
}

pub(super) fn seqs_of(payloads: &[Vec<u8>]) -> Vec<WindowSequence> {
    payloads.iter().map(|p| payload_seq(p)).collect()
}

#[test]
fn steady_tone_stays_only_long() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
    for f in 0..20 {
        let pcm = sine_frame(f, 0.5);
        let payload = enc.encode_frame(&[&pcm])?;
        assert_eq!(
            payload_seq(&payload),
            WindowSequence::OnlyLong,
            "frame {f} switched on a steady tone"
        );
    }
    Ok(())
}

#[test]
fn attack_walks_start_short_stop() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
    let mut payloads = Vec::new();
    for f in 0..16 {
        let mut pcm = sine_frame(f, 0.2);
        if f == 10 {
            add_click(&mut pcm, 600, 0.8);
        }
        payloads.push(enc.encode_frame(&[&pcm])?);
    }
    let seqs = seqs_of(&payloads);
    for (f, &s) in seqs.iter().enumerate() {
        let want = match f {
            10 => WindowSequence::LongStart,
            11 => WindowSequence::EightShort,
            12 => WindowSequence::LongStop,
            _ => WindowSequence::OnlyLong,
        };
        assert_eq!(s, want, "frame {f}");
    }
    Ok(())
}

#[test]
fn repeated_attacks_stay_short() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
    let mut payloads = Vec::new();
    for f in 0..16 {
        let mut pcm = sine_frame(f, 0.15);
        match f {
            10 => add_click(&mut pcm, 600, 0.6),
            11 => add_click(&mut pcm, 300, 0.9),
            12 => add_click(&mut pcm, 800, 1.1),
            _ => {}
        }
        payloads.push(enc.encode_frame(&[&pcm])?);
    }
    let seqs = seqs_of(&payloads);
    assert_eq!(seqs[10], WindowSequence::LongStart);
    assert_eq!(seqs[11], WindowSequence::EightShort);
    assert_eq!(seqs[12], WindowSequence::EightShort, "still attacking");
    assert_eq!(seqs[13], WindowSequence::LongStop);
    assert_eq!(seqs[14], WindowSequence::OnlyLong);
    Ok(())
}

#[test]
fn short_frames_decode_and_fit() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
    let mut dec = StreamDecoder::new();
    let mut saw_short = false;
    for f in 0..14 {
        let mut pcm = sine_frame(f, 0.3);
        if f == 6 {
            add_click(&mut pcm, 700, 0.9);
        }
        let payload = enc.encode_frame(&[&pcm])?;
        assert!(payload.len() <= MAX_PAYLOAD_BYTES);
        saw_short |= payload_seq(&payload) == WindowSequence::EightShort;
        let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 1, 1, &payload)?;
        assert_eq!(frame.planar.len(), 1);
        assert_eq!(frame.planar[0].len(), LONG_WINDOW_LEN);
    }
    assert!(saw_short, "fixture never went short");
    Ok(())
}

#[test]
fn stereo_attack_on_one_channel_switches_the_pair() -> Result<()> {
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    let mut dec = StreamDecoder::new();
    let mut saw_short = false;
    for f in 0..12 {
        let l = sine_frame(f, 0.3);
        let mut r = sine_frame(f, 0.3);
        if f == 7 {
            add_click(&mut r, 700, 0.9); // right channel only
        }
        let payload = enc.encode_frame(&[&l, &r])?;
        assert!(payload.len() <= MAX_PAYLOAD_BYTES);
        // CPE: one shared ics_info — the OR'd attack decision.
        let mut br = BitReader::new(&payload);
        assert_eq!(br.read(3).expect("id"), 1, "expected CPE");
        br.read(4).expect("tag");
        assert!(br.read_bit().expect("common_window"));
        let ics = IcsInfo::parse(&mut br, 3, true).expect("ics_info");
        if ics.window_sequence == WindowSequence::EightShort {
            saw_short = true;
        }
        let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 2, 1, &payload)?;
        assert_eq!(frame.planar.len(), 2, "CPE must yield two planes");
    }
    assert!(saw_short, "single-channel attack did not switch the pair");
    Ok(())
}

/// Decode a stream of payloads, concatenated.
pub(super) fn decode_all(enc: &LcEncoder, payloads: &[Vec<u8>]) -> Result<Vec<f32>> {
    let mut dec = StreamDecoder::new();
    let mut out = Vec::new();
    for p in payloads {
        let frame = dec.decode_raw_data_block(2, enc.fs_index(), 48_000, 1, 1, p)?;
        out.extend_from_slice(&frame.planar[0]);
    }
    Ok(out)
}

#[test]
fn block_switching_reduces_pre_echo() -> Result<()> {
    // Silence, then a click at sample 700 of frame 5 (absolute 5820), then
    // silence. The click lands in the LongStart window's zeroed tail, so
    // only frame 6's short windows code it; long-only smears its
    // quantization noise over the whole 2048-sample long transform.
    let mut pcm = [vec![0.0f32; 14 * LONG_WINDOW_LEN]];
    add_click(&mut pcm[0], 5 * LONG_WINDOW_LEN + 700, 0.9);
    let encode = |switching: bool| -> Result<Vec<Vec<u8>>> {
        let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
        enc.set_block_switching(switching);
        let mut payloads = Vec::new();
        for f in 0..14 {
            let frame: Vec<f32> = pcm[0][f * LONG_WINDOW_LEN..(f + 1) * LONG_WINDOW_LEN].to_vec();
            payloads.push(enc.encode_frame(&[&frame])?);
        }
        Ok(payloads)
    };
    let switched = encode(true)?;
    let long_only = encode(false)?;
    let enc = LcEncoder::new(48_000, 1, 128_000)?;
    let dec_switched = decode_all(&enc, &switched)?;
    let dec_long = decode_all(&enc, &long_only)?;
    // Decoded sample i ≈ input i − 1024 (one-frame priming). The click sits
    // at decoded ≈ 5820 + 1024 = 6844; measure the 480 samples (10 ms)
    // before it — pure codec noise there, the input is digital silence.
    let click_dec = 5 * LONG_WINDOW_LEN + 700 + LONG_WINDOW_LEN;
    let pre = click_dec - 480..click_dec;
    let e_switched: f64 = dec_switched[pre.clone()]
        .iter()
        .map(|&x| f64::from(x) * f64::from(x))
        .sum();
    let e_long: f64 = dec_long[pre]
        .iter()
        .map(|&x| f64::from(x) * f64::from(x))
        .sum();
    eprintln!(
        "pre-echo energy: long-only {e_long:.6}, block-switched {e_switched:.6}, \
         ratio {:.1} dB",
        10.0 * (e_long / e_switched.max(1e-12)).log10()
    );
    assert!(
        e_switched * 4.0 < e_long,
        "block switching should cut pre-echo by ≥6 dB (long {e_long:.6} vs switched {e_switched:.6})"
    );
    Ok(())
}
