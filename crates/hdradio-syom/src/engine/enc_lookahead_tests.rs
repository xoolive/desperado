//! One-frame attack lookahead: the window sequence shifts one frame
//! earlier on early-in-frame attacks, stream-start and flush edge cases
//! keep the causal decision, and the pre-echo A/B against the causal
//! default (the weakness this option exists for).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::super::error::Result;
use super::super::ics::WindowSequence;
use super::super::swb::LONG_WINDOW_LEN;
use super::LcEncoder;
use super::enc_block_tests::{add_click, decode_all, seqs_of, sine_frame};

/// Run `frames` through the lookahead path; returns one payload per frame.
fn encode_lookahead(frames: &[Vec<f32>]) -> Result<Vec<Vec<u8>>> {
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?.with_lookahead(true);
    let mut payloads = Vec::new();
    for pcm in frames {
        if let Some(p) = enc.push_frame(&[pcm])? {
            payloads.push(p);
        }
    }
    if let Some(p) = enc.flush()? {
        payloads.push(p);
    }
    Ok(payloads)
}

#[test]
fn early_attack_shifts_sequence_one_frame_earlier() -> Result<()> {
    // Click at sample 100 of frame 5 — inside the LongStart's flat region,
    // the causal weak spot.
    let frames: Vec<Vec<f32>> = (0..12)
        .map(|f| {
            let mut pcm = sine_frame(f, 0.2);
            if f == 5 {
                add_click(&mut pcm, 100, 0.8);
            }
            pcm
        })
        .collect();
    let causal: Vec<WindowSequence> = {
        let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
        let mut payloads = Vec::new();
        for pcm in &frames {
            payloads.push(enc.encode_frame(&[pcm])?);
        }
        seqs_of(&payloads)
    };
    assert_eq!(causal[5], WindowSequence::LongStart, "causal: attack frame");
    assert_eq!(causal[6], WindowSequence::EightShort);
    assert_eq!(causal[7], WindowSequence::LongStop);
    let lookahead = seqs_of(&encode_lookahead(&frames)?);
    assert_eq!(lookahead.len(), frames.len(), "one payload per frame");
    for (f, &s) in lookahead.iter().enumerate() {
        let want = match f {
            4 => WindowSequence::LongStart,  // slope covers the pre-attack tail
            5 => WindowSequence::EightShort, // the attack frame itself is short
            6 => WindowSequence::LongStop,
            _ => WindowSequence::OnlyLong,
        };
        assert_eq!(s, want, "lookahead frame {f}");
    }
    Ok(())
}

#[test]
fn stream_start_attack_still_switches_on_first_frame() -> Result<()> {
    // No frame −1 exists to be the LongStart: frame 0 falls back to its
    // own (causal) attack decision via the held.attack half of the OR.
    let frames: Vec<Vec<f32>> = (0..8)
        .map(|f| {
            let mut pcm = vec![0.0f32; LONG_WINDOW_LEN];
            if f == 0 {
                add_click(&mut pcm, 600, 0.8);
            }
            pcm
        })
        .collect();
    let seqs = seqs_of(&encode_lookahead(&frames)?);
    assert_eq!(seqs[0], WindowSequence::LongStart);
    assert_eq!(seqs[1], WindowSequence::EightShort);
    assert_eq!(seqs[2], WindowSequence::LongStop);
    assert_eq!(seqs[3], WindowSequence::OnlyLong);
    Ok(())
}

#[test]
fn single_frame_input_matches_causal_bytes() -> Result<()> {
    // One frame total: push holds it, flush encodes it with its own attack
    // flag — exactly the causal decision, so the bytes match.
    let mut pcm = sine_frame(0, 0.2);
    add_click(&mut pcm, 600, 0.8);
    let mut causal = LcEncoder::new(48_000, 1, 128_000)?;
    let want = causal.encode_frame(&[&pcm])?;
    let mut enc = LcEncoder::new(48_000, 1, 128_000)?.with_lookahead(true);
    assert!(enc.push_frame(&[&pcm])?.is_none(), "one frame of latency");
    let got = enc.flush()?.expect("flush emits the held frame");
    assert_eq!(got, want);
    assert!(enc.flush()?.is_none(), "nothing left to flush");
    Ok(())
}

#[test]
fn lookahead_reduces_early_attack_pre_echo() -> Result<()> {
    // Silence, then a click at sample 100 of frame 5 (absolute 5220), then
    // silence. Causal: frame 5 is a LongStart whose flat region still
    // covers the click, so the long transform smears quantization noise
    // backward. Lookahead: frame 4's LongStart slope covers the pre-attack
    // tail and frame 5 codes the click on short windows.
    let mut pcm = [vec![0.0f32; 14 * LONG_WINDOW_LEN]];
    add_click(&mut pcm[0], 5 * LONG_WINDOW_LEN + 100, 0.9);
    let frames: Vec<Vec<f32>> = (0..14)
        .map(|f| pcm[0][f * LONG_WINDOW_LEN..(f + 1) * LONG_WINDOW_LEN].to_vec())
        .collect();
    let causal: Vec<Vec<u8>> = {
        let mut enc = LcEncoder::new(48_000, 1, 128_000)?;
        let mut payloads = Vec::new();
        for f in &frames {
            payloads.push(enc.encode_frame(&[f])?);
        }
        payloads
    };
    let lookahead = encode_lookahead(&frames)?;
    let enc = LcEncoder::new(48_000, 1, 128_000)?;
    let dec_causal = decode_all(&enc, &causal)?;
    let dec_lookahead = decode_all(&enc, &lookahead)?;
    // Decoded sample i ≈ input i − 1024 (one-frame priming). The click
    // sits at decoded ≈ 5220 + 1024 = 6244; measure the 480 samples
    // (10 ms) before it — pure codec noise, the input is digital silence.
    let click_dec = 5 * LONG_WINDOW_LEN + 100 + LONG_WINDOW_LEN;
    let pre = click_dec - 480..click_dec;
    let e_causal: f64 = dec_causal[pre.clone()]
        .iter()
        .map(|&x| f64::from(x) * f64::from(x))
        .sum();
    let e_lookahead: f64 = dec_lookahead[pre]
        .iter()
        .map(|&x| f64::from(x) * f64::from(x))
        .sum();
    eprintln!(
        "early-attack pre-echo energy: causal {e_causal:.6}, lookahead {e_lookahead:.6}, \
         improvement {:.1} dB",
        10.0 * (e_causal / e_lookahead.max(1e-12)).log10()
    );
    assert!(
        e_lookahead * 100.0 < e_causal,
        "lookahead should cut early-attack pre-echo by ≥20 dB \
         (causal {e_causal:.6} vs lookahead {e_lookahead:.6})"
    );
    Ok(())
}
