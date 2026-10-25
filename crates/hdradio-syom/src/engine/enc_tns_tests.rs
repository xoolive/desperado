//! enc_tns: emit/parse mirror against the decoder's `tns.rs`, the
//! analysis→inverse roundtrip through `tns::apply` (the decoder is the
//! oracle), the gating behavior, and the wired-in A/B quality check.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{EncTns, MAX_ORDER, decide_frame, decide_long};
use crate::engine::bits::{BitReader, BitWriter};
use crate::engine::decode::StreamDecoder;
use crate::engine::enc_frame::LcEncoder;
use crate::engine::enc_quant::MAX_BANDS;
use crate::engine::error::Result;
use crate::engine::ics::{IcsInfo, WindowSequence, WindowShape};
use crate::engine::swb::{LONG_WINDOW_LEN, long_offsets};
use crate::engine::tns::TnsData;

/// Long-window ICS for 48 kHz (fs_index 3): 49 swb, one window.
fn long_ics() -> IcsInfo {
    IcsInfo {
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Kbd,
        max_sfb: 49,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: [1, 0, 0, 0, 0, 0, 0, 0],
        num_swb: 49,
    }
}

/// Deterministic tonal spectrum: smooth low-frequency sinusoids (a KBD
/// mainlobe stand-in — neighboring bins are strongly correlated) plus a
/// little deterministic noise.
fn tonal_spec() -> [f32; LONG_WINDOW_LEN] {
    let mut spec = [0.0f32; LONG_WINDOW_LEN];
    let mut lcg = 0x1234_5678u32;
    for (i, s) in spec.iter_mut().enumerate() {
        let t = i as f32 / LONG_WINDOW_LEN as f32;
        let v = 900.0 * (2.0 * std::f32::consts::PI * 3.0 * t).sin()
            + 300.0 * (2.0 * std::f32::consts::PI * 11.0 * t).sin();
        lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let n = (lcg >> 9) as f32 / (1u32 << 23) as f32 - 0.5;
        *s = v + 2.0 * n;
    }
    spec
}

#[test]
fn off_emits_only_the_flag() {
    let t = EncTns::off();
    assert_eq!(t.bits(), 1);
    let mut w = BitWriter::new();
    t.emit(&mut w);
    let out = w.finish();
    assert_eq!(out.len(), 1);
    assert_eq!(out[0] & 0x80, 0, "MSB-first flag bit must be 0");
}

#[test]
fn emit_mirrors_decoder_parse() -> Result<()> {
    let offsets = long_offsets(3)?;
    let mut spec = tonal_spec();
    let t = decide_long(&mut spec, offsets, 3, &[true; MAX_BANDS]);
    assert!(t.is_on(), "tonal spectrum must trigger TNS");
    let mut w = BitWriter::new();
    t.emit(&mut w);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert!(br.read_bit()?, "tns_data_present");
    let parsed = TnsData::parse(&mut br, &long_ics())?;
    assert_eq!(parsed.windows.len(), 1);
    let win = &parsed.windows[0];
    assert!(win.coef_res, "4-bit coefficients");
    assert_eq!(win.filters.len(), 1);
    let f = &win.filters[0];
    assert_eq!(usize::from(f.length), offsets.len() - 1, "length = num_swb");
    assert!(!f.direction);
    assert!(!f.coef_compress);
    assert!(f.order >= 1 && usize::from(f.order) <= MAX_ORDER);
    assert_eq!(f.coef.len(), usize::from(f.order));
    // Bit accounting must match the emitted payload exactly.
    assert_eq!(br.bit_position() as usize, t.bits());
    Ok(())
}

#[test]
fn spacer_filter_mirrors_decoder_parse() -> Result<()> {
    // Coded span not reaching the TNS band ceiling: emit must use an
    // order-0 spacer + the active filter, and the decoder's band walk must
    // land the active filter on exactly [start, end).
    let offsets = long_offsets(3)?;
    let mut coded = [false; MAX_BANDS];
    coded[8..=10].fill(true);
    let mut spec = tonal_spec();
    let t = decide_long(&mut spec, offsets, 3, &coded);
    assert!(t.is_on(), "tonal spectrum must trigger TNS");
    assert_eq!(t.band_range(), Some((8, 11)));
    let mut w = BitWriter::new();
    t.emit(&mut w);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert!(br.read_bit()?);
    let parsed = TnsData::parse(&mut br, &long_ics())?;
    let filters = &parsed.windows[0].filters;
    assert_eq!(filters.len(), 2, "spacer + active filter");
    assert_eq!(filters[0].order, 0, "spacer is a no-op");
    assert_eq!(usize::from(filters[0].length), 49 - 11);
    assert_eq!(usize::from(filters[1].length), 11 - 8);
    assert!(filters[1].order >= 1);
    assert_eq!(br.bit_position() as usize, t.bits());
    // And the decoder applies the inverse on exactly the span's bins.
    let mut zero = vec![0.0f32; LONG_WINDOW_LEN];
    crate::engine::tns::apply(&mut zero, &parsed, &long_ics(), 3)?;
    assert!(zero.iter().all(|&x| x == 0.0), "inverse of zeros is zeros");
    Ok(())
}

#[test]
fn analysis_then_decoder_inverse_roundtrips() -> Result<()> {
    let offsets = long_offsets(3)?;
    let orig = tonal_spec();
    let mut spec = orig;
    let t = decide_long(&mut spec, offsets, 3, &[true; MAX_BANDS]);
    assert!(t.is_on());
    // The analysis must actually filter (whiten) the spectrum.
    let (mut e_in, mut e_out) = (0.0f64, 0.0f64);
    for i in 0..usize::from(offsets[40]) {
        e_in += f64::from(orig[i]) * f64::from(orig[i]);
        e_out += f64::from(spec[i]) * f64::from(spec[i]);
    }
    assert!(e_in > 1.585 * e_out, "prediction gain gate");
    // Emit → parse with the decoder → apply its inverse filter.
    let mut w = BitWriter::new();
    t.emit(&mut w);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert!(br.read_bit()?);
    let parsed = TnsData::parse(&mut br, &long_ics())?;
    let mut restored = spec.to_vec();
    crate::engine::tns::apply(&mut restored, &parsed, &long_ics(), 3)?;
    let peak = orig.iter().map(|x| x.abs()).fold(0.0f32, f32::max);
    let err = restored
        .iter()
        .zip(orig.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f32, f32::max);
    assert!(
        err <= 1e-3 * peak,
        "roundtrip max error {err} (peak {peak}) — analysis is not the decoder inverse"
    );
    Ok(())
}

#[test]
fn noise_and_silence_stay_off() -> Result<()> {
    let offsets = long_offsets(3)?;
    let mut silence = [0.0f32; LONG_WINDOW_LEN];
    let t = decide_long(&mut silence, offsets, 3, &[true; MAX_BANDS]);
    assert!(!t.is_on(), "silence must stay off");
    assert!(
        silence.iter().all(|&x| x == 0.0),
        "off leaves spec untouched"
    );
    // Deterministic white noise: flat spectrum, no prediction gain.
    let mut noise = [0.0f32; LONG_WINDOW_LEN];
    let mut lcg = 0x0BAD_F00Du32;
    for s in noise.iter_mut() {
        lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *s = (lcg >> 9) as f32 / (1u32 << 23) as f32 - 0.5;
    }
    let orig = noise;
    let t = decide_long(&mut noise, offsets, 3, &[true; MAX_BANDS]);
    assert!(!t.is_on(), "white noise must stay off");
    assert_eq!(noise, orig, "off leaves spec untouched");
    Ok(())
}

#[test]
fn short_frames_and_disabled_stay_off() -> Result<()> {
    let offsets = long_offsets(3)?;
    let mut specs = [tonal_spec(), tonal_spec()];
    let coded = [[true; MAX_BANDS]; 2];
    let off = decide_frame(
        &mut specs,
        2,
        WindowSequence::EightShort,
        offsets,
        3,
        &coded,
        true,
    );
    assert!(
        !off[0].is_on() && !off[1].is_on(),
        "short frames: no TNS (v1)"
    );
    let off = decide_frame(
        &mut specs,
        2,
        WindowSequence::OnlyLong,
        offsets,
        3,
        &coded,
        false,
    );
    assert!(!off[0].is_on() && !off[1].is_on(), "A/B knob off");
    let on = decide_frame(
        &mut specs,
        2,
        WindowSequence::LongStart,
        offsets,
        3,
        &coded,
        true,
    );
    assert!(
        on[0].is_on() && on[1].is_on(),
        "long family: tonal triggers"
    );
    Ok(())
}

/// Tremolo fixture: an 880 Hz tone under 8 Hz full-depth AM, correlated
/// stereo. Steady tones/sweeps barely whiten (their KBD-MDCT spectra have
/// little AR structure — lavc keeps TNS off there too); the intra-frame
/// envelope movement of a tremolo is what spectral LPC can shape, so this
/// is the fixture that exercises TNS-on mid-stream.
fn tremolo_frames(n: usize) -> Vec<(Vec<f32>, Vec<f32>)> {
    let mut frames = Vec::with_capacity(n);
    for f in 0..n {
        let (mut l, mut r) = (vec![0.0f32; LONG_WINDOW_LEN], vec![0.0f32; LONG_WINDOW_LEN]);
        for i in 0..LONG_WINDOW_LEN {
            let t = (f * LONG_WINDOW_LEN + i) as f32 / 48_000.0;
            let env = 0.5 + 0.5 * (2.0 * std::f32::consts::PI * 8.0 * t).sin();
            let v = 0.4 * env * (2.0 * std::f32::consts::PI * 880.0 * t).sin();
            l[i] = v;
            r[i] = 0.8 * v;
        }
        frames.push((l, r));
    }
    frames
}

#[test]
fn tremolo_frames_trigger_tns() -> Result<()> {
    // Guards that a real signal exercises TNS-on mid-stream (the enc48
    // golden's tail is this fixture's twin).
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    let frames = tremolo_frames(24);
    let mut on_frames = 0;
    for (l, r) in &frames {
        enc.encode_frame(&[l, r])?;
        if enc.tns_on() {
            on_frames += 1;
        }
    }
    assert!(
        on_frames >= 6,
        "only {on_frames}/24 tremolo frames used TNS"
    );
    Ok(())
}

/// Steady sweeps stay off — matches lavc (which never fires TNS on a pure
/// tone): the fixture asserts the gate doesn't fire on tonal-but-static
/// content.
#[test]
fn steady_sweep_mostly_keeps_tns_off() -> Result<()> {
    let mut phase = 0.0f32;
    let mut enc = LcEncoder::new(48_000, 2, 128_000)?;
    let mut on_frames = 0;
    for f in 1..12 {
        let (mut l, mut r) = (vec![0.0f32; LONG_WINDOW_LEN], vec![0.0f32; LONG_WINDOW_LEN]);
        for i in 0..LONG_WINDOW_LEN {
            let t = (f * LONG_WINDOW_LEN + i) as f32 / (12 * LONG_WINDOW_LEN) as f32;
            phase += 2.0 * std::f32::consts::PI * (200.0 + 3_800.0 * t) / 48_000.0;
            if phase >= std::f32::consts::TAU {
                phase -= std::f32::consts::TAU;
            }
            let v = 0.4 * crate::engine::det_math::sincos(phase).0;
            l[i] = v;
            r[i] = 0.8 * v;
        }
        enc.encode_frame(&[&l, &r])?;
        if enc.tns_on() {
            on_frames += 1;
        }
    }
    assert!(
        on_frames <= 2,
        "{on_frames}/11 steady sweep frames used TNS"
    );
    Ok(())
}

/// Decode payloads through the shipped decoder; per-channel overall and
/// segmental (per-frame mean) SNR (dB) against the input with the
/// one-frame encoder delay accounted for. `DecodedFrame` is in the
/// engine's s16 domain (×32768).
fn ab_snr(
    payloads: &[Vec<u8>],
    frames: &[(Vec<f32>, Vec<f32>)],
    fs_index: u8,
) -> Result<([f64; 2], [f64; 2])> {
    let mut dec = StreamDecoder::new();
    let (mut ps, mut pe) = ([0.0f64; 2], [0.0f64; 2]);
    let mut seg = [Vec::new(), Vec::new()];
    for (t, payload) in payloads.iter().enumerate() {
        let frame = dec.decode_raw_data_block(2, fs_index, 48_000, 2, 1, payload)?;
        if t < 2 || t + 1 >= frames.len() {
            continue; // priming + one-frame delay; drop the tail frame
        }
        for ch in 0..2 {
            let want: &[f32] = if ch == 0 {
                &frames[t - 1].0
            } else {
                &frames[t - 1].1
            };
            let (mut fs, mut fe) = (0.0f64, 0.0f64);
            for (&g, &w) in frame.planar[ch].iter().zip(want.iter()) {
                let w = f64::from(w) * 32768.0;
                ps[ch] += w * w;
                pe[ch] += (w - f64::from(g)) * (w - f64::from(g));
                fs += w * w;
                fe += (w - f64::from(g)) * (w - f64::from(g));
            }
            seg[ch].push(10.0 * (fs / fe.max(1.0)).log10());
        }
    }
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    Ok((
        [
            10.0 * (ps[0] / pe[0].max(1.0)).log10(),
            10.0 * (ps[1] / pe[1].max(1.0)).log10(),
        ],
        [mean(&seg[0]), mean(&seg[1])],
    ))
}

#[test]
fn tns_never_hurts_tremolo() -> Result<()> {
    // A/B at 64 kbps stereo (tight budget, where TNS matters): identical
    // tremolo frames with TNS on vs the test knob off. The tremolo is
    // TNS-neutral on plain SNR (shaping moves noise in time, it does not
    // remove it) — this guards against regressions; the speech fixture
    // below is where the win shows.
    let frames = tremolo_frames(24);
    let encode_all = |on: bool| -> Result<Vec<Vec<u8>>> {
        let mut enc = LcEncoder::new(48_000, 2, 64_000)?;
        enc.set_tns(on);
        frames
            .iter()
            .map(|(l, r)| enc.encode_frame(&[l, r]))
            .collect()
    };
    let off = encode_all(false)?;
    let on = encode_all(true)?;
    let (snr_off, seg_off) = ab_snr(&off, &frames, 3)?;
    let (snr_on, seg_on) = ab_snr(&on, &frames, 3)?;
    eprintln!(
        "tremolo @64k stereo: TNS off {:.1}/{:.1} dB (seg {:.1}/{:.1}), \
         on {:.1}/{:.1} dB (seg {:.1}/{:.1})",
        snr_off[0], snr_off[1], seg_off[0], seg_off[1], snr_on[0], snr_on[1], seg_on[0], seg_on[1],
    );
    for ch in 0..2 {
        assert!(
            snr_on[ch] > snr_off[ch] - 1.0 && seg_on[ch] > seg_off[ch] - 1.0,
            "ch{ch}: TNS on {:.1}/{:.1} dB must not regress off {:.1}/{:.1} dB",
            snr_on[ch],
            seg_on[ch],
            snr_off[ch],
            seg_off[ch]
        );
    }
    Ok(())
}

#[test]
fn tns_improves_speech_lowrate_snr() -> Result<()> {
    // Real speech (the committed lecture fixture, 12 stereo frames): TNS
    // fires on the voiced onsets where the envelope moves in-frame, and
    // measurably wins at 64 kbps stereo.
    let m4a = &include_bytes!("../goldens/lecture.m4a")[..];
    let dec = crate::decode_with(m4a, &crate::DecodeOptions::unbounded()).expect("lecture decode");
    let pcm = &dec.channels;
    let frames: Vec<(Vec<f32>, Vec<f32>)> = (0..pcm[0].len() / LONG_WINDOW_LEN)
        .map(|t| {
            (
                pcm[0][t * LONG_WINDOW_LEN..(t + 1) * LONG_WINDOW_LEN].to_vec(),
                pcm[1][t * LONG_WINDOW_LEN..(t + 1) * LONG_WINDOW_LEN].to_vec(),
            )
        })
        .collect();
    let encode_all = |on: bool| -> Result<(Vec<Vec<u8>>, usize)> {
        let mut enc = LcEncoder::new(48_000, 2, 64_000)?;
        enc.set_tns(on);
        let mut n_on = 0;
        let payloads = frames
            .iter()
            .map(|(l, r)| {
                let p = enc.encode_frame(&[l, r]);
                if enc.tns_on() {
                    n_on += 1;
                }
                p
            })
            .collect::<Result<_>>()?;
        Ok((payloads, n_on))
    };
    let (off, _) = encode_all(false)?;
    let (on, n_on) = encode_all(true)?;
    assert!(n_on > 0, "speech must trigger TNS somewhere");
    let (snr_off, seg_off) = ab_snr(&off, &frames, 3)?;
    let (snr_on, seg_on) = ab_snr(&on, &frames, 3)?;
    eprintln!(
        "lecture @64k stereo ({n_on} TNS frames): off {:.1}/{:.1} dB (seg {:.1}/{:.1}), \
         on {:.1}/{:.1} dB (seg {:.1}/{:.1})",
        snr_off[0], snr_off[1], seg_off[0], seg_off[1], snr_on[0], snr_on[1], seg_on[0], seg_on[1],
    );
    assert!(
        snr_on[0] > snr_off[0] + 1.0,
        "TNS on {:.1} dB should clearly beat off {:.1} dB on speech",
        snr_on[0],
        snr_off[0]
    );
    for ch in 0..2 {
        assert!(
            seg_on[ch] >= seg_off[ch] - 0.2,
            "ch{ch}: segmental {:.1} vs {:.1} dB",
            seg_on[ch],
            seg_off[ch]
        );
    }
    Ok(())
}

#[test]
fn tns_on_residual_stays_bounded() -> Result<()> {
    // Stability: the decoder's inverse filter of our quantized filter must
    // not blow up — decoded energy tracks input energy on tonal content
    // (DecodedFrame is s16-domain: ×32768² on the input energy). 32k mono
    // is a tight-but-sane budget; the drop-cap regime below that is a
    // pre-existing pathology TNS does not worsen (measured identical).
    let frames = tremolo_frames(12);
    let mut enc = LcEncoder::new(48_000, 1, 32_000)?;
    let mut dec = StreamDecoder::new();
    for (t, _) in frames.iter().enumerate() {
        let payload = enc.encode_frame(&[&frames[t].0])?;
        let frame = dec.decode_raw_data_block(2, 3, 48_000, 1, 1, &payload)?;
        if t < 2 {
            continue;
        }
        // Decoded frame t is input frame t-1 (one-frame encoder delay).
        let e_in: f64 = frames[t - 1]
            .0
            .iter()
            .map(|&x| f64::from(x) * f64::from(x))
            .sum();
        let e_out: f64 = frame.planar[0]
            .iter()
            .map(|&x| f64::from(x) * f64::from(x))
            .sum();
        assert!(
            e_out < 4.0 * e_in * 32768.0 * 32768.0,
            "frame {t}: decoded energy {e_out} blew up vs input {e_in}"
        );
    }
    Ok(())
}

/// 7350 Hz (fs_index 12) has no TNS band table in the decoder — the encoder
/// must never emit TNS there (it can't even be constructed: the swb table
/// lookup fails first, so this exercises the defensive guard directly).
#[test]
fn rate_without_tns_table_stays_off() -> Result<()> {
    assert!(long_offsets(12).is_err(), "no swb table at fs_index 12");
    let offsets = long_offsets(11)?;
    let mut spec = tonal_spec();
    let orig = spec;
    let t = decide_long(&mut spec, offsets, 12, &[true; MAX_BANDS]);
    assert!(!t.is_on(), "fs_index 12 must keep TNS off");
    assert_eq!(spec, orig);
    Ok(())
}
