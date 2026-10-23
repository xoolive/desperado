//! Transient (block-switching) lavc golden: a castanet fixture whose encode
//! must walk LongStart → EightShort → LongStop and decode-match ffmpeg.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::encode_tests::assert_decode_matches_lavc_pub;
use crate::{EncodeOptions, encode, encode_with};

/// Deterministic transient fixture: a 440 Hz tone bed (det_math sine, as in
/// `encode_tests::lavc_fixture`) with three castanet clicks (32-sample LCG
/// bursts with a deterministic `det_math::exp2` decay) at different
/// sub-block positions. Stereo 48 kHz, 0.6 s.
fn transient_fixture() -> Vec<Vec<f32>> {
    let n = 28_800usize;
    let mut l = vec![0.0f32; n];
    let mut r = vec![0.0f32; n];
    let mut phase = 0.0f32;
    for i in 0..n {
        phase += 2.0 * std::f32::consts::PI * 440.0 / 48_000.0;
        if phase >= std::f32::consts::TAU {
            phase -= std::f32::consts::TAU;
        }
        let v = 0.15 * crate::engine::det_math::sincos(phase).0;
        l[i] = v;
        r[i] = 0.7 * v;
    }
    let mut lcg = 0xC1C4_5EEDu32;
    for &at in &[7_200usize, 12_000, 16_800] {
        for k in 0..32 {
            lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let u = (lcg >> 9) as f32 / (1u32 << 23) as f32 * 2.0 - 1.0;
            let env = crate::engine::det_math::exp2(-0.3 * k as f32);
            l[at + k] += 0.8 * env * u;
            r[at + k] += 0.55 * env * u;
        }
    }
    vec![l, r]
}

/// Count `ics_info` window sequences in an ADTS stream (SCE/CPE bodies).
fn window_sequence_counts(adts: &[u8]) -> (usize, usize, usize, usize) {
    use crate::engine::adts::AdtsHeader;
    use crate::engine::bits::BitReader;
    use crate::engine::ics::{IcsInfo, WindowSequence};
    let (mut long, mut start, mut short, mut stop) = (0, 0, 0, 0);
    let mut pos = 0usize;
    while pos + 8 < adts.len() {
        let (hdr, off) = AdtsHeader::parse(&adts[pos..]).expect("adts header");
        let payload = &adts[pos + off..pos + hdr.aac_frame_length as usize];
        let mut br = BitReader::new(payload);
        let id = br.read(3).expect("element id");
        br.read(4).expect("tag");
        if id == 1 {
            assert!(br.read_bit().expect("common_window"));
        } else {
            br.read(8).expect("global_gain");
        }
        let ics = IcsInfo::parse(&mut br, 3, id == 1).expect("ics_info");
        match ics.window_sequence {
            WindowSequence::OnlyLong => long += 1,
            WindowSequence::LongStart => start += 1,
            WindowSequence::EightShort => short += 1,
            WindowSequence::LongStop => stop += 1,
        }
        pos += hdr.aac_frame_length as usize;
    }
    assert_eq!(pos, adts.len(), "trailing bytes after last ADTS frame");
    (long, start, short, stop)
}

/// Offline mint: `MINT_GOLDENS=1 cargo test --lib mint_lavc_transient_golden`,
/// then `ffmpeg -y -i src/goldens/enc48t.adts -f s16le src/goldens/enc48t.lavc.s16`.
#[test]
fn mint_lavc_transient_golden() {
    if std::env::var("MINT_GOLDENS").is_err() {
        return;
    }
    let pcm = transient_fixture();
    let adts = encode(&pcm, 48_000).expect("encode");
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/goldens");
    std::fs::write(dir.join("enc48t.adts"), adts).expect("write golden");
}

#[test]
fn lavc_matches_our_decode_of_our_transient_adts() {
    let adts = include_bytes!("goldens/enc48t.adts");
    let lavc = include_bytes!("goldens/enc48t.lavc.s16");
    let pcm = transient_fixture();
    let fresh = encode(&pcm, 48_000).expect("encode");
    // The transient golden must actually exercise block switching: each
    // castanet click walks LongStart → EightShort → LongStop.
    let (long, start, short, stop) = window_sequence_counts(&fresh);
    eprintln!("transient golden sequences long/start/short/stop: {long}/{start}/{short}/{stop}");
    assert!(
        long > 0 && start >= 3 && short >= 3 && stop >= 3,
        "sequences long/start/short/stop: {long}/{start}/{short}/{stop}"
    );
    // Layer 1 — byte-exactness tripwire (see the enc48 oracle).
    assert_eq!(
        fresh.as_slice(),
        &adts[..],
        "encoder output drifted from the committed transient golden; re-mint"
    );
    // Layer 2 — lavc decode equivalence.
    assert_decode_matches_lavc_pub(&fresh, lavc);
}

/// Offline mint: `MINT_GOLDENS=1 cargo test --lib mint_lavc_lookahead_golden`,
/// then `ffmpeg -y -i src/goldens/enc48l.adts -f s16le src/goldens/enc48l.lavc.s16`.
/// Same transient fixture as `enc48t`, encoded with one-frame lookahead on
/// (`enc_frame` module docs): each LongStart shifts one frame earlier, so
/// the first click (sample 7200 = 32 samples into frame 7 — inside the
/// LongStart flat region, the causal weak spot) is coded on short windows.
#[test]
fn mint_lavc_lookahead_golden() {
    if std::env::var("MINT_GOLDENS").is_err() {
        return;
    }
    let pcm = transient_fixture();
    let opts = EncodeOptions::adts().with_lookahead(true);
    let adts = encode_with(&pcm, 48_000, &opts).expect("encode");
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/goldens");
    std::fs::write(dir.join("enc48l.adts"), adts).expect("write golden");
}

#[test]
fn lavc_matches_our_decode_of_our_lookahead_adts() {
    let adts = include_bytes!("goldens/enc48l.adts");
    let lavc = include_bytes!("goldens/enc48l.lavc.s16");
    let pcm = transient_fixture();
    let opts = EncodeOptions::adts().with_lookahead(true);
    let fresh = encode_with(&pcm, 48_000, &opts).expect("encode");
    // The lookahead golden must actually exercise block switching.
    let (long, start, short, stop) = window_sequence_counts(&fresh);
    eprintln!("lookahead golden sequences long/start/short/stop: {long}/{start}/{short}/{stop}");
    assert!(
        long > 0 && start >= 3 && short >= 3 && stop >= 3,
        "sequences long/start/short/stop: {long}/{start}/{short}/{stop}"
    );
    // Layer 1 — byte-exactness tripwire (see the enc48 oracle).
    assert_eq!(
        fresh.as_slice(),
        &adts[..],
        "encoder output drifted from the committed lookahead golden; re-mint"
    );
    // Layer 2 — lavc decode equivalence (measured 1 LSB s16 / ~71-74 dB).
    assert_decode_matches_lavc_pub(&fresh, lavc);
}
