//! Multichannel (3.0 / 4.0 / 5.0 / 5.1) AAC-LC lavc goldens + speech mono rule.
//!
//! Fixtures: 0.4 s, 48 kHz, one sine per channel — FL 440, FR 880, FC 330,
//! LFE 100, BL/BC 550, BR 660 Hz — WAV planes in layout order, encoded with
//! ffmpeg's native AAC-LC encoder (aac_at/afconvert drop 4.0's back center).
//! lavc decodes each config to its default layout, so the golden plane order
//! is FL FR FC / FL FR FC BC / FL FR FC BL BR / FL FR FC LFE BL BR; syom
//! emits the same order (see `engine::channel_map`).
//!
//! Generated offline from a scratch dir (oracle; tests never spawn ffmpeg):
//!
//!   python3 - <<'EOF'   # mc51_src.wav (mc30/40/50: truncate the freq list)
//!   import numpy as np, wave
//!   sr = 48000; t = np.arange(int(sr*0.4))/sr
//!   fr = [440, 880, 330, 100, 550, 660]          # FL FR FC LFE BL BR
//!   d = np.stack([0.16*32767*np.sin(2*np.pi*f*t) for f in fr], 1).astype('<i2')
//!   w = wave.open('mc51_src.wav','wb'); w.setnchannels(len(fr))
//!   w.setsampwidth(2); w.setframerate(sr); w.writeframes(d.tobytes()); w.close()
//!   EOF
//!   ffmpeg -guess_layout_max 0 -i mc51_src.wav -af aformat=channel_layouts=5.1 \
//!       -c:a aac -b:a 256000 mc51.m4a
//!   ffmpeg -i mc51.m4a -c:a copy -f adts mc51.adts
//!   ffmpeg -i mc51.adts -f s16le -acodec pcm_s16le mc51.s16      # untrimmed
//!   ffmpeg -i mc51.m4a  -f s16le -acodec pcm_s16le mc51_m4a.s16  # elst-trimmed
//!
//! mc30/mc40/mc50 likewise with layouts 3.0/4.0/5.0 (m4a golden minted for
//! mc51 only, to cover the ISOBMFF + ASC path).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::decode_tests::assert_native_matches_lavc_with;
use super::{AacError, DecodeOptions};

#[test]
fn mc30_adts_native_matches_lavc_golden() -> Result<(), AacError> {
    assert_native_matches_lavc_with(
        include_bytes!("goldens/mc30.adts"),
        include_bytes!("goldens/mc30.s16"),
        48_000,
        "mc30-adts",
        &DecodeOptions::unbounded(),
    )
}

#[test]
fn mc40_adts_native_matches_lavc_golden() -> Result<(), AacError> {
    assert_native_matches_lavc_with(
        include_bytes!("goldens/mc40.adts"),
        include_bytes!("goldens/mc40.s16"),
        48_000,
        "mc40-adts",
        &DecodeOptions::unbounded(),
    )
}

#[test]
fn mc50_adts_native_matches_lavc_golden() -> Result<(), AacError> {
    assert_native_matches_lavc_with(
        include_bytes!("goldens/mc50.adts"),
        include_bytes!("goldens/mc50.s16"),
        48_000,
        "mc50-adts",
        &DecodeOptions::unbounded(),
    )
}

#[test]
fn mc51_adts_native_matches_lavc_golden() -> Result<(), AacError> {
    assert_native_matches_lavc_with(
        include_bytes!("goldens/mc51.adts"),
        include_bytes!("goldens/mc51.s16"),
        48_000,
        "mc51-adts",
        &DecodeOptions::unbounded(),
    )
}

#[test]
fn mc51_m4a_native_matches_lavc_golden() -> Result<(), AacError> {
    assert_native_matches_lavc_with(
        include_bytes!("goldens/mc51.m4a"),
        include_bytes!("goldens/mc51_m4a.s16"),
        48_000,
        "mc51-m4a",
        &DecodeOptions::unbounded(),
    )
}

/// Speech caps on 5.1 must not blow up: the one mono rule is the arithmetic
/// mean of the non-LFE planes (canonical 5.1 order: LFE is plane 3).
#[test]
fn mc51_speech_mono_is_mean_of_non_lfe_planes() -> Result<(), AacError> {
    let split = crate::decode_with(
        include_bytes!("goldens/mc51.adts"),
        &DecodeOptions::unbounded(),
    )?;
    assert_eq!(split.channels.len(), 6, "5.1 split plane count");
    for (input, label) in [
        (&include_bytes!("goldens/mc51.adts")[..], "adts"),
        (&include_bytes!("goldens/mc51.m4a")[..], "m4a"),
    ] {
        let mono = crate::decode_with(input, &DecodeOptions::speech())?;
        assert_eq!(mono.channels.len(), 1, "{label} speech not mono");
        let m = &mono.channels[0];
        assert!(
            m.iter().any(|s| s.abs() > 1e-3),
            "{label} speech mono silent"
        );
        if label == "adts" {
            assert_eq!(m.len(), split.channels[0].len(), "{label} mono length");
            let non_lfe = [0usize, 1, 2, 4, 5];
            for (i, &mv) in m.iter().enumerate() {
                let mean = non_lfe.iter().map(|&c| split.channels[c][i]).sum::<f32>() / 5.0;
                assert!(
                    (mv - mean).abs() <= 1e-6,
                    "{label} sample {i}: {mv} vs non-LFE mean {mean}"
                );
            }
        }
    }
    Ok(())
}
