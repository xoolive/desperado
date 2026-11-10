//! LC tools: TNS, MS, intensity, PNS, pulse, ICS sequences, SCE/CPE.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::bits::BitWriter;
use super::decode::StreamDecoder;
use super::error::Error;
use super::ics::{IcsInfo, WindowSequence, WindowShape};
use super::pns::{self, Lcg};
use super::section::{INTENSITY_HCB, NOISE_HCB, SectionData};
use super::sf::ScaleFactors;
use super::spectrum::{self, PulseData};
use super::stereo::{self, MsInfo, MsMask};
use super::tns::{self, TnsData, TnsFilter, TnsWindow};

fn long_ics(max_sfb: u8) -> IcsInfo {
    IcsInfo {
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: [1, 0, 0, 0, 0, 0, 0, 0],
        num_swb: 49,
    }
}

#[test]
fn pulse_adds_amplitude_on_zero_bin() -> Result<(), Error> {
    let mut quant = vec![0i32; 1024];
    let pulse = PulseData {
        start_sfb: 0,
        pulses: vec![(0, 3)],
    };
    spectrum::apply_pulse(&mut quant, 3, &pulse)?;
    // k starts at swb 0 offset 0, plus offset 0 → bin 0; quant[0] was 0 so subtract amp.
    assert_eq!(quant[0], -3);
    quant[0] = 2;
    spectrum::apply_pulse(&mut quant, 3, &pulse)?;
    assert_eq!(quant[0], 5);
    assert!(spectrum::invquant(9000).abs() > 1.0);
    assert!(spectrum::sf_gain(300) > 0.0);
    Ok(())
}

#[test]
fn ms_dematrix_is_m_plus_s() -> Result<(), Error> {
    let ics = long_ics(1);
    let mut left = vec![0.0f32; 1024];
    let mut right = vec![0.0f32; 1024];
    left[0] = 3.0; // m
    right[0] = 1.0; // s
    let sec = SectionData {
        sfb_cb: vec![vec![1]],
    };
    let ms = MsInfo {
        mask: MsMask::All,
        used: vec![],
    };
    stereo::apply_ms(&mut left, &mut right, &ics, &sec, &sec, &ms, 3)?;
    assert!((left[0] - 4.0).abs() < 1e-12);
    assert!((right[0] - 2.0).abs() < 1e-12);
    Ok(())
}

#[test]
fn intensity_scales_right_from_left() -> Result<(), Error> {
    let ics = long_ics(1);
    let mut left = vec![0.0f32; 1024];
    let mut right = vec![0.0f32; 1024];
    left[0] = 8.0;
    let right_sec = SectionData {
        sfb_cb: vec![vec![INTENSITY_HCB]],
    };
    let sf = ScaleFactors {
        sf: vec![vec![100]],
        is_pos: vec![vec![0]],
        noise_nrg: vec![vec![0]],
    };
    let ms = MsInfo {
        mask: MsMask::Off,
        used: vec![],
    };
    stereo::apply_intensity(&left, &mut right, &ics, &right_sec, &sf, &ms, 3)?;
    assert!((right[0] - 8.0).abs() < 1e-9, "is_pos=0 → scale 1");
    Ok(())
}

#[test]
fn pns_fills_noise_band_energy() -> Result<(), Error> {
    let ics = long_ics(1);
    let mut spec = vec![0.0f32; 1024];
    let sections = SectionData {
        sfb_cb: vec![vec![NOISE_HCB]],
    };
    let sf = ScaleFactors {
        sf: vec![vec![0]],
        is_pos: vec![vec![0]],
        noise_nrg: vec![vec![0]],
    };
    let mut rng = Lcg::new();
    pns::apply(&mut spec, &ics, &sections, &sf, 3, &mut rng, None)?;
    let nrg: f32 = spec.iter().map(|x| x * x).sum();
    // |target| = 2^(0.25*0) = 1, width=4 → L2 = 1
    assert!((nrg.sqrt() - 1.0).abs() < 1e-5, "PNS L2 {nrg}");
    Ok(())
}

#[test]
fn pns_noise_band_matches_lavc_lcg() -> Result<(), Error> {
    // FFmpeg libavcodec float PNS LCG (aacdec_proc_template.c): seed 0x1f2e3d4c,
    // LCG 1664525/1013904223, int-as-float. Scale is +2^(nrg/4)/sqrt(energy)
    // so PCM polarity matches lavc (lavc's spectral minus is an IMDCT artifact).
    // 48 kHz long sfb 0 is 4 bins. nrg=0 → |L2|=1, first bin positive.
    let ics = long_ics(1);
    let mut spec = vec![0.0f32; 1024];
    let sections = SectionData {
        sfb_cb: vec![vec![NOISE_HCB]],
    };
    let sf = ScaleFactors {
        sf: vec![vec![0]],
        is_pos: vec![vec![0]],
        noise_nrg: vec![vec![0]],
    };
    let mut rng = Lcg::new();
    pns::apply(&mut spec, &ics, &sections, &sf, 3, &mut rng, None)?;
    let got = [spec[0], spec[1], spec[2], spec[3]].map(f64::from);
    let want = [0.443_143_16, -0.548_071_74, 0.398_949_89, -0.586_583_67];
    for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
        assert!(
            (g - w).abs() < 1e-6,
            "PNS bin {i} {g} != lavc {w} (seed/mapping drift)"
        );
    }
    Ok(())
}

#[test]
fn tns_identity_order_zero_is_noop() -> Result<(), Error> {
    let ics = long_ics(2);
    let mut spec = vec![0.0f32; 1024];
    spec[10] = 1.0;
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 2,
                order: 0,
                direction: false,
                coef_compress: false,
                coef: vec![],
            }],
        }],
    };
    tns::apply(&mut spec, &tns, &ics, 3)?;
    assert_eq!(spec[10], 1.0);
    Ok(())
}

#[test]
fn tns_order1_changes_spectrum() -> Result<(), Error> {
    // length=num_swb so the filter starts at sfb 0 (bins 0..). Order-1 AR
    // leaves spec[0] (empty history) and must move spec[1]. Stub Ok(()) fails.
    let ics = long_ics(40);
    let mut spec = vec![0.0f32; 1024];
    spec[0] = 1.0;
    spec[1] = 0.5;
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![TnsFilter {
                length: ics.num_swb,
                order: 1,
                direction: false,
                coef_compress: false,
                coef: vec![4],
            }],
        }],
    };
    tns::apply(&mut spec, &tns, &ics, 3)?;
    assert!(
        (spec[1] - 0.5).abs() > 1e-9,
        "order-1 TNS left spec[1]={}; filter is a stub if unchanged",
        spec[1]
    );
    Ok(())
}

#[test]
fn kbd_window_differs_from_sine_on_nonzero_spec() -> Result<(), Error> {
    use super::filterbank::Filterbank;
    let mut spec = vec![0.0f32; 1024];
    spec[3] = 4_000_000.0;
    let mut sine_ics = long_ics(1);
    sine_ics.window_shape = WindowShape::Sine;
    let mut kbd_ics = long_ics(1);
    kbd_ics.window_shape = WindowShape::Kbd;
    // First-frame left half is sine (no previous shape). Difference is the
    // right half, which is overlap into frame 2.
    let mut fb_s = Filterbank::new();
    let _ = fb_s.synthesize(&spec, &sine_ics)?;
    let pcm_sine = fb_s.synthesize(&spec, &sine_ics)?;
    let mut fb_k = Filterbank::new();
    let _ = fb_k.synthesize(&spec, &kbd_ics)?;
    let pcm_kbd = fb_k.synthesize(&spec, &kbd_ics)?;
    let max_d = pcm_sine
        .iter()
        .zip(pcm_kbd.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f32, f32::max);
    assert!(
        max_d > 1.0,
        "KBD vs sine PCM max abs {max_d} — window is unused if this is 0"
    );
    Ok(())
}

/// Minimal LC SCE: silence, ONLY_LONG, 48 kHz, max_sfb=1, book 0.
fn silent_sce_payload() -> Vec<u8> {
    let mut w = BitWriter::new();
    w.write(0, 3); // SCE
    w.write(0, 4); // tag
    w.write(100, 8); // global_gain
    w.write_bit(false); // reserved
    w.write(0, 2); // only_long
    w.write_bit(false); // sine
    w.write(1, 6); // max_sfb
    w.write_bit(false); // no predictor
    w.write(0, 4); // ZERO_HCB
    w.write(1, 5); // sect_len 1
    // no scalefactor (zero band)
    w.write_bit(false); // pulse
    w.write_bit(false); // tns
    w.write_bit(false); // gain
    w.write(7, 3); // END
    w.finish()
}

#[test]
fn sce_silent_frame_is_1024_zeros() -> Result<(), Error> {
    let mut dec = StreamDecoder::new();
    let payload = silent_sce_payload();
    let frame = dec.decode_raw_data_block(2, 3, 48_000, 1, 1, &payload)?;
    assert_eq!(frame.channels, 1);
    assert_eq!(frame.planar[0].len(), 1024);
    assert!(frame.planar[0].iter().all(|&x| x.abs() < 1e-9));
    Ok(())
}

#[test]
fn cpe_common_window_two_channels() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(1, 3); // CPE
    w.write(0, 4);
    w.write_bit(true); // common_window
    // ics
    w.write_bit(false);
    w.write(0, 2);
    w.write_bit(false);
    w.write(1, 6);
    w.write_bit(false);
    w.write(0, 2); // ms_mask off
    for _ in 0..2 {
        w.write(100, 8);
        w.write(0, 4);
        w.write(1, 5);
        w.write_bit(false);
        w.write_bit(false);
        w.write_bit(false);
    }
    w.write(7, 3);
    let payload = w.finish();
    let mut dec = StreamDecoder::new();
    let frame = dec.decode_raw_data_block(2, 3, 48_000, 2, 1, &payload)?;
    assert_eq!(frame.channels, 2);
    assert_eq!(frame.planar[0].len(), 1024);
    assert_eq!(frame.planar[1].len(), 1024);
    Ok(())
}

#[test]
fn long_start_and_stop_synthesize() -> Result<(), Error> {
    use super::filterbank::Filterbank;
    let mut fb = Filterbank::new();
    let spec = vec![0.0f32; 1024];
    for seq in [
        WindowSequence::LongStart,
        WindowSequence::EightShort,
        WindowSequence::LongStop,
    ] {
        let mut ics = long_ics(1);
        ics.window_sequence = seq;
        if seq.is_eight_short() {
            ics.num_windows = 8;
            ics.num_window_groups = 8;
            ics.window_group_length = [1; 8];
            ics.num_swb = 14;
        }
        let pcm = fb.synthesize(&spec, &ics)?;
        assert_eq!(pcm.len(), 1024);
    }
    Ok(())
}

#[test]
fn start_stop_windows_change_second_frame_pcm() -> Result<(), Error> {
    // First-frame left half is the same window; the right half becomes overlap.
    // LongStart then LongStop must differ from OnlyLong then OnlyLong on frame 2.
    use super::filterbank::Filterbank;
    let mut spec = vec![0.0f32; 1024];
    spec[3] = 4_000_000.0;
    let mut start = long_ics(1);
    start.window_sequence = WindowSequence::LongStart;
    let mut stop = long_ics(1);
    stop.window_sequence = WindowSequence::LongStop;
    let long = long_ics(1);

    let mut fb_ss = Filterbank::new();
    let _ = fb_ss.synthesize(&spec, &start)?;
    let pcm_ss = fb_ss.synthesize(&spec, &stop)?;

    let mut fb_ll = Filterbank::new();
    let _ = fb_ll.synthesize(&spec, &long)?;
    let pcm_ll = fb_ll.synthesize(&spec, &long)?;

    let max_d = pcm_ss
        .iter()
        .zip(pcm_ll.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f32, f32::max);
    assert!(
        max_d > 1.0,
        "start/stop overlap max abs {max_d} — treated as only_long if this is 0"
    );
    Ok(())
}

#[test]
fn dse_pce_lfe_with_silent_end() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(4, 3); // DSE
    w.write(0, 4);
    w.write_bit(false);
    w.write(0, 8);
    w.write(3, 3); // LFE
    w.write(0, 4);
    w.write(100, 8);
    w.write_bit(false);
    w.write(0, 2);
    w.write_bit(false);
    w.write(1, 6);
    w.write_bit(false);
    w.write(0, 4);
    w.write(1, 5);
    w.write_bit(false);
    w.write_bit(false);
    w.write_bit(false);
    w.write(7, 3);
    let mut dec = StreamDecoder::new();
    let frame = dec.decode_raw_data_block(2, 3, 48_000, 1, 1, &w.finish())?;
    assert_eq!(frame.channels, 1);
    assert!(dec.decode_raw_data_block(1, 3, 48_000, 1, 1, &[0]).is_err());
    Ok(())
}

#[test]
fn cce_then_silent_sce_skips_coupling() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(2, 3); // CCE
    w.write(0, 4);
    w.write_bit(true);
    w.write(0, 3);
    w.write_bit(false);
    w.write(0, 4);
    w.write(100, 8);
    w.write_bit(false);
    w.write(0, 2);
    w.write_bit(false);
    w.write(1, 6);
    w.write_bit(false);
    w.write(0, 4);
    w.write(1, 5);
    w.write_bit(false);
    w.write_bit(false);
    w.write_bit(false);
    w.write(0, 3); // SCE
    w.write(0, 4);
    w.write(100, 8);
    w.write_bit(false);
    w.write(0, 2);
    w.write_bit(false);
    w.write(1, 6);
    w.write_bit(false);
    w.write(0, 4);
    w.write(1, 5);
    w.write_bit(false);
    w.write_bit(false);
    w.write_bit(false);
    w.write(7, 3);
    let mut dec = StreamDecoder::new();
    let frame = dec.decode_raw_data_block(2, 3, 48_000, 1, 1, &w.finish())?;
    assert_eq!(frame.channels, 1);
    Ok(())
}

#[test]
fn skip_fil_dse_pce_consume_empty_bodies() -> Result<(), Error> {
    use super::bits::BitReader;
    use super::channel_map::parse_pce;
    use super::skip::{skip_dse, skip_fil};
    let mut w = BitWriter::new();
    w.write(0, 4); // FIL count 0
    w.write(0, 4); // DSE tag
    w.write_bit(false);
    w.write(0, 8); // DSE count 0
    w.write(0, 4 + 2 + 4 + 4 + 4 + 4 + 2 + 3 + 4); // PCE header zeros
    w.write_bit(false);
    w.write_bit(false);
    w.write_bit(false);
    w.write(0, 8); // comment len 0
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    skip_fil(&mut br)?;
    skip_dse(&mut br)?;
    let _ = parse_pce(&mut br)?;
    Ok(())
}

#[test]
fn skip_fil_escape_dse_align_pce_mix() -> Result<(), Error> {
    use super::bits::BitReader;
    use super::channel_map::parse_pce;
    use super::skip::{fill_count, skip_dse};
    let mut w = BitWriter::new();
    w.write(15, 4);
    w.write(2, 8);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert_eq!(fill_count(&mut br)?, 16);

    let mut w = BitWriter::new();
    w.write(0, 4);
    w.write_bit(true);
    w.write(0, 8);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    skip_dse(&mut br)?;

    let mut w = BitWriter::new();
    w.write(0, 4 + 2 + 4 + 4 + 4 + 4 + 2 + 3 + 4);
    w.write_bit(true);
    w.write(0, 4);
    w.write_bit(true);
    w.write(0, 4);
    w.write_bit(true);
    w.write(0, 3);
    w.write_bit(false);
    w.write(0, 8);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let _ = parse_pce(&mut br)?;
    Ok(())
}
