//! ICS grouping, ADTS, ASC, section_data.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::adts::AdtsHeader;
use super::asc::AudioSpecificConfig;
use super::bits::{BitReader, BitWriter};
use super::decode::StreamDecoder;
use super::error::Error;
use super::ics::{IcsInfo, WindowSequence};
use super::ics_body::parse_ics;
use super::raw_data_block::IdSynEle;
use super::section::SectionData;
use super::stereo::MsInfo;

#[test]
fn adts_lc_48k_mono_header() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(0xFFF, 12);
    w.write_bit(false); // MPEG-4
    w.write(0, 2); // layer
    w.write_bit(true); // no CRC
    w.write(1, 2); // LC
    w.write(3, 4); // 48 kHz
    w.write_bit(false);
    w.write(1, 3); // mono
    w.write(0, 4); // original/home/copyright
    w.write(7, 13); // frame length = header
    w.write(0x7FF, 11);
    w.write(0, 2);
    let bytes = w.finish();
    let (hdr, off) = AdtsHeader::parse(&bytes)?;
    assert_eq!(hdr.audio_object_type(), 2);
    assert_eq!(hdr.sample_rate(), 48_000);
    assert_eq!(hdr.channel_configuration, 1);
    assert_eq!(off, 7);
    Ok(())
}

#[test]
fn asc_lc_is_aot_2() -> Result<(), Error> {
    // AOT=2, fs=3 (48k), chan=1, GA: frameLength=0, depends=0, ext=0
    let mut w = BitWriter::new();
    w.write(2, 5);
    w.write(3, 4);
    w.write(1, 4);
    w.write(0, 3);
    let bytes = w.finish();
    let (asc, _) = AudioSpecificConfig::parse(&bytes)?;
    assert_eq!(asc.aot, 2);
    assert_eq!(asc.sample_rate, 48_000);
    assert_eq!(asc.channel_configuration, 1);
    Ok(())
}

#[test]
fn asc_heaac_aot5_unwraps_lc_core() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(5, 5); // SBR
    w.write(3, 4); // 48 kHz output
    w.write(1, 4); // mono
    w.write(2, 5); // inner LC
    w.write(0, 3); // GA
    let bytes = w.finish();
    let (asc, _) = AudioSpecificConfig::parse(&bytes)?;
    assert_eq!(asc.aot, 2);
    assert!(asc.sbr_present);
    assert_eq!(asc.output_sample_rate, 48_000);
    assert_eq!(asc.sample_rate, 24_000);
    Ok(())
}

#[test]
fn asc_lc_with_trailing_sbr_probe_still_lc() -> Result<(), Error> {
    // AOT 2 + GA zeros + syncExtensionType 0x2b7 / AOT 5 (implicit HE-AAC).
    let mut w = BitWriter::new();
    w.write(2, 5);
    w.write(3, 4);
    w.write(1, 4);
    w.write(0, 3);
    w.write(0x2b7, 11);
    w.write(5, 5);
    w.write_bit(true);
    w.write(3, 4);
    let bytes = w.finish();
    let (asc, _) = AudioSpecificConfig::parse(&bytes)?;
    assert_eq!(asc.aot, 2);
    assert!(asc.sbr_present);
    Ok(())
}

#[test]
fn eight_short_grouping_all_separate() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write_bit(false);
    w.write(2, 2); // eight short
    w.write_bit(false); // sine
    w.write(1, 4); // max_sfb
    w.write(0, 7); // grouping: all new groups
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let ics = IcsInfo::parse(&mut br, 3, false)?;
    assert!(ics.window_sequence.is_eight_short());
    assert_eq!(ics.num_windows, 8);
    assert_eq!(ics.num_window_groups, 8);
    assert_eq!(&ics.window_group_length[..8], &[1, 1, 1, 1, 1, 1, 1, 1]);
    Ok(())
}

#[test]
fn section_single_zero_band() -> Result<(), Error> {
    let mut w = BitWriter::new();
    // ics only_long, max_sfb=1, predictor=0
    w.write_bit(false);
    w.write(0, 2);
    w.write_bit(false);
    w.write(1, 6);
    w.write_bit(false);
    // section: cb=0, len=1 (5 bits)
    w.write(0, 4);
    w.write(1, 5);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let ics = IcsInfo::parse(&mut br, 3, false)?;
    assert_eq!(ics.window_sequence, WindowSequence::OnlyLong);
    let sec = SectionData::parse(&mut br, &ics)?;
    assert_eq!(sec.sfb_cb[0][0], 0);
    Ok(())
}

#[test]
fn lecture_frame1_eight_short_cpe_decodes() -> Result<(), Error> {
    // First real audio packet of the committed AAC_MP4 lecture fixture:
    // CPE, common_window, eight_short, book-11 left. Must not InvalidCodebook(12).
    let p: &[u8] = &[
        0x21, 0x45, 0x6c, 0xff, 0xff, 0xfc, 0xc5, 0x92, 0x96, 0x46, 0x59, 0x29, 0x65, 0x23, 0x7f,
        0xaf, 0x5c, 0x78, 0xff, 0x5f, 0xdf, 0xce, 0xaf, 0xff, 0x8f, 0xdf, 0xce, 0xbd, 0xbf, 0xfa,
        0x7e, 0x3c, 0xe8, 0x6d, 0x4e, 0xcd, 0xba, 0xe6, 0x33, 0x06, 0xaa, 0x4e, 0xab, 0x3b, 0x9a,
        0x5b, 0xd1, 0x4a, 0x74, 0xc9, 0x59, 0x5a, 0x2c, 0x3b, 0x1f, 0xcb, 0x65, 0xff, 0x96, 0xc1,
        0x14, 0x68, 0x08, 0x6d, 0xe9, 0x90, 0x48, 0x28, 0x9e, 0x77, 0x5d, 0x69, 0x88, 0xe6, 0xab,
        0x11, 0x08, 0x62, 0x67, 0xca, 0x26, 0x7c, 0xa2, 0x61, 0x2a, 0x1c, 0xb0, 0x9f, 0x80, 0xf3,
        0x8b, 0x38, 0x12, 0x13, 0x30, 0x34, 0x68, 0xd1, 0xad, 0xfe, 0xbd, 0x71, 0xe3, 0xfd, 0x7f,
        0x7f, 0x3a, 0xbf, 0xfe, 0x3f, 0x7f, 0x3a, 0xf6, 0xff, 0xe9, 0xf8, 0xf3, 0xa0, 0x00, 0x00,
        0x00, 0x00, 0x01, 0xc0,
    ];
    let mut br = BitReader::new(p);
    assert_eq!(br.read(3)?, 1); // CPE
    let _tag = br.read(4)?;
    assert!(br.read_bit()?); // common_window
    let ics = IcsInfo::parse(&mut br, 3, true)?;
    assert!(ics.window_sequence.is_eight_short());
    assert_eq!(ics.max_sfb, 5);
    assert_eq!(ics.num_window_groups, 4);
    let _ms = MsInfo::parse(&mut br, &ics)?;
    let left = parse_ics(&mut br, 3, 2, Some(&ics))?;
    let right = parse_ics(&mut br, 3, 2, Some(&ics))?;
    assert!(
        left.spec.iter().any(|&x| x != 0.0),
        "left ICS of lecture frame 1 was silent"
    );
    assert_eq!(right.spec.len(), left.spec.len());

    let mut dec = StreamDecoder::new();
    let frame = dec.decode_raw_data_block(2, 3, 48_000, 2, 1, p)?;
    assert_eq!(frame.channels, 2);
    assert_eq!(frame.planar[0].len(), 1024);
    assert_eq!(frame.planar[1].len(), 1024);
    Ok(())
}

#[test]
fn tns48_adts_has_nonzero_order_tns() -> Result<(), Error> {
    // Hand-built LC SCE with order-1 TNS. Stubbing TNS must fail the
    // lavc golden in extract_tests; this asserts the bitstream carries it.
    let data: &[u8] = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/src/goldens/tns48.adts"
    ));
    let mut pos = 0usize;
    let mut order_gt0 = 0usize;
    let mut tns_some = 0usize;
    let mut sce_ok = 0usize;
    let mut sce_err = 0usize;
    while pos + 7 < data.len() {
        let (hdr, off) = AdtsHeader::parse(&data[pos..])?;
        let fl = usize::from(hdr.aac_frame_length);
        if pos.saturating_add(fl) > data.len() {
            break;
        }
        let payload = data
            .get(pos.saturating_add(off)..pos.saturating_add(fl))
            .ok_or(Error::UnexpectedEnd)?;
        let mut br = BitReader::new(payload);
        if br.bits_remaining() >= 7 && IdSynEle::from_bits(br.read(3)? as u8) == IdSynEle::Sce {
            let _tag = br.read(4)?;
            match parse_ics(&mut br, hdr.sampling_frequency_index, 2, None) {
                Ok(body) => {
                    sce_ok += 1;
                    if let Some(tns) = body.tns {
                        tns_some += 1;
                        if tns
                            .windows
                            .iter()
                            .any(|w| w.filters.iter().any(|f| f.order > 0))
                        {
                            order_gt0 += 1;
                        }
                    }
                }
                Err(_) => sce_err += 1,
            }
        }
        pos = pos.saturating_add(fl);
    }
    assert!(
        order_gt0 > 0,
        "tns48.adts has no order>0 TNS (order_gt0={order_gt0} tns_some={tns_some} sce_ok={sce_ok} sce_err={sce_err})"
    );
    Ok(())
}

#[test]
fn parse_ics_rejects_gain_control() -> Result<(), Error> {
    let mut w = BitWriter::new();
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
    w.write_bit(true);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert!(parse_ics(&mut br, 3, 2, None).is_err());
    Ok(())
}

#[test]
fn parse_ics_pulse_on_long_window() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(100, 8);
    w.write_bit(false);
    w.write(0, 2);
    w.write_bit(false);
    w.write(1, 6);
    w.write_bit(false);
    w.write(0, 4);
    w.write(1, 5);
    w.write_bit(true);
    w.write(0, 2);
    w.write(0, 6);
    w.write(0, 5);
    w.write(1, 4);
    w.write_bit(false);
    w.write_bit(false);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    let body = parse_ics(&mut br, 3, 2, None)?;
    assert_eq!(body.spec.len(), 1024);
    Ok(())
}

#[test]
fn parse_ics_rejects_pulse_on_eight_short() -> Result<(), Error> {
    let mut w = BitWriter::new();
    w.write(100, 8);
    w.write_bit(false);
    w.write(2, 2);
    w.write_bit(false);
    w.write(1, 4);
    w.write(0, 7);
    w.write(0, 4);
    w.write(1, 3);
    w.write_bit(true);
    let bytes = w.finish();
    let mut br = BitReader::new(&bytes);
    assert!(parse_ics(&mut br, 3, 2, None).is_err());
    Ok(())
}
