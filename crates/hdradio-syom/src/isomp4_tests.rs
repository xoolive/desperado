//! Extra ISOBMFF branches: 64-bit sizes, esds flags, co64, elst/mdhd v1, wave.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{
    BoxHdr, extract_asc, parse_elst_start, parse_esds, parse_mdhd_timescale, parse_stco,
    parse_stsc, read_box, read_desc_len, sniff_is_m4a,
};
use crate::error::AacError;

fn ok<T>(r: Result<T, AacError>) -> T {
    match r {
        Ok(v) => v,
        Err(e) => panic!("{e:?}"),
    }
}

fn esds_body(asc: &[u8]) -> Vec<u8> {
    let mut dsi = vec![0x05u8, asc.len() as u8];
    dsi.extend_from_slice(asc);
    let mut dcd = vec![0x04u8, (13 + dsi.len()) as u8];
    dcd.extend_from_slice(&[0x40, 0x15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    dcd.extend_from_slice(&dsi);
    let mut esd = vec![0x03u8, (3 + dcd.len()) as u8, 0x00, 0x01, 0x00];
    esd.extend_from_slice(&dcd);
    let mut body = vec![0u8; 4];
    body.extend_from_slice(&esd);
    body
}

#[test]
fn read_box_64bit_and_until_eof_and_undersize() {
    let mut data = vec![0, 0, 0, 1];
    data.extend_from_slice(b"mdat");
    data.extend_from_slice(&24u64.to_be_bytes());
    data.extend_from_slice(&[0u8; 8]);
    let (hdr, typ) = ok(read_box(&data, 0)).expect("64-bit box");
    assert_eq!(&typ, b"mdat");
    assert_eq!(hdr.content_start, 16);
    assert_eq!(hdr.content_end, 24);

    let mut eof = vec![0, 0, 0, 0];
    eof.extend_from_slice(b"mdatXXXX");
    let (hdr, typ) = ok(read_box(&eof, 0)).expect("until-eof box");
    assert_eq!(&typ, b"mdat");
    assert_eq!(hdr.content_end, eof.len());

    let small = {
        let mut v = 4u32.to_be_bytes().to_vec();
        v.extend_from_slice(b"mdat");
        v
    };
    assert!(read_box(&small, 0).is_err());
    assert!(read_box(&[0, 0, 0, 1, b'm', b'd', b'a', b't'], 0).is_err());
}

#[test]
fn sniff_m4a_rejects_short_and_tiny_size() {
    assert!(!sniff_is_m4a(&[]));
    assert!(!sniff_is_m4a(b"1234"));
    let mut d = 4u32.to_be_bytes().to_vec();
    d.extend_from_slice(b"ftypM4A ");
    assert!(!sniff_is_m4a(&d));
    let mut d = 20u32.to_be_bytes().to_vec();
    d.extend_from_slice(b"ftyp");
    d.extend_from_slice(b"M4A ");
    d.extend_from_slice(&[0; 8]);
    assert!(sniff_is_m4a(&d));
}

#[test]
fn desc_len_rejects_four_continuations() {
    let data = [0x80, 0x80, 0x80, 0x80];
    assert!(read_desc_len(&data, 0).is_err());
}

#[test]
fn esds_errors_and_optional_flags() {
    assert!(parse_esds(&[0, 0, 0, 0, 0x04, 0]).is_err());

    let mut esds = vec![0u8; 4];
    esds.extend_from_slice(&[0x03, 8, 0x00, 0x01, 0x00, 0x04, 0x02, 0x40, 0x15]);
    assert!(parse_esds(&esds).is_err());

    let mut inner = esds_body(&[0x12, 0x10]);
    // ES flags at body[4+1+1+2] = skip version(4)+tag(1)+len(1)+ES_ID(2) → flags.
    let flags_at = 4 + 1 + 1 + 2;
    inner[flags_at] = 0xE0;
    inner.splice(
        flags_at + 1..flags_at + 1,
        [0x00, 0x02, 3, b'u', b'r', b'l', 0x00, 0x03],
    );
    inner[5] = (inner.len() - 6) as u8;
    assert_eq!(ok(parse_esds(&inner)), vec![0x12, 0x10]);

    let mut bad_ot = esds_body(&[0x12, 0x10]);
    let ot_at = 4 + 1 + 1 + 3 + 1 + 1;
    bad_ot[ot_at] = 0x6B;
    assert!(parse_esds(&bad_ot).is_err());

    let mut empty_asc = esds_body(&[]);
    empty_asc[5] = (empty_asc.len() - 6) as u8;
    assert!(parse_esds(&esds_body(&[])).is_err());

    let mut no_dsi = esds_body(&[0x12, 0x10]);
    // Drop the 0x05 descriptor: keep DecoderConfig 13-byte prefix only.
    let dcd_tag = no_dsi.iter().position(|&b| b == 0x04).unwrap_or(0);
    no_dsi.truncate(dcd_tag + 2 + 13);
    no_dsi[dcd_tag + 1] = 13;
    no_dsi[5] = (no_dsi.len() - 6) as u8;
    assert!(parse_esds(&no_dsi).is_err());
    let _ = empty_asc;
}

#[test]
fn extract_asc_from_wave_wrapper() {
    let esds = esds_body(&[0x12, 0x10]);
    let mut esds_box = ((esds.len() + 8) as u32).to_be_bytes().to_vec();
    esds_box.extend_from_slice(b"esds");
    esds_box.extend_from_slice(&esds);
    let mut wave = ((esds_box.len() + 8) as u32).to_be_bytes().to_vec();
    wave.extend_from_slice(b"wave");
    wave.extend_from_slice(&esds_box);
    let children = BoxHdr {
        content_start: 0,
        content_end: wave.len(),
    };
    assert_eq!(ok(extract_asc(&wave, children)), vec![0x12, 0x10]);
}

#[test]
fn stsc_empty_and_co64() {
    let mut stsc = vec![0u8; 4];
    stsc.extend_from_slice(&0u32.to_be_bytes());
    assert!(parse_stsc(&stsc).is_err());

    let mut co64 = vec![0u8; 4];
    co64.extend_from_slice(&1u32.to_be_bytes());
    co64.extend_from_slice(&0u32.to_be_bytes());
    co64.extend_from_slice(&0x1000u32.to_be_bytes());
    let offs = ok(parse_stco(&co64, true));
    assert_eq!(offs, vec![0x1000]);
}

#[test]
fn elst_v1_and_unsupported_and_negative_then_zero() {
    let mut v1 = vec![1u8, 0, 0, 0];
    v1.extend_from_slice(&1u32.to_be_bytes());
    v1.extend_from_slice(&0u64.to_be_bytes());
    v1.extend_from_slice(&1024u64.to_be_bytes());
    v1.extend_from_slice(&0x0001_0000u32.to_be_bytes());
    assert_eq!(ok(parse_elst_start(&v1)), 1024);

    let mut bad = vec![2u8, 0, 0, 0];
    bad.extend_from_slice(&1u32.to_be_bytes());
    bad.extend_from_slice(&[0u8; 12]);
    assert!(parse_elst_start(&bad).is_err());

    let mut skip = vec![0u8; 4];
    skip.extend_from_slice(&2u32.to_be_bytes());
    skip.extend_from_slice(&10u32.to_be_bytes());
    skip.extend_from_slice(&(-1i32 as u32).to_be_bytes());
    skip.extend_from_slice(&0x0001_0000u32.to_be_bytes());
    skip.extend_from_slice(&10u32.to_be_bytes());
    skip.extend_from_slice(&0u32.to_be_bytes());
    skip.extend_from_slice(&0x0001_0000u32.to_be_bytes());
    assert_eq!(ok(parse_elst_start(&skip)), 0);
}

#[test]
fn mdhd_v1_and_unsupported() {
    let mut v1 = vec![1u8, 0, 0, 0];
    v1.extend_from_slice(&[0u8; 16]);
    v1.extend_from_slice(&48_000u32.to_be_bytes());
    assert_eq!(ok(parse_mdhd_timescale(&v1)), 48_000);
    assert!(parse_mdhd_timescale(&[3u8, 0, 0, 0]).is_err());
}
