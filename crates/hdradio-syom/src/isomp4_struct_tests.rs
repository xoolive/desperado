//! Crafted-structure ISOBMFF tests: malformed input is a clean error.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;

fn bx(typ: &[u8; 4], body: &[u8]) -> Vec<u8> {
    let mut v = ((body.len() + 8) as u32).to_be_bytes().to_vec();
    v.extend_from_slice(typ);
    v.extend_from_slice(body);
    v
}

fn ftyp() -> Vec<u8> {
    bx(b"ftyp", b"M4A \x00\x00\x00\x00M4A mp42")
}

/// stbl containing only an stsz with the given default size and count.
fn stbl_with_stsz(default_size: u32, count: u32) -> Vec<u8> {
    let mut stsz = Vec::new();
    stsz.extend_from_slice(&[0u8; 4]); // version/flags
    stsz.extend_from_slice(&default_size.to_be_bytes());
    stsz.extend_from_slice(&count.to_be_bytes());
    let mut stbl_body = bx(b"stsz", &stsz);
    // parse_stsz is called with the box body directly.
    stbl_body.drain(0..8);
    stbl_body
}

#[test]
fn test_stsz_default_size_count_capped() {
    // A stsz declaring 4G samples at a constant size must not allocate
    // 16 GiB: the total sample bytes are fenced by the file length.
    let body = stbl_with_stsz(1, u32::MAX);
    let err = match parse_stsz(&body, body.len()) {
        Ok(_) => panic!("huge stsz count must fail"),
        Err(e) => e,
    };
    assert!(format!("{err:?}").contains("stsz"));
    // A plausible count passes.
    let body = stbl_with_stsz(1, 4);
    assert_eq!(
        parse_stsz(&body, 64).expect("small count"),
        vec![1, 1, 1, 1]
    );
}

#[test]
fn test_stsz_constant_size_fenced_by_file_len() {
    // Regression: the fence used to measure `count` against the stsz
    // body (always 12 bytes for constant size), rejecting every
    // legitimate CBR file with more than 12 frames.
    let body = stbl_with_stsz(4, 100);
    assert_eq!(parse_stsz(&body, 4096).expect("legit CBR").len(), 100);
    // A crafted count whose samples cannot fit in the file is rejected.
    let body = stbl_with_stsz(4, 2000);
    assert!(parse_stsz(&body, 4096).is_err());
}

/// AudioSpecificConfig for AAC-LC, 44100 Hz, stereo.
const ASC: [u8; 2] = [0x12, 0x10];

/// Build an `esds` box body (version/flags + descriptor tree) wrapping
/// the given ASC. All descriptor lengths here fit one length byte.
fn esds_body(asc: &[u8]) -> Vec<u8> {
    let mut dsi = vec![0x05u8, asc.len() as u8];
    dsi.extend_from_slice(asc);
    let mut dcd = vec![0x04u8, (13 + dsi.len()) as u8];
    dcd.extend_from_slice(&[0x40, 0x15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    dcd.extend_from_slice(&dsi);
    let mut esd = vec![0x03u8, (3 + dcd.len()) as u8, 0x00, 0x01, 0x00];
    esd.extend_from_slice(&dcd);
    let mut body = vec![0u8; 4]; // version/flags
    body.extend_from_slice(&esd);
    body
}

/// An `mp4a` AudioSampleEntry (version 0) carrying the given child boxes.
fn mp4a_entry(children: &[u8]) -> Vec<u8> {
    let mut fixed = vec![0u8; 6]; // reserved
    fixed.extend_from_slice(&1u16.to_be_bytes()); // data_reference_index
    fixed.extend_from_slice(&0u16.to_be_bytes()); // version 0
    fixed.extend_from_slice(&[0u8; 6]); // revision + vendor
    fixed.extend_from_slice(&2u16.to_be_bytes()); // channelcount
    fixed.extend_from_slice(&16u16.to_be_bytes()); // samplesize
    fixed.extend_from_slice(&[0u8; 4]); // pre_defined + reserved
    fixed.extend_from_slice(&(44100u32 << 16).to_be_bytes()); // samplerate
    let mut entry = fixed;
    entry.extend_from_slice(children);
    bx(b"mp4a", &entry)
}

/// A `trak` with a `soun` handler and a complete constant-size sample
/// table in a single chunk; `entry` is the boxed stsd sample entry.
fn soun_trak(entry: Vec<u8>, sample_count: u32, sample_size: u32, chunk_offset: u32) -> Vec<u8> {
    let mut stsd_body = vec![0u8; 4]; // version/flags
    stsd_body.extend_from_slice(&1u32.to_be_bytes());
    stsd_body.extend_from_slice(&entry);

    let mut stts = vec![0u8; 4];
    stts.extend_from_slice(&1u32.to_be_bytes());
    stts.extend_from_slice(&sample_count.to_be_bytes());
    stts.extend_from_slice(&1024u32.to_be_bytes());

    let mut stsc = vec![0u8; 4];
    stsc.extend_from_slice(&1u32.to_be_bytes());
    stsc.extend_from_slice(&1u32.to_be_bytes()); // first_chunk
    stsc.extend_from_slice(&sample_count.to_be_bytes()); // samples_per_chunk
    stsc.extend_from_slice(&1u32.to_be_bytes()); // sample_description_index

    let mut stsz = vec![0u8; 4];
    stsz.extend_from_slice(&sample_size.to_be_bytes()); // constant sample size
    stsz.extend_from_slice(&sample_count.to_be_bytes());

    let mut stco = vec![0u8; 4];
    stco.extend_from_slice(&1u32.to_be_bytes());
    stco.extend_from_slice(&chunk_offset.to_be_bytes());

    let mut stbl_body = bx(b"stsd", &stsd_body);
    stbl_body.extend_from_slice(&bx(b"stts", &stts));
    stbl_body.extend_from_slice(&bx(b"stsc", &stsc));
    stbl_body.extend_from_slice(&bx(b"stsz", &stsz));
    stbl_body.extend_from_slice(&bx(b"stco", &stco));

    let mut hdlr = vec![0u8; 8]; // version/flags + pre_defined
    hdlr.extend_from_slice(b"soun");
    hdlr.extend_from_slice(&[0u8; 12]);

    let mut mdhd = vec![0u8; 4];
    mdhd.extend_from_slice(&[0u8; 8]);
    mdhd.extend_from_slice(&48_000u32.to_be_bytes());
    mdhd.extend_from_slice(&0u32.to_be_bytes());
    mdhd.extend_from_slice(&0x55c4_0000u32.to_be_bytes());

    let mut mdia_body = bx(b"mdhd", &mdhd);
    mdia_body.extend_from_slice(&bx(b"hdlr", &hdlr));
    mdia_body.extend_from_slice(&bx(b"minf", &bx(b"stbl", &stbl_body)));
    bx(b"trak", &bx(b"mdia", &mdia_body))
}

fn aac_trak_with_elst(
    sample_count: u32,
    sample_size: u32,
    chunk_offset: u32,
    media_time: i32,
) -> Vec<u8> {
    let trak = aac_trak(sample_count, sample_size, chunk_offset);
    // Insert edts/elst after the 8-byte trak header, before mdia.
    let mut elst = vec![0u8; 4];
    elst.extend_from_slice(&1u32.to_be_bytes());
    elst.extend_from_slice(&250u32.to_be_bytes());
    elst.extend_from_slice(&(media_time as u32).to_be_bytes());
    elst.extend_from_slice(&0x0001_0000u32.to_be_bytes());
    let edts = bx(b"edts", &bx(b"elst", &elst));
    let mut out = trak[..8].to_vec();
    out.extend_from_slice(&edts);
    out.extend_from_slice(&trak[8..]);
    let size = out.len() as u32;
    out[..4].copy_from_slice(&size.to_be_bytes());
    out
}

fn aac_trak(sample_count: u32, sample_size: u32, chunk_offset: u32) -> Vec<u8> {
    soun_trak(
        mp4a_entry(&bx(b"esds", &esds_body(&ASC))),
        sample_count,
        sample_size,
        chunk_offset,
    )
}

/// A complete M4A: ftyp + moov(`extra_trak`? + AAC trak) + mdat with
/// `sample_count * sample_size` payload bytes. The stco offset is patched
/// in a second pass once the moov length is known.
fn m4a(sample_count: u32, sample_size: u32, extra_trak: Option<&[u8]>) -> Vec<u8> {
    let build = |off: u32| {
        let mut moov_body = Vec::new();
        if let Some(t) = extra_trak {
            moov_body.extend_from_slice(t);
        }
        moov_body.extend_from_slice(&aac_trak(sample_count, sample_size, off));
        let mut data = ftyp();
        data.extend_from_slice(&bx(b"moov", &moov_body));
        data
    };
    let mdat_off = (build(0).len() + 8) as u32;
    let mut data = build(mdat_off);
    data.extend_from_slice(&bx(
        b"mdat",
        &vec![0xAAu8; (sample_count * sample_size) as usize],
    ));
    data
}

#[test]
fn test_happy_path_full_box_walk() {
    // Always-on (no ffmpeg gate): a programmatically built CBR M4A —
    // stsd→esds→ASC, stts/stsc/stsz/stco → sample offsets — must parse
    // end to end. 100 constant-size samples is also the MAJOR-2
    // regression shape: ffmpeg movenc writes stsz without a table.
    let data = m4a(100, 7, None);
    let track = parse_aac_track(&data).expect("valid constant-size M4A must parse");
    assert_eq!(track.asc, ASC);
    assert_eq!(track.total_samples, 100);
    assert_eq!(track.frames.len(), 100);
    assert_eq!(track.skip_samples(48_000), 0);
    let mdat_payload = (data.len() - 700) as u64;
    for (i, &(off, len)) in track.frames.iter().enumerate() {
        assert_eq!(len, 7);
        assert_eq!(off, mdat_payload + (i * 7) as u64);
    }
}

#[test]
fn test_broken_sound_track_skipped_for_valid_aac() {
    // A non-AAC soun track ahead of a valid AAC track must not sink the
    // file: track selection skips it (symphonia parity).
    let alac = soun_trak(bx(b"alac", &[0u8; 28]), 100, 7, 0);
    let data = m4a(100, 7, Some(&alac));
    let track = parse_aac_track(&data).expect("valid AAC track behind a broken one");
    assert_eq!(track.frames.len(), 100);
}

#[test]
fn test_elst_media_time_is_skip_samples() {
    let build = |off: u32| {
        let mut data = ftyp();
        data.extend_from_slice(&bx(b"moov", &aac_trak_with_elst(4, 7, off, 1024)));
        data
    };
    let mdat_off = (build(0).len() + 8) as u32;
    let mut data = build(mdat_off);
    data.extend_from_slice(&bx(b"mdat", &[0xAAu8; 28]));
    let track = parse_aac_track(&data).expect("elst M4A must parse");
    assert_eq!(track.edit_start, 1024);
    assert_eq!(track.media_timescale, 48_000);
    assert_eq!(track.skip_samples(48_000), 1024);
    assert_eq!(track.frames.len(), 4);
}

#[test]
fn test_broken_sound_track_only_is_clean_error() {
    // Only a broken sound track: the same clean error as no audio track.
    let alac = soun_trak(bx(b"alac", &[0u8; 28]), 100, 7, 0);
    let mut data = ftyp();
    data.extend_from_slice(&bx(b"moov", &alac));
    let err = match parse_aac_track(&data) {
        Ok(_) => panic!("file with no usable AAC track must fail"),
        Err(e) => e,
    };
    assert!(format!("{err:?}").contains("no AAC audio track"), "{err:?}");
}

#[test]
fn test_sinf_wrapped_esds_is_clean_error() {
    // CENC-style layout: the esds hides inside a sinf wrapper, which is
    // never descended into, so the track is unusable — a clean refusal.
    let sinf = bx(b"sinf", &bx(b"esds", &esds_body(&ASC)));
    let trak = soun_trak(mp4a_entry(&sinf), 4, 7, 0);
    let mut data = ftyp();
    data.extend_from_slice(&bx(b"moov", &trak));
    let err = match parse_aac_track(&data) {
        Ok(_) => panic!("sinf-wrapped esds must fail"),
        Err(e) => e,
    };
    assert!(format!("{err:?}").contains("no AAC audio track"), "{err:?}");
}

#[test]
fn test_no_moov_is_clean_error() {
    let mut data = ftyp();
    data.extend_from_slice(&bx(b"mdat", &[0u8; 64]));
    let err = match parse_aac_track(&data) {
        Ok(_) => panic!("no moov must fail"),
        Err(e) => e,
    };
    assert!(format!("{err:?}").contains("no moov box"));
}

#[test]
fn test_fragmented_rejected() {
    let mut data = ftyp();
    data.extend_from_slice(&bx(b"moof", &[0u8; 8]));
    data.extend_from_slice(&bx(b"moov", &[]));
    let err = match parse_aac_track(&data) {
        Ok(_) => panic!("fragmented must fail"),
        Err(e) => e,
    };
    assert!(format!("{err:?}").contains("fragmented"));
}

#[test]
fn test_truncated_box_is_clean_error() {
    // ftyp declaring a size past the input end.
    let mut data = (0x1000u32.to_be_bytes()).to_vec();
    data.extend_from_slice(b"ftypM4A ");
    let err = match parse_aac_track(&data) {
        Ok(_) => panic!("truncated box must fail"),
        Err(e) => e,
    };
    let msg = format!("{err:?}");
    assert!(
        msg.contains("past end") || msg.contains("no moov box"),
        "{msg}"
    );
}

#[test]
fn test_garbage_never_panics() {
    let mut state = 0x1234_5678_9abc_def0u64;
    let mut rng = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    for _ in 0..2000 {
        let len = (rng() % 512) as usize;
        let mut data: Vec<u8> = (0..len).map(|_| (rng() >> 32) as u8).collect();
        // Half the cases get a plausible ftyp prefix so the box walk runs.
        if len >= 20 && rng() % 2 == 0 {
            data[..4].copy_from_slice(&20u32.to_be_bytes());
            data[4..8].copy_from_slice(b"ftyp");
        }
        let _ = parse_aac_track(&data); // Err is fine; a panic is a bug.
    }
}

#[test]
fn test_video_track_only_is_clean_error() {
    // A moov with a vide (not soun) track: hdlr says "vide", so no AAC
    // track exists.
    let mut hdlr = vec![0u8; 8];
    hdlr.extend_from_slice(b"vide");
    hdlr.extend_from_slice(&[0u8; 12]);
    let mdia = bx(b"mdia", &bx(b"hdlr", &hdlr));
    let trak = bx(b"trak", &mdia);
    let moov = bx(b"moov", &trak);
    let mut data = ftyp();
    data.extend_from_slice(&moov);
    let err = match parse_aac_track(&data) {
        Ok(_) => panic!("video-only must fail"),
        Err(e) => e,
    };
    assert!(format!("{err:?}").contains("no AAC audio track"));
}

#[test]
fn test_drm_sample_entry_rejected() {
    // stsd with an `enca` (encrypted) entry: the DRM track is unusable,
    // so with no other sound track the file fails the same clean way as
    // having no audio track at all.
    let mut stsd = Vec::new();
    stsd.extend_from_slice(&[0u8; 4]); // version/flags
    stsd.extend_from_slice(&1u32.to_be_bytes()); // one entry
    stsd.extend_from_slice(&bx(b"enca", &[0u8; 28]));
    let stbl = bx(b"stbl", &bx(b"stsd", &stsd));
    let minf = bx(b"minf", &stbl);
    let mut hdlr = vec![0u8; 8];
    hdlr.extend_from_slice(b"soun");
    hdlr.extend_from_slice(&[0u8; 12]);
    let mut mdia_body = bx(b"hdlr", &hdlr);
    mdia_body.extend_from_slice(&minf);
    let mdia = bx(b"mdia", &mdia_body);
    let trak = bx(b"trak", &mdia);
    let moov = bx(b"moov", &trak);
    let mut data = ftyp();
    data.extend_from_slice(&moov);
    let err = match parse_aac_track(&data) {
        Ok(_) => panic!("DRM must fail"),
        Err(e) => e,
    };
    assert!(format!("{err:?}").contains("no AAC audio track"), "{err:?}");
}
