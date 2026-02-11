// Unit tests for RDS decoder
// This file is included via #[cfg(test)] mod in rds.rs

use super::*;

#[test]
fn test_rds_syndrome_zero() {
    // A valid RDS block with correct checkword should have syndrome matching offset
    // For testing, let's verify that syndrome of 0 returns 0 (trivial case)
    let word = 0u32;
    let syn = rds_syndrome(word);
    assert_eq!(syn, 0, "Syndrome of zero should be zero");
}

#[test]
fn test_rds_syndrome_offset_word_a() {
    // Test that syndrome of offset word A equals SYNDROME_A
    // This is how RDS encoding works: for data=0, checkword=offset_word
    // produces the expected syndrome
    let syn = rds_syndrome(OFFSET_WORD_A as u32);
    assert_eq!(
        syn, SYNDROME_A,
        "Syndrome of OFFSET_WORD_A should be SYNDROME_A"
    );
}

#[test]
fn test_rds_syndrome_offset_word_b() {
    let syn = rds_syndrome(OFFSET_WORD_B as u32);
    assert_eq!(
        syn, SYNDROME_B,
        "Syndrome of OFFSET_WORD_B should be SYNDROME_B"
    );
}

#[test]
fn test_rds_syndrome_offset_word_c() {
    let syn = rds_syndrome(OFFSET_WORD_C as u32);
    assert_eq!(
        syn, SYNDROME_C,
        "Syndrome of OFFSET_WORD_C should be SYNDROME_C"
    );
}

#[test]
fn test_rds_syndrome_offset_word_d() {
    let syn = rds_syndrome(OFFSET_WORD_D as u32);
    assert_eq!(
        syn, SYNDROME_D,
        "Syndrome of OFFSET_WORD_D should be SYNDROME_D"
    );
}

#[test]
fn test_rds_syndrome_data_only() {
    // Test with arbitrary 16-bit data in upper 16 bits
    let data = 0x1234u16;
    let word26 = (data as u32) << 10;
    let syn = rds_syndrome(word26);
    // Syndrome should be non-zero (no checkword added yet)
    assert!(
        syn > 0,
        "Data without checkword should have non-zero syndrome"
    );
}

#[test]
fn test_rds_offset_for_syndrome_a() {
    let result = rds_offset_for_syndrome(SYNDROME_A);
    assert_eq!(result, Some('A'), "SYNDROME_A should return 'A'");
}

#[test]
fn test_rds_offset_for_syndrome_b() {
    let result = rds_offset_for_syndrome(SYNDROME_B);
    assert_eq!(result, Some('B'), "SYNDROME_B should return 'B'");
}

#[test]
fn test_rds_offset_for_syndrome_c() {
    let result = rds_offset_for_syndrome(SYNDROME_C);
    assert_eq!(result, Some('C'), "SYNDROME_C should return 'C'");
}

#[test]
fn test_rds_offset_for_syndrome_c_prime() {
    let result = rds_offset_for_syndrome(SYNDROME_C_PRIME);
    assert_eq!(result, Some('c'), "SYNDROME_C_PRIME should return 'c'");
}

#[test]
fn test_rds_offset_for_syndrome_d() {
    let result = rds_offset_for_syndrome(SYNDROME_D);
    assert_eq!(result, Some('D'), "SYNDROME_D should return 'D'");
}

#[test]
fn test_rds_offset_for_syndrome_invalid() {
    let result = rds_offset_for_syndrome(0x123);
    assert_eq!(result, None, "Invalid syndrome should return None");
}

#[test]
fn test_rds_data_from_word() {
    let data = 0x1234u16;
    let word26 = (data as u32) << 10 | 0x3FF; // data in bits 25..10, junk in 9..0
    let extracted = rds_data_from_word(word26);
    assert_eq!(extracted, data, "Should extract upper 16 bits as data");
}

#[test]
fn test_rds_data_from_word_zero() {
    let word26 = 0x000003FFu32; // zero data, junk checkword
    let extracted = rds_data_from_word(word26);
    assert_eq!(extracted, 0, "Should extract zero data correctly");
}

#[test]
fn test_rds_data_from_word_all_ones() {
    let word26 = 0x03FFFFFFu32; // all bits set in 26-bit word
    let extracted = rds_data_from_word(word26);
    assert_eq!(extracted, 0xFFFF, "Should extract all-ones data correctly");
}

#[test]
fn test_rds_parser_new() {
    let parser = RdsParser::new();
    assert_eq!(parser.shift, 0);
    assert_eq!(parser.shift_len, 0);
    assert_eq!(parser.ps, [b' '; 8]);
    assert_eq!(parser.ps_received_mask, 0);
    assert!(!parser.is_synced);
}

#[test]
fn test_rds_parser_verbose_mode() {
    let mut parser = RdsParser::new();
    assert!(!parser.verbose, "Verbose should be false by default");

    parser.set_verbose(true);
    assert!(
        parser.verbose,
        "Verbose should be true after set_verbose(true)"
    );

    parser.set_verbose(false);
    assert!(
        !parser.verbose,
        "Verbose should be false after set_verbose(false)"
    );
}

#[test]
fn test_rds_parser_station_name_empty() {
    let parser = RdsParser::new();
    assert_eq!(
        parser.station_name(),
        None,
        "Station name should be None initially"
    );
}

#[test]
fn test_rds_parser_radio_text_empty() {
    let parser = RdsParser::new();
    assert_eq!(
        parser.radio_text(),
        None,
        "Radio text should be None initially"
    );
}

#[test]
fn test_rds_parser_program_type() {
    let parser = RdsParser::new();
    assert_eq!(
        parser.station_info().program_type,
        0,
        "Program type should be 0 initially"
    );
    assert_eq!(
        parser.program_type_name(),
        "None",
        "Program type name should be 'None'"
    );
}

#[test]
fn test_rds_parser_push_bits_insufficient() {
    // Push fewer than 26 bits, should not detect any blocks
    let mut parser = RdsParser::new();
    let bits: Vec<u8> = vec![1, 0, 1, 0, 1]; // 5 bits
    parser.push_bits(&bits);
    let (_, blocks, _) = parser.stats();
    assert_eq!(blocks, 0, "Should not detect blocks with <26 bits");
}

#[test]
fn test_rds_parser_push_bits_invalid_block() {
    // Push 26 random bits that won't match any offset syndrome
    let mut parser = RdsParser::new();
    let bits: Vec<u8> = vec![0; 26]; // all zeros
    parser.push_bits(&bits);
    // All zeros has syndrome 0, which doesn't match any offset
    // So no blocks should be detected
    let (_, blocks, _) = parser.stats();
    assert_eq!(blocks, 0, "Random bits should not produce valid blocks");
}

#[test]
fn test_rds_parser_rt_needs_multiple_segments() {
    // RT needs at least 2 segments to be considered valid
    let mut parser = RdsParser::new();
    parser.rt_received_mask[0] = true;
    assert!(
        parser.radio_text().is_none(),
        "1 segment should not be enough for RT"
    );

    parser.rt_received_mask[1] = true;
    parser.rt[0] = b'H';
    parser.rt[1] = b'I';
    assert!(
        parser.radio_text().is_some(),
        "2 segments should be enough for RT"
    );
}

#[test]
fn test_rds_pty_names() {
    // Verify some PTY names are correct
    assert_eq!(PTY_NAMES[0], "None");
    assert_eq!(PTY_NAMES[1], "News");
    assert_eq!(PTY_NAMES[11], "Classical");
}

#[test]
fn test_rds_di_flags_names() {
    // Test DI flag names
    assert_eq!(DIFlags::flag_name(0), "dynamic_pty");
    assert_eq!(DIFlags::flag_name(1), "compressed");
    assert_eq!(DIFlags::flag_name(2), "artificial_head");
    assert_eq!(DIFlags::flag_name(3), "stereo");
    assert_eq!(DIFlags::flag_name(4), "unknown");
}

#[test]
fn test_rds_di_flags_set_get() {
    let mut di = DIFlags::default();
    assert!(!di.stereo);

    di.set_flag(3, true); // stereo
    assert!(di.get_flag(3));
    assert!(di.stereo);

    di.set_flag(0, true); // dynamic_pty
    assert!(di.dynamic_pty);
}

#[test]
fn test_rds_di_flags_as_string() {
    let di = DIFlags {
        stereo: true,
        ..Default::default()
    };
    let s = di.as_string();
    assert!(s.contains("stereo=true"));
    assert!(s.contains("dynamic_pty=false"));
}

#[test]
fn test_rds_slc_variant_names() {
    // Test SLC variant name lookup
    assert_eq!(RdsParser::slc_variant_name(0), "ECC and PI");
    assert_eq!(RdsParser::slc_variant_name(3), "Language");
    assert_eq!(RdsParser::slc_variant_name(255), "Unknown");
}

/// Helper: create a valid RDS block by finding the correct checkword
/// that produces the expected syndrome for the given offset type.
fn create_rds_block(data: u16, target_syndrome: u16) -> u32 {
    let data_part = (data as u32) << 10;
    let data_syndrome = rds_syndrome(data_part);

    // We need to find checkword such that:
    // rds_syndrome(data_part | checkword) == target_syndrome
    // Due to linearity: data_syndrome XOR checkword_syndrome == target_syndrome
    // So we need checkword_syndrome = data_syndrome XOR target_syndrome
    let needed_checkword_syndrome = data_syndrome ^ target_syndrome;

    // Search for checkword (brute force - only 1024 possibilities)
    for checkword in 0u32..1024 {
        if rds_syndrome(checkword) == needed_checkword_syndrome {
            return data_part | checkword;
        }
    }

    // Fallback (should never happen for valid syndromes)
    panic!(
        "Could not find valid checkword for syndrome 0x{:03X}",
        target_syndrome
    );
}

/// Helper to convert a 26-bit word to bit array (MSB first)
fn word_to_bits(word: u32) -> Vec<u8> {
    (0..26).rev().map(|i| ((word >> i) & 1) as u8).collect()
}

#[test]
fn test_create_and_decode_valid_block_a() {
    // Test that a valid block A is detected and counted
    let mut parser = RdsParser::new();
    let data = 0x1234u16;
    let block = create_rds_block(data, SYNDROME_A);
    let bits = word_to_bits(block);

    parser.push_bits(&bits);

    let (_, blocks, _) = parser.stats();
    assert_eq!(blocks, 1, "Should have detected one valid block");
    // With sync-based approach, a single block doesn't acquire sync
    assert!(
        !parser.is_synced,
        "Should not be synced after just one block"
    );
}

#[test]
fn test_create_and_decode_valid_block_b() {
    // Test that a valid block B is detected
    let mut parser = RdsParser::new();
    let data = 0x5678u16;
    let block = create_rds_block(data, SYNDROME_B);
    let bits = word_to_bits(block);

    parser.push_bits(&bits);

    let (_, blocks, _) = parser.stats();
    assert_eq!(blocks, 1, "Should have detected one valid block");
}

#[test]
fn test_rds_parser_debug_format() {
    let mut parser = RdsParser::new();
    parser.ps = *b"TESTFM  ";
    let debug_str = format!("{:?}", parser);
    assert!(
        debug_str.contains("TESTFM"),
        "Debug format should show PS name"
    );
}

#[test]
fn test_rds_sync_acquisition_with_full_group() {
    // Test sync acquisition by feeding two complete A,B,C,D sequences
    // The first group is used for sync acquisition (may be incomplete)
    // The second group should be fully decoded
    let mut parser = RdsParser::new();

    // Create 4 blocks in sequence: A, B, C, D (first group - for sync)
    let block_a = create_rds_block(0xF212, SYNDROME_A); // PI code
    let block_b = create_rds_block(0x0408, SYNDROME_B); // Group 0A, segment 0
    let block_c = create_rds_block(0xE20E, SYNDROME_C); // AF data
    let block_d = create_rds_block(0x2020, SYNDROME_D); // PS "  "

    // Second group (same data, should be fully decoded)
    let block_a2 = create_rds_block(0xF212, SYNDROME_A);
    let block_b2 = create_rds_block(0x0408, SYNDROME_B);
    let block_c2 = create_rds_block(0xE20E, SYNDROME_C);
    let block_d2 = create_rds_block(0x2020, SYNDROME_D);

    // Convert to bits and feed in sequence
    let mut all_bits = Vec::new();
    all_bits.extend(word_to_bits(block_a));
    all_bits.extend(word_to_bits(block_b));
    all_bits.extend(word_to_bits(block_c));
    all_bits.extend(word_to_bits(block_d));
    all_bits.extend(word_to_bits(block_a2));
    all_bits.extend(word_to_bits(block_b2));
    all_bits.extend(word_to_bits(block_c2));
    all_bits.extend(word_to_bits(block_d2));

    parser.push_bits(&all_bits);

    let (_, blocks, groups) = parser.stats();
    assert!(
        blocks >= 6,
        "Should have detected at least 6 blocks, got {}",
        blocks
    );
    assert!(
        parser.is_synced,
        "Should have acquired sync after A,B,C,D sequence"
    );
    assert!(
        groups >= 1,
        "Should have decoded at least 1 group, got {}",
        groups
    );
}

#[test]
fn test_rds_parser_default() {
    let parser = RdsParser::default();
    assert_eq!(parser.shift, 0);
    assert_eq!(parser.shift_len, 0);
}

// Group parsing tests
// Note: Full end-to-end group parsing tests are challenging because the RDS decoder
// uses a sliding window that can find false positives when perfect back-to-back blocks
// are fed without proper timing/gaps. The tests below focus on verifying the core
// group detection and parsing logic works correctly.

#[test]
fn test_rds_parser_station_name_setter() {
    // Test PS name assembly using update_ps which requires sequential segments
    let mut parser = RdsParser::new();

    // Simulate receiving all 4 segments in sequence via update_ps
    // This triggers the new sequential tracking and caching behavior
    // Segment 0: "AB"
    parser.update_ps(0, b'A', b'B');
    // Segment 1: "CD"
    parser.update_ps(1, b'C', b'D');
    // Segment 2: "EF"
    parser.update_ps(2, b'E', b'F');
    // Segment 3: "GH"
    parser.update_ps(3, b'G', b'H');

    let ps = parser.station_name();
    assert!(
        ps.is_some(),
        "PS should be Some after all 4 segments received sequentially"
    );
    assert_eq!(ps.unwrap(), "ABCDEFGH");
}

#[test]
fn test_rds_parser_radiotext_assembly() {
    // Test RT assembly manually
    let mut parser = RdsParser::new();

    // Set some RT segments
    parser.rt[0] = b'H';
    parser.rt[1] = b'E';
    parser.rt[2] = b'L';
    parser.rt[3] = b'L';
    parser.rt_received_mask[0] = true;

    parser.rt[4] = b'O';
    parser.rt[5] = b' ';
    parser.rt[6] = b'W';
    parser.rt[7] = b'O';
    parser.rt_received_mask[1] = true;

    let rt = parser.radio_text();
    assert!(rt.is_some());
    assert!(rt.unwrap().starts_with("HELLO WO"));
}

#[test]
fn test_rds_parser_di_flags_extraction() {
    // Test that DI flags are properly set via set_flag
    let mut di = DIFlags::default();

    // Segment 0 sets dynamic_pty
    di.set_flag(0, true);
    assert!(di.dynamic_pty);
    assert!(!di.compressed);
    assert!(!di.artificial_head);
    assert!(!di.stereo);

    // Segment 3 sets stereo
    di.set_flag(3, true);
    assert!(di.stereo);
}

#[test]
fn test_rds_real_station_99_1_rfm() {
    // Test with real values captured from 99.1 MHz RFM station
    // PI = 0xF212, Group 0A, TP=true, PTY=0, DI stereo=true
    // This tests the Group 0A parsing logic directly

    let mut parser = RdsParser::new();
    parser.set_verbose(false);

    // Manually construct Group 0A data based on real station values
    // Block 1 (A): PI code = 0xF212
    // Block 2 (B): 0x0408 = group_type=0, version=0, TP=1, PTY=0, TA=0, music=1, DI=0, segment=0
    // Block 3 (C): 0xE20E = AF codes
    // Block 4 (D): 0x2020 = "  " (space space)

    let datas: [u16; 4] = [0xF212, 0x0408, 0xE20E, 0x2020];

    // Call handle_group directly to test parsing (all blocks valid)
    parser.handle_group(datas, [true, true, true, true]);

    // Verify extracted data
    assert!(parser.station_info.is_traffic_program, "TP should be true");
    assert!(
        !parser.station_info.is_traffic_announcement,
        "TA should be false"
    );
    assert_eq!(parser.station_info.program_type, 0, "PTY should be 0");
    assert!(parser.station_info.is_music, "Music flag should be true");

    // Check that PS segment 0 was set to "  "
    assert_eq!(parser.ps[0], 0x20, "PS char 0 should be space");
    assert_eq!(parser.ps[1], 0x20, "PS char 1 should be space");
    assert_eq!(
        parser.ps_received_mask & 1,
        1,
        "PS segment 0 should be received"
    );
}

#[test]
fn test_rds_comprehensive_real_station_sequence() {
    // Simulate a complete sequence of Group 0A messages to build PS name "  RFM   "
    let mut parser = RdsParser::new();

    // Group 0A segment 0: "  " (spaces)
    let group0: [u16; 4] = [0xF212, 0x0408, 0xE20E, 0x2020]; // segment 0, chars "  "
    parser.handle_group(group0, [true, true, true, true]);

    // Group 0A segment 1: "RF"
    let group1: [u16; 4] = [0xF212, 0x0409, 0xE20E, 0x5246]; // segment 1, chars "RF"
    parser.handle_group(group1, [true, true, true, true]);

    // Group 0A segment 2: "M "
    let group2: [u16; 4] = [0xF212, 0x040A, 0xE20E, 0x4D20]; // segment 2, chars "M "
    parser.handle_group(group2, [true, true, true, true]);

    // Group 0A segment 3: "  " (spaces)
    let group3: [u16; 4] = [0xF212, 0x040B, 0xE20E, 0x2020]; // segment 3, chars "  "
    parser.handle_group(group3, [true, true, true, true]);

    // Now PS should be complete: "  RFM   "
    let ps = parser.station_name();
    assert!(ps.is_some(), "PS should be available after all segments");
    let ps_str = ps.unwrap();
    assert!(
        ps_str.contains("RFM"),
        "PS should contain 'RFM', got: '{}'",
        ps_str
    );
}

#[test]
fn test_rds_parser_stats() {
    let mut parser = RdsParser::new();

    let (bits, blocks, groups) = parser.stats();
    assert_eq!(bits, 0);
    assert_eq!(blocks, 0);
    assert_eq!(groups, 0);

    // Push some bits
    parser.push_bits(&[0, 1, 0, 1]);
    let (bits, _, _) = parser.stats();
    assert_eq!(bits, 4);
}

#[test]
fn test_rds_parser_has_data() {
    let mut parser = RdsParser::new();
    assert!(!parser.has_data(), "Should have no data initially");

    // Simulate receiving a group
    parser.groups_decoded = 1;
    assert!(parser.has_data(), "Should have data after group decoded");
}
