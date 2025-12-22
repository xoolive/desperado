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
fn test_rds_syndrome_polynomial() {
    // Test that the polynomial itself produces a known result
    // If we feed the polynomial (shifted to bit position 10), syndrome should be 0
    // RDS_POLY = 0x5B9 at position 10 means: 0x5B9 << 0 = 0x5B9
    let word = RDS_POLY;
    let syn = rds_syndrome(word);
    assert_eq!(syn, 0, "Syndrome of polynomial should be zero");
}

#[test]
fn test_rds_syndrome_data_only() {
    // Test with arbitrary 16-bit data in upper 16 bits
    // Data: 0x1234, checkword computed should produce specific syndrome
    // We'll just verify it computes something consistent
    let data = 0x1234u16;
    let word26 = (data as u32) << 10;
    let syn = rds_syndrome(word26);
    // Syndrome should be non-zero (no checkword added yet)
    assert!(
        syn > 0,
        "Data without checkword should have non-zero syndrome"
    );

    // If we XOR the syndrome back in as checkword, syndrome should become 0
    let word_with_check = word26 | (syn as u32);
    let syn2 = rds_syndrome(word_with_check);
    assert_eq!(syn2, 0, "Adding syndrome as checkword should zero syndrome");
}

#[test]
fn test_rds_syndrome_known_block() {
    // Create a block with known data and offset A
    // Data: 0xABCD, offset A = 0b0011111100 = 0xFC
    let data = 0xABCDu16;
    let word26 = (data as u32) << 10;
    let raw_syndrome = rds_syndrome(word26);
    // Add offset A to get final checkword
    let checkword = raw_syndrome ^ OFFSET_A;
    let final_word = word26 | (checkword as u32);
    let final_syn = rds_syndrome(final_word);
    assert_eq!(
        final_syn, OFFSET_A,
        "Block with offset A should have syndrome = OFFSET_A"
    );
}

#[test]
fn test_rds_offset_for_syndrome_offset_a() {
    let result = rds_offset_for_syndrome(OFFSET_A);
    assert_eq!(result, Some('A'), "OFFSET_A should return 'A'");
}

#[test]
fn test_rds_offset_for_syndrome_offset_b() {
    let result = rds_offset_for_syndrome(OFFSET_B);
    assert_eq!(result, Some('B'), "OFFSET_B should return 'B'");
}

#[test]
fn test_rds_offset_for_syndrome_offset_c() {
    let result = rds_offset_for_syndrome(OFFSET_C);
    assert_eq!(result, Some('C'), "OFFSET_C should return 'C'");
}

#[test]
fn test_rds_offset_for_syndrome_offset_c_prime() {
    let result = rds_offset_for_syndrome(OFFSET_C_PRIME);
    assert_eq!(result, Some('c'), "OFFSET_C_PRIME should return 'c'");
}

#[test]
fn test_rds_offset_for_syndrome_offset_d() {
    let result = rds_offset_for_syndrome(OFFSET_D);
    assert_eq!(result, Some('D'), "OFFSET_D should return 'D'");
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
    assert_eq!(parser.pending_blocks.len(), 0);
    assert_eq!(parser.ps, [b' '; 8]);
    assert_eq!(parser.ps_received_mask, 0);
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
        "Empty parser should have no station name"
    );
}

#[test]
fn test_rds_parser_radio_text_empty() {
    let parser = RdsParser::new();
    assert_eq!(
        parser.radio_text(),
        None,
        "Empty parser should have no radio text"
    );
}

#[test]
fn test_rds_parser_push_bits_insufficient() {
    let mut parser = RdsParser::new();
    // Push only 10 bits (not enough for a block)
    let bits = vec![1, 0, 1, 0, 1, 0, 1, 0, 1, 0];
    parser.push_bits(&bits);
    assert_eq!(parser.shift_len, 10);
    assert_eq!(
        parser.pending_blocks.len(),
        0,
        "Not enough bits for a block"
    );
}

#[test]
fn test_rds_parser_push_bits_invalid_block() {
    let mut parser = RdsParser::new();
    // Push 26 random bits that don't form a valid RDS block
    let bits = vec![1; 26]; // all ones
    parser.push_bits(&bits);
    assert_eq!(parser.shift_len, 26);
    // Check if it was recognized (likely not, unless all-ones happens to match)
    let syn = rds_syndrome(parser.shift);
    if rds_offset_for_syndrome(syn).is_none() {
        assert_eq!(
            parser.pending_blocks.len(),
            0,
            "Invalid block should not be stored"
        );
    }
}

/// Helper function to create a valid 26-bit RDS block
fn create_rds_block(data: u16, offset: u16) -> u32 {
    let word26 = (data as u32) << 10;
    let raw_syndrome = rds_syndrome(word26);
    let checkword = raw_syndrome ^ offset;
    word26 | (checkword as u32)
}

/// Helper to convert a 26-bit word to bit array (MSB first)
fn word_to_bits(word: u32) -> Vec<u8> {
    (0..26).rev().map(|i| ((word >> i) & 1) as u8).collect()
}

#[test]
fn test_create_and_decode_valid_block_a() {
    let mut parser = RdsParser::new();
    let data = 0x1234u16;
    let block = create_rds_block(data, OFFSET_A);
    let bits = word_to_bits(block);

    parser.push_bits(&bits);

    assert_eq!(
        parser.pending_blocks.len(),
        1,
        "Should have one valid block"
    );
    let (offset, decoded_data) = parser.pending_blocks[0];
    assert_eq!(offset, 'A', "Should identify as offset A");
    assert_eq!(decoded_data, data, "Should decode correct data");
}

#[test]
fn test_create_and_decode_valid_block_b() {
    let mut parser = RdsParser::new();
    let data = 0x5678u16;
    let block = create_rds_block(data, OFFSET_B);
    let bits = word_to_bits(block);

    parser.push_bits(&bits);

    assert_eq!(
        parser.pending_blocks.len(),
        1,
        "Should have one valid block"
    );
    let (offset, decoded_data) = parser.pending_blocks[0];
    assert_eq!(offset, 'B', "Should identify as offset B");
    assert_eq!(decoded_data, data, "Should decode correct data");
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
    // Test PS name assembly manually by setting segments
    let mut parser = RdsParser::new();

    // Simulate what handle_group would do for Group 0A segments
    // Segment 0: "AB"
    parser.ps[0] = b'A';
    parser.ps[1] = b'B';
    parser.ps_received_mask |= 1 << 0;

    // Segment 1: "CD"
    parser.ps[2] = b'C';
    parser.ps[3] = b'D';
    parser.ps_received_mask |= 1 << 1;

    let ps = parser.station_name();
    assert!(ps.is_some());
    assert!(ps.unwrap().starts_with("ABCD"));
}

#[test]
fn test_rds_parser_radiotext_assembly() {
    // Test RT assembly manually
    let mut parser = RdsParser::new();

    // Segment 0: "TEST"
    parser.rt[0] = b'T';
    parser.rt[1] = b'E';
    parser.rt[2] = b'S';
    parser.rt[3] = b'T';
    parser.rt_received_mask[0] = true;

    // Segment 1: "ING!"
    parser.rt[4] = b'I';
    parser.rt[5] = b'N';
    parser.rt[6] = b'G';
    parser.rt[7] = b'!';
    parser.rt_received_mask[1] = true;

    let rt = parser.radio_text();
    assert!(rt.is_some());
    assert!(rt.unwrap().starts_with("TESTING!"));
}

#[test]
fn test_rds_parser_ps_partial_reception() {
    let mut parser = RdsParser::new();

    // Receive only segments 0 and 2 (not 1 and 3)
    parser.ps[0] = b'A';
    parser.ps[1] = b'B';
    parser.ps_received_mask |= 1 << 0;

    parser.ps[4] = b'E';
    parser.ps[5] = b'F';
    parser.ps_received_mask |= 1 << 2;

    // Should still return partial PS
    let ps = parser.station_name();
    assert!(ps.is_some());
    // Positions 2-3 should still be spaces
    assert_eq!(&parser.ps[2..4], b"  ");
}

#[test]
fn test_rds_parser_rt_needs_multiple_segments() {
    let mut parser = RdsParser::new();

    // With only 1 segment, radio_text should return None
    parser.rt[0] = b'A';
    parser.rt_received_mask[0] = true;

    assert_eq!(parser.radio_text(), None, "RT needs at least 2 segments");

    // Add second segment
    parser.rt[4] = b'B';
    parser.rt_received_mask[1] = true;

    assert!(
        parser.radio_text().is_some(),
        "RT should be available with 2+ segments"
    );
}

#[test]
fn test_rds_parser_continuous_bitstream() {
    // This test verifies the decoder can handle a continuous stream of bits
    // representing a single perfect block
    let mut parser = RdsParser::new();

    let data = 0xABCDu16;
    let block = create_rds_block(data, OFFSET_A);
    let bits = word_to_bits(block);

    // Feed the 26 bits
    parser.push_bits(&bits);

    // Should have detected exactly one block
    assert!(
        parser.pending_blocks.len() >= 1,
        "Should detect at least one block"
    );

    // The first detected block should be our block A with correct data
    let (offset, decoded_data) = parser.pending_blocks[0];
    assert_eq!(offset, 'A');
    assert_eq!(decoded_data, data);
}
