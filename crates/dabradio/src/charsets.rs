//! EBU Latin character set conversion for DAB labels.
//!
//! DAB service labels use the EBU Latin character set (charset 0)
//! defined in ETSI TS 101 756, Table 1.
//! This maps byte values 0x00-0xFF to Unicode code points.

/// Convert EBU Latin (charset 0) bytes to a UTF-8 string.
pub fn ebu_latin_to_utf8(bytes: &[u8]) -> String {
    bytes.iter().map(|&b| ebu_to_char(b)).collect()
}

/// Map a single EBU Latin byte to a Unicode character.
fn ebu_to_char(b: u8) -> char {
    // The EBU Latin table is mostly ASCII-compatible for 0x20-0x7E.
    // Control chars and 0x80+ have special mappings.
    // We implement the most common mappings; rare ones fall back to replacement char.
    match b {
        // Standard ASCII printable range
        0x20..=0x7E => b as char,
        // Common EBU Latin extensions
        0x80 => '\u{00E1}', // a-acute
        0x81 => '\u{00E0}', // a-grave
        0x82 => '\u{00E9}', // e-acute
        0x83 => '\u{00E8}', // e-grave
        0x84 => '\u{00ED}', // i-acute
        0x85 => '\u{00EC}', // i-grave
        0x86 => '\u{00F3}', // o-acute
        0x87 => '\u{00F2}', // o-grave
        0x88 => '\u{00FA}', // u-acute
        0x89 => '\u{00F9}', // u-grave
        0x8A => '\u{00D1}', // N-tilde
        0x8B => '\u{00C7}', // C-cedilla
        0x8C => '\u{015E}', // S-cedilla
        0x8D => '\u{00DF}', // sharp-s
        0x8E => '\u{00A1}', // inverted !
        0x8F => '\u{0132}', // IJ ligature
        0x90 => '\u{00E2}', // a-circumflex
        0x91 => '\u{00E4}', // a-diaeresis
        0x92 => '\u{00EA}', // e-circumflex
        0x93 => '\u{00EB}', // e-diaeresis
        0x94 => '\u{00EE}', // i-circumflex
        0x95 => '\u{00EF}', // i-diaeresis
        0x96 => '\u{00F4}', // o-circumflex
        0x97 => '\u{00F6}', // o-diaeresis
        0x98 => '\u{00FB}', // u-circumflex
        0x99 => '\u{00FC}', // u-diaeresis
        0x9A => '\u{00F1}', // n-tilde
        0x9B => '\u{00E7}', // c-cedilla
        0x9C => '\u{015F}', // s-cedilla
        0x9D => '\u{011F}', // g-breve
        0x9E => '\u{0131}', // dotless-i
        0x9F => '\u{0133}', // ij ligature
        0xA0 => '\u{00AA}', // feminine ordinal
        0xA1 => '\u{03B1}', // alpha
        0xA8 => '\u{00A3}', // pound sign
        0xAD => '\u{00A4}', // currency sign
        // Space / control characters
        0x00..=0x1F | 0x7F => ' ',
        // For unmapped bytes, use the Unicode replacement or try Latin-1 fallback
        _ => {
            // Many EBU values above 0xA0 roughly correspond to ISO 8859-1 / Latin-1
            char::from_u32(b as u32).unwrap_or('\u{FFFD}')
        }
    }
}
