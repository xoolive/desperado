//! EBU Latin character set conversion for DAB labels.
//!
//! DAB service labels use the EBU Latin character set (charset 0)
//! defined in ETSI TS 101 756, Table 1.
//! This maps byte values 0x00-0xFF to Unicode code points.

/// Convert EBU Latin (charset 0) bytes to a UTF-8 string.
pub fn ebu_latin_to_utf8(bytes: &[u8]) -> String {
    bytes.iter().map(|&b| ebu_to_char(b)).collect()
}

fn ebu_to_char(b: u8) -> char {
    EBU_LATIN[b as usize]
}

static EBU_LATIN: [char; 256] = [
    // 0x00–0x0F
    '\u{FFFD}', // 0x00 — reserved
    '\u{0118}', // 0x01 Ę
    '\u{012E}', // 0x02 Į
    '\u{0172}', // 0x03 Ų
    '\u{0102}', // 0x04 Ă
    '\u{0116}', // 0x05 Ė
    '\u{010E}', // 0x06 Ď
    '\u{0218}', // 0x07 Ș
    '\u{021A}', // 0x08 Ț
    '\u{010A}', // 0x09 Ċ
    '\u{FFFD}', // 0x0A — reserved
    '\u{FFFD}', // 0x0B — reserved
    '\u{0120}', // 0x0C Ġ
    '\u{0139}', // 0x0D Ĺ
    '\u{017B}', // 0x0E Ż
    '\u{0143}', // 0x0F Ń

    // 0x10–0x1F
    '\u{0105}', // 0x10 ą
    '\u{0119}', // 0x11 ę
    '\u{012F}', // 0x12 į
    '\u{0173}', // 0x13 ų
    '\u{0103}', // 0x14 ă
    '\u{0117}', // 0x15 ė
    '\u{010F}', // 0x16 ď
    '\u{0219}', // 0x17 ș
    '\u{021B}', // 0x18 ț
    '\u{010B}', // 0x19 ċ
    '\u{0147}', // 0x1A Ň
    '\u{011A}', // 0x1B Ě
    '\u{0121}', // 0x1C ġ
    '\u{013A}', // 0x1D ĺ
    '\u{017C}', // 0x1E ż
    '\u{FFFD}', // 0x1F — reserved

    // 0x20–0x2F
    ' ',        // 0x20 ␠
    '!',        // 0x21 !
    '"',        // 0x22 "
    '#',        // 0x23 #
    '\u{0142}', // 0x24 ł
    '%',        // 0x25 %
    '&',        // 0x26 &
    '\'',       // 0x27 '
    '(',        // 0x28 (
    ')',        // 0x29 )
    '*',        // 0x2A *
    '+',        // 0x2B +
    ',',        // 0x2C ,
    '-',        // 0x2D -
    '.',        // 0x2E .
    '/',        // 0x2F /

    // 0x30–0x3F
    '0','1','2','3','4','5','6','7',
    '8','9',':',';','<','=','>','?',

    // 0x40–0x4F
    '@','A','B','C','D','E','F','G',
    'H','I','J','K','L','M','N','O',

    // 0x50–0x5F
    'P','Q','R','S','T','U','V','W',
    'X','Y','Z','[','\u{016E}',']','\u{0141}','_',
    //                 Ů           Ł

    // 0x60–0x6F
    '\u{0104}', // 0x60 Ą
    'a','b','c','d','e','f','g',
    'h','i','j','k','l','m','n','o',

    // 0x70–0x7F
    'p','q','r','s','t','u','v','w',
    'x','y','z',
    '\u{00AB}', // 0x7B «
    '\u{016F}', // 0x7C ů
    '\u{00BB}', // 0x7D »
    '\u{013D}', // 0x7E Ľ
    '\u{0126}', // 0x7F Ħ

    // 0x80–0x8F
    '\u{00E1}', // á
    '\u{00E0}', // à
    '\u{00E9}', // é
    '\u{00E8}', // è
    '\u{00ED}', // í
    '\u{00EC}', // ì
    '\u{00F3}', // ó
    '\u{00F2}', // ò
    '\u{00FA}', // ú
    '\u{00F9}', // ù
    '\u{00D1}', // Ñ
    '\u{00C7}', // Ç
    '\u{015E}', // Ş
    '\u{00DF}', // ß
    '\u{00A1}', // ¡
    '\u{0178}', // Ÿ

    // 0x90–0x9F
    '\u{00E2}', // â
    '\u{00E4}', // ä
    '\u{00EA}', // ê
    '\u{00EB}', // ë
    '\u{00EE}', // î
    '\u{00EF}', // ï
    '\u{00F4}', // ô
    '\u{00F6}', // ö
    '\u{00FB}', // û
    '\u{00FC}', // ü
    '\u{00F1}', // ñ
    '\u{00E7}', // ç
    '\u{015F}', // ş
    '\u{011F}', // ğ
    '\u{0131}', // ı
    '\u{00FF}', // ÿ

    // 0xA0–0xAF
    '\u{0136}', // Ķ
    '\u{0145}', // Ņ
    '\u{00A9}', // ©
    '\u{0122}', // Ģ
    '\u{011E}', // Ğ
    '\u{011B}', // ě
    '\u{0148}', // ň
    '\u{0151}', // ő
    '\u{0150}', // Ő
    '\u{20AC}', // €
    '\u{00A3}', // £
    '$',        // $
    '\u{0100}', // Ā
    '\u{0112}', // Ē
    '\u{012A}', // Ī
    '\u{016A}', // Ū

    // 0xB0–0xBF
    '\u{0137}', // ķ
    '\u{0146}', // ņ
    '\u{013B}', // Ļ
    '\u{0123}', // ģ
    '\u{013C}', // ļ
    '\u{0130}', // İ
    '\u{0144}', // ń
    '\u{0171}', // ű
    '\u{0170}', // Ű
    '\u{00BF}', // ¿
    '\u{013E}', // ľ
    '\u{00B0}', // °
    '\u{0101}', // ā
    '\u{0113}', // ē
    '\u{012B}', // ī
    '\u{016B}', // ū

    // 0xC0–0xCF
    '\u{00C1}','\u{00C0}','\u{00C9}','\u{00C8}',
    '\u{00CD}','\u{00CC}','\u{00D3}','\u{00D2}',
    '\u{00DA}','\u{00D9}','\u{0158}','\u{010C}',
    '\u{0160}','\u{017D}','\u{00D0}','\u{013F}',

    // 0xD0–0xDF
    '\u{00C2}','\u{00C4}','\u{00CA}','\u{00CB}',
    '\u{00CE}','\u{00CF}','\u{00D4}','\u{00D6}',
    '\u{00DB}','\u{00DC}','\u{0159}','\u{010D}',
    '\u{0161}','\u{017E}','\u{0111}','\u{0140}',

    // 0xE0–0xEF
    '\u{00C3}','\u{00C5}','\u{00C6}','\u{0152}',
    '\u{0177}','\u{00DD}','\u{00D5}','\u{00D8}',
    '\u{00DE}','\u{014A}','\u{0154}','\u{0106}',
    '\u{015A}','\u{0179}','\u{0164}','\u{00F0}',

    // 0xF0–0xFF
    '\u{00E3}','\u{00E5}','\u{00E6}','\u{0153}',
    '\u{0175}','\u{00FD}','\u{00F5}','\u{00F8}',
    '\u{00FE}','\u{014B}','\u{0155}','\u{0107}',
    '\u{015B}','\u{017A}','\u{0165}','\u{0127}',
];
///
/// Map a single EBU Latin byte to a Unicode character.
fn ebu_to_char2(b: u8) -> char {
    // The EBU Latin table is mostly ASCII-compatible for 0x20-0x7E.
    // Control chars and 0x80+ have special mappings.
    // We implement the most common mappings; rare ones fall back to replacement char.
    // ETSI TS 101 756, Table 1 — Complete EBU Latin based repertoire
    match b {
        // Standard ASCII printable range
        0x20..=0x7E => b as char,

        // Row 8: lowercase accented vowels (0x80-0x89)
        0x80 => '\u{00E1}', // á
        0x81 => '\u{00E0}', // à
        0x82 => '\u{00E9}', // é
        0x83 => '\u{00E8}', // è
        0x84 => '\u{00ED}', // í
        0x85 => '\u{00EC}', // ì
        0x86 => '\u{00F3}', // ó
        0x87 => '\u{00F2}', // ò
        0x88 => '\u{00FA}', // ú
        0x89 => '\u{00F9}', // ù
        // Row 8 continued: special characters (0x8A-0x8F)
        0x8A => '\u{00D1}', // Ñ
        0x8B => '\u{00C7}', // Ç
        0x8C => '\u{015E}', // Ş
        0x8D => '\u{00DF}', // ß
        0x8E => '\u{00A1}', // ¡
        0x8F => '\u{0132}', // IJ

        // Row 9: lowercase circumflex/diaeresis (0x90-0x99)
        0x90 => '\u{00E2}', // â
        0x91 => '\u{00E4}', // ä
        0x92 => '\u{00EA}', // ê
        0x93 => '\u{00EB}', // ë
        0x94 => '\u{00EE}', // î
        0x95 => '\u{00EF}', // ï
        0x96 => '\u{00F4}', // ô
        0x97 => '\u{00F6}', // ö
        0x98 => '\u{00FB}', // û
        0x99 => '\u{00FC}', // ü
        // Row 9 continued (0x9A-0x9F)
        0x9A => '\u{00F1}', // ñ
        0x9B => '\u{00E7}', // ç
        0x9C => '\u{015F}', // ş
        0x9D => '\u{011F}', // ğ
        0x9E => '\u{0131}', // ı (dotless i)
        0x9F => '\u{0133}', // ij

        // Row A: currency and special (0xA0-0xAF)
        0xA0 => '\u{00AA}', // ª
        0xA1 => '\u{03B1}', // α
        0xA2 => '\u{00A9}', // ©
        0xA3 => '\u{2030}', // ‰
        0xA4 => '\u{011E}', // Ğ
        0xA5 => '\u{011B}', // ě
        0xA6 => '\u{0148}', // ň
        0xA7 => '\u{0151}', // ő
        0xA8 => '\u{00A3}', // £
        0xA9 => '\u{0024}', // $
        0xAA => '\u{0142}', // ł
        0xAB => '\u{0141}', // Ł
        0xAC => '\u{00DF}', // ß (duplicate)
        0xAD => '\u{00A4}', // ¤
        0xAE => '\u{0110}', // Đ
        0xAF => '\u{0111}', // đ

        // Row C: uppercase accented vowels (0xC0-0xC9)
        0xC0 => '\u{00C1}', // Á
        0xC1 => '\u{00C0}', // À
        0xC2 => '\u{00C9}', // É
        0xC3 => '\u{00C8}', // È
        0xC4 => '\u{00CD}', // Í
        0xC5 => '\u{00CC}', // Ì
        0xC6 => '\u{00D3}', // Ó
        0xC7 => '\u{00D2}', // Ò
        0xC8 => '\u{00DA}', // Ú
        0xC9 => '\u{00D9}', // Ù
        // Row C continued (0xCA-0xCF)
        0xCA => '\u{0158}', // Ř
        0xCB => '\u{010C}', // Č
        0xCC => '\u{0160}', // Š
        0xCD => '\u{017D}', // Ž
        0xCE => '\u{00D0}', // Ð
        0xCF => '\u{013F}', // Ŀ

        // Row D: uppercase circumflex/diaeresis (0xD0-0xD9)
        0xD0 => '\u{00C2}', // Â
        0xD1 => '\u{00C4}', // Ä
        0xD2 => '\u{00CA}', // Ê
        0xD3 => '\u{00CB}', // Ë
        0xD4 => '\u{00CE}', // Î
        0xD5 => '\u{00CF}', // Ï
        0xD6 => '\u{00D4}', // Ô
        0xD7 => '\u{00D6}', // Ö
        0xD8 => '\u{00DB}', // Û
        0xD9 => '\u{00DC}', // Ü
        // Row D continued (0xDA-0xDF)
        0xDA => '\u{0159}', // ř
        0xDB => '\u{010D}', // č
        0xDC => '\u{0161}', // š
        0xDD => '\u{017E}', // ž
        0xDE => '\u{00F0}', // ð
        0xDF => '\u{0140}', // ŀ

        // Space / control characters
        0x00..=0x1F | 0x7F => ' ',

        // Unmapped bytes: try Latin-1 interpretation
        _ => char::from_u32(b as u32).unwrap_or('\u{FFFD}'),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ebu_latin_ascii() {
        assert_eq!(ebu_latin_to_utf8(b"Hello World"), "Hello World");
    }

    #[test]
    fn test_ebu_latin_french_accents_lowercase() {
        // Real DLS from RCF OCCITANIE (6B, SId 0xF20C):
        // "...face à une guerre qui inquiète"
        // Raw bytes: 0x81 = à (a-grave), 0x83 = è (e-grave)
        let raw: &[u8] = &[
            0x66, 0x61, 0x63, 0x65, 0x20, // "face "
            0x81, // à
            0x20, 0x75, 0x6E, 0x65, 0x20, // " une "
            0x67, 0x75, 0x65, 0x72, 0x72, 0x65, 0x20, // "guerre "
            0x71, 0x75, 0x69, 0x20, // "qui "
            0x69, 0x6E, 0x71, 0x75, 0x69, // "inqui"
            0x83, // è
            0x74, 0x65, // "te"
        ];
        assert_eq!(ebu_latin_to_utf8(raw), "face à une guerre qui inquiète");
    }

    #[test]
    fn test_ebu_latin_french_accents_uppercase() {
        // Real DLS from RCF OCCITANIE (6B, SId 0xF20C):
        // "Écoute dans la nuit - ... transformé votre vie ?"
        // Raw bytes: 0xC2 = É (E-acute uppercase), 0x82 = é (e-acute lowercase)
        let raw: &[u8] = &[
            0xC2, // É
            0x63, 0x6F, 0x75, 0x74, 0x65, 0x20, // "coute "
            0x74, 0x72, 0x61, 0x6E, 0x73, 0x66, 0x6F, 0x72, 0x6D, // "transform"
            0x82, // é
        ];
        assert_eq!(ebu_latin_to_utf8(raw), "Écoute transformé");
    }

    #[test]
    fn test_ebu_latin_lowercase_accented_vowels() {
        assert_eq!(ebu_to_char(0x80), 'á');
        assert_eq!(ebu_to_char(0x81), 'à');
        assert_eq!(ebu_to_char(0x82), 'é');
        assert_eq!(ebu_to_char(0x83), 'è');
        assert_eq!(ebu_to_char(0x84), 'í');
        assert_eq!(ebu_to_char(0x85), 'ì');
        assert_eq!(ebu_to_char(0x86), 'ó');
        assert_eq!(ebu_to_char(0x87), 'ò');
        assert_eq!(ebu_to_char(0x88), 'ú');
        assert_eq!(ebu_to_char(0x89), 'ù');
    }

    #[test]
    fn test_ebu_latin_uppercase_accented_vowels() {
        assert_eq!(ebu_to_char(0xC0), 'Á');
        assert_eq!(ebu_to_char(0xC1), 'À');
        assert_eq!(ebu_to_char(0xC2), 'É');
        assert_eq!(ebu_to_char(0xC3), 'È');
        assert_eq!(ebu_to_char(0xC4), 'Í');
        assert_eq!(ebu_to_char(0xC5), 'Ì');
        assert_eq!(ebu_to_char(0xC6), 'Ó');
        assert_eq!(ebu_to_char(0xC7), 'Ò');
        assert_eq!(ebu_to_char(0xC8), 'Ú');
        assert_eq!(ebu_to_char(0xC9), 'Ù');
    }

    #[test]
    fn test_ebu_latin_special_chars() {
        assert_eq!(ebu_to_char(0x8A), 'Ñ'); // N-tilde
        assert_eq!(ebu_to_char(0x8B), 'Ç'); // C-cedilla
        assert_eq!(ebu_to_char(0x8D), 'ß'); // sharp-s
        assert_eq!(ebu_to_char(0x9A), 'ñ'); // n-tilde
        assert_eq!(ebu_to_char(0x9B), 'ç'); // c-cedilla
    }

    #[test]
    fn test_ebu_latin_circumflex_diaeresis() {
        assert_eq!(ebu_to_char(0x90), 'â'); // a-circumflex
        assert_eq!(ebu_to_char(0x91), 'ä'); // a-diaeresis
        assert_eq!(ebu_to_char(0x92), 'ê'); // e-circumflex
        assert_eq!(ebu_to_char(0x93), 'ë'); // e-diaeresis
        assert_eq!(ebu_to_char(0x94), 'î'); // i-circumflex
        assert_eq!(ebu_to_char(0x96), 'ô'); // o-circumflex
        assert_eq!(ebu_to_char(0x97), 'ö'); // o-diaeresis
        assert_eq!(ebu_to_char(0x98), 'û'); // u-circumflex
        assert_eq!(ebu_to_char(0x99), 'ü'); // u-diaeresis
    }

    #[test]
    fn test_ebu_latin_control_chars_to_space() {
        // Control characters 0x00-0x1F should map to space
        for b in 0x00..=0x1Fu8 {
            assert_eq!(ebu_to_char(b), ' ', "byte 0x{:02X} should be space", b);
        }
        assert_eq!(ebu_to_char(0x7F), ' ');
    }
}
