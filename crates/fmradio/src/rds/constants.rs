//! RDS protocol constants and lookup tables from the RDS standard (IEC 62106:2015, EN 50067:1998).

/// RDS CRC polynomial (g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1)
/// Hex shown in many references: 0x5B9. See pysdr / RDS standard discussion.
#[allow(dead_code)]
pub(super) const RDS_POLY: u32 = 0x5B9; // 10-bit polynomial (bits 9..0)
#[allow(dead_code)]
pub(super) const RDS_POLY_DEGREE: usize = 10;

/// Offset words (d9..d0) for A, B, C, C', D (as 10-bit values).
/// Values from the RDS standard (IEC 62106:2015 Table B.1).
/// These are added during encoding.
#[allow(dead_code)]
pub(super) const OFFSET_WORD_A: u16 = 0b0011111100; // A offset word
#[allow(dead_code)]
pub(super) const OFFSET_WORD_B: u16 = 0b0110011000; // B offset word
#[allow(dead_code)]
pub(super) const OFFSET_WORD_C: u16 = 0b0101101000; // C offset word
#[allow(dead_code)]
pub(super) const OFFSET_WORD_C_PRIME: u16 = 0b1101010000; // C' offset word
#[allow(dead_code)]
pub(super) const OFFSET_WORD_D: u16 = 0b0110110100; // D offset word

/// Expected syndromes for valid RDS blocks (IEC 62106:2015 section B.3.1).
/// These are the values returned by the syndrome calculation for error-free blocks.
/// Derived from redsea's getOffsetForSyndrome function.
pub(super) const SYNDROME_A: u16 = 0b1111011000; // 0x3D8 = 984
pub(super) const SYNDROME_B: u16 = 0b1111010100; // 0x3D4 = 980
pub(super) const SYNDROME_C: u16 = 0b1001011100; // 0x25C = 604
pub(super) const SYNDROME_C_PRIME: u16 = 0b1111001100; // 0x3CC = 972
pub(super) const SYNDROME_D: u16 = 0b1001011000; // 0x258 = 600

/// Maximum tolerable block error rate (out of 100). If exceeded over 50 blocks, sync is lost.
/// A lower value means redsea gives up faster in noisy conditions.
/// Redsea uses 85, meaning 42/50 errors triggers sync loss.
pub(super) const MAX_TOLERABLE_BLER: usize = 85;
/// Maximum errors tolerated over 50 blocks (kMaxTolerableBLER / 2)
pub(super) const MAX_ERRORS_OVER_50: usize = MAX_TOLERABLE_BLER / 2; // 42

/// Offset words as array for FEC lookup (indexed by offset enum)
pub(super) const OFFSET_WORDS: [u32; 5] = [
    0b0011111100, // A
    0b0110011000, // B
    0b0101101000, // C
    0b1101010000, // C' (Cprime)
    0b0110110100, // D
];

/// RDS offset types (A, B, C, C', D)
#[derive(Debug, Clone, Copy)]
pub(super) enum OffsetType {
    A = 0,
    B = 1,
    C = 2,
    Cprime = 3,
    D = 4,
}

impl OffsetType {
    pub(super) fn from_char(c: char) -> Option<Self> {
        match c {
            'A' => Some(OffsetType::A),
            'B' => Some(OffsetType::B),
            'C' => Some(OffsetType::C),
            'c' => Some(OffsetType::Cprime),
            'D' => Some(OffsetType::D),
            _ => None,
        }
    }
}

/// Parity check matrix for syndrome calculation (EN 50067:1998 section B.1.1)
/// Each row is a 10-bit value. Matrix multiplication is calculated by
/// modulo-two addition of all rows where the corresponding bit in the input is 1.
pub(super) const PARITY_CHECK_MATRIX: [u16; 26] = [
    0b1000000000, // bit 25 (MSB)
    0b0100000000,
    0b0010000000,
    0b0001000000,
    0b0000100000,
    0b0000010000,
    0b0000001000,
    0b0000000100,
    0b0000000010,
    0b0000000001, // bit 16
    0b1011011100, // bit 15
    0b0101101110,
    0b0010110111,
    0b1010000111,
    0b1110011111,
    0b1100010011,
    0b1101010101,
    0b1101110110,
    0b0110111011,
    0b1000000001, // bit 6
    0b1111011100, // bit 5
    0b0111101110,
    0b0011110111,
    0b1010100111,
    0b1110001111,
    0b1100011011, // bit 0 (LSB)
];

/// Program Type (PTY) names - indices 0-31
pub(super) const PTY_NAMES: &[&str] = &[
    "None",
    "News",
    "Information",
    "Sports",
    "Talk",
    "Rock",
    "Classic Rock",
    "Adult Hits",
    "MOR",
    "Easy Listening",
    "Lite Classics",
    "Classical",
    "Rhythm & Blues",
    "Soft Rhythm & Blues",
    "Language",
    "Religious Music",
    "Religious Talk",
    "Personality",
    "Public Affairs",
    "Drama",
    "Easy Listening (alt)",
    "Reggae",
    "Heritage",
    "Children",
    "Social Affairs",
    "Documentary",
    "Unknown (25)",
    "Unknown (26)",
    "Unknown (27)",
    "Unknown (28)",
    "Unknown (29)",
    "Unknown (30)",
];

/// Language names - 128 languages (RDS standard EN 50067:1998)
pub(super) const LANGUAGE_NAMES: &[&str] = &[
    "Unknown",
    "Albanian",
    "Breton",
    "Catalan",
    "Croatian",
    "Welsh",
    "Czech",
    "Danish",
    "German",
    "English",
    "Spanish",
    "Esperanto",
    "Estonian",
    "Basque",
    "Faroese",
    "French",
    "Frisian",
    "Irish",
    "Gaelic",
    "Galician",
    "Icelandic",
    "Italian",
    "Lappish",
    "Latin",
    "Latvian",
    "Luxembourgian",
    "Lithuanian",
    "Hungarian",
    "Maltese",
    "Dutch",
    "Norwegian",
    "Occitan",
    "Polish",
    "Portuguese",
    "Romanian",
    "Romansh",
    "Serbian",
    "Slovak",
    "Slovene",
    "Finnish",
    "Swedish",
    "Turkish",
    "Flemish",
    "Walloon",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "Background",
    "",
    "",
    "",
    "",
    "Zulu",
    "Vietnamese",
    "Uzbek",
    "Urdu",
    "Ukrainian",
    "Thai",
    "Telugu",
    "Tatar",
    "Tamil",
    "Tadzhik",
    "Swahili",
    "SrananTongo",
    "Somali",
    "Sinhalese",
    "Shona",
    "Serbo-Croat",
    "Ruthenian",
    "Russian",
    "Quechua",
    "Pushtu",
    "Punjabi",
    "Persian",
    "Papamiento",
    "Oriya",
    "Nepali",
    "Ndebele",
    "Marathi",
    "Moldovian",
    "Malaysian",
    "Malagasay",
    "Macedonian",
    "Laotian",
    "Korean",
    "Khmer",
    "Kazakh",
    "Kannada",
    "Japanese",
    "Indonesian",
    "Hindi",
    "Hebrew",
    "Hausa",
    "Gurani",
    "Gujurati",
    "Greek",
    "Georgian",
    "Fulani",
    "Dari",
    "Churash",
    "Chinese",
    "Burmese",
    "Bulgarian",
    "Bengali",
    "Belorussian",
    "Bambora",
    "Azerbaijan",
    "Assamese",
    "Armenian",
    "Arabic",
    "Amharic",
];

/// Slow Labeling Code (SLC) variant names for Group 1A
pub(super) const SLC_VARIANTS: &[&str] = &[
    "ECC and PI",       // 0: Extended Country Code
    "TMC ID",           // 1: Traffic Message Channel ID
    "Pager",            // 2: (deprecated)
    "Language",         // 3: Broadcast language
    "Reserved 4",       // 4: Not assigned
    "Reserved 5",       // 5: Not assigned
    "Broadcaster Bits", // 6: Broadcaster-specific
    "EWS",              // 7: Emergency Warning System
];

/// ODA (Open Data Application) applications - 65 known apps mapped by ID
pub(super) const ODA_APPS: &[(u16, &str)] = &[
    (0x0000, "None"),
    (0x0093, "Cross referencing DAB within RDS"),
    (0x0BCB, "Leisure & Practical Info for Drivers"),
    (0x0C24, "ELECTRABEL-DSM 7"),
    (0x0CC1, "Wireless Playground broadcast control signal"),
    (0x0D45, "RDS-TMC: ALERT-C / EN ISO 14819-1"),
    (0x0D8B, "ELECTRABEL-DSM 18"),
    (0x0E2C, "ELECTRABEL-DSM 3"),
    (0x0E31, "ELECTRABEL-DSM 13"),
    (0x0F87, "ELECTRABEL-DSM 2"),
    (0x125F, "I-FM-RDS for fixed and mobile devices"),
    (0x1BDA, "ELECTRABEL-DSM 1"),
    (0x1C5E, "ELECTRABEL-DSM 20"),
    (0x1C68, "ITIS In-vehicle data base"),
    (0x1CB1, "ELECTRABEL-DSM 10"),
    (0x1D47, "ELECTRABEL-DSM 4"),
    (0x1DC2, "CITIBUS 4"),
    (0x1DC5, "Encrypted TTI using ALERT-Plus"),
    (0x1E8F, "ELECTRABEL-DSM 17"),
    (0x4400, "RDS-Light"),
    (0x4AA1, "RASANT"),
    (0x4AB7, "ELECTRABEL-DSM 9"),
    (0x4BA2, "ELECTRABEL-DSM 5"),
    (0x4BD7, "RadioText+ (RT+)"),
    (0x4BD8, "RadioText Plus / RT+ for eRT"),
    (0x4C59, "CITIBUS 2"),
    (0x4D87, "Radio Commerce System (RCS)"),
    (0x4D95, "ELECTRABEL-DSM 16"),
    (0x4D9A, "ELECTRABEL-DSM 11"),
    (0x50DD, "To warn people in case of disasters or emergency"),
    (0x5757, "Personal weather station"),
    (0x6363, "Hybradio RDS-Net (for testing use, only)"),
    (0x6365, "RDS2 – 9 bit AF lists ODA"),
    (0x6552, "Enhanced RadioText (eRT)"),
    (0x6A7A, "Warning receiver"),
    (0x7373, "Enhanced early warning system"),
    (0xA112, "NL Alert system"),
    (0xA911, "Data FM Selective Multipoint Messaging"),
    (0xABCF, "RF Power Monitoring"),
    (0xC350, "NRSC Song Title and Artist"),
    (0xC3A1, "Personal Radio Service"),
    (0xC3B0, "iTunes Tagging"),
    (0xC3C3, "NAVTEQ Traffic Plus"),
    (0xC4D4, "eEAS"),
    (0xC549, "Smart Grid Broadcast Channel"),
    (0xC563, "ID Logic"),
    (0xC6A7, "Veil Enabled Interactive Device"),
    (0xC737, "Utility Message Channel (UMC)"),
    (0xCB73, "CITIBUS 1"),
    (0xCB97, "ELECTRABEL-DSM 14"),
    (0xCC21, "CITIBUS 3"),
    (0xCD46, "RDS-TMC: ALERT-C"),
    (0xCD47, "RDS-TMC: ALERT-C"),
    (0xCD9E, "ELECTRABEL-DSM 8"),
    (0xCE6B, "Encrypted TTI using ALERT-Plus"),
    (0xE123, "APS Gateway"),
    (0xE1C1, "Action code"),
    (0xE319, "ELECTRABEL-DSM 12"),
    (0xE411, "Beacon downlink"),
    (0xE440, "ELECTRABEL-DSM 15"),
    (0xE4A6, "ELECTRABEL-DSM 19"),
    (0xE5D7, "ELECTRABEL-DSM 6"),
    (0xE911, "EAS open protocol"),
    (0xFF7F, "RFT: Station logo"),
    (0xFF80, "RFT+ (work title)"),
];
