use serde::Serialize;
use std::fmt;
use tracing::{debug, trace};

/// RDS CRC polynomial (g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1)
/// Hex shown in many references: 0x5B9. See pysdr / RDS standard discussion.
#[allow(dead_code)]
const RDS_POLY: u32 = 0x5B9; // 10-bit polynomial (bits 9..0)
#[allow(dead_code)]
const RDS_POLY_DEGREE: usize = 10;

/// Offset words (d9..d0) for A, B, C, C', D (as 10-bit values).
/// Values from the RDS standard (IEC 62106:2015 Table B.1).
/// These are added during encoding.
#[allow(dead_code)]
const OFFSET_WORD_A: u16 = 0b0011111100; // A offset word
#[allow(dead_code)]
const OFFSET_WORD_B: u16 = 0b0110011000; // B offset word
#[allow(dead_code)]
const OFFSET_WORD_C: u16 = 0b0101101000; // C offset word
#[allow(dead_code)]
const OFFSET_WORD_C_PRIME: u16 = 0b1101010000; // C' offset word
#[allow(dead_code)]
const OFFSET_WORD_D: u16 = 0b0110110100; // D offset word

/// Expected syndromes for valid RDS blocks (IEC 62106:2015 section B.3.1).
/// These are the values returned by the syndrome calculation for error-free blocks.
/// Derived from redsea's getOffsetForSyndrome function.
const SYNDROME_A: u16 = 0b1111011000; // 0x3D8 = 984
const SYNDROME_B: u16 = 0b1111010100; // 0x3D4 = 980
const SYNDROME_C: u16 = 0b1001011100; // 0x25C = 604
const SYNDROME_C_PRIME: u16 = 0b1111001100; // 0x3CC = 972
const SYNDROME_D: u16 = 0b1001011000; // 0x258 = 600

/// Maximum tolerable block error rate (out of 100). If exceeded over 50 blocks, sync is lost.
/// A lower value means redsea gives up faster in noisy conditions.
/// Redsea uses 85, meaning 42/50 errors triggers sync loss.
const MAX_TOLERABLE_BLER: usize = 85;
/// Maximum errors tolerated over 50 blocks (kMaxTolerableBLER / 2)
const MAX_ERRORS_OVER_50: usize = MAX_TOLERABLE_BLER / 2; // 42

/// Offset words as array for FEC lookup (indexed by offset enum)
const OFFSET_WORDS: [u32; 5] = [
    0b0011111100, // A
    0b0110011000, // B
    0b0101101000, // C
    0b1101010000, // C' (Cprime)
    0b0110110100, // D
];

/// Offset enum for FEC (matches OFFSET_WORDS indexing)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum OffsetType {
    A = 0,
    B = 1,
    C = 2,
    Cprime = 3,
    D = 4,
}

impl OffsetType {
    fn from_char(c: char) -> Option<Self> {
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

/// Running sum over a sliding window of N elements.
/// Used to track block error rate over last 50 blocks (like redsea's RunningSum).
struct RunningSum<const N: usize> {
    buffer: [bool; N],
    pos: usize,
    sum: usize,
    filled: bool,
}

impl<const N: usize> RunningSum<N> {
    fn new() -> Self {
        Self {
            buffer: [false; N],
            pos: 0,
            sum: 0,
            filled: false,
        }
    }

    /// Push a new value, removing oldest if buffer is full
    fn push(&mut self, value: bool) {
        // Subtract the old value at this position
        if self.filled && self.buffer[self.pos] {
            self.sum = self.sum.saturating_sub(1);
        }
        // Add new value
        self.buffer[self.pos] = value;
        if value {
            self.sum += 1;
        }
        // Advance position
        self.pos = (self.pos + 1) % N;
        if self.pos == 0 {
            self.filled = true;
        }
    }

    /// Get current sum of errors in the window
    fn get_sum(&self) -> usize {
        self.sum
    }

    /// Clear the running sum
    fn clear(&mut self) {
        self.buffer = [false; N];
        self.pos = 0;
        self.sum = 0;
        self.filled = false;
    }
}

/// FEC (Forward Error Correction) lookup table for 1-2 bit burst errors.
/// For each offset type, stores (syndrome, error_vector) pairs.
/// Based on redsea's makeErrorLookupTable() in block_sync.cc
struct FecTable {
    /// Lookup tables for each offset type (A, B, C, C', D)
    tables: [Vec<(u16, u32)>; 5],
}

impl FecTable {
    /// Build the FEC lookup table at compile time / first use
    fn new() -> Self {
        let mut tables: [Vec<(u16, u32)>; 5] = Default::default();

        // For each offset type
        for (offset_idx, &offset_word) in OFFSET_WORDS.iter().enumerate() {
            // Try 1-bit and 2-bit burst errors at each position
            for &error_bits in &[0b1u32, 0b11u32] {
                for shift in 0..26u32 {
                    let error_vector = (error_bits << shift) & 0x03FF_FFFF; // 26-bit mask
                                                                            // XOR error with offset to get syndrome for this error pattern
                    let syndrome = rds_syndrome(error_vector ^ offset_word);
                    tables[offset_idx].push((syndrome, error_vector));
                }
            }
        }

        Self { tables }
    }

    /// Try to correct burst errors in a block.
    /// Returns Some(corrected_bits) if correction succeeded, None otherwise.
    fn try_correct(&self, raw_block: u32, expected_offset: OffsetType) -> Option<u32> {
        let syndrome = rds_syndrome(raw_block);
        let table = &self.tables[expected_offset as usize];

        for &(synd, error_vector) in table {
            if synd == syndrome {
                return Some(raw_block ^ error_vector);
            }
        }
        None
    }
}

/// Global FEC table (lazily initialized)
fn get_fec_table() -> &'static FecTable {
    use std::sync::OnceLock;
    static FEC_TABLE: OnceLock<FecTable> = OnceLock::new();
    FEC_TABLE.get_or_init(FecTable::new)
}

/// Block length in bits
const BLOCK_LENGTH: u32 = 26;

/// A sync pulse records a valid block detection with its offset type and bit position
#[derive(Clone, Copy, Debug, Default)]
struct SyncPulse {
    /// Offset type ('A', 'B', 'C', 'c', 'D', or '\0' for invalid)
    offset: char,
    /// Bit position where this block was detected
    bit_position: u32,
}

impl SyncPulse {
    /// Check if this pulse could realistically follow another pulse
    /// Based on redsea's SyncPulse::couldFollow()
    fn could_follow(&self, other: &SyncPulse) -> bool {
        // Invalid pulses can't form sequences
        if self.offset == '\0' || other.offset == '\0' {
            return false;
        }

        // Calculate distance in bits (handles wraparound)
        let sync_distance = self.bit_position.wrapping_sub(other.bit_position);

        // Distance must be a multiple of block length
        if sync_distance % BLOCK_LENGTH != 0 {
            return false;
        }

        // Distance must be within 6 blocks (allows for some missed blocks)
        let blocks_apart = sync_distance / BLOCK_LENGTH;
        if blocks_apart > 6 || blocks_apart == 0 {
            return false;
        }

        // Check that offsets follow the correct cyclic pattern
        // Block numbers: A=0, B=1, C/c=2, D=3
        let block_num = |off: char| -> u32 {
            match off {
                'A' => 0,
                'B' => 1,
                'C' | 'c' => 2,
                'D' => 3,
                _ => 0,
            }
        };

        let other_block = block_num(other.offset);
        let this_block = block_num(self.offset);

        // The expected block number after `blocks_apart` blocks
        let expected_block = (other_block + blocks_apart) % 4;

        expected_block == this_block
    }
}

/// Buffer of recent sync pulses for robust sync acquisition
/// Stores the last 4 detected valid blocks and looks for patterns
/// Based on redsea's SyncPulseBuffer
struct SyncPulseBuffer {
    pulses: [SyncPulse; 4],
}

impl SyncPulseBuffer {
    fn new() -> Self {
        Self {
            pulses: [SyncPulse::default(); 4],
        }
    }

    /// Push a new sync pulse, shifting out the oldest
    fn push(&mut self, offset: char, bit_position: u32) {
        // Shift all pulses left (oldest drops off)
        for i in 0..3 {
            self.pulses[i] = self.pulses[i + 1];
        }
        // Add new pulse at the end
        self.pulses[3] = SyncPulse {
            offset,
            bit_position,
        };
    }

    /// Search for three sync pulses in the correct cyclic rhythm
    /// Returns true if we found a valid sequence (not necessarily consecutive)
    fn is_sequence_found(&self) -> bool {
        let third = &self.pulses[3]; // Most recent pulse

        // Try all combinations of first and second pulses
        for i_first in 0..2 {
            for i_second in (i_first + 1)..3 {
                if third.could_follow(&self.pulses[i_second])
                    && self.pulses[i_second].could_follow(&self.pulses[i_first])
                {
                    return true;
                }
            }
        }
        false
    }

    /// Clear the buffer
    fn clear(&mut self) {
        self.pulses = [SyncPulse::default(); 4];
    }
}

/// Program Type (PTY) names - indices 0-31
const PTY_NAMES: &[&str] = &[
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
const LANGUAGE_NAMES: &[&str] = &[
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
const SLC_VARIANTS: &[&str] = &[
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
const ODA_APPS: &[(u16, &str)] = &[
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

/// Decoder Identification (DI) flags - accumulated over 4 Group 0A messages (one flag per message)
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DIFlags {
    /// Dynamic PTY (segment 0)
    pub dynamic_pty: bool,
    /// Compressed audio (segment 1)
    pub compressed: bool,
    /// Artificial head stereo simulation (segment 2)
    pub artificial_head: bool,
    /// Stereo audio (segment 3)
    pub stereo: bool,
}

impl DIFlags {
    /// Get the name of a DI flag by segment address (0-3)
    pub fn flag_name(segment: usize) -> &'static str {
        match segment {
            0 => "dynamic_pty",
            1 => "compressed",
            2 => "artificial_head",
            3 => "stereo",
            _ => "unknown",
        }
    }

    /// Get or set a DI flag by segment address
    pub fn set_flag(&mut self, segment: usize, value: bool) {
        match segment {
            0 => self.dynamic_pty = value,
            1 => self.compressed = value,
            2 => self.artificial_head = value,
            3 => self.stereo = value,
            _ => {}
        }
    }

    /// Get a DI flag by segment address
    pub fn get_flag(&self, segment: usize) -> bool {
        match segment {
            0 => self.dynamic_pty,
            1 => self.compressed,
            2 => self.artificial_head,
            3 => self.stereo,
            _ => false,
        }
    }

    /// Format DI flags as a string
    pub fn as_string(&self) -> String {
        format!(
            "dynamic_pty={} compressed={} artificial_head={} stereo={}",
            self.dynamic_pty, self.compressed, self.artificial_head, self.stereo
        )
    }
}

/// Expanded Country Info struct for PIN/ECC data (Group 1A)
#[derive(Debug, Clone, Default)]
pub struct ProgramItemInfo {
    /// Program Item Number (16-bit value)
    pub number: u16,
    /// Day of program start (1-31)
    pub day: u8,
    /// Hour of program start (0-23)
    pub hour: u8,
    /// Minute of program start (0-59)
    pub minute: u8,
}

/// Open Data Application (ODA) info from Group 3A
#[derive(Debug, Clone, Default)]
pub struct ODAInfo {
    /// Target group type for this ODA (0-15)
    pub target_group_type: u8,
    /// ODA Application ID (16-bit code)
    pub app_id: u16,
    /// ODA Message from Block 3
    pub message: u16,
}

/// Clock/Time/Date info from Group 4A
#[derive(Debug, Clone, Default)]
pub struct ClockTimeInfo {
    /// Year (1900-2100+)
    pub year: u16,
    /// Month (1-12)
    pub month: u8,
    /// Day (1-31)
    pub day: u8,
    /// Hour UTC (0-23)
    pub hour: u8,
    /// Minute UTC (0-59)
    pub minute: u8,
    /// Local time offset in half-hours (-14.0 to +14.0)
    pub local_offset: f32,
}

/// Enhanced Other Network (EON) info from Group 14A
#[derive(Debug, Clone, Default)]
pub struct EONInfo {
    /// PI code of other network
    pub pi: u16,
    /// EON variant type (0-15)
    pub variant: u8,
    /// Program Type of other network (variant 13)
    pub program_type: Option<u8>,
    /// Has linkage flag (variant 12)
    pub has_linkage: Option<bool>,
    /// Linkage Set Number (variant 12)
    pub linkage_set: Option<u16>,
    /// Alternative frequency from variant 4
    pub alt_frequency: Option<u8>,
}

/// Convert a 26-bit MSB-first word into a 10-bit syndrome (remainder).
/// Parity check matrix for syndrome calculation (EN 50067:1998 section B.1.1)
/// Each row is a 10-bit value. Matrix multiplication is calculated by
/// modulo-two addition of all rows where the corresponding bit in the input is 1.
const PARITY_CHECK_MATRIX: [u16; 26] = [
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

/// Calculate syndrome for a 26-bit RDS block using parity check matrix multiplication.
/// Input: 26-bit value where bit 25 is MSB (first transmitted bit) and bit 0 is LSB (last).
/// Returns: 10-bit syndrome value.
fn rds_syndrome(word26: u32) -> u16 {
    // EN 50067:1998, section B.1.1: Matrix multiplication is "calculated by
    // the modulo-two addition of all the rows of the matrix for which the
    // corresponding coefficient in the vector is 1."
    let mut result: u16 = 0;
    for k in 0..26 {
        let bit = (word26 >> k) & 1;
        if bit != 0 {
            result ^= PARITY_CHECK_MATRIX[25 - k];
        }
    }
    result
}

/// Try to identify which offset (if any) matches this 26-bit block.
/// Returns Some(offset_id_char) where offset_id_char is 'A', 'B', 'C', 'c' (C'), or 'D'.
fn rds_offset_for_syndrome(s: u16) -> Option<char> {
    if s == SYNDROME_A {
        Some('A')
    } else if s == SYNDROME_B {
        Some('B')
    } else if s == SYNDROME_C {
        Some('C')
    } else if s == SYNDROME_C_PRIME {
        Some('c')
    }
    // we use lowercase c for C'
    else if s == SYNDROME_D {
        Some('D')
    } else {
        None
    }
}

/// Convert 26 bits (msb-first) into the 16-bit data word (upper 16 bits).
fn rds_data_from_word(word26: u32) -> u16 {
    ((word26 >> 10) & 0xFFFF) as u16
}

/// JSON output for an RDS group (similar to redsea format)
#[derive(Debug, Clone, Serialize)]
pub struct RdsGroupJson {
    /// PI code as hex string (e.g., "0xF212")
    pub pi: String,
    /// Group type (e.g., "0A", "2A", "4A")
    pub group: String,
    /// Traffic Program flag
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tp: Option<bool>,
    /// Program type name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub prog_type: Option<String>,
    /// Program Service name (8 chars)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ps: Option<String>,
    /// RadioText (up to 64 chars)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radiotext: Option<String>,
    /// Traffic Announcement flag
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ta: Option<bool>,
    /// Music/Speech flag
    #[serde(skip_serializing_if = "Option::is_none")]
    pub music: Option<bool>,
    /// Alternative frequencies list
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt_freqs: Option<Vec<String>>,
    /// Clock time info
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clock_time: Option<String>,
    /// Partial group flag (some blocks missing)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub partial: Option<bool>,
    /// Raw hex blocks for debugging
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw: Option<String>,
}

impl RdsGroupJson {
    /// Create a new empty RDS group JSON
    pub fn new(pi: u16, group_type: &str) -> Self {
        Self {
            pi: format!("0x{:04X}", pi),
            group: group_type.to_string(),
            tp: None,
            prog_type: None,
            ps: None,
            radiotext: None,
            ta: None,
            music: None,
            alt_freqs: None,
            clock_time: None,
            partial: None,
            raw: None,
        }
    }

    /// Serialize to JSON string
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_else(|_| "{}".to_string())
    }
}

/// RDS parser state: assemble validated blocks into groups and parse group types
pub struct RdsParser {
    /// Shift register of bits (most recently received bit is LSB)
    shift: u32,
    /// How many bits currently valid in shift (max 26)
    shift_len: usize,
    /// Program Service name buffer (8 chars)
    ps: [u8; 8],
    ps_received_mask: u8, // bit mask of which 2-char segments received (4 segments)
    /// Sequential character count (0-8): how many chars received in order from position 0
    ps_sequential_len: usize,
    /// Previous character position (for sequential tracking, like redsea's prev_pos_)
    ps_prev_pos: Option<usize>,
    /// Cached last complete PS string (returned consistently once complete)
    ps_last_complete: Option<String>,
    /// Radiotext buffer (64 chars for 2A), track received segments
    rt: Vec<u8>,
    rt_received_mask: Vec<bool>,
    /// Enable verbose debug output
    verbose: bool,
    /// Station metadata extracted from RDS
    station_info: StationInfo,
    /// Count of successfully decoded groups (for debugging)
    groups_decoded: u32,
    /// Count of valid blocks received (for debugging)
    blocks_received: u32,
    /// Count of bits pushed (for debugging)
    bits_pushed: u64,
    /// Block synchronization state
    is_synced: bool,
    /// Expected next offset (when synced)
    expected_offset: char,
    /// Bits until next block boundary (when synced, count down from 26)
    bits_until_next_block: u8,
    /// Current group being assembled (4 blocks)
    current_group: [(bool, u16); 4], // (is_received, data)
    /// Sliding window of block errors over last 50 blocks (for sync loss detection)
    /// Replaces the old consecutive bad_block_count for more robust sync
    block_error_sum: RunningSum<50>,
    /// Enable FEC (Forward Error Correction) for 1-2 bit burst errors
    use_fec: bool,
    /// Sync pulse buffer for robust sync acquisition (like redsea)
    sync_buffer: SyncPulseBuffer,
    /// Total bit count (for sync pulse tracking)
    bitcount: u32,
    /// Queue of pending JSON outputs (groups to be printed)
    json_output_queue: Vec<RdsGroupJson>,
    /// Enable JSON output mode (disables verbose text output)
    json_mode: bool,
}

/// Extracted station metadata from RDS group 0A/0B
#[derive(Debug, Clone, Default)]
pub struct StationInfo {
    /// Traffic Program flag (TP)
    pub is_traffic_program: bool,
    /// Traffic Announcement flag (TA)
    pub is_traffic_announcement: bool,
    /// Program Type (0-31, see PTY_NAMES)
    pub program_type: u8,
    /// Decoder ID flags (4 flags: dynamic_pty, compressed, artificial_head, stereo)
    pub di_flags: DIFlags,
    /// Music flag
    pub is_music: bool,
    /// Alternative Frequency list (Method A: simple list, up to 25 frequencies in 10kHz units)
    pub af_list: Vec<u16>, // 0 = invalid/filler, 1-204 valid, 225-249 = list signifier

    // Group 1A data
    /// Program Item Number (PIN) from Group 1A Block 4
    pub program_item: Option<ProgramItemInfo>,
    /// Extended Country Code (ECC) from Group 1A Block 3 variant 0
    pub extended_country_code: Option<u8>,
    /// Language code from Group 1A Block 3 variant 3
    pub language_code: Option<u8>,
    /// TMC ID from Group 1A Block 3 variant 1
    pub tmc_id: Option<u16>,
    /// Has linkage flag (Group 1A Block 3 bit 15)
    pub has_linkage: bool,

    // Group 3A data
    /// Open Data Application (ODA) info from Group 3A
    pub oda_info: Option<ODAInfo>,

    // Group 4A data
    /// Clock/Time/Date info from Group 4A
    pub clock_time: Option<ClockTimeInfo>,

    // Group 14A data
    /// Enhanced Other Network (EON) info from Group 14A
    pub eon_info: Option<EONInfo>,
}

impl fmt::Debug for RdsParser {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ps = String::from_utf8_lossy(&self.ps);
        write!(f, "RdsParser {{ PS='{}' }}", ps)
    }
}

impl RdsParser {
    pub fn new() -> Self {
        Self {
            shift: 0,
            shift_len: 0,
            ps: [b' '; 8],
            ps_received_mask: 0,
            ps_sequential_len: 0,
            ps_prev_pos: None,
            ps_last_complete: None,
            rt: vec![b' '; 64],
            rt_received_mask: vec![false; 16], // 16 segments of 4 chars for RT 2A
            verbose: false,
            station_info: StationInfo::default(),
            groups_decoded: 0,
            blocks_received: 0,
            bits_pushed: 0,
            is_synced: false,
            expected_offset: 'A',
            bits_until_next_block: 1, // Check immediately on first bit
            current_group: [(false, 0); 4],
            block_error_sum: RunningSum::new(),
            use_fec: true, // Enable FEC by default (like redsea)
            sync_buffer: SyncPulseBuffer::new(),
            bitcount: 0,
            json_output_queue: Vec::new(),
            json_mode: false,
        }
    }

    /// Get debugging statistics
    pub fn stats(&self) -> (u64, u32, u32) {
        (self.bits_pushed, self.blocks_received, self.groups_decoded)
    }

    /// Enable JSON output mode (outputs RDS groups as JSON to stdout)
    pub fn set_json_mode(&mut self, enabled: bool) {
        self.json_mode = enabled;
        if enabled {
            self.verbose = false; // Disable verbose when JSON mode is on
        }
    }

    /// Take pending JSON outputs (drains the queue)
    pub fn take_json_outputs(&mut self) -> Vec<RdsGroupJson> {
        std::mem::take(&mut self.json_output_queue)
    }

    /// Check if any groups have been decoded
    pub fn has_data(&self) -> bool {
        self.groups_decoded > 0
    }

    /// Enable verbose debug output showing all received groups
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// Get the current station name (PS) if complete
    /// PS name is 8 characters transmitted as 4 segments of 2 characters each.
    /// Returns the cached last complete string once all 8 characters have been
    /// received sequentially (like redsea does).
    pub fn station_name(&self) -> Option<String> {
        // Return the cached complete string if we have one
        self.ps_last_complete.clone()
    }

    /// Get the current radio text (RT) if at least some segments have been received
    pub fn radio_text(&self) -> Option<String> {
        let have = self.rt_received_mask.iter().filter(|&&x| x).count();
        if have >= 2 {
            let rt_text = String::from_utf8_lossy(&self.rt).trim_end().to_string();
            if !rt_text.is_empty() {
                Some(rt_text)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Get the station metadata (TP, TA, PTY, DI, AF list)
    pub fn station_info(&self) -> &StationInfo {
        &self.station_info
    }

    /// Get the program type name (PTY)
    pub fn program_type_name(&self) -> &str {
        PTY_NAMES[self.station_info.program_type as usize]
    }

    /// Get the language name for Group 1A language code
    pub fn language_name(&self) -> Option<&'static str> {
        self.station_info.language_code.and_then(|code| {
            if (code as usize) < LANGUAGE_NAMES.len() {
                let name = LANGUAGE_NAMES[code as usize];
                if !name.is_empty() {
                    Some(name)
                } else {
                    None
                }
            } else {
                None
            }
        })
    }

    /// Get the SLC (Slow Labeling Code) variant name for Group 1A
    pub fn slc_variant_name(variant: u8) -> &'static str {
        if (variant as usize) < SLC_VARIANTS.len() {
            SLC_VARIANTS[variant as usize]
        } else {
            "Unknown"
        }
    }

    /// Get the ODA application name by ID
    pub fn oda_app_name(app_id: u16) -> &'static str {
        for &(id, name) in ODA_APPS {
            if id == app_id {
                return name;
            }
        }
        "(Unknown ODA)"
    }

    /// Feed raw RDS bits (0/1) into the parser. Bits must be in the transmitted order (MSB-first).
    /// Typically you call this with the bit sequence produced by your BPSK symbol slicer.
    ///
    /// This implements block synchronization similar to redsea:
    /// - When not synced: check every bit position for valid blocks
    /// - When synced: only check at 26-bit intervals
    /// - Sync is acquired when 3 valid blocks are found in correct cyclic rhythm
    /// - Sync is lost after too many errors in a sliding window
    pub fn push_bits(&mut self, bits: &[u8]) {
        self.bits_pushed += bits.len() as u64;

        for &b in bits {
            // Shift in the new bit
            self.shift = ((self.shift << 1) & ((1u32 << 26) - 1)) | (b as u32 & 1);
            if self.shift_len < 26 {
                self.shift_len += 1;
            }
            self.bitcount = self.bitcount.wrapping_add(1);

            if self.shift_len < 26 {
                continue;
            }

            // We have 26 bits - check if we should process them
            self.bits_until_next_block = self.bits_until_next_block.saturating_sub(1);

            if self.is_synced {
                // When synced, only check at block boundaries
                if self.bits_until_next_block == 0 {
                    self.process_block_at_boundary();
                    self.bits_until_next_block = 26;
                }
            } else {
                // When not synced, check every bit position for sync acquisition
                self.try_acquire_sync();
            }
        }
    }

    /// Try to acquire sync using SyncPulseBuffer (like redsea)
    /// Looks for 3 valid blocks in correct cyclic rhythm (not necessarily consecutive)
    fn try_acquire_sync(&mut self) {
        let synd = rds_syndrome(self.shift);
        if let Some(offset) = rds_offset_for_syndrome(synd) {
            // Found a valid block - push to sync buffer
            self.sync_buffer.push(offset, self.bitcount);
            self.blocks_received += 1;

            // Check if we have a valid sequence of 3 blocks
            if self.sync_buffer.is_sequence_found() {
                // Sync acquired!
                self.is_synced = true;
                self.bits_until_next_block = 26;
                self.block_error_sum.clear();
                self.expected_offset = match offset {
                    'A' => 'B',
                    'B' => 'C',
                    'C' | 'c' => 'D',
                    'D' => 'A',
                    _ => 'A',
                };

                // Store this block in current group
                let block_idx = match offset {
                    'A' => 0,
                    'B' => 1,
                    'C' | 'c' => 2,
                    'D' => 3,
                    _ => 0,
                };
                let data = rds_data_from_word(self.shift);
                self.current_group[block_idx] = (true, data);

                debug!(
                    offset = %offset,
                    "RDS sync acquired (SyncPulseBuffer)"
                );

                // If we just completed a group (got D), process it
                if offset == 'D' {
                    self.process_current_group();
                }
            }
        }
    }

    /// Process block when we're synced and at a block boundary
    fn process_block_at_boundary(&mut self) {
        let synd = rds_syndrome(self.shift);
        let detected_offset = rds_offset_for_syndrome(synd);

        // Check if the detected offset matches expected (allowing C/c substitution)
        // If expecting C, also accept C' (lowercase 'c')
        let matches_expected = match detected_offset {
            Some(off) => off == self.expected_offset || (self.expected_offset == 'C' && off == 'c'),
            None => false,
        };

        // Track if block had errors (for sliding window)
        let had_errors = !matches_expected;
        self.block_error_sum.push(had_errors);

        // EN 50067:1998, section C.1.2: Sync is dropped when too many syndromes failed
        if self.block_error_sum.get_sum() > MAX_ERRORS_OVER_50 {
            self.is_synced = false;
            self.block_error_sum.clear();
            self.sync_buffer.clear(); // Reset sync buffer for re-acquisition
            debug!(
                errors = self.block_error_sum.get_sum(),
                threshold = MAX_ERRORS_OVER_50,
                "RDS sync lost"
            );
            return;
        }

        // Adjust expected offset for C' if needed
        if self.expected_offset == 'C' && detected_offset == Some('c') {
            // We were expecting C but got C' - that's valid for type B groups
        }

        let mut block_data: Option<u16> = None;
        let mut block_is_valid = false;

        if matches_expected {
            // Block matched expected offset perfectly
            block_data = Some(rds_data_from_word(self.shift));
            block_is_valid = true;
            self.blocks_received += 1;
        } else if self.use_fec {
            // Try FEC error correction (1-2 bit burst errors)
            if let Some(expected_type) = OffsetType::from_char(self.expected_offset) {
                if let Some(corrected) = get_fec_table().try_correct(self.shift, expected_type) {
                    // Verify correction: recalculate syndrome should match expected offset
                    let corrected_synd = rds_syndrome(corrected);
                    let corrected_offset = rds_offset_for_syndrome(corrected_synd);
                    if corrected_offset == Some(self.expected_offset)
                        || (self.expected_offset == 'C' && corrected_offset == Some('c'))
                    {
                        block_data = Some(rds_data_from_word(corrected));
                        block_is_valid = true;
                        self.blocks_received += 1;
                        trace!(
                            block = %self.expected_offset,
                            original = format!("0x{:06X}", self.shift),
                            corrected = format!("0x{:06X}", corrected),
                            "RDS FEC corrected block"
                        );
                    }
                }
            }
        }

        // Store block in current group (or mark as not received)
        let block_idx = match self.expected_offset {
            'A' => 0,
            'B' => 1,
            'C' => 2,
            'D' => 3,
            _ => 0,
        };

        if block_is_valid {
            self.current_group[block_idx] = (true, block_data.unwrap());
        } else {
            self.current_group[block_idx] = (false, 0);
            trace!(
                expected = %self.expected_offset,
                syndrome = format!("0x{:03X}", synd),
                detected = ?detected_offset,
                "RDS bad block"
            );
        }

        // Advance expected offset
        let next_offset = match self.expected_offset {
            'A' => 'B',
            'B' => 'C',
            'C' => 'D',
            'D' => {
                // Completed a group - process it
                self.process_current_group();
                'A'
            }
            _ => 'A',
        };
        self.expected_offset = next_offset;
    }

    /// Process the current 4-block group if we have enough valid blocks
    fn process_current_group(&mut self) {
        // Count how many blocks we received
        let received_count = self.current_group.iter().filter(|(r, _)| *r).count();

        // Require at least blocks A and B (PI and group type info) to process
        if self.current_group[0].0 && self.current_group[1].0 {
            let datas: [u16; 4] = [
                self.current_group[0].1,
                self.current_group[1].1,
                if self.current_group[2].0 {
                    self.current_group[2].1
                } else {
                    0
                },
                if self.current_group[3].0 {
                    self.current_group[3].1
                } else {
                    0
                },
            ];

            // Track which blocks are actually valid (received with good CRC)
            let block_valid = [
                self.current_group[0].0,
                self.current_group[1].0,
                self.current_group[2].0,
                self.current_group[3].0,
            ];

            if received_count < 4 {
                trace!(blocks = received_count, "RDS partial group");
            }

            self.handle_group(datas, block_valid);
        }

        // Reset for next group
        self.current_group = [(false, 0); 4];
    }

    /// Check if a byte is a valid RDS PS/RT character (printable ASCII or space)
    #[inline]
    fn is_valid_rds_char(b: u8) -> bool {
        // Accept printable ASCII (0x20-0x7E) and common RDS extended chars (0x80-0xFF)
        // Reject control characters (0x00-0x1F except 0x0D which is string terminator)
        b >= 0x20 || b == 0x0D
    }

    /// Update PS buffer with a new segment, tracking sequential reception.
    /// Like redsea, we store all characters and filter on output.
    pub fn update_ps(&mut self, seg: usize, ch1: u8, ch2: u8) {
        // Validate segment index
        if seg > 3 {
            return;
        }

        let pos = seg * 2; // Character position (0, 2, 4, 6)

        // Store the characters (like redsea, store any value - filter on output)
        self.ps[pos] = ch1;
        self.ps[pos + 1] = ch2;
        self.ps_received_mask |= 1 << seg;

        // Track sequential reception exactly like redsea's RDSString::set()
        // First character of segment (position pos)
        // Condition: pos == 0 || (pos == prev_pos + 1 && sequential_length == pos)
        if pos == 0
            || (self.ps_prev_pos.map_or(false, |p| pos == p + 1) && self.ps_sequential_len == pos)
        {
            self.ps_sequential_len = pos + 1;
        }
        self.ps_prev_pos = Some(pos);

        // Second character of segment (position pos + 1)
        let pos2 = pos + 1;
        if pos2 == 0
            || (self.ps_prev_pos.map_or(false, |p| pos2 == p + 1) && self.ps_sequential_len == pos2)
        {
            self.ps_sequential_len = pos2 + 1;
        }
        self.ps_prev_pos = Some(pos2);

        // Check if PS is complete (all 8 chars received sequentially)
        if self.ps_sequential_len >= 8 {
            // Cache the complete string, filtering invalid chars like redsea does
            let ps_filtered: Vec<u8> = self
                .ps
                .iter()
                .map(|&b| {
                    // Replace control chars (except 0x0D terminator) with space
                    if b >= 0x20 || b == 0x0D {
                        b
                    } else {
                        b' '
                    }
                })
                .collect();
            let ps_str = String::from_utf8_lossy(&ps_filtered).to_string();
            let trimmed = ps_str.trim();
            if !trimmed.is_empty() {
                self.ps_last_complete = Some(trimmed.to_string());
            }
        }

        // Verbose debug output (converted to tracing)
        if self.verbose {
            debug!(
                "  [PS] Segment {}/4: 0x{:02X} 0x{:02X} = '{}{}' (mask: {:04b}, seq_len: {})",
                seg,
                ch1,
                ch2,
                if ch1.is_ascii_graphic() || ch1 == b' ' {
                    ch1 as char
                } else {
                    '?'
                },
                if ch2.is_ascii_graphic() || ch2 == b' ' {
                    ch2 as char
                } else {
                    '?'
                },
                self.ps_received_mask,
                self.ps_sequential_len
            );
        }
    }

    /// Process a full 4-block group (datas array: block1..block4 data words).
    /// block_valid indicates which blocks passed CRC (like redsea's group.has()).
    fn handle_group(&mut self, datas: [u16; 4], block_valid: [bool; 4]) {
        self.groups_decoded += 1;

        // block 1 is PI (program identification)
        let pi = datas[0];
        let block2 = datas[1];
        let block3 = datas[2];
        let block4 = datas[3];

        // Convenience flags for block validity (like redsea's group.has(BLOCK3), etc.)
        let has_block3 = block_valid[2];
        let has_block4 = block_valid[3];

        // group type: bits 15..12 of block2 (MSB-first)
        let group_type = ((block2 >> 12) & 0x0F) as u8;
        // version (A=0, B=1) is bit 11
        let version = ((block2 >> 11) & 0x01) as u8;
        // for PS and RT addressing we need the low bits:
        // for PS: C1 C0 usually located in block2 bits b1..b0 (we use block2 & 0x03)
        let c_bits = (block2 & 0x03) as u8;
        // for RT version A segment address is low 4 bits
        let rt_seg_addr = (block2 & 0x0F) as usize;

        // Extract common RDS flags from Block 2 (bits 15..0):
        // Bit 15..12: Group type
        // Bit 11: Version (A/B)
        // Bit 10: TP (Traffic Program)
        // Bit 9..5: PTY (Program Type, 5 bits = 0-31)
        // Bit 4: TA (Traffic Announcement)
        // Bit 3: Music flag
        // Bit 2: DI or other use (varies by group)
        // Bit 1..0: Varies by group (PS segment, RT segment address, etc.)

        // Extract metadata (valid for most group types)
        self.station_info.is_traffic_program = (block2 & 0x0400) != 0; // Bit 10
        self.station_info.program_type = ((block2 >> 5) & 0x1F) as u8; // Bits 9..5
        self.station_info.is_traffic_announcement = (block2 & 0x0010) != 0; // Bit 4
        self.station_info.is_music = (block2 & 0x0008) != 0; // Bit 3

        // Format group type string (e.g., "0A", "2B")
        let group_type_str = format!("{}{}", group_type, if version == 0 { 'A' } else { 'B' });

        // Create JSON output structure
        let mut json_out = RdsGroupJson::new(pi, &group_type_str);
        json_out.tp = Some(self.station_info.is_traffic_program);
        json_out.prog_type = Some(self.program_type_name().to_string());
        json_out.ta = Some(self.station_info.is_traffic_announcement);
        json_out.music = Some(self.station_info.is_music);

        // Add partial flag if not all blocks received
        if !block_valid.iter().all(|&v| v) {
            json_out.partial = Some(true);
        }

        // Log with tracing (always, at debug level)
        debug!(
            group = %group_type_str,
            pi = format!("0x{:04X}", pi),
            tp = self.station_info.is_traffic_program,
            pty = self.station_info.program_type,
            "RDS group received"
        );

        // Verbose debug output (converted to tracing)
        if self.verbose {
            debug!(
                tp = self.station_info.is_traffic_program,
                ta = self.station_info.is_traffic_announcement,
                pty = self.station_info.program_type,
                pty_name = %self.program_type_name(),
                music = self.station_info.is_music,
                di = %self.station_info.di_flags.as_string(),
                "RDS flags"
            );
        }

        match (group_type, version) {
            (0, 0) => {
                // 0A - PS name (2 ASCII chars in block4) + AF list in block3 + DI flags
                // Block 2: bits 1-0 = segment address (0-3), bit 2 = DI flag for this segment
                // Each segment 0-3 received provides one of the 4 DI flags
                let seg = c_bits as usize; // 0..3

                // Extract DI flag from Block 2 bit 2
                let di_flag = (block2 & 0x0004) != 0; // bit 2
                self.station_info.di_flags.set_flag(seg, di_flag);

                // Verbose debug output (converted to tracing)
        if self.verbose {
                    debug!(
                        "  [DI] Segment {}: {} = {}",
                        seg,
                        DIFlags::flag_name(seg),
                        di_flag
                    );
                }

                // Extract AF codes from block3
                let af1 = ((block3 >> 8) & 0xFF) as u8;
                let af2 = (block3 & 0xFF) as u8;

                // AF code to frequency conversion: 87.5 + code*0.1 MHz (code 1-204)
                // Stored in units of 10 kHz: 8750 + code*10
                if af1 > 0 && af1 < 205 {
                    let freq_10khz = 8750u16 + (af1 as u16) * 10;
                    if !self.station_info.af_list.contains(&freq_10khz) {
                        self.station_info.af_list.push(freq_10khz);
                    }
                }
                if af2 > 0 && af2 < 205 {
                    let freq_10khz = 8750u16 + (af2 as u16) * 10;
                    if !self.station_info.af_list.contains(&freq_10khz) {
                        self.station_info.af_list.push(freq_10khz);
                    }
                }

                // PS name in block4 (2 ASCII chars)
                // Only update PS if block4 was actually received (like redsea's group.has(BLOCK4))
                if has_block4 {
                    // High byte (bits 15-8) → first char, Low byte (bits 7-0) → second char
                    // NO BIT INVERSION - use raw byte values like redsea does
                    let ch1 = ((block4 >> 8) & 0xFF) as u8; // High byte first
                    let ch2 = (block4 & 0xFF) as u8; // Low byte second

                    self.update_ps(seg, ch1, ch2);
                }
            }
            (0, 1) => {
                // 0B - PS variant (block4 has chars)
                // Only update PS if block4 was actually received
                if has_block4 {
                    // High byte (bits 15-8) → first char, Low byte (bits 7-0) → second char
                    let ch1 = ((block4 >> 8) & 0xFF) as u8;
                    let ch2 = (block4 & 0xFF) as u8;
                    let seg = c_bits as usize;

                    self.update_ps(seg, ch1, ch2);
                }
            }
            (2, 0) => {
                // 2A - Radiotext (4 chars per group from block3 and block4)
                // Only update RT if we have both block3 and block4
                if has_block3 && has_block4 {
                    // Byte order: high byte first, then low byte (same as PS)
                    let seg = rt_seg_addr & 0x0F;
                    // NO XOR inversion - use raw bytes like redsea does
                    let a = ((block3 >> 8) & 0xFF) as u8; // block3 high byte
                    let b = (block3 & 0xFF) as u8; // block3 low byte
                    let c = ((block4 >> 8) & 0xFF) as u8; // block4 high byte
                    let d = (block4 & 0xFF) as u8; // block4 low byte
                    let pos = seg * 4;
                    if pos + 3 < self.rt.len() {
                        self.rt[pos] = a;
                        self.rt[pos + 1] = b;
                        self.rt[pos + 2] = c;
                        self.rt[pos + 3] = d;
                        self.rt_received_mask[seg] = true;

                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            let received_count =
                                self.rt_received_mask.iter().filter(|&&x| x).count();
                            debug!(
                            "  [RT] Segment {}/16: 0x{:02X} 0x{:02X} 0x{:02X} 0x{:02X} = '{}{}{}{}' ({}/16 segments)",
                            seg,
                            a,
                            b,
                            c,
                            d,
                            if a.is_ascii_graphic() || a == b' ' {
                                a as char
                            } else {
                                '?'
                            },
                            if b.is_ascii_graphic() || b == b' ' {
                                b as char
                            } else {
                                '?'
                            },
                            if c.is_ascii_graphic() || c == b' ' {
                                c as char
                            } else {
                                '?'
                            },
                            if d.is_ascii_graphic() || d == b' ' {
                                d as char
                            } else {
                                '?'
                            },
                            received_count
                        );
                        }
                    }
                }
            }
            (2, 1) => {
                // 2B - Radiotext version B (only 2 chars per group from block4)
                // Byte order: high byte first, then low byte
                let seg = rt_seg_addr & 0x0F;
                // NO XOR inversion - use raw bytes like redsea does
                let c = ((block4 >> 8) & 0xFF) as u8; // block4 high byte
                let d = (block4 & 0xFF) as u8; // block4 low byte
                let pos = seg * 2; // 2B uses 2 chars per segment, not 4
                if pos + 1 < self.rt.len() {
                    self.rt[pos] = c;
                    self.rt[pos + 1] = d;
                    self.rt_received_mask[seg] = true;

                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        let received_count = self.rt_received_mask.iter().filter(|&&x| x).count();
                        debug!(
                            "  [RT] Segment {}/16: 0x{:02X} 0x{:02X} = '{}{}' ({}/16 segments)",
                            seg,
                            c,
                            d,
                            if c.is_ascii_graphic() || c == b' ' {
                                c as char
                            } else {
                                '?'
                            },
                            if d.is_ascii_graphic() || d == b' ' {
                                d as char
                            } else {
                                '?'
                            },
                            received_count
                        );
                    }
                }
            }
            (1, 0) => {
                // 1A - Program Item Number (PIN) + Extended Country Code or Language
                // Block 3: Variant identification and variant data
                // Block 4: PIN (Program Item Number)

                let linkage_la = (block3 >> 15) & 0x01;
                self.station_info.has_linkage = linkage_la != 0;

                let slc_variant = ((block3 >> 12) & 0x07) as u8;

                // Decode PIN from Block 4
                // PIN format: bits 15..11 = day (1-31), bits 10..6 = hour (0-23), bits 5..0 = minute (0-59)
                let day = ((block4 >> 11) & 0x1F) as u8;
                let hour = ((block4 >> 6) & 0x1F) as u8;
                let minute = (block4 & 0x3F) as u8;

                // Validate PIN
                if (1..=31).contains(&day) && hour <= 24 && minute <= 59 {
                    self.station_info.program_item = Some(ProgramItemInfo {
                        number: block4,
                        day,
                        hour,
                        minute,
                    });
                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        debug!(
                            "  [PIN] Day {} {:02}:{:02} (0x{:04X})",
                            day, hour, minute, block4
                        );
                    }
                }

                // Decode variant-specific data from Block 3 (bits 11..0)
                match slc_variant {
                    0 => {
                        // ECC and PI-based country code
                        let ecc = (block3 & 0xFF) as u8;
                        let cc = (pi >> 12) & 0x0F;
                        self.station_info.extended_country_code = Some(ecc);
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!(
                                "  [SLC] {} - ECC=0x{:02X}, CC={} (country code from PI)",
                                Self::slc_variant_name(slc_variant),
                                ecc,
                                cc
                            );
                        }
                    }
                    1 => {
                        // TMC ID (Traffic Message Channel)
                        let tmc_id = block3 & 0xFFF;
                        self.station_info.tmc_id = Some(tmc_id);
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!(
                                "  [SLC] {} - TMC ID=0x{:03X}",
                                Self::slc_variant_name(slc_variant),
                                tmc_id
                            );
                        }
                    }
                    2 => {
                        // Pager (deprecated in RDS2)
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!(
                                "  [SLC] {} - (Pager, deprecated)",
                                Self::slc_variant_name(slc_variant)
                            );
                        }
                    }
                    3 => {
                        // Language code
                        let lang_code = (block3 & 0xFF) as u8;
                        self.station_info.language_code = Some(lang_code);
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            let lang_name = if lang_code < LANGUAGE_NAMES.len() as u8 {
                                LANGUAGE_NAMES[lang_code as usize]
                            } else {
                                "Unknown"
                            };
                            debug!(
                                "  [SLC] {} - Language=0x{:02X} ({})",
                                Self::slc_variant_name(slc_variant),
                                lang_code,
                                lang_name
                            );
                        }
                    }
                    6 => {
                        // Broadcaster bits
                        let bits = block3 & 0x7FF;
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!(
                                "  [SLC] {} - Broadcaster bits=0x{:03X}",
                                Self::slc_variant_name(slc_variant),
                                bits
                            );
                        }
                    }
                    7 => {
                        // Emergency Warning System (EWS)
                        let ews = block3 & 0xFFF;
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!(
                                "  [SLC] {} - EWS=0x{:03X}",
                                Self::slc_variant_name(slc_variant),
                                ews
                            );
                        }
                    }
                    _ => {
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!(
                                "  [SLC] {} (variant {})",
                                Self::slc_variant_name(slc_variant),
                                slc_variant
                            );
                        }
                    }
                }
            }
            (3, 0) => {
                // 3A - Open Data Application (ODA) Registration
                // Only version A is valid for ODA
                let target_group_type = (block2 & 0x1F) as u8; // Bits 4..0
                let oda_app_id = block4;
                let oda_message = block3;

                // Verbose debug output (converted to tracing)
        if self.verbose {
                    debug!(
                        "  [ODA] Target Group: {}A, App ID: 0x{:04X} ({})",
                        target_group_type,
                        oda_app_id,
                        Self::oda_app_name(oda_app_id)
                    );
                }

                // Store ODA info for reference
                self.station_info.oda_info = Some(ODAInfo {
                    target_group_type,
                    app_id: oda_app_id,
                    message: oda_message,
                });

                // Handle specific ODA applications
                match oda_app_id {
                    0x4BD7 => {
                        // RadioText+ (RT+)
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            let cb = (oda_message >> 12) & 0x01 != 0;
                            let scb = (oda_message >> 8) & 0x0F;
                            let template_num = (oda_message & 0xFF) as u8;
                            debug!(
                                "    [RT+] CB={}, SCB=0x{:X}, Template={}",
                                cb, scb, template_num
                            );
                        }
                    }
                    0x6552 => {
                        // Enhanced RadioText (eRT)
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            let encoding = if (oda_message & 0x01) != 0 {
                                "UTF-8"
                            } else {
                                "UCS2"
                            };
                            let direction = if (oda_message & 0x02) != 0 {
                                "RTL"
                            } else {
                                "LTR"
                            };
                            debug!("    [eRT] Encoding={}, Direction={}", encoding, direction);
                        }
                    }
                    0xCD46 | 0xCD47 => {
                        // RDS-TMC (Traffic Message Channel)
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!("    [TMC] ALERT-C message: 0x{:04X}", oda_message);
                        }
                    }
                    _ => {}
                }
            }
            (4, 0) => {
                // 4A - Clock/Time and Date
                // Only version A is valid for time/date

                // Extract Modified Julian Date from Block 2-3
                // Block 2 bits 0-15, Block 3 bit 1 (17 bits total)
                let mjd_high = block2 as u32; // 16 bits
                let mjd_low = ((block3 >> 1) & 0x0001) as u32; // 1 bit from Block 3 bit 1
                let mjd = (mjd_low << 16) | mjd_high;

                // MJD to Gregorian calendar conversion
                // Using standard algorithm from RDS spec
                let mjd_f = mjd as f64;

                // Check for invalid date
                if mjd_f < 15079.0 {
                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        debug!("  [Time] Invalid MJD: {}", mjd);
                    }
                    return;
                }

                // MJD to calendar date conversion
                let mut year_utc = ((mjd_f - 15078.2) / 365.25) as i32;
                let mut month_utc =
                    ((mjd_f - 14956.1 - (year_utc as f64 * 365.25)) / 30.6001) as i32;
                let day_utc =
                    (mjd_f - 14956.0 - (year_utc as f64 * 365.25) - (month_utc as f64 * 30.6001))
                        as u8;

                if month_utc == 14 || month_utc == 15 {
                    year_utc += 1;
                    month_utc -= 12;
                }
                year_utc += 1900;
                month_utc -= 1;

                // Extract hour and minute from Block 3-4
                // Hour: bits 12..8 of result spanning Block 3 bit 0 + Block 4 bits 15..12
                let hour_utc = ((block3 & 0x0001) << 4) | ((block4 >> 12) & 0x0F);
                let hour_utc = hour_utc as u8;

                // Minute: bits 5..0 of Block 4
                let minute_utc = ((block4 >> 6) & 0x3F) as u8;

                // Local time offset: bits 5..0 with sign bit 5
                let offset_sign = ((block4 >> 5) & 0x01) != 0; // 1 = negative
                let offset_half_hours = (block4 & 0x1F) as u8;
                let local_offset = (offset_half_hours as f32) * 0.5;
                let local_offset = if offset_sign {
                    -local_offset
                } else {
                    local_offset
                };

                // Validate and store
                if (1..=31).contains(&day_utc)
                    && hour_utc <= 23
                    && minute_utc <= 59
                    && year_utc >= 1900
                {
                    self.station_info.clock_time = Some(ClockTimeInfo {
                        year: year_utc as u16,
                        month: month_utc as u8,
                        day: day_utc,
                        hour: hour_utc,
                        minute: minute_utc,
                        local_offset,
                    });

                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        debug!(
                            "  [Time] {}-{:02}-{:02} {:02}:{:02} UTC, Offset: {:+.1}h",
                            year_utc, month_utc, day_utc, hour_utc, minute_utc, local_offset
                        );
                    }
                }
            }
            (10, 0) => {
                // 10A - 8-Character PTY Name (PTYN)
                // Segment address: bits 5..4 of Block 2 (2 bits = 0-3)
                let seg = ((block2 >> 4) & 0x03) as usize;

                // Characters from Block 3-4 (inverted)
                let block3_inv = block3 ^ 0xFFFF;
                let block4_inv = block4 ^ 0xFFFF;
                let c1 = (block3_inv & 0xFF) as u8;
                let c2 = ((block3_inv >> 8) & 0xFF) as u8;
                let c3 = (block4_inv & 0xFF) as u8;
                let c4 = ((block4_inv >> 8) & 0xFF) as u8;

                // Verbose debug output (converted to tracing)
        if self.verbose {
                    debug!(
                        "  [PTYN] Segment {}/4: chars at positions {} {}: '{}{}{}{}' (0x{:02X} 0x{:02X} 0x{:02X} 0x{:02X})",
                        seg,
                        seg * 4,
                        seg * 4 + 3,
                        if c1.is_ascii_graphic() || c1 == b' ' {
                            c1 as char
                        } else {
                            '?'
                        },
                        if c2.is_ascii_graphic() || c2 == b' ' {
                            c2 as char
                        } else {
                            '?'
                        },
                        if c3.is_ascii_graphic() || c3 == b' ' {
                            c3 as char
                        } else {
                            '?'
                        },
                        if c4.is_ascii_graphic() || c4 == b' ' {
                            c4 as char
                        } else {
                            '?'
                        },
                        c1,
                        c2,
                        c3,
                        c4
                    );
                }
            }
            (14, 0) => {
                // 14A - Enhanced Other Network (EON) information
                // Block 4 contains PI of other network (required)
                let on_pi = block4;
                let eon_variant = (block2 & 0x0F) as u8; // Bits 3..0

                // Create base EON info with PI
                let mut eon_data = EONInfo {
                    pi: on_pi,
                    variant: eon_variant,
                    program_type: None,
                    has_linkage: None,
                    linkage_set: None,
                    alt_frequency: None,
                };

                // Verbose debug output (converted to tracing)
        if self.verbose {
                    debug!(
                        "  [EON] PI=0x{:04X}, Variant {} (Other Network)",
                        on_pi, eon_variant
                    );
                }

                // Variants 0-3: PS name segments
                if eon_variant <= 3 {
                    let b3_h = ((block3 >> 8) & 0xFF) as u8;
                    let b3_l = (block3 & 0xFF) as u8;
                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        debug!(
                            "    [EON-PS] Segment {}: chars at {} {}: '{}{}' (0x{:02X} 0x{:02X})",
                            eon_variant,
                            eon_variant * 2,
                            eon_variant * 2 + 1,
                            if b3_h.is_ascii_graphic() || b3_h == b' ' {
                                b3_h as char
                            } else {
                                '?'
                            },
                            if b3_l.is_ascii_graphic() || b3_l == b' ' {
                                b3_l as char
                            } else {
                                '?'
                            },
                            b3_h,
                            b3_l
                        );
                    }
                }
                // Variant 4: Alternative Frequency list
                else if eon_variant == 4 {
                    let af1 = ((block3 >> 8) & 0xFF) as u8;
                    let af2 = (block3 & 0xFF) as u8;
                    eon_data.alt_frequency = Some(af1);
                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        debug!("    [EON-AF] AF1=0x{:02X}, AF2=0x{:02X}", af1, af2);
                    }
                }
                // Variants 5-9: Frequency in kilohertz
                else if (5..=9).contains(&eon_variant) {
                    let freq_code = (block3 & 0xFF) as u8;
                    // Frequency: 87.5 + code*0.1 MHz for codes 1-204
                    if freq_code > 0 && freq_code < 205 {
                        let freq_mhz = 87.5 + (freq_code as f32) * 0.1;
                        // Verbose debug output (converted to tracing)
        if self.verbose {
                            debug!(
                                "    [EON-Freq] Variant {} = {:.1} MHz",
                                eon_variant, freq_mhz
                            );
                        }
                    }
                }
                // Variant 12: Linkage information
                else if eon_variant == 12 {
                    let has_linkage = (block3 >> 15) & 0x01 != 0;
                    let lsn = block3 & 0x0FFF;
                    eon_data.has_linkage = Some(has_linkage);
                    if has_linkage && lsn != 0 {
                        eon_data.linkage_set = Some(lsn);
                    }
                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        debug!(
                            "    [EON-Link] Has linkage: {}, LSN=0x{:03X}",
                            has_linkage, lsn
                        );
                    }
                }
                // Variant 13: Program Type and TA
                else if eon_variant == 13 {
                    let pty = ((block3 >> 11) & 0x1F) as u8;
                    let ta = (block3 & 0x0001) != 0;
                    eon_data.program_type = Some(pty);
                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        debug!(
                            "    [EON-PTY] PTY={} ({}), TA={}",
                            pty,
                            if (pty as usize) < PTY_NAMES.len() {
                                PTY_NAMES[pty as usize]
                            } else {
                                "Unknown"
                            },
                            ta
                        );
                    }
                }
                // Variant 14: PIN (Program Item Number)
                else if eon_variant == 14 {
                    // Verbose debug output (converted to tracing)
        if self.verbose {
                        let day = ((block3 >> 11) & 0x1F) as u8;
                        let hour = ((block3 >> 6) & 0x1F) as u8;
                        let minute = (block3 & 0x3F) as u8;
                        debug!("    [EON-PIN] Day {}, {:02}:{:02}", day, hour, minute);
                    }
                }
                // Variant 15: Broadcaster data
                else if eon_variant == 15 && self.verbose {
                    debug!("    [EON-Data] Broadcaster data: 0x{:04X}", block3);
                }

                self.station_info.eon_info = Some(eon_data);
            }
            _ => {
                // For other group types, log at trace level
                trace!(group = %group_type_str, "Unhandled RDS group type");
            }
        }

        // Finalize JSON output with current state
        // Add PS name if complete
        if let Some(ps) = self.station_name() {
            json_out.ps = Some(ps);
        }
        // Add radiotext if available
        if let Some(rt) = self.radio_text() {
            json_out.radiotext = Some(rt);
        }
        // Add alternative frequencies if any
        if !self.station_info.af_list.is_empty() {
            let af_strs: Vec<String> = self
                .station_info
                .af_list
                .iter()
                .map(|&f| format!("{:.1}", f as f64 / 100.0))
                .collect();
            json_out.alt_freqs = Some(af_strs);
        }

        // Queue JSON output
        if self.json_mode {
            self.json_output_queue.push(json_out);
        }
    }
}

impl Default for RdsParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "rds_tests.rs"]
mod tests;
