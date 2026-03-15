//! DAB Mode I parameters and channel frequency table.
//! Reference: ETSI EN 300 401

use std::f64::consts::PI;

// Mode I parameters (the only mode used in practice)
pub const T_U: usize = 2048; // FFT size / useful symbol duration in samples
pub const T_G: usize = 504; // Guard interval (cyclic prefix) in samples
pub const T_S: usize = T_U + T_G; // Total symbol duration: 2552 samples
pub const T_NULL: usize = 2656; // Null symbol duration in samples
pub const T_F: usize = 196608; // Transmission frame duration in samples
pub const L_SYMBOLS: usize = 76; // Non-null symbols per frame: 1 PRS + 75 data
pub const K: usize = 1536; // Number of active carriers
pub const SAMPLE_RATE: u32 = 2_048_000; // 2.048 MHz
pub const CARRIER_DIFF: f64 = 1000.0; // Carrier spacing in Hz

// FIC parameters
pub const FIC_SYMBOLS: usize = 3; // Symbols 1..3 carry FIC data
pub const FIBS_PER_FIC: usize = 3; // 3 FIBs per FIC (Mode I)
pub const FIB_LENGTH: usize = 32; // bytes per FIB (30 data + 2 CRC)
pub const FIB_CRC_LENGTH: usize = 2;

// CIF parameters (used by MSC decoding)
#[allow(dead_code)]
pub const CIFS_PER_FRAME: usize = 4; // Common Interleaved Frames per transmission frame
#[allow(dead_code)]
pub const CU_SIZE: usize = 64; // bits per Capacity Unit

/// DAB Band III channel frequencies (MHz).
/// Returns None for unknown channel names.
pub fn channel_frequency(channel: &str) -> Option<u32> {
    let freq_khz: u32 = match channel {
        "5A" => 174_928,
        "5B" => 176_640,
        "5C" => 178_352,
        "5D" => 180_064,
        "6A" => 181_936,
        "6B" => 183_648,
        "6C" => 185_360,
        "6D" => 187_072,
        "7A" => 188_928,
        "7B" => 190_640,
        "7C" => 192_352,
        "7D" => 194_064,
        "8A" => 195_936,
        "8B" => 197_648,
        "8C" => 199_360,
        "8D" => 201_072,
        "9A" => 202_928,
        "9B" => 204_640,
        "9C" => 206_352,
        "9D" => 208_064,
        "10A" => 209_936,
        "10B" => 211_648,
        "10C" => 213_360,
        "10D" => 215_072,
        "11A" => 216_928,
        "11B" => 218_640,
        "11C" => 220_352,
        "11D" => 222_064,
        "12A" => 223_936,
        "12B" => 225_648,
        "12C" => 227_360,
        "12D" => 229_072,
        "13A" => 230_784,
        "13B" => 232_496,
        "13C" => 234_208,
        "13D" => 235_776,
        "13E" => 237_488,
        "13F" => 239_200,
        _ => return None,
    };
    Some(freq_khz * 1000)
}

/// Phase Reference Symbol table entry from ETSI EN 300 401 Table 44.
struct PrsEntry {
    kmin: i32,
    kmax: i32,
    i: usize,
    n: i32,
}

/// Mode I PRS table from ETSI EN 300 401 Table 44.
/// Each entry defines a carrier range [kmin, kmax] with parameters i and n.
/// Reference: welle.io src/backend/phasetable.cpp
const MODE_I_TABLE: &[PrsEntry] = &[
    PrsEntry {
        kmin: -768,
        kmax: -737,
        i: 0,
        n: 1,
    },
    PrsEntry {
        kmin: -736,
        kmax: -705,
        i: 1,
        n: 2,
    },
    PrsEntry {
        kmin: -704,
        kmax: -673,
        i: 2,
        n: 0,
    },
    PrsEntry {
        kmin: -672,
        kmax: -641,
        i: 3,
        n: 1,
    },
    PrsEntry {
        kmin: -640,
        kmax: -609,
        i: 0,
        n: 3,
    },
    PrsEntry {
        kmin: -608,
        kmax: -577,
        i: 1,
        n: 2,
    },
    PrsEntry {
        kmin: -576,
        kmax: -545,
        i: 2,
        n: 2,
    },
    PrsEntry {
        kmin: -544,
        kmax: -513,
        i: 3,
        n: 3,
    },
    PrsEntry {
        kmin: -512,
        kmax: -481,
        i: 0,
        n: 2,
    },
    PrsEntry {
        kmin: -480,
        kmax: -449,
        i: 1,
        n: 1,
    },
    PrsEntry {
        kmin: -448,
        kmax: -417,
        i: 2,
        n: 2,
    },
    PrsEntry {
        kmin: -416,
        kmax: -385,
        i: 3,
        n: 3,
    },
    PrsEntry {
        kmin: -384,
        kmax: -353,
        i: 0,
        n: 1,
    },
    PrsEntry {
        kmin: -352,
        kmax: -321,
        i: 1,
        n: 2,
    },
    PrsEntry {
        kmin: -320,
        kmax: -289,
        i: 2,
        n: 3,
    },
    PrsEntry {
        kmin: -288,
        kmax: -257,
        i: 3,
        n: 3,
    },
    PrsEntry {
        kmin: -256,
        kmax: -225,
        i: 0,
        n: 2,
    },
    PrsEntry {
        kmin: -224,
        kmax: -193,
        i: 1,
        n: 2,
    },
    PrsEntry {
        kmin: -192,
        kmax: -161,
        i: 2,
        n: 2,
    },
    PrsEntry {
        kmin: -160,
        kmax: -129,
        i: 3,
        n: 1,
    },
    PrsEntry {
        kmin: -128,
        kmax: -97,
        i: 0,
        n: 1,
    },
    PrsEntry {
        kmin: -96,
        kmax: -65,
        i: 1,
        n: 3,
    },
    PrsEntry {
        kmin: -64,
        kmax: -33,
        i: 2,
        n: 1,
    },
    PrsEntry {
        kmin: -32,
        kmax: -1,
        i: 3,
        n: 2,
    },
    PrsEntry {
        kmin: 1,
        kmax: 32,
        i: 0,
        n: 3,
    },
    PrsEntry {
        kmin: 33,
        kmax: 64,
        i: 3,
        n: 1,
    },
    PrsEntry {
        kmin: 65,
        kmax: 96,
        i: 2,
        n: 1,
    },
    PrsEntry {
        kmin: 97,
        kmax: 128,
        i: 1,
        n: 1,
    }, // bug fix by Jorgen Scott 2014-09-03
    PrsEntry {
        kmin: 129,
        kmax: 160,
        i: 0,
        n: 2,
    },
    PrsEntry {
        kmin: 161,
        kmax: 192,
        i: 3,
        n: 2,
    },
    PrsEntry {
        kmin: 193,
        kmax: 224,
        i: 2,
        n: 1,
    },
    PrsEntry {
        kmin: 225,
        kmax: 256,
        i: 1,
        n: 0,
    },
    PrsEntry {
        kmin: 257,
        kmax: 288,
        i: 0,
        n: 2,
    },
    PrsEntry {
        kmin: 289,
        kmax: 320,
        i: 3,
        n: 2,
    },
    PrsEntry {
        kmin: 321,
        kmax: 352,
        i: 2,
        n: 3,
    },
    PrsEntry {
        kmin: 353,
        kmax: 384,
        i: 1,
        n: 3,
    },
    PrsEntry {
        kmin: 385,
        kmax: 416,
        i: 0,
        n: 0,
    },
    PrsEntry {
        kmin: 417,
        kmax: 448,
        i: 3,
        n: 2,
    },
    PrsEntry {
        kmin: 449,
        kmax: 480,
        i: 2,
        n: 1,
    },
    PrsEntry {
        kmin: 481,
        kmax: 512,
        i: 1,
        n: 3,
    },
    PrsEntry {
        kmin: 513,
        kmax: 544,
        i: 0,
        n: 3,
    },
    PrsEntry {
        kmin: 545,
        kmax: 576,
        i: 3,
        n: 3,
    },
    PrsEntry {
        kmin: 577,
        kmax: 608,
        i: 2,
        n: 3,
    },
    PrsEntry {
        kmin: 609,
        kmax: 640,
        i: 1,
        n: 0,
    },
    PrsEntry {
        kmin: 641,
        kmax: 672,
        i: 0,
        n: 3,
    },
    PrsEntry {
        kmin: 673,
        kmax: 704,
        i: 3,
        n: 0,
    },
    PrsEntry {
        kmin: 705,
        kmax: 736,
        i: 2,
        n: 1,
    },
    PrsEntry {
        kmin: 737,
        kmax: 768,
        i: 1,
        n: 1,
    },
];

/// h0..h3 tables from ETSI EN 300 401 Table 44.
/// Used to compute the phase of each PRS carrier.
const H0: [i32; 32] = [
    0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1, 0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1,
];
const H1: [i32; 32] = [
    0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0, 0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0,
];
const H2: [i32; 32] = [
    0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3, 0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3,
];
const H3: [i32; 32] = [
    0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2, 0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2,
];

/// Look up h_table(i, j) from the ETSI tables.
fn h_table(i: usize, j: usize) -> i32 {
    match i {
        0 => H0[j],
        1 => H1[j],
        2 => H2[j],
        3 => H3[j],
        _ => panic!("invalid i in h_table: {}", i),
    }
}

/// Compute the phase (radians) for PRS carrier k using ETSI EN 300 401 Table 44.
fn get_phi(k: i32) -> f64 {
    for entry in MODE_I_TABLE {
        if entry.kmin <= k && k <= entry.kmax {
            let k_prime = entry.kmin;
            let j = (k - k_prime) as usize;
            return PI / 2.0 * (h_table(entry.i, j) + entry.n) as f64;
        }
    }
    panic!("invalid carrier k={} in get_phi", k);
}

/// Phase reference table for Mode I (2048 carriers).
/// Returns a frequency-domain vector of complex values representing the PRS.
/// Carrier k maps to FFT bin: if k > 0 then bin = k, else bin = T_U + k.
/// Reference: ETSI EN 300 401 Table 44, welle.io phasereference.cpp
pub fn phase_reference_table() -> Vec<num_complex::Complex<f32>> {
    let mut table = vec![num_complex::Complex::new(0.0f32, 0.0); T_U];

    for k in 1..=(K / 2) as i32 {
        // Positive carrier k → FFT bin k
        let phi_pos = get_phi(k);
        let (sin_p, cos_p) = phi_pos.sin_cos();
        table[k as usize] = num_complex::Complex::new(cos_p as f32, sin_p as f32);

        // Negative carrier -k → FFT bin T_U - k
        let phi_neg = get_phi(-k);
        let (sin_n, cos_n) = phi_neg.sin_cos();
        table[T_U - k as usize] = num_complex::Complex::new(cos_n as f32, sin_n as f32);
    }

    table
}

/// CRC-16 for FIB (CCITT polynomial x^16 + x^12 + x^5 + 1, init 0xFFFF, inverted).
pub fn fib_crc_valid(fib: &[u8]) -> bool {
    if fib.len() < FIB_LENGTH {
        return false;
    }
    let data = &fib[..FIB_LENGTH - FIB_CRC_LENGTH]; // 30 bytes
    let crc_bytes = &fib[FIB_LENGTH - FIB_CRC_LENGTH..FIB_LENGTH]; // 2 bytes

    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc ^= 0xFFFF;

    let received = ((crc_bytes[0] as u16) << 8) | (crc_bytes[1] as u16);
    crc == received
}
