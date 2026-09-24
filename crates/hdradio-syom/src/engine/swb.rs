//! Scalefactor-band offsets — ISO/IEC 14496-3 Tables 4.129–4.141.

use super::error::{Error, Result};

/// Long-window spectrum length (1024-line family).
pub const LONG_WINDOW_LEN: usize = 1024;
/// Short-window spectrum length (one of eight).
pub const SHORT_WINDOW_LEN: usize = 128;

/// Table 4.129 — 44.1 / 48 kHz long (49 SWB).
const LONG_48K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 160,
    176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704,
    736, 768, 800, 832, 864, 896, 928, 1024,
];
/// Table 4.131 — 32 kHz long (51 SWB).
const LONG_32K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 160,
    176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704,
    736, 768, 800, 832, 864, 896, 928, 960, 992, 1024,
];
/// Table 4.132 — 8 kHz long (40 SWB).
const LONG_8K: &[u16] = &[
    0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144, 156, 172, 188, 204, 220, 236, 252, 268,
    288, 308, 328, 348, 372, 396, 420, 448, 476, 508, 544, 580, 620, 664, 712, 764, 820, 880, 944,
    1024,
];
/// Table 4.134 — 11.025 / 12 / 16 kHz long (43 SWB).
const LONG_16K: &[u16] = &[
    0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 100, 112, 124, 136, 148, 160, 172, 184, 196, 212,
    228, 244, 260, 280, 300, 320, 344, 368, 396, 424, 456, 492, 532, 572, 616, 664, 716, 772, 832,
    896, 960, 1024,
];
/// Table 4.136 — 22.05 / 24 kHz long (47 SWB).
const LONG_24K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 52, 60, 68, 76, 84, 92, 100, 108, 116, 124, 136,
    148, 160, 172, 188, 204, 220, 240, 260, 284, 308, 336, 364, 396, 432, 468, 508, 552, 600, 652,
    704, 768, 832, 896, 960, 1024,
];
/// Table 4.138 — 64 kHz long (47 SWB).
const LONG_64K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 100, 112, 124, 140,
    156, 172, 192, 216, 240, 268, 304, 344, 384, 424, 464, 504, 544, 584, 624, 664, 704, 744, 784,
    824, 864, 904, 944, 984, 1024,
];
/// Table 4.140 — 88.2 / 96 kHz long (41 SWB).
const LONG_96K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 96, 108, 120, 132,
    144, 156, 172, 188, 212, 240, 276, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024,
];

/// Table 4.130 — 32 / 44.1 / 48 kHz short (14 SWB).
const SHORT_48K: &[u16] = &[0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 128];
/// Table 4.133 — 8 kHz short (15 SWB).
const SHORT_8K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 60, 72, 88, 108, 128,
];
/// Table 4.135 — 11.025 / 12 / 16 kHz short (15 SWB).
const SHORT_16K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 60, 72, 88, 108, 128,
];
/// Table 4.137 — 22.05 / 24 kHz short (15 SWB).
const SHORT_24K: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 36, 44, 52, 64, 76, 92, 108, 128,
];
/// Table 4.139 / 4.141 — 64 / 88.2 / 96 kHz short (12 SWB).
const SHORT_64K: &[u16] = &[0, 4, 8, 12, 16, 20, 24, 32, 40, 48, 64, 92, 128];

const LONG: [&[u16]; 12] = [
    LONG_96K, LONG_96K, LONG_64K, LONG_48K, LONG_48K, LONG_32K, LONG_24K, LONG_24K, LONG_16K,
    LONG_16K, LONG_16K, LONG_8K,
];
const SHORT: [&[u16]; 12] = [
    SHORT_64K, SHORT_64K, SHORT_64K, SHORT_48K, SHORT_48K, SHORT_48K, SHORT_24K, SHORT_24K,
    SHORT_16K, SHORT_16K, SHORT_16K, SHORT_8K,
];

/// Long-window `swb_offset` for `samplingFrequencyIndex` 0..=11.
pub fn long_offsets(fs_index: u8) -> Result<&'static [u16]> {
    LONG.get(fs_index as usize)
        .copied()
        .ok_or(Error::UnsupportedSampleRateIndex(fs_index))
}

/// Short-window `swb_offset` for `samplingFrequencyIndex` 0..=11.
pub fn short_offsets(fs_index: u8) -> Result<&'static [u16]> {
    SHORT
        .get(fs_index as usize)
        .copied()
        .ok_or(Error::UnsupportedSampleRateIndex(fs_index))
}
