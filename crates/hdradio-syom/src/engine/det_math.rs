//! Deterministic elementary math for the encoder decision path.
//!
//! IEEE 754 pins `+ - * / sqrt` to exact rounding, so they are bit-identical
//! on every platform; `powf` / `log2` / `exp2` / `atan` / `sin` / `cos` are
//! libm calls that may differ by 1 ulp across platforms (observed: macOS
//! arm64 vs Linux x86_64 flip `atan` and some `sin`/`cos` twiddles), which
//! can flip a scalefactor or a quantization rounding and drift the encoded
//! bytes. The encoder therefore routes every transcendental in its decision
//! path through the replacements below — each is built from exactly-rounded
//! ops over literals, so the encoder output is byte-identical across
//! platforms. Accuracy is a few ulp, plenty for the psy / quantization
//! decisions these feed.

use std::f32::consts::{FRAC_2_PI, FRAC_PI_2, FRAC_PI_6, LN_2, LOG2_E, SQRT_2};

/// log2(10) — for `10^x = 2^(x·log2(10))`.
pub const LOG2_10: f32 = std::f32::consts::LOG2_10;
/// ln(2)/16 — the residual scale of [`exp2`]'s sixteenth-split.
const LN_2_OVER_16: f32 = LN_2 / 16.0;
/// √3, for the `atan` argument reduction.
const SQRT_3: f32 = 1.732_050_8;
/// tan(π/12) = 2 − √3 — the `atan` reduction threshold.
const TAN_PI_12: f32 = 0.267_949_2;
/// π/2 split into a hi f32 and the remaining tail (Cody–Waite), so the
/// `sincos` quadrant reduction keeps ~full f32 accuracy.
const PIO2_HI: f32 = FRAC_PI_2;
const PIO2_LO: f32 = -4.371_139e-8; // π/2 − PIO2_HI

/// `2^(j/16)`, j = 0..16 — decimal literals round to the same f32 on every
/// platform (float-literal parsing is correctly rounded).
static TWO_POW_J_16: [f32; 16] = [
    1.0,
    1.044_273_7,
    1.090_507_7,
    1.138_788_6,
    1.189_207_1,
    1.241_857_8,
    1.296_839_6,
    1.354_255_6,
    SQRT_2,
    1.476_826_2,
    1.542_210_8,
    1.610_490_3,
    1.681_792_9,
    1.756_252_2,
    1.834_008_1,
    1.915_206_6,
];

/// `2^x` for finite `x`, deterministic. Splits `16x = n + f` (n integer,
/// f ∈ [0,1)) so `2^x = 2^q · 2^(j/16) · e^r` with `r = f·ln2/16` tiny enough
/// for a 5th-order Taylor (truncation < 1e-11). When `16x` is an exact
/// integer — the scalefactor-gain case `x = −3k/16` — the residual is
/// exactly 0 and the result is one exact-rounded product of table literals.
pub fn exp2(x: f32) -> f32 {
    let y = x * 16.0;
    if y < -2400.0 {
        return 0.0; // below the f32 subnormal floor
    }
    let n = y.floor() as i32;
    let r = (y - n as f32) * LN_2_OVER_16;
    let q = n.div_euclid(16);
    let j = n.rem_euclid(16) as usize;
    let scale = if q < -126 {
        // 2^q is subnormal: form it via a normal factor so `powi`'s
        // reciprocal doesn't flush it to zero.
        2f32.powi(q + 64) * 5.421_011e-20 // 2^(q+64) · 2^−64
    } else {
        2f32.powi(q) // exact power of two
    };
    // e^r, |r| < ln2/16 ≈ 0.0434: Horner of the 5th-order Taylor.
    let p = 1.0 + r * (1.0 + r * (0.5 + r * (1.0 / 6.0 + r * (1.0 / 24.0 + r * (1.0 / 120.0)))));
    scale * TWO_POW_J_16[j] * p
}

/// `log2(x)` for `x > 0` (subnormals included), deterministic. Frexp-style
/// decomposition via the bit pattern, then `ln(m) = 2·atanh(t)` with
/// `t = (m−1)/(m+1)`; range-reducing `m` to [√2/2, √2) keeps |t| ≤ 0.172 so
/// the 9th-order series truncates under 1e-9. Only `+ * /` — every op
/// exactly rounded.
pub fn log2(x: f32) -> f32 {
    debug_assert!(x > 0.0);
    // Lift subnormals into the normal range (2^24 is an exact power of two).
    let (x, bias) = if x < f32::MIN_POSITIVE {
        (x * 16_777_216.0, -24)
    } else {
        (x, 0)
    };
    let bits = x.to_bits();
    let mut e = ((bits >> 23) & 0xff) as i32 - 127 + bias;
    let mut m = f32::from_bits((bits & 0x007f_ffff) | 0x3f80_0000); // [1, 2)
    if m >= SQRT_2 {
        m *= 0.5; // exact halving → m ∈ [√2/2, √2)
        e += 1;
    }
    let t = (m - 1.0) / (m + 1.0);
    let t2 = t * t;
    // ln(m) = 2t·(1 + t²/3 + t⁴/5 + t⁶/7 + t⁸/9)
    let s = t * (1.0 + t2 * (1.0 / 3.0 + t2 * (1.0 / 5.0 + t2 * (1.0 / 7.0 + t2 * (1.0 / 9.0)))));
    e as f32 + 2.0 * LOG2_E * s
}

/// `atan(x)`, deterministic. Reduces to |t| ≤ tan(π/12) via the reciprocal
/// and π/6-shift identities, then the alternating Taylor series (next term
/// < 2e-10). Division is exactly rounded, so the reductions are too.
pub fn atan(x: f32) -> f32 {
    if x < 0.0 {
        return -atan(-x);
    }
    if x > 1.0 {
        return FRAC_PI_2 - atan(1.0 / x);
    }
    let (t, shift) = if x > TAN_PI_12 {
        // atan(x) = π/6 + atan((√3·x − 1)/(√3 + x))
        ((x * SQRT_3 - 1.0) / (x + SQRT_3), FRAC_PI_6)
    } else {
        (x, 0.0)
    };
    shift + atan_poly(t)
}

/// `atan(t)` for |t| ≤ tan(π/12): Taylor to t^13, remainder < t^15/15 ≈ 2e-10.
fn atan_poly(t: f32) -> f32 {
    let t2 = t * t;
    t * (1.0
        + t2 * (-1.0 / 3.0
            + t2 * (1.0 / 5.0
                + t2 * (-1.0 / 7.0 + t2 * (1.0 / 9.0 + t2 * (-1.0 / 11.0 + t2 * (1.0 / 13.0)))))))
}

/// `(sin a, cos a)` for a ∈ [0, 2π), deterministic — the MDCT/FFT twiddle
/// domain. Nearest-quadrant reduction with a Cody–Waite π/2 keeps the
/// residual |r| ≤ π/4 + 1 ulp, where the Taylor polys err under 1e-12.
pub fn sincos(a: f32) -> (f32, f32) {
    debug_assert!((0.0..std::f32::consts::TAU).contains(&a));
    // k = round(a / (π/2)) ∈ 0..=4.
    let k = (a * FRAC_2_PI + 0.5).floor() as i32;
    let kf = k as f32;
    let r = (a - kf * PIO2_HI) - kf * PIO2_LO;
    let (s, c) = (sin_poly(r), cos_poly(r));
    match k.rem_euclid(4) {
        0 => (s, c),
        1 => (c, -s),
        2 => (-s, -c),
        _ => (-c, s),
    }
}

/// `sin(r)` for |r| ≤ π/4 + ε: Taylor to r^13, remainder < 2e-14.
fn sin_poly(r: f32) -> f32 {
    let r2 = r * r;
    r * (1.0
        + r2 * (-1.0 / 6.0
            + r2 * (1.0 / 120.0
                + r2 * (-1.0 / 5040.0
                    + r2 * (1.0 / 362_880.0 + r2 * (-1.0 / 39_916_800.0 + r2 / 6_227_020_800.0))))))
}

/// `cos(r)` for |r| ≤ π/4 + ε: Taylor to r^12, remainder < 5e-13.
fn cos_poly(r: f32) -> f32 {
    let r2 = r * r;
    1.0 + r2
        * (-1.0 / 2.0
            + r2 * (1.0 / 24.0
                + r2 * (-1.0 / 720.0
                    + r2 * (1.0 / 40_320.0 + r2 * (-1.0 / 3_628_800.0 + r2 / 479_001_600.0)))))
}

/// `|x|^0.75` from two exactly-rounded sqrts: `√|x| · √(√|x|)`.
pub fn pow_three_quarter(x: f32) -> f32 {
    let r = x.abs().sqrt();
    r * r.sqrt()
}

/// Per-stage FFT twiddles `e^{+i·2πk/len}` — same layout as
/// [`super::imdct::twiddle_table`] but built from [`sincos`] so the forward
/// MDCT's twiddles are platform-deterministic.
pub fn twiddle_table(n: usize) -> (Vec<f32>, Vec<f32>) {
    let mut re = Vec::with_capacity(n);
    let mut im = Vec::with_capacity(n);
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = 2.0 * std::f32::consts::PI / len as f32;
        for k in 0..half {
            let (s, c) = sincos(ang * k as f32);
            re.push(c);
            im.push(s);
        }
        len *= 2;
    }
    (re, im)
}

#[cfg(test)]
#[path = "det_math_tests.rs"]
mod det_math_tests;
