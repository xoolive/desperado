//! Per-band M/S stereo decision for the LC encoder — the forward twin of
//! [`super::stereo`] (ISO/IEC 14496-3 §4.6.8, Table 4.4).
//!
//! Per scalefactor band (long windows) or per (window, band) — 8 groups of
//! 1 window, flattened window-major as `g * n_sfb + b` (short windows) — the
//! band is coded M/S when the side signal is clearly cheaper than the
//! weaker original channel:
//!
//! ```text
//! ms_used[b]  ⇔  3 · Σs²  <  min(Σl², Σr²)      s = (l−r)/2
//! ```
//!
//! The ×3 margin (~4.8 dB) keeps uncorrelated content on L/R with room to
//! spare (there `3·Σs² ≈ 1.5·min(Σl², Σr²)` — narrow-band energy noise of
//! ~1/√n_bins cannot flip it), while correlated content concentrates into
//! mid and passes by orders of magnitude. Equivalently, equal-amplitude
//! bands need correlation ρ > 1/3 to go M/S. This is the per-band
//! refinement of the previous whole-frame energy comparison, and by
//! construction never scores worse than either whole-pair choice under the
//! same metric (`Σ min(a_b, b_b) ≤ min(Σ a_b, Σ b_b)`). Energy sums run in
//! f64 — IEEE `+` / `*` only, so the decision is platform-deterministic
//! without `det_math` transcendentals.
//!
//! Bands with no signal in either channel decode identically either way
//! (both sides quantize to all-zero, and `m ± s` of zeros is zero), so
//! their bits are folded toward unanimity: a one-texture frame still emits
//! the 2-bit whole-pair mask (0 or 2) instead of `1` + bitmask.
//!
//! The decision runs once per frame, before the rate loop, on the MDCT
//! spectra; chosen bands are transformed in place (`m = (l+r)/2`,
//! `s = (l−r)/2`). The mask form is frozen for the frame: bands the hard
//! cap later drops keep their `ms_used` bit (the decoder skips them).

use super::bits::BitWriter;
use super::enc_quant::{MAX_FLAT_SHORT, MAX_GROUPS};
use super::swb::{LONG_WINDOW_LEN, SHORT_WINDOW_LEN};

/// −60 dB relative-to-loudest floor for the no-signal fold (same ratio the
/// psy model uses to drop bands).
const QUIET_FLOOR: f64 = 1e-6;

/// Wire form of one frame's M/S decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaskForm {
    /// `ms_mask_present = 0` — no M/S.
    Off,
    /// `ms_mask_present = 2` — every band M/S.
    All,
    /// `ms_mask_present = 1` — the `ms_used` bitmask follows.
    PerBand,
}

/// One CPE frame's per-band M/S decision, in the exact order the decoder
/// reads the `ms_mask_present = 1` bitmask (`stereo::MsInfo::parse`): band
/// order for long windows, group-major `(group, band)` for short windows.
pub struct MsBands {
    used: [bool; MAX_FLAT_SHORT],
    /// Live entries: `n_bands` (long) or `8 * n_sfb` (short); 0 for mono.
    n: usize,
}

impl MsBands {
    /// No decision (mono / SCE) — never emitted.
    pub fn off() -> Self {
        Self {
            used: [false; MAX_FLAT_SHORT],
            n: 0,
        }
    }

    /// Wire form: unanimous decisions collapse to the 2-bit whole-pair
    /// masks, a split decision emits the per-band bitmask.
    pub fn form(&self) -> MaskForm {
        let live = &self.used[..self.n];
        if live.iter().all(|&u| u) && self.n > 0 {
            MaskForm::All
        } else if live.iter().any(|&u| u) {
            MaskForm::PerBand
        } else {
            MaskForm::Off
        }
    }

    /// Bits the mask occupies in the CPE header (replaces the fixed 2).
    pub fn overhead_bits(&self) -> usize {
        match self.form() {
            MaskForm::PerBand => 2 + self.n,
            MaskForm::Off | MaskForm::All => 2,
        }
    }

    /// Emit `ms_mask_present` and, for [`MaskForm::PerBand`], the `ms_used`
    /// bitmask — mirroring `stereo::MsInfo::parse`.
    pub fn emit(&self, w: &mut BitWriter) {
        match self.form() {
            MaskForm::Off => w.write(0, 2),
            MaskForm::All => w.write(2, 2),
            MaskForm::PerBand => {
                w.write(1, 2);
                for &u in &self.used[..self.n] {
                    w.write_bit(u);
                }
            }
        }
    }

    /// Re-apply this frame's long-window M/S transform to a second spectrum
    /// pair: the encoder's psy snapshot (taken before TNS analysis) must
    /// see the same coded bands as the transmitted spectra.
    pub fn apply_long_to(&self, specs: &mut [[f32; LONG_WINDOW_LEN]; 2], offsets: &[u16]) {
        apply_long(specs, offsets, &self.used[..self.n]);
    }

    /// Test / debug access to one decision bit.
    #[cfg(test)]
    pub fn used(&self, i: usize) -> bool {
        self.used[i]
    }
}

/// M/S decision margin: the side energy must be 3× (~4.8 dB) under the
/// weaker original channel (module docs).
const MS_MARGIN: f64 = 3.0;

/// Per-band energies of the pair and the winning decision for one band:
/// M/S iff `MS_MARGIN·e_side < min(e_left, e_right)` (module docs).
fn band_prefers_ms(e_left: f64, e_right: f64, e_side: f64) -> bool {
    MS_MARGIN * e_side < e_left.min(e_right)
}

/// Fold no-signal bands toward unanimity (module docs) and freeze `used`.
fn fold_quiet(used: &mut [bool], quiet: &[bool]) {
    let mut any_loud = false;
    let mut all_ms = true;
    for (&q, &u) in quiet.iter().zip(used.iter()) {
        if !q {
            any_loud = true;
            all_ms &= u;
        }
    }
    if !any_loud {
        used.fill(false); // digital silence: Off
        return;
    }
    if all_ms {
        used.fill(true); // every loud band is M/S: All
    }
    // Otherwise mixed: quiet bands keep their (false) metric outcome.
}

/// Long-window decision over `specs` (`[l, r]`, 1024 bins each), applying
/// the M/S transform in place on the chosen bands. `per_band = false` is
/// the test-only whole-pair A/B mode: one decision from the grand totals.
pub fn decide_long(
    specs: &mut [[f32; LONG_WINDOW_LEN]; 2],
    offsets: &[u16],
    per_band: bool,
) -> MsBands {
    let n_bands = offsets.len() - 1;
    let [l, r] = specs;
    let mut e_l = [0.0f64; MAX_FLAT_SHORT];
    let mut e_r = [0.0f64; MAX_FLAT_SHORT];
    let mut e_s = [0.0f64; MAX_FLAT_SHORT];
    let mut out = MsBands {
        used: [false; MAX_FLAT_SHORT],
        n: n_bands,
    };
    let mut loudest = 0.0f64;
    for b in 0..n_bands {
        let (lo, hi) = (usize::from(offsets[b]), usize::from(offsets[b + 1]));
        for (&lv, &rv) in l[lo..hi].iter().zip(r[lo..hi].iter()) {
            let s = f64::from((lv - rv) * 0.5);
            e_l[b] += f64::from(lv) * f64::from(lv);
            e_r[b] += f64::from(rv) * f64::from(rv);
            e_s[b] += s * s;
        }
        loudest = loudest.max(e_l[b].max(e_r[b]));
        out.used[b] = band_prefers_ms(e_l[b], e_r[b], e_s[b]);
    }
    let mut quiet = [false; MAX_FLAT_SHORT];
    for b in 0..n_bands {
        quiet[b] = e_l[b].max(e_r[b]) < loudest * QUIET_FLOOR;
    }
    if per_band {
        fold_quiet(&mut out.used[..n_bands], &quiet[..n_bands]);
    } else {
        let (mut tl, mut tr, mut ts) = (0.0f64, 0.0f64, 0.0f64);
        for b in 0..n_bands {
            tl += e_l[b];
            tr += e_r[b];
            ts += e_s[b];
        }
        out.used[..n_bands].fill(band_prefers_ms(tl, tr, ts));
    }
    apply_long(specs, offsets, &out.used[..n_bands]);
    out
}

/// Apply `(m, s) = ((l+r)/2, (l−r)/2)` on the chosen long-window bands.
fn apply_long(specs: &mut [[f32; LONG_WINDOW_LEN]; 2], offsets: &[u16], used: &[bool]) {
    let [l, r] = specs;
    for (b, &u) in used.iter().enumerate() {
        if !u {
            continue;
        }
        let (lo, hi) = (usize::from(offsets[b]), usize::from(offsets[b + 1]));
        for (lv, rv) in l[lo..hi].iter_mut().zip(r[lo..hi].iter_mut()) {
            let (a, b2) = (*lv, *rv);
            *lv = (a + b2) * 0.5;
            *rv = (a - b2) * 0.5;
        }
    }
}

/// Short-window decision over the window-major `specs`: one bit per
/// (window, band), flattened `g * n_sfb + b` — group `g` is window `g`
/// (no `scale_factor_grouping`), the decoder's exact bitmask read order.
pub fn decide_short(
    specs: &mut [[f32; LONG_WINDOW_LEN]; 2],
    offsets: &[u16],
    per_band: bool,
) -> MsBands {
    let n_sfb = offsets.len() - 1;
    let n = MAX_GROUPS * n_sfb;
    let [l, r] = specs;
    let mut e_l = [0.0f64; MAX_FLAT_SHORT];
    let mut e_r = [0.0f64; MAX_FLAT_SHORT];
    let mut e_s = [0.0f64; MAX_FLAT_SHORT];
    let mut loudest = 0.0f64;
    let mut out = MsBands {
        used: [false; MAX_FLAT_SHORT],
        n,
    };
    for w in 0..MAX_GROUPS {
        for b in 0..n_sfb {
            let idx = w * n_sfb + b;
            let lo = w * SHORT_WINDOW_LEN + usize::from(offsets[b]);
            let hi = w * SHORT_WINDOW_LEN + usize::from(offsets[b + 1]);
            for (&lv, &rv) in l[lo..hi].iter().zip(r[lo..hi].iter()) {
                let s = f64::from((lv - rv) * 0.5);
                e_l[idx] += f64::from(lv) * f64::from(lv);
                e_r[idx] += f64::from(rv) * f64::from(rv);
                e_s[idx] += s * s;
            }
            loudest = loudest.max(e_l[idx].max(e_r[idx]));
            out.used[idx] = band_prefers_ms(e_l[idx], e_r[idx], e_s[idx]);
        }
    }
    let mut quiet = [false; MAX_FLAT_SHORT];
    for i in 0..n {
        quiet[i] = e_l[i].max(e_r[i]) < loudest * QUIET_FLOOR;
    }
    if per_band {
        fold_quiet(&mut out.used[..n], &quiet[..n]);
    } else {
        let (mut tl, mut tr, mut ts) = (0.0f64, 0.0f64, 0.0f64);
        for i in 0..n {
            tl += e_l[i];
            tr += e_r[i];
            ts += e_s[i];
        }
        out.used[..n].fill(band_prefers_ms(tl, tr, ts));
    }
    // Transform the chosen (window, band) regions in place.
    let [l, r] = specs;
    for w in 0..MAX_GROUPS {
        for b in 0..n_sfb {
            if !out.used[w * n_sfb + b] {
                continue;
            }
            let lo = w * SHORT_WINDOW_LEN + usize::from(offsets[b]);
            let hi = w * SHORT_WINDOW_LEN + usize::from(offsets[b + 1]);
            for (lv, rv) in l[lo..hi].iter_mut().zip(r[lo..hi].iter_mut()) {
                let (a, b2) = (*lv, *rv);
                *lv = (a + b2) * 0.5;
                *rv = (a - b2) * 0.5;
            }
        }
    }
    out
}

#[cfg(test)]
#[path = "enc_ms_tests.rs"]
mod enc_ms_tests;
