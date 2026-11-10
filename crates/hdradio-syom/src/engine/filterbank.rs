//! Filterbank: windows + overlap-add — ISO/IEC 14496-3 §4.6.11.3.2 / §4.6.11.3.3.

use super::error::{Error, Result};
use super::ics::{IcsInfo, WindowSequence, WindowShape};
use super::imdct::imdct_into_f32;
use super::swb::{LONG_WINDOW_LEN, SHORT_WINDOW_LEN};
use std::sync::LazyLock;

const N_L: usize = 2048;
const N_S: usize = 256;

fn to_array<const N: usize>(v: Vec<f64>) -> [f32; N] {
    let mut a = [0.0f32; N];
    for (d, s) in a.iter_mut().zip(v) {
        *d = s as f32;
    }
    a
}

static SINE_LONG: LazyLock<[f32; 1024]> = LazyLock::new(|| to_array(sine_left(1024)));
static SINE_SHORT: LazyLock<[f32; 128]> = LazyLock::new(|| to_array(sine_left(128)));
static KBD_LONG: LazyLock<[f32; 1024]> = LazyLock::new(|| to_array(kbd_left(1024, 4.0)));
static KBD_SHORT: LazyLock<[f32; 128]> = LazyLock::new(|| to_array(kbd_left(128, 6.0)));

fn bessel_i0(x: f64) -> f64 {
    let half_x = x / 2.0;
    let mut term = 1.0f64;
    let mut sum = 1.0f64;
    let mut k = 1.0f64;
    loop {
        term *= (half_x / k) * (half_x / k);
        sum += term;
        if term <= sum * 1e-18 || k > 256.0 {
            break;
        }
        k += 1.0;
    }
    sum
}

fn kbd_left(half: usize, alpha: f64) -> Vec<f64> {
    let quarter = half as f64 / 2.0;
    let denom = bessel_i0(std::f64::consts::PI * alpha);
    let kernel: Vec<f64> = (0..=half)
        .map(|n| {
            let t = (n as f64 - quarter) / quarter;
            let rad = (1.0 - t * t).max(0.0);
            bessel_i0(std::f64::consts::PI * alpha * rad.sqrt()) / denom
        })
        .collect();
    let total: f64 = kernel.iter().sum();
    let mut running = 0.0f64;
    let mut out = Vec::with_capacity(half);
    for &w in kernel.iter().take(half) {
        running += w;
        out.push((running / total).sqrt());
    }
    out
}

fn sine_left(half: usize) -> Vec<f64> {
    let n = (2 * half) as f64;
    (0..half)
        .map(|i| (std::f64::consts::PI / n * (i as f64 + 0.5)).sin())
        .collect()
}

fn half(n: usize, shape: WindowShape) -> &'static [f32] {
    match (n, shape) {
        (N_L, WindowShape::Sine) => SINE_LONG.as_slice(),
        (N_L, WindowShape::Kbd) => KBD_LONG.as_slice(),
        (N_S, WindowShape::Sine) => SINE_SHORT.as_slice(),
        (N_S, WindowShape::Kbd) => KBD_SHORT.as_slice(),
        _ => SINE_LONG.as_slice(),
    }
}

/// Window left half for the encoder's analysis side (mirrors [`half`]; the
/// right half of an OnlyLong window is this reversed).
pub(crate) fn window_left(n: usize, shape: WindowShape) -> &'static [f32] {
    half(n, shape)
}

/// Per-channel overlap-add state.
#[derive(Clone, Debug)]
pub struct Filterbank {
    overlap: [f32; LONG_WINDOW_LEN],
    prev_shape: Option<WindowShape>,
    scratch: [f32; N_L],
}

impl Default for Filterbank {
    fn default() -> Self {
        Self::new()
    }
}

impl Filterbank {
    /// Zero overlap, previous shape sine (lavc `use_kb_window[1]` starts at 0).
    #[must_use]
    pub fn new() -> Self {
        Self {
            overlap: [0.0; LONG_WINDOW_LEN],
            prev_shape: None,
            scratch: [0.0; N_L],
        }
    }

    /// Synthesize 1024 PCM samples from a 1024-bin window-major spectrum.
    #[cfg(test)]
    pub fn synthesize(&mut self, spec: &[f32], ics: &IcsInfo) -> Result<Vec<f32>> {
        let mut out = Vec::new();
        self.synthesize_into(spec, ics, &mut out)?;
        Ok(out)
    }

    /// Write one frame into `dst`.
    pub fn synthesize_into(
        &mut self,
        spec: &[f32],
        ics: &IcsInfo,
        dst: &mut Vec<f32>,
    ) -> Result<()> {
        self.windowed(spec, ics)?;
        let half_n = LONG_WINDOW_LEN;
        dst.resize(half_n, 0.0);
        let (left, right) = self.scratch.split_at(half_n);
        for ((d, o), (&s, &next)) in dst
            .iter_mut()
            .zip(self.overlap.iter_mut())
            .zip(left.iter().zip(right.iter()))
        {
            *d = s + *o;
            *o = next;
        }
        self.prev_shape = Some(ics.window_shape);
        Ok(())
    }

    fn windowed(&mut self, spec: &[f32], ics: &IcsInfo) -> Result<()> {
        let left = self.prev_shape.unwrap_or(WindowShape::Sine);
        let right = ics.window_shape;
        match ics.window_sequence {
            WindowSequence::OnlyLong | WindowSequence::LongStart | WindowSequence::LongStop => {
                self.long_windowed(spec, left, right, ics.window_sequence)
            }
            WindowSequence::EightShort => self.short_windowed(spec, left, right),
        }
    }

    fn long_windowed(
        &mut self,
        spec: &[f32],
        left: WindowShape,
        right: WindowShape,
        seq: WindowSequence,
    ) -> Result<()> {
        if spec.len() != LONG_WINDOW_LEN {
            return Err(Error::FilterbankInvalid);
        }
        imdct_into_f32(spec, &mut self.scratch);
        apply_long_window(&mut self.scratch, left, right, seq);
        Ok(())
    }

    fn short_windowed(
        &mut self,
        spec: &[f32],
        left: WindowShape,
        right: WindowShape,
    ) -> Result<()> {
        if spec.len() != 8 * SHORT_WINDOW_LEN {
            return Err(Error::FilterbankInvalid);
        }
        self.scratch.fill(0.0);
        let start = (N_L - N_S) / 4;
        let hop = N_S / 2;
        let mut x = [0.0f32; N_S];
        for j in 0..8 {
            let coeffs = &spec[j * SHORT_WINDOW_LEN..(j + 1) * SHORT_WINDOW_LEN];
            imdct_into_f32(coeffs, &mut x);
            let this_left = if j == 0 { left } else { right };
            let lh = half(N_S, this_left);
            let rh = half(N_S, right);
            let base = start + j * hop;
            for n in 0..N_S / 2 {
                self.scratch[base + n] += x[n] * lh[n];
            }
            for n in 0..N_S / 2 {
                self.scratch[base + N_S / 2 + n] += x[N_S / 2 + n] * rh[N_S / 2 - 1 - n];
            }
        }
        Ok(())
    }
}

fn apply_long_window(z: &mut [f32], left: WindowShape, right: WindowShape, seq: WindowSequence) {
    let half_l = N_L / 2;
    let lh = half(N_L, left);
    let rh = half(N_L, right);
    let sh_l = half(N_S, left);
    let sh_r = half(N_S, right);
    match seq {
        WindowSequence::OnlyLong => {
            for i in 0..half_l {
                z[i] *= lh[i];
                z[half_l + i] *= rh[half_l - 1 - i];
            }
        }
        WindowSequence::LongStart => {
            for i in 0..half_l {
                z[i] *= lh[i];
            }
            let b = (3 * N_L - N_S) / 4;
            for (m, i) in (0..N_S / 2).enumerate() {
                z[b + m] *= sh_r[N_S / 2 - 1 - i];
            }
            z[b + N_S / 2..].fill(0.0);
        }
        WindowSequence::LongStop => {
            let a = (N_L - N_S) / 4;
            z[..a].fill(0.0);
            for (m, &sv) in sh_l.iter().enumerate() {
                z[a + m] *= sv;
            }
            for i in 0..half_l {
                z[half_l + i] *= rh[half_l - 1 - i];
            }
        }
        WindowSequence::EightShort => {}
    }
}
