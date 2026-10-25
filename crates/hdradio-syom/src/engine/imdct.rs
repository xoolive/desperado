//! IMDCT — ISO/IEC 14496-3 §4.6.11.3.1, plus an N/4 IFFT fast path.
//!
//! Pre/post twiddles live here. The fast path is proven against the naive sum
//! in `imdct_tests.rs`.

use std::sync::LazyLock;

#[derive(Clone, Copy)]
struct C {
    re: f32,
    im: f32,
}

impl C {
    fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    #[inline(always)]
    fn mul(self, o: Self) -> Self {
        Self {
            re: self.re * o.re - self.im * o.im,
            im: self.re * o.im + self.im * o.re,
        }
    }
}

/// Bit-reversal permutation for an `n`-point radix-2 FFT (shared with the
/// forward MDCT).
pub(crate) fn bitrev_table(n: usize) -> Vec<u16> {
    let mut t = vec![0u16; n];
    let mut j = 0usize;
    for slot in t.iter_mut() {
        *slot = j as u16;
        let mut bit = n >> 1;
        while bit != 0 && j >= bit {
            j -= bit;
            bit >>= 1;
        }
        j += bit;
    }
    t
}

/// Twiddles `e^{+i·2πk/len}` per stage (shared with the forward MDCT, which
/// negates the imaginary half for the forward FFT).
pub(crate) fn twiddle_table(n: usize) -> (Vec<f32>, Vec<f32>) {
    let mut re = Vec::with_capacity(n);
    let mut im = Vec::with_capacity(n);
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = 2.0 * std::f32::consts::PI / len as f32;
        for k in 0..half {
            let a = ang * k as f32;
            re.push(a.cos());
            im.push(a.sin());
        }
        len *= 2;
    }
    (re, im)
}

/// Unnormalized inverse radix-2 FFT, SoA + precomputed twiddles. With the
/// twiddle imaginary parts negated this is the forward FFT (the MDCT uses
/// it that way).
pub(crate) fn ifft_soa(
    re: &mut [f32],
    im: &mut [f32],
    bitrev: &[u16],
    tw_re: &[f32],
    tw_im: &[f32],
) {
    let n = re.len();
    for (i, &rev) in bitrev.iter().enumerate() {
        let j = rev as usize;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2usize;
    let mut off = 0usize;
    while len <= n {
        let half = len / 2;
        for i in (0..n).step_by(len) {
            let mut k = 0usize;
            #[cfg(target_arch = "aarch64")]
            {
                // SAFETY: NEON is baseline on aarch64; slices are in-bounds by k+3 < half.
                while k + 4 <= half {
                    unsafe {
                        ifft_butterfly4(re, im, i, k, half, off, tw_re, tw_im);
                    }
                    k += 4;
                }
            }
            while k < half {
                let wr = tw_re[off + k];
                let wi = tw_im[off + k];
                let j = i + k + half;
                let tr = wr * re[j] - wi * im[j];
                let ti = wr * im[j] + wi * re[j];
                let ur = re[i + k];
                let ui = im[i + k];
                re[j] = ur - tr;
                im[j] = ui - ti;
                re[i + k] = ur + tr;
                im[i + k] = ui + ti;
                k += 1;
            }
        }
        off += half;
        len *= 2;
    }
}

#[cfg(target_arch = "aarch64")]
#[target_feature(enable = "neon")]
#[allow(clippy::too_many_arguments)]
unsafe fn ifft_butterfly4(
    re: &mut [f32],
    im: &mut [f32],
    i: usize,
    k: usize,
    half: usize,
    off: usize,
    tw_re: &[f32],
    tw_im: &[f32],
) {
    use std::arch::aarch64::*;
    let j = i + k + half;
    unsafe {
        // SAFETY: NEON (baseline aarch64); caller ensures k+3 < half and
        // i+k+3, j+3 are in-range for `re`/`im`/`tw_*`.
        let wr = vld1q_f32(tw_re.as_ptr().add(off + k));
        let wi = vld1q_f32(tw_im.as_ptr().add(off + k));
        let rj = vld1q_f32(re.as_ptr().add(j));
        let ij = vld1q_f32(im.as_ptr().add(j));
        // Separate multiply + add/sub, NOT vfmaq/vfmsq: fused rounding would
        // differ from the scalar path by 1 ulp per butterfly, making the
        // transform (and the encoder's MDCT output) platform-dependent.
        let tr = vsubq_f32(vmulq_f32(wr, rj), vmulq_f32(wi, ij));
        let ti = vaddq_f32(vmulq_f32(wr, ij), vmulq_f32(wi, rj));
        let ur = vld1q_f32(re.as_ptr().add(i + k));
        let ui = vld1q_f32(im.as_ptr().add(i + k));
        vst1q_f32(re.as_mut_ptr().add(j), vsubq_f32(ur, tr));
        vst1q_f32(im.as_mut_ptr().add(j), vsubq_f32(ui, ti));
        vst1q_f32(re.as_mut_ptr().add(i + k), vaddq_f32(ur, tr));
        vst1q_f32(im.as_mut_ptr().add(i + k), vaddq_f32(ui, ti));
    }
}

/// Naive §4.6.11.3.1 sum.
///
/// `x[n] = (2/N) · Σ_k spec[k] · cos((2π/N)·(n + n0)·(k + 1/2))`
/// with `n0 = (N/2 + 1)/2`, `N = n_transform`.
#[cfg(test)]
#[must_use]
pub fn imdct_naive(spec: &[f64], n_transform: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; n_transform];
    imdct_naive_into(spec, &mut out);
    out
}

fn imdct_naive_into(spec: &[f64], out: &mut [f64]) {
    let n = out.len();
    if n == 0 {
        return;
    }
    let n0 = (n / 2 + 1) as f64 / 2.0;
    let scale = 2.0 / n as f64;
    let step = 2.0 * std::f64::consts::PI / n as f64;
    for (n_i, slot) in out.iter_mut().enumerate() {
        let np = n_i as f64 + n0;
        let mut acc = 0.0f64;
        for (k, &c) in spec.iter().enumerate() {
            acc += c * (step * np * (k as f64 + 0.5)).cos();
        }
        *slot = scale * acc;
    }
}

fn naive_f32(spec: &[f32], out: &mut [f32]) {
    let spec64: Vec<f64> = spec.iter().copied().map(f64::from).collect();
    let mut tmp = vec![0.0f64; out.len()];
    imdct_naive_into(&spec64, &mut tmp);
    for (d, s) in out.iter_mut().zip(tmp) {
        *d = s as f32;
    }
}

/// Fast IMDCT into `out` (f32 time domain).
pub fn imdct_into_f32(spec: &[f32], out: &mut [f32]) {
    let n = out.len();
    if spec.len() != n / 2 {
        naive_f32(spec, out);
        return;
    }
    match n {
        256 => PLAN_256.apply_into(spec, out),
        2048 => PLAN_2048.apply_into(spec, out),
        _ => naive_f32(spec, out),
    }
}

/// Fast IMDCT into f64 `out` (tests / legacy).
#[cfg(test)]
pub fn imdct_into(spec: &[f64], out: &mut [f64]) {
    use std::cell::RefCell;
    thread_local! {
        static SPEC: RefCell<Vec<f32>> = const { RefCell::new(Vec::new()) };
        static TMP: RefCell<Vec<f32>> = const { RefCell::new(Vec::new()) };
    }
    SPEC.with(|sc| {
        TMP.with(|tc| {
            let mut s = sc.borrow_mut();
            let mut t = tc.borrow_mut();
            s.clear();
            s.extend(spec.iter().map(|&x| x as f32));
            t.resize(out.len(), 0.0);
            imdct_into_f32(&s, &mut t);
            for (d, &v) in out.iter_mut().zip(t.iter()) {
                *d = f64::from(v);
            }
        });
    });
}

/// Allocate and run [`imdct_into`].
#[cfg(test)]
#[must_use]
pub fn imdct(spec: &[f64], n_transform: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; n_transform];
    imdct_into(spec, &mut out);
    out
}

struct Plan {
    n: usize,
    pre: Vec<C>,
    post: Vec<C>,
    bitrev: Vec<u16>,
    tw_re: Vec<f32>,
    tw_im: Vec<f32>,
}

static PLAN_256: LazyLock<Plan> = LazyLock::new(|| Plan::new(256));
static PLAN_2048: LazyLock<Plan> = LazyLock::new(|| Plan::new(2048));

impl Plan {
    fn new(n: usize) -> Self {
        let n4 = n / 4;
        let two_pi_n = 2.0 * std::f32::consts::PI / n as f32;
        let mut pre = Vec::with_capacity(n4);
        let mut post = Vec::with_capacity(n4);
        for k in 0..n4 {
            let a = two_pi_n * (k as f32 + 0.125);
            let c = C::new(a.cos(), a.sin());
            pre.push(c);
            post.push(c);
        }
        let (tw_re, tw_im) = twiddle_table(n4);
        Self {
            n,
            pre,
            post,
            bitrev: bitrev_table(n4),
            tw_re,
            tw_im,
        }
    }

    fn apply_into(&self, spec: &[f32], out: &mut [f32]) {
        match self.n {
            2048 => {
                let mut re = [0.0f32; 512];
                let mut im = [0.0f32; 512];
                self.apply_soa(spec, out, &mut re, &mut im);
            }
            256 => {
                let mut re = [0.0f32; 64];
                let mut im = [0.0f32; 64];
                self.apply_soa(spec, out, &mut re, &mut im);
            }
            _ => naive_f32(spec, out),
        }
    }

    fn apply_soa(&self, spec: &[f32], out: &mut [f32], re: &mut [f32], im: &mut [f32]) {
        let n = self.n;
        let n2 = n / 2;
        let n4 = n / 4;
        for k in 0..n4 {
            let z = C::new(spec[n2 - 2 * k - 1], spec[2 * k]).mul(self.pre[k]);
            re[k] = z.re;
            im[k] = z.im;
        }
        ifft_soa(re, im, &self.bitrev, &self.tw_re, &self.tw_im);
        for k in 0..n4 {
            let z = C::new(re[k], im[k]).mul(self.post[k]);
            re[k] = z.re;
            im[k] = z.im;
        }
        let scale = 2.0 / n as f32;
        let half = n4 / 2;
        for p in 0..half {
            let i = p * 2;
            out[i] = scale * im[half + p];
            out[n2 + i] = scale * re[half + p];
        }
        for p in 0..half {
            let i = p * 2 + 1;
            out[i] = -scale * re[half - 1 - p];
            out[n2 + i] = -scale * im[half - 1 - p];
        }
        for n_i in 0..n4 {
            out[n2 - 1 - n_i] = -out[n_i];
            out[n - 1 - n_i] = out[n2 + n_i];
        }
    }
}

#[cfg(test)]
#[path = "imdct_tests.rs"]
mod imdct_tests;
