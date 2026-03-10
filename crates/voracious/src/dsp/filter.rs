//! Digital filters for VOR signal processing

use desperado::dsp::filters::LowPassFir;
use num_complex::Complex;
use rustfft::FftPlanner;

#[derive(Debug, Clone)]
pub struct ButterworthFilter {
    coeffs: Vec<f64>,
    real_state: Vec<f64>,
    imag_state: Vec<f64>,
    real_pos: usize,
    imag_pos: usize,
}

impl ButterworthFilter {
    pub fn lowpass(cutoff: f64, sample_rate: f64, order: usize) -> Self {
        let taps = taps_from_order(order);
        let fir = LowPassFir::new(cutoff as f32, sample_rate as f32, taps);
        let coeffs = fir.coefficients().iter().map(|&c| c as f64).collect();
        Self::from_coeffs(coeffs)
    }

    pub fn bandpass(low: f64, high: f64, sample_rate: f64, order: usize) -> Self {
        let taps = taps_from_order(order);
        let lp_high = LowPassFir::new(high as f32, sample_rate as f32, taps);
        let lp_low = LowPassFir::new(low as f32, sample_rate as f32, taps);

        let coeffs = lp_high
            .coefficients()
            .iter()
            .zip(lp_low.coefficients().iter())
            .map(|(h, l)| (*h - *l) as f64)
            .collect();

        Self::from_coeffs(coeffs)
    }

    fn from_coeffs(coeffs: Vec<f64>) -> Self {
        let len = coeffs.len();
        Self {
            coeffs,
            real_state: vec![0.0; len],
            imag_state: vec![0.0; len],
            real_pos: 0,
            imag_pos: 0,
        }
    }

    pub fn filter(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(fir_step(
                x,
                &self.coeffs,
                &mut self.real_state,
                &mut self.real_pos,
            ));
        }
        output
    }

    pub fn filter_complex(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            let r = fir_step(
                x.re as f64,
                &self.coeffs,
                &mut self.real_state,
                &mut self.real_pos,
            );
            let i = fir_step(
                x.im as f64,
                &self.coeffs,
                &mut self.imag_state,
                &mut self.imag_pos,
            );
            output.push(Complex::new(r as f32, i as f32));
        }
        output
    }
}

fn taps_from_order(order: usize) -> usize {
    let base = (order.max(1) * 16) + 1;
    if base % 2 == 1 { base } else { base + 1 }
}

fn fir_step(x: f64, coeffs: &[f64], state: &mut [f64], pos: &mut usize) -> f64 {
    state[*pos] = x;

    let mut y = 0.0;
    let mut idx = *pos;
    for &c in coeffs {
        y += c * state[idx];
        idx = if idx == 0 { state.len() - 1 } else { idx - 1 };
    }

    *pos += 1;
    if *pos == state.len() {
        *pos = 0;
    }

    y
}

pub fn decimate(input: &[f64], factor: usize) -> Vec<f64> {
    if factor <= 1 {
        return input.to_vec();
    }
    input.iter().step_by(factor).copied().collect()
}

pub fn hilbert_transform(signal: &[f64]) -> Vec<Complex<f64>> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(n);
    let ifft = planner.plan_fft_inverse(n);

    let mut spectrum: Vec<Complex<f64>> = signal
        .iter()
        .map(|&x| Complex::<f64>::new(x, 0.0))
        .collect();

    fft.process(&mut spectrum);

    if n.is_multiple_of(2) {
        for x in spectrum.iter_mut().take(n / 2).skip(1) {
            *x *= 2.0;
        }
        for x in spectrum.iter_mut().skip(n / 2 + 1) {
            *x = Complex::new(0.0, 0.0);
        }
    } else {
        for x in spectrum.iter_mut().take(n.div_ceil(2)).skip(1) {
            *x *= 2.0;
        }
        for x in spectrum.iter_mut().skip(n.div_ceil(2)) {
            *x = Complex::new(0.0, 0.0);
        }
    }

    ifft.process(&mut spectrum);

    let scale = 1.0 / n as f64;
    for x in &mut spectrum {
        *x *= scale;
    }

    spectrum
}

pub fn envelope(signal: &[Complex<f64>]) -> Vec<f64> {
    signal.iter().map(|c| c.norm()).collect()
}
