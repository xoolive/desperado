/// A DSP block that applies a complex rotation to a sequence of samples.
///
/// The `Rotate` struct maintains an internal complex rotation state (`rot`)
/// and a multiplier (`mult`) that represents the rotation per sample, determined
/// by the given angle in radians. Each call to `process` rotates the input
/// complex samples by the current rotation, updating the rotation for each sample.
///
/// # Example
/// ```
/// use num_complex::Complex;
/// use desperado::dsp::Rotate;
///
/// let mut rotator = Rotate::new(std::f32::consts::FRAC_PI_2); // 90 degrees per sample
/// let input = vec![Complex::new(1.0, 0.0); 4];
/// let output = rotator.process(&input);
/// ```
use num_complex::Complex;

use crate::dsp::DspBlock;

pub struct Rotate {
    /// Current complex rotation factor
    rot: Complex<f32>,
    /// Per-sample rotation multiplier
    mult: Complex<f32>,
}

impl Rotate {
    /// Create a new Rotate DSP block with the specified rotation angle in radians
    pub fn new(angle: f32) -> Self {
        Self {
            rot: Complex::new(1.0, 0.0),
            mult: Complex::new(angle.cos(), angle.sin()),
        }
    }
}

impl DspBlock for Rotate {
    /// Process a slice of complex samples, applying the rotation
    fn process(&mut self, data: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let mut out = Vec::with_capacity(data.len());
        for &s in data {
            out.push(s * self.rot);
            self.rot *= self.mult;
        }
        if self.rot.norm() > 0.0 {
            self.rot /= self.rot.norm();
        }
        out
    }
}
