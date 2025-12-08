/// Digital Signal Processing (DSP) module.
///
/// This module provides core DSP building blocks and traits for processing
/// streams of complex-valued samples, typically used in software-defined radio
/// and signal analysis applications.
///
/// # Modules
/// - `afc`: Automatic Frequency Control related functionality.
/// - `rds`: Radio Data System processing.
/// - `rotate`: Utilities for rotating complex samples.
///
/// # Traits
/// - [`DspBlock`]: A trait representing a generic DSP processing block that
///   operates on slices of complex samples.
use num_complex::Complex;

pub mod afc;
pub mod rds;
pub mod rotate;

pub trait DspBlock {
    fn process(&mut self, data: &[Complex<f32>]) -> Vec<Complex<f32>>;
}
