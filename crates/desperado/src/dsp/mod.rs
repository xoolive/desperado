/// Digital Signal Processing (DSP) module.
///
/// This module provides core DSP building blocks and traits for processing
/// streams of complex-valued and real-valued samples, commonly used in
/// software-defined radio (SDR) and digital signal analysis applications.
///
/// # Overview
///
/// The DSP module is organized into specialized submodules, each providing
/// specific signal processing functionality. These modules can be combined
/// to create complete signal processing pipelines for applications like
/// FM demodulation, frequency analysis, and real-time audio streaming.
///
/// ## DSP Pipeline Example
///
/// A typical FM radio demodulation pipeline might look like:
///
/// ```text
/// I/Q Samples → Rotate → Decimator → AFC → Phase Extractor → LowPass Filter → Deemphasis → Audio
///                  ↓         ↓          ↓          ↓               ↓              ↓
///              Freq Shift  Downsample  Freq Corr  FM Demod      Audio Filter   De-emphasis
/// ```
///
/// # Modules
///
/// ## Signal Manipulation
/// - [`rotate`]: Complex rotation for frequency shifting and offset correction
/// - [`decimator`]: Sample rate reduction with anti-aliasing filtering
/// - [`filters`]: Digital filter implementations (FIR low-pass, etc.)
///
/// ## Carrier and Timing Recovery
/// - [`nco`]: Numerically Controlled Oscillator with Phase-Locked Loop (PLL)
/// - [`symsync`]: Polyphase Symbol Synchronizer for timing recovery
/// - [`agc`]: Automatic Gain Control for amplitude normalization
///
/// ## Advanced Features
/// - [`afc`]: Automatic Frequency Control for frequency offset correction
///
/// Note: FM demodulation (phase extraction, de-emphasis), RDS decoding,
/// and adaptive resampling have been moved to the `fmradio` crate for better modularity.
///
/// # Traits
///
/// - [`DspBlock`]: A trait representing a generic DSP processing block that
///   operates on slices of complex samples and produces complex output.
///
/// # Feature Flags
///
/// None currently. Adaptive resampling has been moved to the `fmradio` crate.
///
/// # Examples
///
/// ## Basic Frequency Shifting
///
/// ```
/// use desperado::dsp::{DspBlock, rotate::Rotate};
/// use num_complex::Complex;
/// use std::f32::consts::PI;
///
/// // Shift frequency by 200 kHz at 2 MHz sample rate
/// let sample_rate = 2_000_000.0;
/// let freq_offset = 200_000.0;
/// let angle = -2.0 * PI * freq_offset / sample_rate;
///
/// let mut rotator = Rotate::new(angle);
/// let samples = vec![Complex::new(1.0, 0.0); 100];
/// let shifted = rotator.process(&samples);
/// ```
///
/// # Performance Considerations
///
/// - **Decimation**: Use decimation early in the pipeline to reduce computational load
/// - **Filter Length**: Longer FIR filters provide better frequency response but cost more CPU
/// - **Batch Processing**: Process larger chunks of samples for better throughput
/// - **Feature Flags**: Only enable features you need to minimize dependencies
///
/// # Thread Safety
///
/// DSP blocks maintain internal state and are **not** thread-safe. Each thread should
/// have its own instance of DSP blocks. For parallel processing, create separate instances
/// per thread or use synchronization primitives.
use num_complex::Complex;

pub mod afc;
pub mod agc;
pub mod decimator;
pub mod filters;
pub mod nco;
pub mod rotate;
pub mod symsync;

/// Trait for DSP blocks that process complex-valued signals.
///
/// This trait represents a generic DSP processing block that takes a slice
/// of complex samples and produces a vector of complex samples. DSP blocks
/// typically maintain internal state between calls to `process()`.
///
/// # Examples
///
/// ```
/// use desperado::dsp::{DspBlock, rotate::Rotate};
/// use num_complex::Complex;
///
/// let mut rotator = Rotate::new(0.1);
/// let input = vec![Complex::new(1.0, 0.0); 10];
/// let output = rotator.process(&input);
/// assert_eq!(output.len(), 10);
/// ```
pub trait DspBlock {
    /// Process a block of complex samples.
    ///
    /// Takes a slice of input samples and returns a vector of processed samples.
    /// The output length may differ from the input length depending on the
    /// operation (e.g., decimation reduces the length).
    ///
    /// # Arguments
    ///
    /// * `data` - Input complex samples
    ///
    /// # Returns
    ///
    /// A vector of processed complex samples
    fn process(&mut self, data: &[Complex<f32>]) -> Vec<Complex<f32>>;
}
