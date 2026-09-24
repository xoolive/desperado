//! Streaming station selection and sample-rate conversion for NRSC-5 FM.

use std::f32::consts::PI;

use desperado::dsp::{DspBlock, resampler::ComplexResampler, rotate::Rotate};
use num_complex::Complex;

use crate::protocol::config::{NRSC5_FM_SAMPLE_RATE_HZ, ReceiverConfig};

/// Converts capture-rate I/Q into station-centred NRSC-5 FM-rate I/Q.
pub struct FrontEnd {
    rotate: Rotate,
    resampler: Option<ComplexResampler>,
    input_samples: u64,
    output_samples: u64,
}

impl FrontEnd {
    /// Build a station-selecting front end.
    ///
    /// A station below the capture centre has a negative baseband offset, so
    /// multiplying by `exp(-j 2π offset / sample_rate)` moves it to DC.
    pub fn new(config: &ReceiverConfig) -> Result<Self, String> {
        let angle =
            -2.0 * PI * config.station_offset_hz() as f32 / config.input.sample_rate_hz as f32;
        Ok(Self {
            rotate: Rotate::new(angle),
            resampler: (config.input.sample_rate_hz != NRSC5_FM_SAMPLE_RATE_HZ)
                .then(|| {
                    ComplexResampler::new(config.input.sample_rate_hz, NRSC5_FM_SAMPLE_RATE_HZ)
                })
                .transpose()?,
            input_samples: 0,
            output_samples: 0,
        })
    }

    /// Rotate the selected station to baseband and resample it to 744,188 S/s.
    pub fn process(&mut self, samples: &[Complex<f32>]) -> Vec<Complex<f32>> {
        self.input_samples += samples.len() as u64;
        let rotated = self.rotate.process(samples);
        let mut output = if let Some(resampler) = &mut self.resampler {
            resampler.process(&rotated)
        } else {
            rotated
        };
        // NRSC-5's FM-sideband convention is conjugated relative to the
        // GQRX/Desperado complex-I/Q convention. Keep that conversion here,
        // before PHY timing and FFT processing, rather than scattering signs
        // through carrier and FEC code.
        for sample in &mut output {
            *sample = sample.conj();
        }
        self.output_samples += output.len() as u64;
        output
    }

    /// Number of raw capture samples processed.
    pub fn input_samples(&self) -> u64 {
        self.input_samples
    }

    /// Number of NRSC-5-rate samples produced.
    pub fn output_samples(&self) -> u64 {
        self.output_samples
    }
}
