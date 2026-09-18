//! Stereo PCM16 WAV I/Q input.
//!
//! WAV-IQ is distinct from decoded-audio WAV: the left channel is the I
//! component and the right channel is the Q component of each complex sample.

use std::path::Path;

use num_complex::Complex;

use crate::error;

/// A streaming reader for stereo signed-PCM16 WAV I/Q captures.
pub struct WavIqReader {
    reader: hound::WavReader<std::io::BufReader<std::fs::File>>,
    sample_rate: u32,
    chunk_size: usize,
}

impl WavIqReader {
    /// Open a WAV-IQ capture.
    ///
    /// The file must be stereo signed PCM16, with I in the left channel and Q
    /// in the right channel. The WAV header supplies the sample rate.
    pub fn from_file<P: AsRef<Path>>(path: P, chunk_size: usize) -> error::Result<Self> {
        let reader = hound::WavReader::open(path)
            .map_err(|error| error::Error::format(format!("invalid WAV-IQ input: {error}")))?;
        let spec = reader.spec();
        if spec.channels != 2
            || spec.sample_format != hound::SampleFormat::Int
            || spec.bits_per_sample != 16
        {
            return Err(error::Error::format(format!(
                "WAV-IQ input must be stereo signed PCM16; got {} channel(s), {:?}, {} bits",
                spec.channels, spec.sample_format, spec.bits_per_sample
            )));
        }

        Ok(Self {
            reader,
            sample_rate: spec.sample_rate,
            chunk_size,
        })
    }

    /// Sample rate declared by the WAV header.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }
}

impl Iterator for WavIqReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut samples = Vec::with_capacity(self.chunk_size);
        let mut pcm = self.reader.samples::<i16>();

        for _ in 0..self.chunk_size {
            let i = match pcm.next() {
                Some(Ok(sample)) => sample,
                Some(Err(error)) => return Some(Err(error::Error::format(error.to_string()))),
                None => break,
            };
            let q = match pcm.next() {
                Some(Ok(sample)) => sample,
                Some(Err(error)) => return Some(Err(error::Error::format(error.to_string()))),
                None => {
                    return Some(Err(error::Error::format(
                        "WAV-IQ data ended between I and Q channel samples",
                    )));
                }
            };
            samples.push(Complex::new(i as f32 / 32768.0, q as f32 / 32768.0));
        }

        (!samples.is_empty()).then_some(Ok(samples))
    }
}
