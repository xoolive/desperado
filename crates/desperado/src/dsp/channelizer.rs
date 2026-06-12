//! Multi-channel complex I/Q channelization utilities.
//!
//! This module provides a simple streaming channelizer that translates one
//! wideband complex I/Q stream into multiple narrowband streams. Each channel is
//! mixed to baseband with an [`Nco`] and then resampled with a
//! [`ComplexResampler`]. It is intentionally a generic SDR building block; users
//! can associate the returned `channel_index` with their own metadata.
//!
//! [`Nco`]: crate::dsp::nco::Nco
//! [`ComplexResampler`]: crate::dsp::resampler::ComplexResampler

use crate::dsp::nco::Nco;
use crate::dsp::resampler::ComplexResampler;
use num_complex::Complex;

/// Narrowband samples produced for one configured channel.
#[derive(Debug, Clone)]
pub struct ChannelizedChunk {
    /// Index of the channel in the configuration slice passed to [`Channelizer::new`].
    pub channel_index: usize,
    /// Narrowband complex samples for this channel.
    pub samples: Vec<Complex<f32>>,
}

/// Description of a channel to extract from a wideband I/Q stream.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelSpec {
    offset_hz: f64,
}

impl ChannelSpec {
    /// Create a channel from an offset, in Hz, relative to the input stream center.
    pub fn offset_hz(offset_hz: impl Into<f64>) -> Self {
        Self {
            offset_hz: offset_hz.into(),
        }
    }

    /// Create a channel from absolute center/channel frequencies, in Hz.
    pub fn absolute_hz(center_hz: impl Into<f64>, channel_hz: impl Into<f64>) -> Self {
        Self::offset_hz(channel_hz.into() - center_hz.into())
    }

    /// Return the channel offset, in Hz, relative to the input stream center.
    pub fn offset(&self) -> f64 {
        self.offset_hz
    }
}

/// Streaming per-channel NCO + resampler channelizer.
pub struct Channelizer {
    channels: Vec<ChannelizerChannel>,
}

struct ChannelizerChannel {
    nco: Option<Nco>,
    resampler: ComplexResampler,
}

impl Channelizer {
    /// Create a channelizer from channel offsets.
    ///
    /// `input_rate_hz` is the wideband stream sample rate. `output_rate_hz` is
    /// the sample rate produced for each narrowband channel.
    pub fn new(
        input_rate_hz: u32,
        output_rate_hz: u32,
        channels: &[ChannelSpec],
    ) -> Result<Self, String> {
        let mut out = Vec::with_capacity(channels.len());
        for channel in channels {
            let offset_hz = channel.offset();
            out.push(ChannelizerChannel {
                nco: (offset_hz.abs() > 1.0).then(|| Nco::new(offset_hz, input_rate_hz as f64)),
                resampler: ComplexResampler::new(input_rate_hz, output_rate_hz)?,
            });
        }
        Ok(Self { channels: out })
    }

    /// Create a channelizer from absolute center/channel frequencies.
    pub fn from_absolute_frequencies(
        center_hz: u32,
        input_rate_hz: u32,
        output_rate_hz: u32,
        channels_hz: &[u32],
    ) -> Result<Self, String> {
        let specs: Vec<_> = channels_hz
            .iter()
            .map(|&channel_hz| ChannelSpec::absolute_hz(center_hz as f64, channel_hz as f64))
            .collect();
        Self::new(input_rate_hz, output_rate_hz, &specs)
    }

    /// Number of configured output channels.
    pub fn len(&self) -> usize {
        self.channels.len()
    }

    /// Whether no output channels are configured.
    pub fn is_empty(&self) -> bool {
        self.channels.is_empty()
    }

    /// Process one wideband chunk and return one narrowband chunk per channel.
    pub fn process(&mut self, input: &[Complex<f32>]) -> Vec<ChannelizedChunk> {
        let mut out = Vec::with_capacity(self.channels.len());
        for (channel_index, channel) in self.channels.iter_mut().enumerate() {
            let mixed: Vec<Complex<f32>> = if let Some(nco) = channel.nco.as_mut() {
                input
                    .iter()
                    .map(|s| {
                        let (re, im) = nco.mix_down_complex(s.re, s.im);
                        nco.step();
                        Complex::new(re, im)
                    })
                    .collect()
            } else {
                input.to_vec()
            };
            out.push(ChannelizedChunk {
                channel_index,
                samples: channel.resampler.process(&mixed),
            });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn absolute_frequency_spec_computes_offset() {
        let spec = ChannelSpec::absolute_hz(100_000_000.0, 100_025_000.0);
        assert_eq!(spec.offset(), 25_000.0);
    }

    #[test]
    fn channelizer_returns_one_chunk_per_channel() {
        let channels = [
            ChannelSpec::offset_hz(0.0),
            ChannelSpec::offset_hz(25_000.0),
        ];
        let mut channelizer = Channelizer::new(1_000_000, 100_000, &channels).unwrap();
        let input = vec![Complex::new(1.0, 0.0); 1000];
        let out = channelizer.process(&input);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].channel_index, 0);
        assert_eq!(out[1].channel_index, 1);
        assert!(!out[0].samples.is_empty());
        assert!(!out[1].samples.is_empty());
    }
}
