//! DAB-specific resampling paths tuned for Airspy and RTL front-ends.
//!
//! This module contains the exact rate-matching logic for converting
//! various SDR front-end sample rates to the DAB native rate (2.048 MHz).
//! The paths are carefully chosen to preserve OFDM timing integrity.

use desperado::dsp::DspBlock;
use desperado::dsp::decimator::Decimator;
use desperado::dsp::resampler::ComplexResampler;
use num_complex::Complex;

use dabradio::constants;

const WELLE_INTERMEDIATE_RATE: u32 = 4_096_000;

/// Number of taps for the wideband channel-selection FIR. Enough to reject
/// adjacent multiplexes (~1.7 MHz away) while keeping the ±0.768 MHz DAB band
/// in a flat passband; evaluated at the decimated rate so the cost is modest.
const SELECTION_LPF_TAPS: usize = 255;

pub(crate) struct DabResampler {
    path: Path,
}

enum Path {
    Passthrough,
    Half(AveragingDecimator2),
    Direct(ComplexResampler),
    ResampleThenHalf {
        resampler: ComplexResampler,
        decimator: AveragingDecimator2,
    },
    HalfThenResampleThenHalf {
        pre_decimator: AveragingDecimator2,
        resampler: ComplexResampler,
        post_decimator: AveragingDecimator2,
    },
    /// Fallback for arbitrary/high wideband input rates (e.g. 16 MS/s GQRX
    /// captures, or a high-rate HACKRF live source) that don't match one of the
    /// tuned SDR paths above.
    ///
    /// A single **decimating** channel-selection FIR (`Decimator`) rejects
    /// adjacent multiplexes and any out-of-band energy above the decimated
    /// output Nyquist *before* decimation (so nothing aliases), while being
    /// evaluated at the decimated output rate (1/D as often) — far cheaper than
    /// a full-rate selection LPF. A `ComplexResampler` then fractional-rate-
    /// matches to the DAB native rate. The cutoff sits just below the decimated
    /// Nyquist so the ±0.768 MHz DAB band is in the flat passband (a weak
    /// multiplex needs this), with DC removal (applied upstream) handling the
    /// capture-center spike. Kept as a catch-all so the tuned paths above remain
    /// untouched (no regression for Airspy/RTL front-ends).
    SelectAndResample {
        decimator: Decimator,
        resampler: ComplexResampler,
    },
}

impl DabResampler {
    /// Create a DAB resampler for the given SDR input sample rate.
    ///
    /// Uses hardcoded rate ranges tuned for known SDR front-ends:
    /// - 2.048 MHz: passthrough (RTL-SDR native DAB rate)
    /// - ~4.096 MHz: 2:1 averaging (Airspy)
    /// - ~2.4 MHz: direct polyphase resample (RTL-SDR)
    /// - ~3 MHz: resample to 4.096 MHz then halve
    /// - ~6 MHz: halve, resample to 4.096 MHz, halve (Airspy HF+)
    pub(crate) fn new(input_rate_hz: u32) -> Result<Self, String> {
        if input_rate_hz == 0 {
            return Err("Input sample rate must be non-zero".to_string());
        }

        let path = if input_rate_hz == constants::SAMPLE_RATE {
            Path::Passthrough
        } else if (4_000_000..=4_200_000).contains(&input_rate_hz) {
            Path::Half(AveragingDecimator2::new())
        } else if (5_800_000..=6_200_000).contains(&input_rate_hz) {
            let half_rate = input_rate_hz / 2;
            Path::HalfThenResampleThenHalf {
                pre_decimator: AveragingDecimator2::new(),
                resampler: ComplexResampler::new(half_rate, WELLE_INTERMEDIATE_RATE)?,
                post_decimator: AveragingDecimator2::new(),
            }
        } else if (2_900_000..=3_100_000).contains(&input_rate_hz) {
            Path::ResampleThenHalf {
                resampler: ComplexResampler::new(input_rate_hz, WELLE_INTERMEDIATE_RATE)?,
                decimator: AveragingDecimator2::new(),
            }
        } else if (2_300_000..=2_500_000).contains(&input_rate_hz) {
            Path::Direct(ComplexResampler::new(
                input_rate_hz,
                constants::SAMPLE_RATE,
            )?)
        } else {
            // Arbitrary / wideband rate (e.g. 16 MS/s GQRX file, high-rate
            // HACKRF). A single decimating selection FIR both anti-aliases
            // (rejecting everything above the decimated output Nyquist *before*
            // decimation, so strong out-of-band multiplexes can't alias in) and
            // selects the wanted channel, evaluated only every Dth sample —
            // far cheaper than a full-rate LPF. Pick the largest integer D that
            // keeps the decimated rate >= ~2.2 MS/s. The cutoff sits just below
            // the decimated Nyquist so the ±0.768 MHz DAB band is in the flat
            // passband; ComplexResampler finishes the fractional rate-match.
            let min_decimated_rate = 2_200_000;
            if let Some(d) = (2..=64u32)
                .rev()
                .find(|&d| input_rate_hz / d >= min_decimated_rate)
            {
                let decimated_rate = input_rate_hz / d;
                // Selection cutoff: as high as possible while staying just below the
                // decimated output Nyquist (0.5/D) — maximizes flat passband for the
                // DAB band (0.768 MHz) and still rejects adjacent multiplexes.
                let cutoff_norm = (0.48 / d as f32).min(0.48);
                Path::SelectAndResample {
                    decimator: Decimator::with_params(d as usize, SELECTION_LPF_TAPS, cutoff_norm),
                    resampler: ComplexResampler::new(decimated_rate, constants::SAMPLE_RATE)?,
                }
            } else {
                // Low arbitrary rates cannot be decimated while retaining the
                // complete DAB channel. Resample directly rather than silently
                // halving them and filtering away part of the OFDM spectrum.
                Path::Direct(ComplexResampler::new(
                    input_rate_hz,
                    constants::SAMPLE_RATE,
                )?)
            }
        };

        Ok(Self { path })
    }

    pub(crate) fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        match &mut self.path {
            Path::Passthrough => input.to_vec(),
            Path::Half(decimator) => decimator.process(input),
            Path::Direct(resampler) => resampler.process(input),
            Path::ResampleThenHalf {
                resampler,
                decimator,
            } => {
                let resampled = resampler.process(input);
                decimator.process(&resampled)
            }
            Path::HalfThenResampleThenHalf {
                pre_decimator,
                resampler,
                post_decimator,
            } => {
                let half = pre_decimator.process(input);
                let intermediate = resampler.process(&half);
                post_decimator.process(&intermediate)
            }
            Path::SelectAndResample {
                decimator,
                resampler,
            } => {
                // Decimating selection FIR (anti-alias + adjacent rejection
                // before decimation), then fractional rate-match to 2.048 MS/s.
                let decimated = decimator.process(input);
                resampler.process(&decimated)
            }
        }
    }
}

/// Stateful 2:1 averaging decimator for complex samples.
struct AveragingDecimator2 {
    pending: Option<Complex<f32>>,
}

impl AveragingDecimator2 {
    fn new() -> Self {
        Self { pending: None }
    }

    fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        if input.is_empty() {
            return Vec::new();
        }

        let mut output = Vec::with_capacity(input.len().div_ceil(2));
        let mut idx = 0usize;

        if let Some(prev) = self.pending.take() {
            let pair_avg = (prev + input[0]) * 0.5;
            output.push(pair_avg);
            idx = 1;
        }

        while idx + 1 < input.len() {
            let pair_avg = (input[idx] + input[idx + 1]) * 0.5;
            output.push(pair_avg);
            idx += 2;
        }

        if idx < input.len() {
            self.pending = Some(input[idx]);
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    /// Generate `n` complex samples of a tone at `freq_hz` (amplitude 1) at the
    /// given sample rate, starting at phase 0.
    fn tone(freq_hz: f32, sample_rate: f32, n: usize) -> Vec<Complex<f32>> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq_hz * i as f32 / sample_rate;
                Complex::new(phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Mean per-sample power of a complex buffer.
    fn mean_power(samples: &[Complex<f32>]) -> f32 {
        if samples.is_empty() {
            return 0.0;
        }
        samples.iter().map(|s| s.norm_sqr()).sum::<f32>() / samples.len() as f32
    }

    #[test]
    fn wideband_16msps_is_supported() {
        // Previously rejected; now routed to the SelectAndResample fallback.
        assert!(DabResampler::new(16_000_000).is_ok());
        assert!(DabResampler::new(10_000_000).is_ok());
        assert!(DabResampler::new(20_000_000).is_ok());
    }

    #[test]
    fn select_and_resample_output_rate_is_dab_native() {
        // 16 MS/s -> 2.048 MS/s is a 7.8125:1 ratio. Feed one frame's worth of
        // input (at the input rate) and check the output count matches the DAB
        // native frame size within the resampler's transient margin.
        let input_rate = 16_000_000u32;
        let ratio = input_rate as f64 / constants::SAMPLE_RATE as f64; // 7.8125
        let n_in = (constants::T_F as f64 * ratio).round() as usize;
        let input = tone(0.0, input_rate as f32, n_in);
        let mut r = DabResampler::new(input_rate).unwrap();
        let out = r.process(&input);
        // Expect roughly T_F samples (allow a few % for polyphase transients).
        assert!(
            out.len() > constants::T_F * 90 / 100 && out.len() < constants::T_F * 110 / 100,
            "expected ~{} output samples, got {}",
            constants::T_F,
            out.len()
        );
    }

    #[test]
    fn select_and_resample_rejects_adjacent_multiplex() {
        // A neighbour multiplex downconverted to ~1.7 MHz must be attenuated by
        // the selection LPF (0.9 MHz cutoff) before decimation, while an
        // in-band DC tone passes with near-unity gain.
        let input_rate = 16_000_000u32;
        let n = 1 << 16;
        let mut r = DabResampler::new(input_rate).unwrap();

        let in_band = tone(0.0, input_rate as f32, n);
        let adjacent = tone(1_700_000.0, input_rate as f32, n);

        let out_in = r.process(&in_band);
        // Reset so the adjacent run isn't polluted by in-band history.
        r = DabResampler::new(input_rate).unwrap();
        let out_adj = r.process(&adjacent);

        let p_in = mean_power(&out_in[in_band.len() / 20..]);
        let p_adj = mean_power(&out_adj[adjacent.len() / 20..]);

        assert!(
            p_in > 0.8,
            "in-band DC tone should pass near unity, power={:.4}",
            p_in
        );
        assert!(
            p_adj < p_in * 0.01,
            "adjacent 1.7 MHz tone should be attenuated >20 dB (p_adj={:.4} vs p_in={:.4})",
            p_adj,
            p_in
        );
    }

    #[test]
    fn tuned_sdr_paths_unchanged() {
        // The fallback must not disturb the existing front-end-specific paths.
        assert!(DabResampler::new(constants::SAMPLE_RATE).is_ok()); // Passthrough
        assert!(DabResampler::new(4_096_000).is_ok()); // Half
        assert!(DabResampler::new(2_400_000).is_ok()); // Direct
        assert!(DabResampler::new(6_000_000).is_ok()); // HalfThenResampleThenHalf
    }

    #[test]
    fn low_arbitrary_rate_is_not_destructively_decimated() {
        let resampler = DabResampler::new(1_800_000).unwrap();
        assert!(matches!(resampler.path, Path::Direct(_)));
    }
}
