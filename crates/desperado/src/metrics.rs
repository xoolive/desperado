//! RF signal quality metrics computed from IQ samples.
//!
//! This module provides tools for monitoring and evaluating the quality of
//! received RF signals in real time. It computes metrics such as signal level
//! (dBFS), clipping ratio, DC offset, and I/Q imbalance from raw IQ sample
//! streams.
//!
//! The main entry point is [`RfMetricsCalculator`], which accepts successive
//! chunks of IQ samples and emits [`RfMetrics`] snapshots at a configurable
//! rate, using a sliding window for smoothing.

use num_complex::Complex;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

const DEFAULT_CLIP_THRESHOLD: f32 = 0.98;
const DEFAULT_WINDOW_SECONDS: f64 = 3.0;
const DEFAULT_EMIT_RATE_HZ: f64 = 2.0;
const EPSILON: f64 = 1e-12;

/// Compute chunk-level IQ RMS level in dBFS.
///
/// This is equivalent to `10 * log10(mean(I^2 + Q^2))` and also to
/// `20 * log10(rms_amplitude)`.
pub fn iq_level_dbfs(samples: &[Complex<f32>]) -> f64 {
    if samples.is_empty() {
        return -120.0;
    }
    let avg_power = samples
        .iter()
        .map(|s| f64::from(s.re) * f64::from(s.re) + f64::from(s.im) * f64::from(s.im))
        .sum::<f64>()
        / samples.len() as f64;
    10.0 * avg_power.max(EPSILON).log10()
}

/// RF quality metrics computed from IQ samples.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RfMetrics {
    /// Fraction of samples where either I or Q component is clipped.
    pub clipping_ratio: f64,
    /// Average power expressed in dBFS.
    pub noise_floor_db: f64,
    /// Kurtosis of signal amplitude (non-excess kurtosis).
    pub kurtosis: f64,
    /// Timestamp associated with the metric snapshot.
    pub timestamp: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct MetricsBucket {
    sample_count: usize,
    clipped_count: usize,
    sum_power: f64,
    sum_amp1: f64,
    sum_amp2: f64,
    sum_amp3: f64,
    sum_amp4: f64,
}

impl std::ops::AddAssign for MetricsBucket {
    fn add_assign(&mut self, rhs: Self) {
        self.sample_count += rhs.sample_count;
        self.clipped_count += rhs.clipped_count;
        self.sum_power += rhs.sum_power;
        self.sum_amp1 += rhs.sum_amp1;
        self.sum_amp2 += rhs.sum_amp2;
        self.sum_amp3 += rhs.sum_amp3;
        self.sum_amp4 += rhs.sum_amp4;
    }
}

impl std::ops::SubAssign for MetricsBucket {
    fn sub_assign(&mut self, rhs: Self) {
        self.sample_count -= rhs.sample_count;
        self.clipped_count -= rhs.clipped_count;
        self.sum_power -= rhs.sum_power;
        self.sum_amp1 -= rhs.sum_amp1;
        self.sum_amp2 -= rhs.sum_amp2;
        self.sum_amp3 -= rhs.sum_amp3;
        self.sum_amp4 -= rhs.sum_amp4;
    }
}

/// Streaming calculator for RF metrics over a rolling time window.
#[derive(Debug, Clone)]
pub struct RfMetricsCalculator {
    clip_threshold: f32,
    window_samples: usize,
    emit_every_samples: usize,
    samples_since_emit: usize,
    buckets: VecDeque<MetricsBucket>,
    totals: MetricsBucket,
}

impl RfMetricsCalculator {
    /// Create a calculator with default window (3s) and emit rate (2 Hz).
    pub fn new(sample_rate_hz: f64) -> Self {
        Self::with_config(
            sample_rate_hz,
            DEFAULT_WINDOW_SECONDS,
            DEFAULT_EMIT_RATE_HZ,
            DEFAULT_CLIP_THRESHOLD,
        )
    }

    /// Create a calculator with custom window and emission rates.
    pub fn with_config(
        sample_rate_hz: f64,
        window_seconds: f64,
        emit_rate_hz: f64,
        clip_threshold: f32,
    ) -> Self {
        let window_samples = (sample_rate_hz * window_seconds).max(1.0).round() as usize;
        let emit_every_samples = (sample_rate_hz / emit_rate_hz).max(1.0).round() as usize;

        Self {
            clip_threshold,
            window_samples,
            emit_every_samples,
            samples_since_emit: 0,
            buckets: VecDeque::new(),
            totals: MetricsBucket::default(),
        }
    }

    /// Push a chunk of IQ samples.
    ///
    /// Returns `Some(RfMetrics)` when it's time to emit a new snapshot,
    /// otherwise returns `None`.
    pub fn push_chunk(&mut self, samples: &[Complex<f32>], timestamp: f64) -> Option<RfMetrics> {
        if samples.is_empty() {
            return None;
        }

        let mut bucket = MetricsBucket {
            sample_count: samples.len(),
            ..MetricsBucket::default()
        };

        for sample in samples {
            let i = sample.re;
            let q = sample.im;

            if i.abs() >= self.clip_threshold || q.abs() >= self.clip_threshold {
                bucket.clipped_count += 1;
            }

            let power = f64::from(i) * f64::from(i) + f64::from(q) * f64::from(q);
            let amp = power.sqrt();

            bucket.sum_power += power;
            bucket.sum_amp1 += amp;
            bucket.sum_amp2 += amp * amp;
            bucket.sum_amp3 += amp * amp * amp;
            bucket.sum_amp4 += amp * amp * amp * amp;
        }

        self.samples_since_emit += bucket.sample_count;
        self.buckets.push_back(bucket);
        self.totals += bucket;

        while self.totals.sample_count > self.window_samples {
            if let Some(oldest) = self.buckets.pop_front() {
                self.totals -= oldest;
            }
        }

        if self.samples_since_emit >= self.emit_every_samples {
            self.samples_since_emit = 0;
            return Some(self.snapshot(timestamp));
        }

        None
    }

    /// Compute a metrics snapshot from the current rolling window.
    pub fn snapshot(&self, timestamp: f64) -> RfMetrics {
        let n = self.totals.sample_count as f64;

        if self.totals.sample_count == 0 {
            return RfMetrics {
                clipping_ratio: 0.0,
                noise_floor_db: f64::NEG_INFINITY,
                kurtosis: 0.0,
                timestamp,
            };
        }

        let mean_amp = self.totals.sum_amp1 / n;
        let m2 = (self.totals.sum_amp2 / n) - mean_amp * mean_amp;
        let m4 = (self.totals.sum_amp4 / n) - (4.0 * mean_amp * self.totals.sum_amp3 / n)
            + (6.0 * mean_amp * mean_amp * self.totals.sum_amp2 / n)
            - (3.0 * mean_amp * mean_amp * mean_amp * mean_amp);

        let kurtosis = if m2 > EPSILON {
            (m4 / (m2 * m2)).max(0.0)
        } else {
            0.0
        };

        let avg_power = self.totals.sum_power / n;
        let noise_floor_db = 10.0 * avg_power.max(EPSILON).log10();

        RfMetrics {
            clipping_ratio: self.totals.clipped_count as f64 / n,
            noise_floor_db,
            kurtosis,
            timestamp,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iq_level_dbfs_empty_is_floor() {
        let s: Vec<Complex<f32>> = Vec::new();
        assert_eq!(iq_level_dbfs(&s), -120.0);
    }

    #[test]
    fn iq_level_dbfs_unit_tone_is_zero_dbfs() {
        let s = vec![Complex::new(1.0, 0.0); 1024];
        let v = iq_level_dbfs(&s);
        assert!(v.abs() < 1e-9);
    }

    #[test]
    fn computes_clipping_ratio() {
        let mut calc = RfMetricsCalculator::with_config(8.0, 1.0, 8.0, 0.98);
        let samples = vec![
            Complex::new(0.5, 0.1),
            Complex::new(1.0, 0.0),
            Complex::new(0.0, -1.0),
            Complex::new(0.2, 0.3),
        ];

        let metrics = calc.push_chunk(&samples, 1.0).expect("expected a snapshot");
        assert!((metrics.clipping_ratio - 0.5).abs() < 1e-9);
    }

    #[test]
    fn computes_finite_noise_floor() {
        let mut calc = RfMetricsCalculator::with_config(4.0, 1.0, 4.0, 0.98);
        let samples = vec![
            Complex::new(0.1, 0.1),
            Complex::new(0.2, 0.1),
            Complex::new(0.1, 0.2),
            Complex::new(0.2, 0.2),
        ];

        let metrics = calc.push_chunk(&samples, 1.0).expect("expected a snapshot");
        assert!(metrics.noise_floor_db.is_finite());
    }

    #[test]
    fn rolling_window_evicts_old_samples() {
        let mut calc = RfMetricsCalculator::with_config(4.0, 1.0, 2.0, 0.98);

        let low = vec![Complex::new(0.1, 0.0), Complex::new(0.1, 0.0)];
        let high = vec![Complex::new(1.0, 0.0), Complex::new(1.0, 0.0)];

        let _ = calc.push_chunk(&low, 0.5);
        let first = calc.push_chunk(&low, 1.0).expect("expected first snapshot");
        assert!(first.clipping_ratio < 0.1);

        let _ = calc.push_chunk(&high, 1.5);
        let second = calc
            .push_chunk(&high, 2.0)
            .expect("expected second snapshot");
        assert!(second.clipping_ratio > 0.9);
    }
}
