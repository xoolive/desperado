//! Adalm Pluto SDR support module
//! (requires the `pluto` feature)
//!
//! This module provides functionality to read I/Q samples from Adalm Pluto SDR
//! devices, both synchronously and asynchronously. It uses the `pluto-sdr`
//! crate to interface with the Adalm Pluto hardware.

use futures::Stream;
use industrial_io::{Buffer, Channel};
use num_complex::Complex;
use pluto_sdr::pluto::{Pluto, RX};
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::sync::mpsc;

use crate::error;

const DEFAULT_BUFFER_SIZE: usize = 32768;

/**
 * Adalm Pluto SDR Configuration
 */
#[derive(Debug, Clone)]
pub struct PlutoConfig {
    /// Device address (e.g., "ip:192.168.2.1" or "usb:1.2.3"); use iio_info -s to find out the proper uri
    pub uri: String,
    /// Center frequency in Hz
    pub center_freq: i64,
    /// Sample rate in Hz
    pub sample_rate: i64,
    /// Tuner gain in dB (None for auto gain)
    pub gain: f64,
}

impl PlutoConfig {
    /// Create a new Adalm Pluto SDR configuration with specified parameters
    pub fn new(address: String, center_freq: i64, sample_rate: i64, gain: f64) -> Self {
        Self {
            uri: address,
            center_freq,
            sample_rate,
            gain,
        }
    }
}

/**
 * Synchronous Adalm Pluto SDR I/Q Reader
 */
pub struct PlutoSdrReader {
    buffer: Buffer,
    rx_i: Channel,
    rx_q: Channel,
    i_samples: Vec<i16>,
    q_samples: Vec<i16>,
    pos: usize,
}

impl PlutoSdrReader {
    pub fn new(config: &PlutoConfig) -> error::Result<Self> {
        let pluto = Pluto::connect(&config.uri).expect("Failed to connect to Pluto");

        // Configure receiver
        let _ = pluto.set_sampling_freq(config.sample_rate);
        let _ = pluto.set_lo_rx(config.center_freq);
        // Set bandwidth to 80% of sample rate (typical)
        //let bandwidth = (config.sample_rate as f64 * 0.8) as i64;
        let bandwidth = config.sample_rate;
        let _ = pluto.set_rf_bandwidth(bandwidth, RX);
        // Set gain
        let _ = pluto.set_hwgain(config.gain, RX);

        // Get RX channels
        let (rx_i, rx_q) = pluto.rx_ch0();

        // Enable both channels
        rx_i.enable();
        rx_q.enable();

        // Create buffer
        let buffer = pluto.create_buffer_rx(DEFAULT_BUFFER_SIZE).unwrap();

        Ok(Self {
            buffer,
            rx_i,
            rx_q,
            i_samples: Vec::new(),
            q_samples: Vec::new(),
            pos: 0,
        })
    }

    fn refill_buffer(&mut self) -> error::Result<()> {
        // Refill the buffer with new samples
        self.buffer
            .refill()
            .map_err(|e| format!("Buffer refill failed: {:?}", e))?;

        // Read I and Q samples
        self.i_samples = self
            .rx_i
            .read::<i16>(&self.buffer)
            .map_err(|e| format!("Failed to read I samples: {:?}", e))?;

        self.q_samples = self
            .rx_q
            .read::<i16>(&self.buffer)
            .map_err(|e| format!("Failed to read Q samples: {:?}", e))?;

        self.pos = 0;
        Ok(())
    }
}

impl Iterator for PlutoSdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        // Refill buffer if we've consumed all samples
        if self.pos >= self.i_samples.len()
            && let Err(e) = self.refill_buffer()
        {
            return Some(Err(std::io::Error::other(e)));
        }

        // Check if we have samples to return
        if self.pos < self.i_samples.len() && self.pos < self.q_samples.len() {
            // Collect all remaining samples in the buffer
            let len = self.i_samples.len().min(self.q_samples.len()) - self.pos;
            let mut samples = Vec::with_capacity(len);
            for _ in 0..len {
                let i = self.i_samples[self.pos];
                let q = self.q_samples[self.pos];
                self.pos += 1;
                let i_norm = i as f32 / 2048.0;
                let q_norm = q as f32 / 2048.0;
                samples.push(Complex::new(i_norm, q_norm));
            }
            Some(Ok(samples))
        } else {
            None
        }
    }
}

/**
 * Asynchronous Adalm Pluto SDR I/Q Stream
 */
pub struct AsyncPlutoSdrReader {
    receiver: mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
    _handle: tokio::task::JoinHandle<()>,
}

impl AsyncPlutoSdrReader {
    pub async fn new(config: &PlutoConfig) -> error::Result<Self> {
        let config = config.clone();

        // Create channel for streaming samples
        let (tx, rx) = mpsc::channel::<error::Result<Vec<Complex<f32>>>>(32);

        // Spawn blocking task for Pluto SDR operations
        let handle = tokio::task::spawn_blocking(move || {
            let result = Self::run_pluto_rx(config, tx);

            if let Err(e) = result {
                eprintln!("Pluto SDR error: {}", e);
            }
        });

        Ok(Self {
            receiver: rx,
            _handle: handle,
        })
    }

    fn run_pluto_rx(
        config: PlutoConfig,
        tx: mpsc::Sender<error::Result<Vec<Complex<f32>>>>,
    ) -> error::Result<()> {
        let pluto = Pluto::connect(&config.uri).expect("Failed to connect to Pluto");

        // Configure receiver
        let _ = pluto.set_sampling_freq(config.sample_rate);
        let _ = pluto.set_lo_rx(config.center_freq);

        let bandwidth = (config.sample_rate as f64 * 0.8) as i64;
        let _ = pluto.set_rf_bandwidth(bandwidth, RX);

        let _ = pluto.set_hwgain(config.gain, RX);

        // Get RX channels
        let (rx_i, rx_q) = pluto.rx_ch0();

        // Enable both channels
        rx_i.enable();
        rx_q.enable();

        // Create buffer
        let mut buffer = pluto
            .create_buffer_rx(DEFAULT_BUFFER_SIZE)
            .map_err(|e| format!("Failed to create buffer: {:?}", e))?;

        // Continuous reading loop
        loop {
            // Refill buffer with new samples
            if let Err(e) = buffer.refill() {
                let _ = tx.blocking_send(Err(std::io::Error::other(format!(
                    "Buffer refill failed: {:?}",
                    e
                ))));
                break;
            }

            // Read I and Q samples
            let i_samples = rx_i
                .read::<i16>(&buffer)
                .map_err(|e| format!("Failed to read I samples: {:?}", e))
                .unwrap_or_default();

            let q_samples = rx_q
                .read::<i16>(&buffer)
                .map_err(|e| format!("Failed to read Q samples: {:?}", e))
                .unwrap_or_default();

            // Convert to complex samples
            let mut samples = Vec::with_capacity(i_samples.len().min(q_samples.len()));
            for (i, q) in i_samples.iter().zip(q_samples.iter()) {
                // Normalize from i16 range to -1.0..1.0
                let i_norm = *i as f32 / 2048.0; // Pluto uses 12-bit samples
                let q_norm = *q as f32 / 2048.0;
                samples.push(Complex::new(i_norm, q_norm));
            }

            // Send samples through channel
            if tx.blocking_send(Ok(samples)).is_err() {
                // Receiver dropped, exit loop
                break;
            }
        }

        Ok(())
    }
}

impl Stream for AsyncPlutoSdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.receiver.poll_recv(cx)
    }
}
