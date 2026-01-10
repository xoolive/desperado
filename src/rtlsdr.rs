//! RTL-SDR I/Q Data Source Module
//! (requires the `rtlsdr` feature)
//!
//! This module provides functionality to read I/Q samples from RTL-SDR devices,
//! both synchronously and asynchronously. It uses the `rtl_sdr_rs` crate to
//! interface with the RTL-SDR hardware.

use futures::Stream;
use num_complex::Complex;
use rtl_sdr_rs::{RtlSdr, TunerGain, DEFAULT_BUF_LENGTH};

use crate::{error, Gain, IqFormat};

/**
 * RTL-SDR Configuration
 */
#[derive(Debug, Clone)]
pub struct RtlSdrConfig {
    /// Device index (0 for first device)
    pub device_index: usize,
    /// Center frequency in Hz
    pub center_freq: u32,
    /// Sample rate in Hz
    pub sample_rate: u32,
    /// Tuner gain (Auto or Manual in dB)
    pub gain: Gain,
    /// Enable bias tee (default: false)
    pub bias_tee: bool,
}

impl RtlSdrConfig {
    /// Create a new RTL-SDR configuration with specified parameters
    pub fn new(device_index: usize, center_freq: u32, sample_rate: u32, gain: Gain) -> Self {
        Self {
            device_index,
            center_freq,
            sample_rate,
            gain,
            bias_tee: false,
        }
    }
}

/**
 * Synchronous RTL-SDR I/Q Reader
 */
pub struct RtlSdrReader {
    rtlsdr: RtlSdr,
    buf: Vec<u8>,
    pos: usize,
    end: usize,
}

impl RtlSdrReader {
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        let mut rtlsdr = RtlSdr::open_with_index(config.device_index)?;
        rtlsdr.set_sample_rate(config.sample_rate)?;
        rtlsdr.set_center_freq(config.center_freq)?;
        match config.gain {
            Gain::Manual(gain_db) => {
                // Convert dB to rtl-sdr units (gain * 10)
                let gain_tenths = (gain_db * 10.0) as i32;
                rtlsdr.set_tuner_gain(TunerGain::Manual(gain_tenths))?
            }
            Gain::Auto => rtlsdr.set_tuner_gain(TunerGain::Auto)?,
        };
        let _ = rtlsdr.set_bias_tee(config.bias_tee);
        rtlsdr.reset_buffer()?;
        Ok(Self {
            rtlsdr,
            buf: vec![0u8; DEFAULT_BUF_LENGTH],
            pos: 0,
            end: 0,
        })
    }
}

impl Iterator for RtlSdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.end {
            match self.rtlsdr.read_sync(&mut self.buf) {
                Ok(bytes_read) => {
                    if bytes_read == 0 {
                        return None; // End of stream
                    }
                    self.pos = 0;
                    self.end = bytes_read;
                }
                Err(e) => return Some(Err(e.into())),
            }
        }

        Some(Ok(crate::convert_bytes_to_complex(
            IqFormat::Cu8,
            &self.buf[self.pos..self.end],
        )))
    }
}

/**
 * Asynchronous RTL-SDR I/Q Reader
 */
pub struct AsyncRtlSdrReader {
    rx: tokio::sync::mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
    _handle: std::thread::JoinHandle<()>,
}

impl AsyncRtlSdrReader {
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        let (tx, rx) = tokio::sync::mpsc::channel::<error::Result<Vec<Complex<f32>>>>(32);
        let (tx_init, rx_init) = std::sync::mpsc::channel::<error::Result<()>>();
        let cfg = config.clone();

        let handle = std::thread::spawn(move || {
            let init_res = (|| -> error::Result<RtlSdr> {
                let mut rtl = RtlSdr::open_with_index(cfg.device_index)?;
                rtl.set_sample_rate(cfg.sample_rate)?;
                rtl.set_center_freq(cfg.center_freq)?;
                match cfg.gain {
                    Gain::Manual(gain_db) => {
                        // Convert dB to rtl-sdr units (gain * 10)
                        let gain_tenths = (gain_db * 10.0) as i32;
                        rtl.set_tuner_gain(TunerGain::Manual(gain_tenths))?
                    }
                    Gain::Auto => rtl.set_tuner_gain(TunerGain::Auto)?,
                };
                let _ = rtl.set_bias_tee(cfg.bias_tee);
                rtl.reset_buffer()?;
                Ok(rtl)
            })();

            match init_res {
                Ok(rtl) => {
                    let _ = tx_init.send(Ok(()));
                    let mut buffer = vec![0u8; DEFAULT_BUF_LENGTH];
                    loop {
                        match rtl.read_sync(&mut buffer) {
                            Ok(bytes_read) => {
                                if bytes_read == 0 {
                                    let _ = tx.blocking_send(Ok(Vec::new()));
                                    return;
                                }
                                let samples = crate::convert_bytes_to_complex(
                                    IqFormat::Cu8,
                                    &buffer[..bytes_read],
                                );
                                if tx.blocking_send(Ok(samples)).is_err() {
                                    return;
                                }
                            }
                            Err(e) => {
                                let _ = tx.blocking_send(Err(e.into()));
                                return;
                            }
                        }
                    }
                }
                Err(e) => {
                    let _ = tx_init.send(Err(e));
                }
            }
        });

        match rx_init.recv() {
            Ok(Ok(())) => Ok(Self {
                rx,
                _handle: handle,
            }),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(error::Error::device("Failed to initialize RTL-SDR device")),
        }
    }
}

impl Stream for AsyncRtlSdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        let this = &mut *self;
        match this.rx.poll_recv(cx) {
            std::task::Poll::Ready(Some(item)) => std::task::Poll::Ready(Some(item)),
            std::task::Poll::Ready(None) => std::task::Poll::Ready(None),
            std::task::Poll::Pending => std::task::Poll::Pending,
        }
    }
}

/// Try to open the first available RTL-SDR device
///
/// This is a convenience function that attempts to open device index 0.
/// Returns an error if no device is found.
///
/// # Examples
///
/// ```no_run
/// use desperado::rtlsdr::RtlSdrConfig;
/// use desperado::Gain;
///
/// let config = RtlSdrConfig {
///     device_index: 0,  // First device
///     center_freq: 1090000000,
///     sample_rate: 2400000,
///     gain: Gain::Auto,
///     bias_tee: false,
/// };
/// ```
pub fn get_first_device_index() -> usize {
    0 // RTL-SDR convention: device 0 is the first available device
}
