//! SoapySDR I/Q Data Source Module
//! (requires the `soapysdr` feature)
//!
//! This module provides functionality to read I/Q samples from SoapySDR devices,
//! both synchronously and asynchronously.

use futures::Stream;
use num_complex::Complex;
use soapysdr::{Args, Device, Direction, Error as SoapyError};

use crate::{Gain, error};

/**
 * SoapySDR Configuration
 */
#[derive(Debug, Clone, PartialEq)]
pub struct SoapyConfig {
    /// Device arguments (e.g., "driver=rtlsdr")
    pub args: String,
    /// Center frequency in Hz
    pub center_freq: f64,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Channel index (typically 0)
    pub channel: usize,
    /// Tuner gain (Auto or Manual in dB)
    pub gain: Gain,
    /// Gain element name (e.g., "TUNER")
    pub gain_element: String,
}

impl SoapyConfig {
    /// Create a new SoapySDR configuration with specified parameters
    pub fn new(args: String, center_freq: f64, sample_rate: f64) -> Self {
        Self {
            args,
            center_freq,
            sample_rate,
            channel: 0,
            gain: Gain::Auto,
            gain_element: "TUNER".to_string(),
        }
    }
}

/**
 * Synchronous SoapySDR I/Q Reader
 */
pub struct SoapySdrReader {
    stream: soapysdr::RxStream<Complex<i16>>,
    buf: Vec<Complex<i16>>,
    pos: usize,
    end: usize,
}

impl SoapySdrReader {
    pub fn new(config: &SoapyConfig) -> error::Result<Self> {
        let device = Device::new(config.args.as_str())?;

        device.set_frequency(Direction::Rx, config.channel, config.center_freq, ())?;
        device.set_sample_rate(Direction::Rx, config.channel, config.sample_rate)?;

        match config.gain {
            Gain::Manual(gain_db) => {
                device.set_gain_element(
                    Direction::Rx,
                    config.channel,
                    config.gain_element.as_str(),
                    gain_db,
                )?;
            }
            Gain::Auto => {
                // SoapySDR uses automatic gain when no manual gain is set
            }
        }

        let mut stream = device.rx_stream::<Complex<i16>>(&[config.channel])?;
        let mtu = stream.mtu()?;
        stream.activate(None)?;

        Ok(Self {
            stream,
            buf: vec![Complex::new(0, 0); mtu],
            pos: 0,
            end: 0,
        })
    }
}

impl Iterator for SoapySdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.end {
            match self.stream.read(&mut [&mut self.buf], 5_000_000) {
                Ok(len) => {
                    if len == 0 {
                        return None; // End of stream
                    }
                    self.pos = 0;
                    self.end = len;
                }
                Err(e) => {
                    return Some(Err(e.into()));
                }
            }
        }

        let samples: Vec<Complex<f32>> = self.buf[self.pos..self.end]
            .iter()
            .map(|c| {
                Complex::new(
                    c.re as f32 / (1 << 15) as f32,
                    c.im as f32 / (1 << 15) as f32,
                )
            })
            .collect();

        self.pos = self.end;
        Some(Ok(samples))
    }
}

/**
 * Asynchronous SoapySDR I/Q Reader
 */
pub struct AsyncSoapySdrReader {
    rx: tokio::sync::mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
    _handle: std::thread::JoinHandle<()>,
}

impl AsyncSoapySdrReader {
    pub fn new(config: &SoapyConfig) -> error::Result<Self> {
        let (tx, rx) = tokio::sync::mpsc::channel::<error::Result<Vec<Complex<f32>>>>(32);
        let (tx_init, rx_init) = std::sync::mpsc::channel::<error::Result<()>>();
        let cfg = config.clone();

        let handle = std::thread::spawn(move || {
            let init_res = (|| -> error::Result<(Device, soapysdr::RxStream<Complex<i16>>)> {
                let device = Device::new(cfg.args.as_str())?;

                device.set_frequency(Direction::Rx, cfg.channel, cfg.center_freq, ())?;
                device.set_sample_rate(Direction::Rx, cfg.channel, cfg.sample_rate)?;

                match cfg.gain {
                    Gain::Manual(gain_db) => {
                        device.set_gain_element(
                            Direction::Rx,
                            cfg.channel,
                            cfg.gain_element.as_str(),
                            gain_db,
                        )?;
                    }
                    Gain::Auto => {
                        // SoapySDR uses automatic gain when no manual gain is set
                    }
                }

                let mut stream = device.rx_stream::<Complex<i16>>(&[cfg.channel])?;
                stream.activate(None)?;

                Ok((device, stream))
            })();

            match init_res {
                Ok((_device, mut stream)) => {
                    let _ = tx_init.send(Ok(()));
                    let mtu = stream.mtu().unwrap_or(16384);
                    let mut buffer = vec![Complex::new(0, 0); mtu];

                    loop {
                        match stream.read(&mut [&mut buffer], 5_000_000) {
                            Ok(len) => {
                                if len == 0 {
                                    let _ = tx.blocking_send(Ok(Vec::new()));
                                    return;
                                }
                                let samples: Vec<Complex<f32>> = buffer[..len]
                                    .iter()
                                    .map(|c| {
                                        Complex::new(
                                            c.re as f32 / (1 << 15) as f32,
                                            c.im as f32 / (1 << 15) as f32,
                                        )
                                    })
                                    .collect();

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
            Err(_) => Err(error::Error::device("Failed to initialize SoapySDR device")),
        }
    }
}

impl Stream for AsyncSoapySdrReader {
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

/// Enumerate available SoapySDR devices
pub fn enumerate_devices(args: &str) -> Result<Vec<Args>, SoapyError> {
    soapysdr::enumerate(args)
}
