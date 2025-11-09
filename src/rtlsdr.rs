use futures::Stream;
use num_complex::Complex;
use rtl_sdr_rs::error::RtlsdrError;
use rtl_sdr_rs::{DEFAULT_BUF_LENGTH, RtlSdr, TunerGain};

use crate::IqFormat;

#[derive(Debug, Clone)]
pub struct RtlSdrConfig {
    /// Device index (0 for first device)
    pub device_index: usize,
    /// Center frequency in Hz
    pub center_freq: u32,
    /// Sample rate in Hz
    pub sample_rate: u32,
    /// Tuner gain (None for AGC, Some(gain) for manual)
    pub gain: Option<i32>,
}

impl RtlSdrConfig {
    /// Create a new RTL-SDR configuration with specified parameters
    pub fn new(device_index: usize, center_freq: u32, sample_rate: u32, gain: Option<i32>) -> Self {
        Self {
            device_index,
            center_freq,
            sample_rate,
            gain,
        }
    }
}

pub struct RtlSdrReader {
    rtlsdr: RtlSdr,
    buf: Vec<u8>,
    pos: usize,
    end: usize,
}

impl RtlSdrReader {
    pub fn new(config: &RtlSdrConfig) -> Result<Self, RtlsdrError> {
        let mut rtlsdr = RtlSdr::open_with_index(config.device_index)?;
        rtlsdr.set_sample_rate(config.sample_rate)?;
        rtlsdr.set_center_freq(config.center_freq)?;
        match config.gain {
            Some(gain) => rtlsdr.set_tuner_gain(TunerGain::Manual(gain))?,
            None => rtlsdr.set_tuner_gain(TunerGain::Auto)?,
        };
        let _ = rtlsdr.set_bias_tee(false);
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
    type Item = Result<Vec<Complex<f32>>, RtlsdrError>;

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
                Err(e) => return Some(Err(e)),
            }
        }

        Some(Ok(crate::convert_bytes_to_complex(
            IqFormat::Cu8,
            &self.buf[self.pos..self.end],
        )))
    }
}

pub struct AsyncRtlSdrReader {
    rx: tokio::sync::mpsc::Receiver<Result<Vec<Complex<f32>>, RtlsdrError>>,
    _handle: std::thread::JoinHandle<()>,
}

impl AsyncRtlSdrReader {
    pub fn new(config: &RtlSdrConfig) -> Result<Self, RtlsdrError> {
        let (tx, rx) = tokio::sync::mpsc::channel::<Result<Vec<Complex<f32>>, RtlsdrError>>(32);
        let (tx_init, rx_init) = std::sync::mpsc::channel::<Result<(), RtlsdrError>>();
        let cfg = config.clone();

        let handle = std::thread::spawn(move || {
            let init_res = (|| -> Result<RtlSdr, RtlsdrError> {
                let mut rtl = RtlSdr::open_with_index(cfg.device_index)?;
                rtl.set_sample_rate(cfg.sample_rate)?;
                rtl.set_center_freq(cfg.center_freq)?;
                match cfg.gain {
                    Some(gain) => rtl.set_tuner_gain(TunerGain::Manual(gain))?,
                    None => rtl.set_tuner_gain(TunerGain::Auto)?,
                };
                let _ = rtl.set_bias_tee(false);
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
                                let _ = tx.blocking_send(Err(e));
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
            Err(e) => Err(RtlsdrError::RtlsdrErr(format!("RecvError {}", e))),
        }
    }
}

impl Stream for AsyncRtlSdrReader {
    type Item = Result<Vec<Complex<f32>>, RtlsdrError>;

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
