//! RTL-SDR I/Q Data Source Module
//! (requires the `rtlsdr` feature)
//!
//! Provides synchronous and asynchronous I/Q readers for RTL-SDR devices,
//! using the vendored `rtl_sdr_rs` crate for hardware access.

use futures::Stream;
use num_complex::Complex;
use rtl_sdr_rs::{
    DEFAULT_ASYNC_BUF_NUMBER, DEFAULT_BUF_LENGTH, DeviceDescriptors, RtlSdr, TunerGain,
};

use crate::{Gain, IqFormat, error};

/// Tokio-side bridge queue depth between the USB reader thread and the async consumer.
/// The inner `into_async_reader` queue (DEFAULT_ASYNC_BUF_NUMBER = 15 chunks) is the
/// real buffer; this just bridges sync → tokio.
const BRIDGE_QUEUE_DEPTH: usize = 4;

/**
 * Device selector for RTL-SDR devices
 */
#[derive(Debug, Clone, PartialEq)]
pub enum DeviceSelector {
    /// Select device by index (0 for first device)
    Index(usize),
    /// Select device by filters (manufacturer, product, serial).
    /// All provided filters must match.
    Filter {
        manufacturer: Option<String>,
        product: Option<String>,
        serial: Option<String>,
    },
}

impl Default for DeviceSelector {
    fn default() -> Self {
        DeviceSelector::Index(0)
    }
}

/**
 * RTL-SDR Configuration
 */
#[derive(Debug, Clone, PartialEq)]
pub struct RtlSdrConfig {
    /// Device selector (index or filters)
    pub device: DeviceSelector,
    /// Center frequency in Hz
    pub center_freq: u32,
    /// Sample rate in Hz
    pub sample_rate: u32,
    /// Tuner gain (Auto or Manual in dB)
    pub gain: Gain,
    /// Enable bias tee (default: false)
    pub bias_tee: bool,
    /// Frequency correction in PPM (default: 0)
    pub freq_correction_ppm: i32,
}

impl RtlSdrConfig {
    /// Create a new RTL-SDR configuration with device index
    pub fn new(device_index: usize, center_freq: u32, sample_rate: u32, gain: Gain) -> Self {
        Self {
            device: DeviceSelector::Index(device_index),
            center_freq,
            sample_rate,
            gain,
            bias_tee: false,
            freq_correction_ppm: 0,
        }
    }

    /// Create a new RTL-SDR configuration with device filters
    pub fn new_with_filters(
        manufacturer: Option<String>,
        product: Option<String>,
        serial: Option<String>,
        center_freq: u32,
        sample_rate: u32,
        gain: Gain,
    ) -> Self {
        Self {
            device: DeviceSelector::Filter {
                manufacturer,
                product,
                serial,
            },
            center_freq,
            sample_rate,
            gain,
            bias_tee: false,
            freq_correction_ppm: 0,
        }
    }
}

/// Control message for dynamic RTL-SDR parameter adjustment.
///
/// Sent to the async reader to adjust device parameters in real-time.
/// Only `Frequency` is supported during streaming; others are logged and ignored.
#[derive(Debug, Clone)]
pub enum RtlSdrMessage {
    /// Retune to center frequency (Hz)
    Frequency(u32),
    /// Change sample rate (Hz) — not supported during streaming
    SampleRate(u32),
    /// Change tuner gain — not supported during streaming
    Gain(Gain),
    /// Change frequency correction (PPM) — not supported during streaming
    FreqCorrection(i32),
}

/// Device information for RTL-SDR devices
#[derive(Debug, Clone, PartialEq)]
pub struct RtlSdrDeviceInfo {
    /// Device index (0-based)
    pub index: usize,
    /// Manufacturer name
    pub manufacturer: String,
    /// Product name
    pub product: String,
    /// Serial number
    pub serial: String,
}

/// List all available RTL-SDR devices.
pub fn list_devices() -> error::Result<Vec<RtlSdrDeviceInfo>> {
    let descriptors = DeviceDescriptors::new()
        .map_err(|e| error::Error::device(format!("Failed to enumerate devices: {e}")))?;
    Ok(descriptors
        .iter()
        .map(|d| RtlSdrDeviceInfo {
            index: d.index,
            manufacturer: d.manufacturer,
            product: d.product,
            serial: d.serial,
        })
        .collect())
}

/// Open an RTL-SDR device based on a selector.
fn open_device_with_selector(selector: &DeviceSelector) -> error::Result<RtlSdr> {
    match selector {
        DeviceSelector::Index(idx) => RtlSdr::open_with_index(*idx).map_err(|e| e.into()),
        DeviceSelector::Filter {
            manufacturer,
            product,
            serial,
        } => {
            let descriptors = DeviceDescriptors::new()
                .map_err(|e| error::Error::device(format!("Failed to enumerate devices: {e}")))?;

            let matching = descriptors.iter().find(|d| {
                manufacturer.as_ref().is_none_or(|m| &d.manufacturer == m)
                    && product.as_ref().is_none_or(|p| &d.product == p)
                    && serial.as_ref().is_none_or(|s| &d.serial == s)
            });

            match matching {
                Some(dev) => RtlSdr::open_with_index(dev.index).map_err(|e| e.into()),
                None => Err(error::Error::device(format!(
                    "No RTL-SDR device found matching filters: \
                     manufacturer={manufacturer:?}, product={product:?}, serial={serial:?}"
                ))),
            }
        }
    }
}

/// Configure an open RTL-SDR device from a `RtlSdrConfig`.
fn configure_rtlsdr(rtlsdr: &mut RtlSdr, config: &RtlSdrConfig) -> error::Result<()> {
    rtlsdr.set_sample_rate(config.sample_rate)?;
    let _ = rtlsdr.set_tuner_bandwidth(config.sample_rate);
    let _ = rtlsdr.set_freq_correction(config.freq_correction_ppm);
    rtlsdr.set_center_freq(config.center_freq)?;
    match config.gain {
        Gain::Manual(gain_db) => {
            let gain_tenths = (gain_db * 10.0) as i32;
            tracing::info!(gain_db, gain_tenths, "Setting manual tuner gain");
            rtlsdr.set_tuner_gain(TunerGain::Manual(gain_tenths))?
        }
        Gain::Auto => {
            tracing::info!("Setting automatic tuner gain");
            rtlsdr.set_tuner_gain(TunerGain::Auto)?
        }
        Gain::Elements(_) => {
            eprintln!(
                "Warning: RTL-SDR does not support element-based gain control, using auto gain"
            );
            rtlsdr.set_tuner_gain(TunerGain::Auto)?
        }
    };
    let _ = rtlsdr.set_bias_tee(config.bias_tee);
    rtlsdr.reset_buffer()?;
    Ok(())
}

/**
 * Synchronous RTL-SDR I/Q Reader
 */
pub struct RtlSdrReader {
    config: RtlSdrConfig,
    buf_size: usize,
    /// Background reader thread receiver (lazily initialized on first `next()` call).
    /// Sends raw u8 bytes to minimize time between USB reads.
    bg_rx: Option<std::sync::mpsc::Receiver<Result<Vec<u8>, String>>>,
}

impl RtlSdrReader {
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        Ok(Self {
            config: config.clone(),
            buf_size: DEFAULT_BUF_LENGTH,
            bg_rx: None,
        })
    }

    fn start_reader_thread(&mut self) -> error::Result<()> {
        let (tx, rx) = std::sync::mpsc::sync_channel::<Result<Vec<u8>, String>>(64);
        let (tx_init, rx_init) = std::sync::mpsc::sync_channel::<Result<(), String>>(1);

        let config = self.config.clone();
        let buf_size = self.buf_size;

        std::thread::spawn(move || {
            let mut rtlsdr = match open_device_with_selector(&config.device) {
                Ok(dev) => dev,
                Err(e) => {
                    let _ = tx_init.send(Err(e.to_string()));
                    return;
                }
            };

            if let Err(e) = configure_rtlsdr(&mut rtlsdr, &config) {
                let _ = tx_init.send(Err(e.to_string()));
                return;
            }
            let _ = tx_init.send(Ok(()));

            let mut buf_a = vec![0u8; buf_size];
            let mut buf_b = vec![0u8; buf_size];
            let mut use_a = true;

            loop {
                let buf = if use_a { &mut buf_a } else { &mut buf_b };
                match rtlsdr.read_sync(buf) {
                    Ok(bytes_read) => {
                        if bytes_read == 0 {
                            continue;
                        }
                        if tx.send(Ok(buf[..bytes_read].to_vec())).is_err() {
                            break;
                        }
                        use_a = !use_a;
                    }
                    Err(e) => {
                        let _ = tx.send(Err(e.to_string()));
                        break;
                    }
                }
            }
        });

        match rx_init.recv() {
            Ok(Ok(())) => {
                self.bg_rx = Some(rx);
                Ok(())
            }
            Ok(Err(msg)) => Err(error::Error::Other(msg)),
            Err(_) => Err(error::Error::device(
                "Failed to initialize RTL-SDR reader thread",
            )),
        }
    }

    /// Retune the reader to a new center frequency.
    pub fn tune(&mut self, center_freq: u32) -> error::Result<()> {
        if self.bg_rx.is_none() {
            self.config.center_freq = center_freq;
            let mut rtlsdr = open_device_with_selector(&self.config.device)?;
            configure_rtlsdr(&mut rtlsdr, &self.config)?;
        }
        Ok(())
    }

    /// Change tuner gain mode/value.
    pub fn set_gain(&mut self, gain: Gain) -> error::Result<()> {
        if self.bg_rx.is_none() {
            self.config.gain = gain;
            let mut rtlsdr = open_device_with_selector(&self.config.device)?;
            configure_rtlsdr(&mut rtlsdr, &self.config)?;
        }
        Ok(())
    }
}

impl Iterator for RtlSdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.bg_rx.is_none()
            && let Err(e) = self.start_reader_thread()
        {
            return Some(Err(e));
        }

        if let Some(ref rx) = self.bg_rx {
            return match rx.recv() {
                Ok(Ok(bytes)) => {
                    let samples = crate::convert_bytes_to_complex(IqFormat::Cu8, &bytes);
                    Some(Ok(samples))
                }
                Ok(Err(msg)) => Some(Err(error::Error::Other(msg))),
                Err(_) => None,
            };
        }
        None
    }
}

/// Asynchronous RTL-SDR I/Q Reader.
///
/// Uses `RtlSdr::into_async_reader()` from the vendored library, which runs USB reads
/// on a dedicated OS thread with a bounded `sync_channel(DEFAULT_ASYNC_BUF_NUMBER)`.
/// A thin bridge thread converts raw bytes to `Complex<f32>` and forwards to a small
/// tokio channel, keeping the tokio worker pool free for decode work.
///
/// Architecture:
/// ```text
///   USB reader thread  ──sync_channel(15)──▶  bridge thread  ──tokio::mpsc(4)──▶  decode loop
///   (into_async_reader)                        (bytes→complex)                     (IqAsyncSource)
/// ```
pub struct AsyncRtlSdrReader {
    /// Control handle for tune/stop on the USB reader thread.
    control: rtl_sdr_rs::AsyncReadControlHandle,
    /// Async receiver for the decode loop.
    samples_rx: tokio::sync::mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
}

impl AsyncRtlSdrReader {
    /// Create a new async RTL-SDR reader.
    ///
    /// Opens and configures the device, then hands ownership to a dedicated USB reader
    /// thread via `into_async_reader`. A bridge thread converts samples and forwards
    /// them to the tokio channel.
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        let mut rtl = open_device_with_selector(&config.device)?;
        configure_rtlsdr(&mut rtl, config)?;

        // 15 concurrent libusb bulk transfers — mirrors C librtlsdr's rtlsdr_read_async
        // so the hardware FIFO never overflows between transfers.
        let handle = rtl
            .into_multi_transfer_reader(DEFAULT_ASYNC_BUF_NUMBER, DEFAULT_BUF_LENGTH)
            .map_err(|e| error::Error::device(format!("Failed to start async reader: {e}")))?;
        let control = handle.control_handle();

        let (samples_tx, samples_rx) = tokio::sync::mpsc::channel(BRIDGE_QUEUE_DEPTH);

        // Bridge thread: receives raw bytes from the USB reader, converts to complex,
        // sends to tokio channel. Exits when the consumer drops `samples_rx`.
        std::thread::Builder::new()
            .name("rtlsdr-bridge".into())
            .spawn(move || {
                for result in handle {
                    let to_send = match result {
                        Ok(bytes) => Ok(crate::convert_bytes_to_complex(IqFormat::Cu8, &bytes)),
                        Err(e) => Err(error::Error::from(e)),
                    };
                    let is_err = to_send.is_err();
                    if samples_tx.blocking_send(to_send).is_err() || is_err {
                        break;
                    }
                }
            })
            .map_err(|e| error::Error::device(format!("Failed to spawn bridge thread: {e}")))?;

        Ok(Self {
            control,
            samples_rx,
        })
    }

    /// Send a control message to the USB reader thread.
    ///
    /// Only `Frequency` is forwarded; other variants are not supported during
    /// streaming and are logged as warnings.
    pub fn adjust(&self, message: RtlSdrMessage) -> error::Result<()> {
        match message {
            RtlSdrMessage::Frequency(freq) => self
                .control
                .tune(freq)
                .map_err(|e| error::Error::device(format!("RTL-SDR tune failed: {e}"))),
            other => {
                tracing::warn!(
                    message = ?other,
                    "RTL-SDR live parameter adjustment not supported during streaming \
                     (only Frequency/tune is); ignoring"
                );
                Ok(())
            }
        }
    }

    /// Retune to a specific center frequency.
    pub fn tune(&self, center_freq: u32) -> error::Result<()> {
        self.adjust(RtlSdrMessage::Frequency(center_freq))
    }
}

impl Stream for AsyncRtlSdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        self.samples_rx.poll_recv(cx)
    }
}

/// Get the index of the first available RTL-SDR device (always 0).
pub fn get_first_device_index() -> usize {
    0
}
