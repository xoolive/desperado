//! RTL-SDR I/Q Data Source Module
//! (requires the `rtlsdr` feature)
//!
//! Provides synchronous and asynchronous I/Q readers for RTL-SDR devices,
//! using the `rs_rtl` crate (pure-Rust nusb backend) for hardware access.
//!
//! The rs_rtl driver keeps multiple USB bulk transfers in-flight simultaneously
//! via nusb's endpoint queue, eliminating the inter-transfer gap that causes
//! RTL2832U FIFO overflow at high sample rates.

use futures::Stream;
use num_complex::Complex;

use crate::{Gain, IqFormat, error};

/// Tokio-side bridge queue depth between the USB reader thread and the async consumer.
///
/// The inner `rs_rtl` streaming queue (RECOMMENDED_QUEUE_DEPTH = 32 chunks) is the
/// primary buffer; this bridges the sync reader thread → tokio async consumer.
///
/// Sized to absorb bursts when the tokio event loop is busy with OFDM/DSP work.
/// At DAB sample rates (2048 kSPS CU8 = 2 MB/s), each 16 KB chunk is ~4 ms,
/// so 32 chunks ≈ 128 ms of headroom.
const BRIDGE_QUEUE_DEPTH: usize = 32;

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
    let devices = rs_rtl::DeviceDescriptors::new()
        .map_err(|e| error::Error::device(format!("Failed to enumerate devices: {e}")))?;
    Ok(devices
        .iter()
        .enumerate()
        .map(|(i, d)| RtlSdrDeviceInfo {
            index: i,
            manufacturer: d.manufacturer.clone().unwrap_or_default(),
            product: d.product.clone().unwrap_or_default(),
            serial: d.serial.clone().unwrap_or_default(),
        })
        .collect())
}

/// Resolve a DeviceSelector to a device index.
fn resolve_device_index(selector: &DeviceSelector) -> error::Result<usize> {
    match selector {
        DeviceSelector::Index(idx) => Ok(*idx),
        DeviceSelector::Filter {
            manufacturer,
            product,
            serial,
        } => {
            let devices = rs_rtl::DeviceDescriptors::new()
                .map_err(|e| error::Error::device(format!("Failed to enumerate devices: {e}")))?;

            let matching = devices.iter().enumerate().find(|(_, d)| {
                manufacturer
                    .as_ref()
                    .is_none_or(|m| d.manufacturer.as_deref() == Some(m.as_str()))
                    && product
                        .as_ref()
                        .is_none_or(|p| d.product.as_deref() == Some(p.as_str()))
                    && serial
                        .as_ref()
                        .is_none_or(|s| d.serial.as_deref() == Some(s.as_str()))
            });

            match matching {
                Some((idx, _)) => Ok(idx),
                None => Err(error::Error::device(format!(
                    "No RTL-SDR device found matching filters: \
                     manufacturer={manufacturer:?}, product={product:?}, serial={serial:?}"
                ))),
            }
        }
    }
}

/// Open and configure an RTL-SDR device from a `RtlSdrConfig`.
fn open_and_configure(config: &RtlSdrConfig) -> error::Result<rs_rtl::RtlSdr> {
    let idx = resolve_device_index(&config.device)?;
    let mut sdr = rs_rtl::RtlSdr::open(rs_rtl::DeviceId::Index(idx))?;

    sdr.set_sample_rate(config.sample_rate)?;
    let _ = sdr.set_bandwidth(config.sample_rate);

    if config.freq_correction_ppm != 0 {
        tracing::warn!(
            ppm = config.freq_correction_ppm,
            "rs-rtl does not support frequency correction PPM; ignoring"
        );
    }

    sdr.set_center_freq(config.center_freq)?;

    match config.gain {
        Gain::Manual(gain_db) => {
            let gain_tenths = (gain_db * 10.0) as i32;
            tracing::info!(gain_db, gain_tenths, "Setting manual tuner gain");
            sdr.set_gain_manual(gain_tenths)?;
        }
        Gain::Auto => {
            tracing::info!("Setting automatic tuner gain");
            sdr.set_gain_auto()?;
        }
        Gain::Elements(_) => {
            tracing::warn!("RTL-SDR does not support element-based gain control, using auto gain");
            sdr.set_gain_auto()?;
        }
    }

    let _ = sdr.set_bias_t(config.bias_tee);

    Ok(sdr)
}

/**
 * Synchronous RTL-SDR I/Q Reader
 */
pub struct RtlSdrReader {
    config: RtlSdrConfig,
    /// Background streaming handle (lazily initialized on first `next()` call).
    bg_rx: Option<std::sync::mpsc::Receiver<Result<Vec<u8>, String>>>,
}

impl RtlSdrReader {
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        Ok(Self {
            config: config.clone(),
            bg_rx: None,
        })
    }

    fn start_reader_thread(&mut self) -> error::Result<()> {
        let (tx, rx) = std::sync::mpsc::sync_channel::<Result<Vec<u8>, String>>(64);
        let (tx_init, rx_init) = std::sync::mpsc::sync_channel::<Result<(), String>>(1);

        let config = self.config.clone();

        std::thread::spawn(move || {
            let mut sdr = match open_and_configure(&config) {
                Ok(dev) => dev,
                Err(e) => {
                    let _ = tx_init.send(Err(e.to_string()));
                    return;
                }
            };

            let reader = match sdr.start_streaming() {
                Ok(r) => r,
                Err(e) => {
                    let _ = tx_init.send(Err(e.to_string()));
                    return;
                }
            };
            let _ = tx_init.send(Ok(()));

            while let Some(data) = reader.recv() {
                if data.is_empty() {
                    continue;
                }
                // Use blocking send for backpressure (lesson 20.1)
                if tx.send(Ok(data)).is_err() {
                    break;
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
        }
        Ok(())
    }

    /// Change tuner gain mode/value.
    pub fn set_gain(&mut self, gain: Gain) -> error::Result<()> {
        if self.bg_rx.is_none() {
            self.config.gain = gain;
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
/// Uses `rs_rtl::RtlSdr::start_streaming()` which runs multi-transfer USB I/O
/// on a dedicated OS thread with a bounded `sync_channel`. A thin bridge thread
/// converts raw bytes to `Complex<f32>` and forwards to a tokio channel,
/// keeping the tokio worker pool free for decode work.
///
/// Architecture:
/// ```text
///   USB streaming thread  ──sync_channel(32)──▶  bridge thread  ──tokio::mpsc(32)──▶  decode loop
///   (nusb multi-transfer)                         (bytes→complex)                      (IqAsyncSource)
/// ```
///
/// The streaming thread uses nusb's endpoint queue with 15 transfers in-flight
/// simultaneously, eliminating inter-transfer gaps. Delivery uses blocking
/// `send()` for backpressure — the USB thread pauses when the queue is full,
/// preventing data loss.
pub struct AsyncRtlSdrReader {
    /// Control handle for tune/gain/stop on the USB reader thread.
    control: rs_rtl::AsyncReadControlHandle,
    /// Async receiver for the decode loop.
    samples_rx: tokio::sync::mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
}

impl AsyncRtlSdrReader {
    /// Create a new async RTL-SDR reader.
    ///
    /// Opens and configures the device, then starts multi-transfer streaming.
    /// A bridge thread converts raw IQ bytes and forwards them to the tokio channel.
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        let mut sdr = open_and_configure(config)?;

        let reader = sdr
            .start_streaming()
            .map_err(|e| error::Error::device(format!("Failed to start streaming: {e}")))?;
        let control = reader.control_handle();

        let (samples_tx, samples_rx) = tokio::sync::mpsc::channel(BRIDGE_QUEUE_DEPTH);

        // Bridge thread: receives raw bytes from the USB streaming thread,
        // converts to complex, sends to tokio channel.
        // Exits when the consumer drops `samples_rx`.
        std::thread::Builder::new()
            .name("rtlsdr-bridge".into())
            .spawn(move || {
                // Keep sdr alive so the device is not dropped while streaming
                let _sdr = sdr;
                while let Some(bytes) = reader.recv() {
                    let samples = Ok(crate::convert_bytes_to_complex(IqFormat::Cu8, &bytes));
                    if samples_tx.blocking_send(samples).is_err() {
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
    pub fn adjust(&self, message: RtlSdrMessage) -> error::Result<()> {
        match message {
            RtlSdrMessage::Frequency(freq) => self
                .control
                .tune(freq)
                .map_err(|e| error::Error::device(format!("RTL-SDR tune failed: {e}"))),
            RtlSdrMessage::Gain(gain) => match gain {
                Gain::Auto => self.control.set_gain_auto().map_err(|e| {
                    error::Error::device(format!("RTL-SDR set auto gain failed: {e}"))
                }),
                Gain::Manual(db) => {
                    let gain_tenths = (db * 10.0) as i32;
                    self.control
                        .set_gain(gain_tenths)
                        .map_err(|e| error::Error::device(format!("RTL-SDR set gain failed: {e}")))
                }
                Gain::Elements(_) => {
                    tracing::warn!("Element-based gain not supported for RTL-SDR; ignoring");
                    Ok(())
                }
            },
            RtlSdrMessage::SampleRate(_rate) => {
                tracing::warn!(
                    "RTL-SDR live sample rate change not supported during streaming; ignoring"
                );
                Ok(())
            }
            RtlSdrMessage::FreqCorrection(ppm) => {
                tracing::warn!(
                    ppm,
                    "RTL-SDR live frequency correction change not supported during streaming; ignoring"
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
