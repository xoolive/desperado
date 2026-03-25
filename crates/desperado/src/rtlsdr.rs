//! RTL-SDR I/Q Data Source Module
//! (requires the `rtlsdr` feature)
//!
//! This module provides functionality to read I/Q samples from RTL-SDR devices,
//! both synchronously and asynchronously. It uses the `rtl_sdr_rs` crate to
//! interface with the RTL-SDR hardware.

use futures::Stream;
use num_complex::Complex;
use rtl_sdr_rs::{DEFAULT_BUF_LENGTH, RtlSdr, TunerGain, error::RtlsdrError};

use crate::{Gain, IqFormat, error};

const ASYNC_QUEUE_CHUNKS: usize = 512;

// WORKAROUND: Temporary device enumeration until rtl-sdr-rs PR #26 is merged
// This module can be removed once PR #26 is available
mod device_workaround {
    use rusb::{Context, DeviceDescriptor, UsbContext};

    // Known RTL-SDR device VID/PID pairs (from rtl-sdr-rs)
    const KNOWN_DEVICES: &[(u16, u16)] = &[
        (0x0bda, 0x2832), // Realtek RTL2832U
        (0x0bda, 0x2838), // Realtek RTL2838
    ];

    pub struct DeviceInfo {
        pub index: usize,
        pub manufacturer: String,
        pub product: String,
        pub serial: String,
    }

    fn is_known_device(vid: u16, pid: u16) -> bool {
        KNOWN_DEVICES.iter().any(|&(v, p)| v == vid && p == pid)
    }

    pub fn enumerate_rtlsdr_devices() -> Result<Vec<DeviceInfo>, rusb::Error> {
        let context = Context::new()?;
        let devices = context.devices()?;
        let mut rtlsdr_devices: Vec<DeviceInfo> = Vec::new();
        let mut index: usize = 0;

        for device in devices.iter() {
            let desc: DeviceDescriptor = match device.device_descriptor() {
                Ok(d) => d,
                Err(_) => continue,
            };

            if !is_known_device(desc.vendor_id(), desc.product_id()) {
                continue;
            }

            // Try to open device to read strings
            let handle = match device.open() {
                Ok(h) => h,
                Err(_) => {
                    // Device exists but we can't open it (permissions, etc.)
                    // Still add it with placeholder strings
                    rtlsdr_devices.push(DeviceInfo {
                        index,
                        manufacturer: format!("VID:{:04x}", desc.vendor_id()),
                        product: format!("PID:{:04x}", desc.product_id()),
                        serial: format!("Unknown-{}", index),
                    });
                    index += 1;
                    continue;
                }
            };

            let manufacturer: String = desc
                .manufacturer_string_index()
                .and_then(|idx| handle.read_string_descriptor_ascii(idx).ok())
                .unwrap_or_else(|| format!("VID:{:04x}", desc.vendor_id()));

            let product: String = desc
                .product_string_index()
                .and_then(|idx| handle.read_string_descriptor_ascii(idx).ok())
                .unwrap_or_else(|| format!("PID:{:04x}", desc.product_id()));

            let serial: String = desc
                .serial_number_string_index()
                .and_then(|idx| handle.read_string_descriptor_ascii(idx).ok())
                .unwrap_or_else(|| format!("Unknown-{}", index));

            rtlsdr_devices.push(DeviceInfo {
                index,
                manufacturer,
                product,
                serial,
            });

            index += 1;
        }

        Ok(rtlsdr_devices)
    }

    pub fn find_device_by_filters(
        manufacturer: &Option<String>,
        product: &Option<String>,
        serial: &Option<String>,
    ) -> Result<Option<usize>, rusb::Error> {
        let devices = enumerate_rtlsdr_devices()?;

        for device in devices {
            let manufacturer_match = manufacturer
                .as_ref()
                .map(|m| device.manufacturer == *m)
                .unwrap_or(true);

            let product_match = product
                .as_ref()
                .map(|p| device.product == *p)
                .unwrap_or(true);

            let serial_match = serial.as_ref().map(|s| device.serial == *s).unwrap_or(true);

            if manufacturer_match && product_match && serial_match {
                return Ok(Some(device.index));
            }
        }

        Ok(None)
    }
}
// END WORKAROUND

/**
 * Device selector for RTL-SDR devices
 */
#[derive(Debug, Clone, PartialEq)]
pub enum DeviceSelector {
    /// Select device by index (0 for first device)
    Index(usize),
    /// Select device by filters (manufacturer, product, serial)
    /// All provided filters must match
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

/// Control message for dynamic RTL-SDR parameter adjustment
///
/// These messages are sent to the async reader task to adjust device parameters
/// in real-time without restarting the reader.
#[derive(Debug, Clone)]
pub enum RtlSdrMessage {
    /// Retune to center frequency (Hz)
    Frequency(u32),
    /// Change sample rate (Hz)
    SampleRate(u32),
    /// Change tuner gain
    Gain(Gain),
    /// Change frequency correction (PPM)
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

/// List all available RTL-SDR devices
///
/// Returns a list of detected RTL-SDR devices with their information.
/// This can be used to discover what devices are available and their properties.
///
/// # Examples
///
/// ```no_run
/// use desperado::rtlsdr::list_devices;
///
/// match list_devices() {
///     Ok(devices) => {
///         println!("Found {} RTL-SDR device(s):", devices.len());
///         for dev in devices {
///             println!("  [{}] {}, {}, SN: {}",
///                      dev.index, dev.manufacturer, dev.product, dev.serial);
///         }
///     }
///     Err(e) => eprintln!("Error listing devices: {}", e),
/// }
/// ```
pub fn list_devices() -> error::Result<Vec<RtlSdrDeviceInfo>> {
    // WORKAROUND: Once rtl-sdr-rs PR #26 is merged, use DeviceDescriptors instead
    device_workaround::enumerate_rtlsdr_devices()
        .map(|devices| {
            devices
                .into_iter()
                .map(|d| RtlSdrDeviceInfo {
                    index: d.index,
                    manufacturer: d.manufacturer,
                    product: d.product,
                    serial: d.serial,
                })
                .collect()
        })
        .map_err(|e| error::Error::device(format!("Failed to enumerate devices: {}", e)))
}

/// Helper function to open RTL-SDR device based on selector
fn open_device_with_selector(selector: &DeviceSelector) -> error::Result<RtlSdr> {
    match selector {
        DeviceSelector::Index(idx) => RtlSdr::open_with_index(*idx).map_err(|e| e.into()),
        DeviceSelector::Filter {
            manufacturer,
            product,
            serial,
        } => {
            // WORKAROUND: Use rusb directly to enumerate and filter devices
            // Once rtl-sdr-rs PR #26 is merged, replace this with:
            //   RtlSdr::open(DeviceId::Serial(serial)) or similar

            let device_index =
                device_workaround::find_device_by_filters(manufacturer, product, serial).map_err(
                    |e| error::Error::device(format!("Failed to enumerate devices: {}", e)),
                )?;

            match device_index {
                Some(idx) => {
                    eprintln!("Found matching RTL-SDR device at index {}", idx);
                    if let (Some(m), Some(p), Some(s)) = (manufacturer, product, serial) {
                        eprintln!("  Manufacturer: {}, Product: {}, Serial: {}", m, p, s);
                    }
                    RtlSdr::open_with_index(idx).map_err(|e| e.into())
                }
                None => Err(error::Error::device(format!(
                    "No RTL-SDR device found matching filters: manufacturer={:?}, product={:?}, serial={:?}",
                    manufacturer, product, serial
                ))),
            }
        }
    }
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

/// Asynchronous RTL-SDR I/Q Reader
///
/// Pure async implementation using `tokio::task::block_in_place()` for blocking USB I/O.
/// No spawned threads - tokio manages the execution.
///
/// This reader accepts control messages to adjust device parameters (frequency, sample rate,
/// gain, PPM) in real-time without restarting the reader.
pub struct AsyncRtlSdrReader {
    /// Channel to send control messages (frequency, sample_rate, gain, etc)
    adjust_tx: tokio::sync::mpsc::UnboundedSender<RtlSdrMessage>,
    /// Channel to receive sample data
    samples_rx: tokio::sync::mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
}

impl AsyncRtlSdrReader {
    /// Create new async RTL-SDR reader
    ///
    /// Sets up the device and wraps it in Arc<Mutex> for shared access between
    /// the async context and the spawned task. Uses `block_in_place()` for
    /// blocking USB operations without blocking the executor.
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        // Setup phase: configure device
        let mut rtl = open_device_with_selector(&config.device)?;
        rtl.set_sample_rate(config.sample_rate)?;
        rtl.set_tuner_bandwidth(config.sample_rate)?;
        rtl.set_freq_correction(config.freq_correction_ppm)?;
        rtl.set_center_freq(config.center_freq)?;

        match config.gain {
            Gain::Manual(gain_db) => {
                let gain_tenths = (gain_db * 10.0) as i32;
                rtl.set_tuner_gain(TunerGain::Manual(gain_tenths))?
            }
            Gain::Auto => rtl.set_tuner_gain(TunerGain::Auto)?,
            Gain::Elements(_) => {
                eprintln!(
                    "Warning: RTL-SDR does not support element-based gain control, using auto gain"
                );
                rtl.set_tuner_gain(TunerGain::Auto)?
            }
        };

        rtl.set_bias_tee(config.bias_tee)?;
        rtl.reset_buffer()?;

        // Wrap device in Arc<Mutex> for thread-safe shared access
        let rtl = std::sync::Arc::new(std::sync::Mutex::new(rtl));

        // Create channels
        let (adjust_tx, adjust_rx) = tokio::sync::mpsc::unbounded_channel::<RtlSdrMessage>();
        let (samples_tx, samples_rx) =
            tokio::sync::mpsc::channel::<error::Result<Vec<Complex<f32>>>>(ASYNC_QUEUE_CHUNKS);

        // Spawn task for sample reading and control message handling
        // No explicit thread spawning - this is a tokio task managed by the executor
        let rtl_clone = rtl.clone();
        tokio::spawn(async move { Self::reader_task(rtl_clone, adjust_rx, samples_tx).await });

        Ok(Self {
            adjust_tx,
            samples_rx,
        })
    }

    /// Async task that handles sample reading and control messages
    ///
    /// - Reads USB data in blocking context via `block_in_place`
    /// - Handles control messages (frequency, gain, etc) non-blockingly
    /// - Uses `tokio::select!` to multiplex both operations
    /// - Device is wrapped in Arc<Mutex> so both reads and control access it safely
    async fn reader_task(
        rtl: std::sync::Arc<std::sync::Mutex<RtlSdr>>,
        mut adjust_rx: tokio::sync::mpsc::UnboundedReceiver<RtlSdrMessage>,
        samples_tx: tokio::sync::mpsc::Sender<error::Result<Vec<Complex<f32>>>>,
    ) {
        let mut buf = vec![0u8; DEFAULT_BUF_LENGTH];

        loop {
            tokio::select! {
                // Continuously try to read samples from device
                result = async {
                    tokio::task::block_in_place(|| {
                        let rtl = match rtl.lock() {
                            Ok(guard) => guard,
                            Err(_poisoned) => {
                                tracing::error!("RTL-SDR device mutex was poisoned");
                                return Err(RtlsdrError::RtlsdrErr(
                                    "Device mutex poisoned".to_string(),
                                ));
                            }
                        };
                        rtl.read_sync(&mut buf)
                    })
                } => {
                    match result {
                        Ok(n) => {
                            if n > 0 {
                                // Convert USB bytes to complex samples
                                let samples = crate::convert_bytes_to_complex(IqFormat::Cu8, &buf[..n]);
                                if samples_tx.send(Ok(samples)).await.is_err() {
                                    // Receiver dropped, exit task
                                    return;
                                }
                            }
                            // n == 0: No data read, shouldn't happen but continue
                        }
                        Err(e) => {
                            let _ = samples_tx.send(Err(e.into())).await;
                            return;
                        }
                    }
                }

                // Handle incoming control messages
                Some(msg) = adjust_rx.recv() => {
                    let result = tokio::task::block_in_place(|| {
                        let mut rtl = match rtl.lock() {
                            Ok(guard) => guard,
                            Err(_poisoned) => {
                                tracing::error!("RTL-SDR device mutex was poisoned");
                                return Err(RtlsdrError::RtlsdrErr(
                                    "Device mutex poisoned".to_string(),
                                ));
                            }
                        };
                        match msg {
                            RtlSdrMessage::Frequency(freq) => {
                                rtl.set_center_freq(freq)
                            }
                            RtlSdrMessage::SampleRate(sr) => {
                                rtl.set_sample_rate(sr)
                            }
                            RtlSdrMessage::Gain(gain) => {
                                match gain {
                                    Gain::Manual(gain_db) => {
                                        let gain_tenths = (gain_db * 10.0) as i32;
                                        rtl.set_tuner_gain(TunerGain::Manual(gain_tenths))
                                    }
                                    Gain::Auto => rtl.set_tuner_gain(TunerGain::Auto),
                                    Gain::Elements(_) => {
                                        // Not supported on RTL-SDR, silently ignore
                                        Ok(())
                                    }
                                }
                            }
                            RtlSdrMessage::FreqCorrection(ppm) => {
                                rtl.set_freq_correction(ppm)
                            }
                        }
                    });

                    if let Err(e) = result {
                        tracing::warn!("Failed to adjust RTL-SDR parameter: {}", e);
                    }
                }
            }
        }
    }

    /// Send a control message to adjust device parameters
    ///
    /// This is non-blocking - the message is queued and processed by the reader task.
    pub fn adjust(&self, message: RtlSdrMessage) -> error::Result<()> {
        self.adjust_tx
            .send(message)
            .map_err(|_| error::Error::device("AsyncRtlSdrReader task closed".to_string()))
    }

    /// Retune to a specific center frequency
    ///
    /// Convenience method that sends a Frequency control message.
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
        let this = &mut *self;
        this.samples_rx.poll_recv(cx)
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
/// use desperado::rtlsdr::{RtlSdrConfig, DeviceSelector};
/// use desperado::Gain;
///
/// // Select by index
/// let config = RtlSdrConfig {
///     device: DeviceSelector::Index(0),  // First device
///     center_freq: 1090000000,
///     sample_rate: 2400000,
///     gain: Gain::Auto,
///     bias_tee: false,
/// };
///
/// // Or select by filters (requires rtl-sdr-rs PR #26)
/// let config_filtered = RtlSdrConfig {
///     device: DeviceSelector::Filter {
///         manufacturer: Some("RTLSDRBlog".to_string()),
///         product: Some("Blog V4".to_string()),
///         serial: Some("00000001".to_string()),
///     },
///     center_freq: 1090000000,
///     sample_rate: 2400000,
///     gain: Gain::Auto,
///     bias_tee: false,
/// };
/// ```
pub fn get_first_device_index() -> usize {
    0 // RTL-SDR convention: device 0 is the first available device
}
