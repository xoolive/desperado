//! RTL-SDR I/Q Data Source Module
//! (requires the `rtlsdr` feature)
//!
//! This module provides functionality to read I/Q samples from RTL-SDR devices,
//! both synchronously and asynchronously. It uses the `rtl_sdr_rs` crate to
//! interface with the RTL-SDR hardware.

use futures::Stream;
use num_complex::Complex;
use rtl_sdr_rs::{DEFAULT_BUF_LENGTH, RtlSdr, TunerGain};

use crate::{Gain, IqFormat, error};

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
        }
    }
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
    rtlsdr: RtlSdr,
    buf: Vec<u8>,
    pos: usize,
    end: usize,
}

impl RtlSdrReader {
    pub fn new(config: &RtlSdrConfig) -> error::Result<Self> {
        let mut rtlsdr = open_device_with_selector(&config.device)?;
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
                let mut rtl = open_device_with_selector(&cfg.device)?;
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
