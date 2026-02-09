//! Airspy SDR I/Q Data Source Module
//! (requires the `airspy` feature)
//!
//! This module provides functionality to read I/Q samples from Airspy devices
//! (R2, Mini, HF+), both synchronously and asynchronously. It uses the `rs_spy`
//! crate to interface with the Airspy hardware.
//!
//! # Sample Format
//!
//! Airspy hardware outputs **REAL samples** from a single ADC sampling the
//! IF (Intermediate Frequency) output from the R820T tuner chip.
//!
//! This module converts real samples to complex I/Q using the `IqConverter`
//! from rs-spy, which implements libairspy-compatible Fs/4 frequency translation:
//!
//! 1. **DC Removal**: High-pass filter to remove DC offset
//! 2. **Fs/4 Translation**: Frequency shift by sample_rate/4
//! 3. **Half-band FIR Filter**: 47-tap filter for I channel
//! 4. **Delay Line**: Matches group delay for Q channel
//!
//! This provides proper I/Q samples without requiring frequency offset compensation.
//!
//! # Example
//!
//! ```no_run
//! use desperado::airspy::{AirspyConfig, AirspySdrReader};
//! use desperado::Gain;
//!
//! let config = AirspyConfig::new(0, 100_000_000, 6_000_000, Gain::Auto);
//! let reader = AirspySdrReader::new(&config)?;
//!
//! for samples in reader.take(10) {
//!     let samples = samples?;
//!     println!("Received {} samples", samples.len());
//! }
//! # Ok::<(), desperado::Error>(())
//! ```

use futures::Stream;
use num_complex::Complex;
use rs_spy::{Airspy, IqConverter, RECOMMENDED_BUFFER_SIZE};

use crate::{error, Gain, GainElementName};

/// Device selector for Airspy devices
#[derive(Debug, Clone, PartialEq)]
pub enum DeviceSelector {
    /// Select device by index (0 for first device)
    Index(usize),
    /// Select device by serial number
    Serial(u64),
}

impl Default for DeviceSelector {
    fn default() -> Self {
        DeviceSelector::Index(0)
    }
}

/// Airspy SDR configuration
///
/// # Sample Rate Note
///
/// Sample rate selection is most reliable when using the default rate (index 0,
/// typically 6 MSPS for Airspy Mini). Switching to non-default sample rates may
/// require the device to be in a specific state. If you encounter sample rate
/// errors, try using the highest available rate (index 0) or power-cycling the device.
#[derive(Debug, Clone, PartialEq)]
pub struct AirspyConfig {
    /// Device selector (index or serial)
    pub device: DeviceSelector,
    /// Center frequency in Hz (24 MHz to 1.8 GHz for Mini)
    pub center_freq: u32,
    /// Sample rate in Hz (must match one of the device's supported rates)
    pub sample_rate: u32,
    /// Gain configuration
    pub gain: Gain,
    /// Enable bias tee / RF bias (DC on antenna port)
    pub bias_tee: bool,
    /// Enable 12-bit sample packing (default: false = 16-bit samples)
    pub packing: bool,
    /// LNA gain (0-14), None for default/auto
    pub lna_gain: Option<u8>,
    /// Mixer gain (0-15), None for default/auto
    pub mixer_gain: Option<u8>,
    /// VGA gain (0-15), None for default/auto
    pub vga_gain: Option<u8>,
}

impl AirspyConfig {
    /// Create a new Airspy configuration with device index
    ///
    /// # Arguments
    ///
    /// * `device_index` - Zero-based device index
    /// * `center_freq` - Center frequency in Hz
    /// * `sample_rate` - Sample rate in Hz (must be supported by device)
    /// * `gain` - Gain configuration (Auto, Manual dB, or per-element)
    pub fn new(device_index: usize, center_freq: u32, sample_rate: u32, gain: Gain) -> Self {
        Self {
            device: DeviceSelector::Index(device_index),
            center_freq,
            sample_rate,
            gain,
            bias_tee: false,
            packing: false,
            lna_gain: None,
            mixer_gain: None,
            vga_gain: None,
        }
    }

    /// Create a new Airspy configuration with serial number selection
    pub fn new_with_serial(serial: u64, center_freq: u32, sample_rate: u32, gain: Gain) -> Self {
        Self {
            device: DeviceSelector::Serial(serial),
            center_freq,
            sample_rate,
            gain,
            bias_tee: false,
            packing: false,
            lna_gain: None,
            mixer_gain: None,
            vga_gain: None,
        }
    }
}

/// Device information for Airspy devices
#[derive(Debug, Clone, PartialEq)]
pub struct AirspyDeviceInfo {
    /// Device index (0-based)
    pub index: usize,
    /// Board ID (0 = AIRSPY, 1 = AIRSPY MINI, etc.)
    pub board_id: u32,
    /// Firmware version string
    pub firmware_version: String,
    /// Part IDs (two u32 values)
    pub part_id: [u32; 2],
    /// Serial number
    pub serial_number: u64,
    /// Supported sample rates in Hz
    pub supported_sample_rates: Vec<u32>,
}

impl AirspyDeviceInfo {
    /// Get a human-readable board name
    pub fn board_name(&self) -> &'static str {
        match self.board_id {
            0 => "AIRSPY",
            1 => "AIRSPY MINI",
            2 => "AIRSPY HF+",
            _ => "AIRSPY (Unknown)",
        }
    }
}

/// List all available Airspy devices
///
/// Returns a list of detected Airspy devices with their information.
/// This can be used to discover what devices are available and their properties.
///
/// # Examples
///
/// ```no_run
/// use desperado::airspy::list_devices;
///
/// match list_devices() {
///     Ok(devices) => {
///         println!("Found {} Airspy device(s):", devices.len());
///         for dev in devices {
///             println!("  [{}] {} - {} (SN: 0x{:016X})",
///                      dev.index, dev.board_name(), dev.firmware_version, dev.serial_number);
///         }
///     }
///     Err(e) => eprintln!("Error listing devices: {}", e),
/// }
/// ```
pub fn list_devices() -> error::Result<Vec<AirspyDeviceInfo>> {
    let device_strings = Airspy::list_devices()
        .map_err(|e| error::Error::device(format!("Failed to enumerate Airspy devices: {}", e)))?;

    let mut devices = Vec::new();
    for (index, _) in device_strings.iter().enumerate() {
        // Open device to query info
        match Airspy::open_by_index(index) {
            Ok(device) => {
                let board_id = device.board_id().unwrap_or(0);
                let firmware_version = device.version().unwrap_or_default();
                let (part_id_1, part_id_2, serial_number) =
                    device.board_partid_serialno().unwrap_or((0, 0, 0));
                let supported_sample_rates = device.supported_sample_rates().unwrap_or_default();

                devices.push(AirspyDeviceInfo {
                    index,
                    board_id,
                    firmware_version,
                    part_id: [part_id_1, part_id_2],
                    serial_number,
                    supported_sample_rates,
                });
            }
            Err(e) => {
                // Device exists but we can't open it (permissions, busy, etc.)
                eprintln!(
                    "Warning: Could not open Airspy device at index {}: {}",
                    index, e
                );
            }
        }
    }

    Ok(devices)
}

/// Helper function to open Airspy device based on selector
fn open_device_with_selector(selector: &DeviceSelector) -> error::Result<Airspy> {
    match selector {
        DeviceSelector::Index(idx) => {
            Airspy::open_by_index(*idx).map_err(|e| error::Error::device(e.to_string()))
        }
        DeviceSelector::Serial(serial) => {
            // Enumerate devices and find matching serial
            let devices = list_devices()?;
            for dev in devices {
                if dev.serial_number == *serial {
                    return Airspy::open_by_index(dev.index)
                        .map_err(|e| error::Error::device(e.to_string()));
                }
            }
            Err(error::Error::device(format!(
                "No Airspy device found with serial 0x{:016X}",
                serial
            )))
        }
    }
}

/// Find sample rate index for a given sample rate value
fn find_sample_rate_index(device: &Airspy, target_rate: u32) -> error::Result<u8> {
    let rates = device
        .supported_sample_rates()
        .map_err(|e| error::Error::device(format!("Failed to get sample rates: {}", e)))?;

    for (index, rate) in rates.iter().enumerate() {
        if *rate == target_rate {
            return Ok(index as u8);
        }
    }

    // If exact match not found, return error with available rates
    let rate_strs: Vec<String> = rates.iter().map(|r| format!("{} Hz", r)).collect();
    Err(error::Error::device(format!(
        "Unsupported sample rate {} Hz. Available rates: {}",
        target_rate,
        rate_strs.join(", ")
    )))
}

/// Configure gain on Airspy device based on Gain enum
fn configure_gain(
    device: &Airspy,
    gain: &Gain,
    lna: Option<u8>,
    mixer: Option<u8>,
    vga: Option<u8>,
) -> error::Result<()> {
    // If individual gains are specified, apply them directly
    if let Some(lna_val) = lna {
        device
            .set_lna_gain(lna_val.min(14))
            .map_err(|e| error::Error::device(format!("Failed to set LNA gain: {}", e)))?;
    }
    if let Some(mixer_val) = mixer {
        device
            .set_mixer_gain(mixer_val.min(15))
            .map_err(|e| error::Error::device(format!("Failed to set Mixer gain: {}", e)))?;
    }
    if let Some(vga_val) = vga {
        device
            .set_vga_gain(vga_val.min(15))
            .map_err(|e| error::Error::device(format!("Failed to set VGA gain: {}", e)))?;
    }

    // If individual gains are specified, skip the normal gain configuration
    if lna.is_some() || mixer.is_some() || vga.is_some() {
        // Disable AGC when using manual gains
        device
            .set_lna_agc(false)
            .map_err(|e| error::Error::device(format!("Failed to disable LNA AGC: {}", e)))?;
        device
            .set_mixer_agc(false)
            .map_err(|e| error::Error::device(format!("Failed to disable Mixer AGC: {}", e)))?;
        return Ok(());
    }

    // Otherwise use the standard gain configuration
    match gain {
        Gain::Auto => {
            // Enable AGC for both LNA and Mixer
            device
                .set_lna_agc(true)
                .map_err(|e| error::Error::device(format!("Failed to enable LNA AGC: {}", e)))?;
            device
                .set_mixer_agc(true)
                .map_err(|e| error::Error::device(format!("Failed to enable Mixer AGC: {}", e)))?;
            // VGA gain: set to middle value when using AGC
            device
                .set_vga_gain(8)
                .map_err(|e| error::Error::device(format!("Failed to set VGA gain: {}", e)))?;
        }
        Gain::Manual(db) => {
            // Map dB value (0-50 range typical) to Airspy's sensitivity preset (0-21)
            // This is an approximation; actual dB depends on frequency and other factors
            let level = ((*db / 50.0) * 21.0).clamp(0.0, 21.0) as u8;
            device.set_sensitivity_gain(level).map_err(|e| {
                error::Error::device(format!("Failed to set sensitivity gain: {}", e))
            })?;
        }
        Gain::Elements(elements) => {
            // Disable AGC when using element-based gain
            device
                .set_lna_agc(false)
                .map_err(|e| error::Error::device(format!("Failed to disable LNA AGC: {}", e)))?;
            device
                .set_mixer_agc(false)
                .map_err(|e| error::Error::device(format!("Failed to disable Mixer AGC: {}", e)))?;

            for element in elements {
                match &element.name {
                    GainElementName::Lna => {
                        // LNA gain: 0-14
                        let gain = (element.value_db as u8).min(14);
                        device.set_lna_gain(gain).map_err(|e| {
                            error::Error::device(format!("Failed to set LNA gain: {}", e))
                        })?;
                    }
                    GainElementName::Mix => {
                        // Mixer gain: 0-15
                        let gain = (element.value_db as u8).min(15);
                        device.set_mixer_gain(gain).map_err(|e| {
                            error::Error::device(format!("Failed to set Mixer gain: {}", e))
                        })?;
                    }
                    GainElementName::Vga => {
                        // VGA gain: 0-15
                        let gain = (element.value_db as u8).min(15);
                        device.set_vga_gain(gain).map_err(|e| {
                            error::Error::device(format!("Failed to set VGA gain: {}", e))
                        })?;
                    }
                    other => {
                        eprintln!(
                            "Warning: Airspy does not support gain element {:?}, ignoring",
                            other
                        );
                    }
                }
            }
        }
    }
    Ok(())
}

/// Synchronous Airspy I/Q Reader
///
/// Provides an iterator-based interface for reading I/Q samples from Airspy devices.
/// Each call to `next()` returns a chunk of Complex<f32> samples.
///
/// # Example
///
/// ```no_run
/// use desperado::airspy::{AirspyConfig, AirspySdrReader};
/// use desperado::Gain;
///
/// let config = AirspyConfig::new(0, 100_000_000, 6_000_000, Gain::Auto);
/// let mut reader = AirspySdrReader::new(&config)?;
///
/// for result in reader.take(10) {
///     let samples = result?;
///     println!("Got {} samples", samples.len());
/// }
/// # Ok::<(), desperado::Error>(())
/// ```
pub struct AirspySdrReader {
    device: Airspy,
    buf: Vec<u8>,
    float_buf: Vec<f32>,
    iq_converter: IqConverter,
    packing: bool,
}

impl AirspySdrReader {
    /// Create a new synchronous Airspy reader
    ///
    /// Opens the device, configures it according to the provided config,
    /// and starts streaming.
    pub fn new(config: &AirspyConfig) -> error::Result<Self> {
        let device = open_device_with_selector(&config.device)?;

        // Find sample rate index before starting (for validation)
        let rate_index = find_sample_rate_index(&device, config.sample_rate)?;

        // Set frequency directly - no offset needed!
        // The IqConverter performs Fs/4 translation which brings the IF signal to baseband.
        device
            .set_freq(config.center_freq)
            .map_err(|e| error::Error::device(format!("Failed to set frequency: {}", e)))?;

        // Configure gain
        configure_gain(
            &device,
            &config.gain,
            config.lna_gain,
            config.mixer_gain,
            config.vga_gain,
        )?;

        // Set RF bias (Bias-T)
        device
            .set_rf_bias(config.bias_tee)
            .map_err(|e| error::Error::device(format!("Failed to set RF bias: {}", e)))?;

        // Set packing mode
        device
            .set_packing(config.packing)
            .map_err(|e| error::Error::device(format!("Failed to set packing: {}", e)))?;

        // Set sample rate BEFORE starting RX if not the default (index 0)
        // Note: Sample rate setting may fail on some firmware versions before streaming starts.
        // If it fails, we try again after start_rx().
        let mut sample_rate_set = false;
        if rate_index != 0 {
            match device.set_sample_rate(rate_index) {
                Ok(()) => sample_rate_set = true,
                Err(_e) => {
                    // Will try again after start_rx
                    // Some firmware versions require streaming to be active first
                }
            }
        } else {
            sample_rate_set = true; // Index 0 is the default
        }

        // Start streaming
        device
            .start_rx()
            .map_err(|e| error::Error::device(format!("Failed to start RX: {}", e)))?;

        // If sample rate wasn't set before, try now after start_rx
        if !sample_rate_set {
            match device.set_sample_rate(rate_index) {
                Ok(()) => {}
                Err(e) => {
                    let supported = device.supported_sample_rates().unwrap_or_default();
                    return Err(error::Error::device(format!(
                        "Failed to set sample rate after start_rx: {}. \
                        Supported rates: {:?}. \
                        Requested: {} Hz (index {})",
                        e, supported, config.sample_rate, rate_index
                    )));
                }
            }
        }

        // Pre-allocate float buffer for sample conversion
        // RECOMMENDED_BUFFER_SIZE bytes / 2 bytes per sample = number of samples
        let max_samples = RECOMMENDED_BUFFER_SIZE / 2;

        Ok(Self {
            device,
            buf: vec![0u8; RECOMMENDED_BUFFER_SIZE],
            float_buf: vec![0.0f32; max_samples],
            iq_converter: IqConverter::new(),
            packing: config.packing,
        })
    }

    /// Get device information
    pub fn device_info(&self) -> error::Result<AirspyDeviceInfo> {
        let board_id = self
            .device
            .board_id()
            .map_err(|e| error::Error::device(e.to_string()))?;
        let firmware_version = self
            .device
            .version()
            .map_err(|e| error::Error::device(e.to_string()))?;
        let (part_id_1, part_id_2, serial_number) = self
            .device
            .board_partid_serialno()
            .map_err(|e| error::Error::device(e.to_string()))?;
        let supported_sample_rates = self
            .device
            .supported_sample_rates()
            .map_err(|e| error::Error::device(e.to_string()))?;

        Ok(AirspyDeviceInfo {
            index: 0, // We don't track the index after opening
            board_id,
            firmware_version,
            part_id: [part_id_1, part_id_2],
            serial_number,
            supported_sample_rates,
        })
    }
}

impl Iterator for AirspySdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.device.read_sync(&mut self.buf) {
            Ok(bytes_read) => {
                if bytes_read == 0 {
                    return None; // End of stream
                }

                if self.packing {
                    // TODO: Implement 12-bit unpacking for packed mode
                    eprintln!(
                        "Warning: 12-bit unpacking not yet implemented, data may be incorrect"
                    );
                }

                // Convert raw bytes to f32 samples for IqConverter
                // Airspy outputs unsigned 12-bit samples (0-4095 range) stored in u16
                // We normalize to [-1.0, 1.0] centered around 2048
                let num_samples = bytes_read / 2;

                // Convert u16 bytes to f32 samples
                for (i, chunk) in self.buf[..bytes_read].chunks_exact(2).enumerate() {
                    let sample = u16::from_le_bytes([chunk[0], chunk[1]]);
                    self.float_buf[i] = (sample as f32 - 2048.0) / 2048.0;
                }

                // Process through IqConverter (Fs/4 translation)
                // This converts real samples to interleaved I/Q
                let samples = self
                    .iq_converter
                    .process_to_complex(&mut self.float_buf[..num_samples]);

                Some(Ok(samples))
            }
            Err(e) => Some(Err(error::Error::device(format!("Read error: {}", e)))),
        }
    }
}

impl Drop for AirspySdrReader {
    fn drop(&mut self) {
        // Stop streaming when reader is dropped
        let _ = self.device.stop_rx();
    }
}

/// Asynchronous Airspy I/Q Reader
///
/// Provides a `futures::Stream`-based interface for reading I/Q samples.
/// Internally spawns a background thread that reads from the device and
/// sends samples through a tokio channel.
///
/// # Example
///
/// ```no_run
/// use desperado::airspy::{AirspyConfig, AsyncAirspySdrReader};
/// use desperado::Gain;
/// use futures::StreamExt;
///
/// # async fn example() -> Result<(), desperado::Error> {
/// let config = AirspyConfig::new(0, 100_000_000, 6_000_000, Gain::Auto);
/// let mut reader = AsyncAirspySdrReader::new(&config)?;
///
/// while let Some(result) = reader.next().await {
///     let samples = result?;
///     println!("Got {} samples", samples.len());
/// }
/// # Ok(())
/// # }
/// ```
pub struct AsyncAirspySdrReader {
    rx: tokio::sync::mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
    _handle: std::thread::JoinHandle<()>,
}

impl AsyncAirspySdrReader {
    /// Create a new asynchronous Airspy reader
    ///
    /// Opens the device in a background thread and streams samples through
    /// an async channel.
    pub fn new(config: &AirspyConfig) -> error::Result<Self> {
        let (tx, rx) = tokio::sync::mpsc::channel::<error::Result<Vec<Complex<f32>>>>(32);
        let (tx_init, rx_init) = std::sync::mpsc::channel::<error::Result<()>>();
        let cfg = config.clone();

        let handle = std::thread::spawn(move || {
            let init_res = (|| -> error::Result<(Airspy, bool)> {
                let device = open_device_with_selector(&cfg.device)?;

                // Find sample rate index before starting (for validation)
                let rate_index = find_sample_rate_index(&device, cfg.sample_rate)?;

                // Set frequency directly - no offset needed!
                // The IqConverter performs Fs/4 translation which brings the IF signal to baseband.
                device
                    .set_freq(cfg.center_freq)
                    .map_err(|e| error::Error::device(format!("Failed to set frequency: {}", e)))?;

                // Configure gain
                configure_gain(
                    &device,
                    &cfg.gain,
                    cfg.lna_gain,
                    cfg.mixer_gain,
                    cfg.vga_gain,
                )?;

                // Set RF bias
                device
                    .set_rf_bias(cfg.bias_tee)
                    .map_err(|e| error::Error::device(format!("Failed to set RF bias: {}", e)))?;

                // Set packing mode
                device
                    .set_packing(cfg.packing)
                    .map_err(|e| error::Error::device(format!("Failed to set packing: {}", e)))?;

                // Set sample rate BEFORE starting RX if not the default (index 0)
                let mut sample_rate_set = false;
                if rate_index != 0 {
                    if device.set_sample_rate(rate_index).is_ok() {
                        sample_rate_set = true;
                    }
                } else {
                    sample_rate_set = true;
                }

                // Start streaming
                device
                    .start_rx()
                    .map_err(|e| error::Error::device(format!("Failed to start RX: {}", e)))?;

                // If sample rate wasn't set before, try now after start_rx
                if !sample_rate_set {
                    device.set_sample_rate(rate_index).map_err(|e| {
                        error::Error::device(format!("Failed to set sample rate: {}", e))
                    })?;
                }

                Ok((device, cfg.packing))
            })();

            match init_res {
                Ok((device, packing)) => {
                    let _ = tx_init.send(Ok(()));
                    let mut buffer = vec![0u8; RECOMMENDED_BUFFER_SIZE];
                    let max_samples = RECOMMENDED_BUFFER_SIZE / 2;
                    let mut float_buf = vec![0.0f32; max_samples];
                    let mut iq_converter = IqConverter::new();

                    loop {
                        match device.read_sync(&mut buffer) {
                            Ok(bytes_read) => {
                                if bytes_read == 0 {
                                    let _ = tx.blocking_send(Ok(Vec::new()));
                                    break;
                                }

                                if packing {
                                    // TODO: Implement 12-bit unpacking for packed mode
                                    eprintln!("Warning: 12-bit unpacking not yet implemented");
                                }

                                // Convert raw bytes to f32 samples
                                let num_samples = bytes_read / 2;
                                for (i, chunk) in buffer[..bytes_read].chunks_exact(2).enumerate() {
                                    let sample = u16::from_le_bytes([chunk[0], chunk[1]]);
                                    float_buf[i] = (sample as f32 - 2048.0) / 2048.0;
                                }

                                // Process through IqConverter (Fs/4 translation)
                                let samples =
                                    iq_converter.process_to_complex(&mut float_buf[..num_samples]);

                                if tx.blocking_send(Ok(samples)).is_err() {
                                    // Receiver dropped, stop streaming
                                    break;
                                }
                            }
                            Err(e) => {
                                let _ = tx.blocking_send(Err(error::Error::device(format!(
                                    "Read error: {}",
                                    e
                                ))));
                                break;
                            }
                        }
                    }

                    // Stop streaming on exit
                    let _ = device.stop_rx();
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
            Err(_) => Err(error::Error::device(
                "Failed to initialize Airspy device: channel closed",
            )),
        }
    }
}

impl Stream for AsyncAirspySdrReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        match self.rx.poll_recv(cx) {
            std::task::Poll::Ready(Some(item)) => std::task::Poll::Ready(Some(item)),
            std::task::Poll::Ready(None) => std::task::Poll::Ready(None),
            std::task::Poll::Pending => std::task::Poll::Pending,
        }
    }
}

/// Get the index of the first available Airspy device
///
/// Returns 0, which is the conventional index for the first device.
pub fn get_first_device_index() -> usize {
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_selector_default() {
        assert_eq!(DeviceSelector::default(), DeviceSelector::Index(0));
    }

    #[test]
    fn test_airspy_config_new() {
        let config = AirspyConfig::new(0, 100_000_000, 6_000_000, Gain::Auto);
        assert_eq!(config.device, DeviceSelector::Index(0));
        assert_eq!(config.center_freq, 100_000_000);
        assert_eq!(config.sample_rate, 6_000_000);
        assert_eq!(config.gain, Gain::Auto);
        assert!(!config.bias_tee);
        assert!(!config.packing);
    }

    #[test]
    fn test_airspy_config_with_serial() {
        let config =
            AirspyConfig::new_with_serial(0x35AC63DC2D8C7A4F, 100_000_000, 6_000_000, Gain::Auto);
        assert_eq!(config.device, DeviceSelector::Serial(0x35AC63DC2D8C7A4F));
    }

    #[test]
    fn test_device_info_board_name() {
        let info = AirspyDeviceInfo {
            index: 0,
            board_id: 1,
            firmware_version: "test".to_string(),
            part_id: [0, 0],
            serial_number: 0,
            supported_sample_rates: vec![],
        };
        assert_eq!(info.board_name(), "AIRSPY MINI");
    }
}
