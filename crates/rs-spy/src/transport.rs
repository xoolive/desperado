//! USB transport layer for Airspy devices.

use crate::error::{Error, Result};
use crate::{AIRSPY_PID, AIRSPY_VID};
use rusb::{Context, Device, DeviceHandle, UsbContext};
use std::time::Duration;

/// Default timeout for USB control transfers (milliseconds).
/// Matches libairspy's LIBUSB_CTRL_TIMEOUT_MS constant.
const USB_TIMEOUT: Duration = Duration::from_millis(500);

// Airspy command codes from libairspy (airspy_commands enum in airspy.h)
// Device info commands
const AIRSPY_BOARD_ID_READ: u8 = 9;
const AIRSPY_VERSION_STRING_READ: u8 = 10;
const AIRSPY_BOARD_PARTID_SERIALNO_READ: u8 = 11;
const AIRSPY_GET_SAMPLERATES: u8 = 25;

// Configuration commands
const AIRSPY_SET_FREQ: u8 = 13;
const AIRSPY_SET_LNA_GAIN: u8 = 14;
const AIRSPY_SET_MIXER_GAIN: u8 = 15;
const AIRSPY_SET_VGA_GAIN: u8 = 16;
const AIRSPY_SET_LNA_AGC: u8 = 17;
const AIRSPY_SET_MIXER_AGC: u8 = 18;
const AIRSPY_SET_SAMPLERATE: u8 = 19;
const AIRSPY_SET_PACKING: u8 = 26;

// Streaming commands
const AIRSPY_RECEIVER_MODE: u8 = 1;

// Receiver modes
const RECEIVER_MODE_OFF: u16 = 0;
const RECEIVER_MODE_RX: u16 = 1;

// Bulk transfer endpoint (IN endpoint 1)
const BULK_ENDPOINT_IN: u8 = 0x81; // LIBUSB_ENDPOINT_IN | 1

/// Recommended buffer size for bulk transfers (256 KB).
/// This matches libairspy's default and provides good throughput.
pub const RECOMMENDED_BUFFER_SIZE: usize = 262144;

/// Number of samples per buffer at 16-bit sample size.
/// For 256 KB buffer: 262144 / 2 = 131072 samples.
pub const SAMPLES_PER_BUFFER: usize = RECOMMENDED_BUFFER_SIZE / 2;

// GPIO commands for Bias-T control
const AIRSPY_GPIO_WRITE: u8 = 4;

// GPIO port/pin for Bias-T (RF bias) - GPIO_PORT1, GPIO_PIN13
const GPIO_PORT1: u8 = 1;
const GPIO_PIN13: u8 = 13;

// Gain lookup tables from libairspy for linearity and sensitivity presets
// Index 0 = gain level 21 (max), Index 21 = gain level 0 (min)
const GAIN_COUNT: usize = 22;

static LINEARITY_VGA_GAINS: [u8; GAIN_COUNT] = [
    13, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 8, 7, 6, 5, 4,
];
static LINEARITY_MIXER_GAINS: [u8; GAIN_COUNT] = [
    12, 12, 11, 9, 8, 7, 6, 6, 5, 0, 0, 1, 0, 0, 2, 2, 1, 1, 1, 1, 0, 0,
];
static LINEARITY_LNA_GAINS: [u8; GAIN_COUNT] = [
    14, 14, 14, 13, 12, 10, 9, 9, 8, 9, 8, 6, 5, 3, 1, 0, 0, 0, 0, 0, 0, 0,
];

static SENSITIVITY_VGA_GAINS: [u8; GAIN_COUNT] = [
    13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4,
];
static SENSITIVITY_MIXER_GAINS: [u8; GAIN_COUNT] = [
    12, 12, 12, 12, 11, 10, 10, 9, 9, 8, 7, 4, 4, 4, 3, 2, 2, 1, 0, 0, 0, 0,
];
static SENSITIVITY_LNA_GAINS: [u8; GAIN_COUNT] = [
    14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 12, 12, 9, 9, 8, 7, 6, 5, 3, 2, 1, 0,
];

/// Airspy device handle.
pub struct Airspy {
    device: DeviceHandle<Context>,
}

impl Airspy {
    /// Open the first available Airspy device.
    ///
    /// # Returns
    ///
    /// - `Ok(Airspy)` if a device was found and opened
    /// - `Err(Error::DeviceNotFound)` if no Airspy device is connected
    /// - `Err(Error::OpenFailed)` if the device could not be opened
    pub fn open_first() -> Result<Self> {
        let context = Context::new()?;
        let devices = context.devices()?;

        for device in devices.iter() {
            let desc = device.device_descriptor()?;

            if desc.vendor_id() == AIRSPY_VID && desc.product_id() == AIRSPY_PID {
                return Self::open_device(&device);
            }
        }

        Err(Error::DeviceNotFound)
    }

    /// Open a specific Airspy device by index.
    ///
    /// # Arguments
    ///
    /// * `index` - Zero-based device index (in order of enumeration)
    pub fn open_by_index(index: usize) -> Result<Self> {
        let context = Context::new()?;
        let devices = context.devices()?;

        let mut count = 0;
        for device in devices.iter() {
            let desc = device.device_descriptor()?;

            if desc.vendor_id() == AIRSPY_VID && desc.product_id() == AIRSPY_PID {
                if count == index {
                    return Self::open_device(&device);
                }
                count += 1;
            }
        }

        Err(Error::DeviceNotFound)
    }

    /// List all available Airspy devices.
    pub fn list_devices() -> Result<Vec<String>> {
        let context = Context::new()?;
        let devices = context.devices()?;
        let mut result = Vec::new();

        for device in devices.iter() {
            let desc = device.device_descriptor()?;

            if desc.vendor_id() == AIRSPY_VID && desc.product_id() == AIRSPY_PID {
                let serial = format!(
                    "Bus {:03} Device {:03}",
                    device.bus_number(),
                    device.address()
                );
                result.push(serial);
            }
        }

        Ok(result)
    }

    /// Open a device handle.
    ///
    /// This follows the libairspy initialization sequence:
    /// 1. Open the USB device
    /// 2. Detach kernel driver (Linux only)
    /// 3. Set configuration to 1
    /// 4. Claim interface 0
    fn open_device(device: &Device<Context>) -> Result<Self> {
        let handle = device.open()?;

        // On Linux, detach kernel driver if attached to interface 0
        #[cfg(target_os = "linux")]
        {
            if handle.kernel_driver_active(0).unwrap_or(false) {
                tracing::debug!("Detaching kernel driver from interface 0");
                if let Err(e) = handle.detach_kernel_driver(0) {
                    tracing::warn!("Failed to detach kernel driver: {}", e);
                }
            }
        }

        // Set configuration to 1
        if let Err(e) = handle.set_active_configuration(1) {
            tracing::debug!("Failed to set configuration (may already be set): {}", e);
        }

        // Claim interface 0
        handle.claim_interface(0)?;

        Ok(Airspy { device: handle })
    }

    /// Get the firmware version string.
    ///
    /// # Returns
    ///
    /// A version string like "AirSpy MINI v1.0.0-rc10-0-g946184a 2016-09-19".
    pub fn version(&self) -> Result<String> {
        let mut buffer = vec![0u8; 128];

        // REQUEST_TYPE_VENDOR | RECIPIENT_DEVICE = 0xC0
        let n = self.control_in(0xC0, AIRSPY_VERSION_STRING_READ, 0, 0, &mut buffer)?;

        if n == 0 {
            return Err(Error::InvalidResponse("Version response empty".to_string()));
        }

        // Parse the null-terminated string
        let version_bytes = &buffer[..n];
        let version_str = String::from_utf8_lossy(version_bytes)
            .trim_end_matches('\0')
            .to_string();

        Ok(version_str)
    }

    /// Get the board ID.
    ///
    /// # Returns
    ///
    /// A numeric board identifier (e.g., 0 for AIRSPY, 1 for AIRSPY MINI).
    pub fn board_id(&self) -> Result<u32> {
        let mut buffer = [0u8; 4];

        // REQUEST_TYPE_VENDOR | RECIPIENT_DEVICE = 0xC0
        let n = self.control_in(0xC0, AIRSPY_BOARD_ID_READ, 0, 0, &mut buffer)?;

        tracing::debug!("board_id response length: {}", n);
        tracing::debug!("board_id buffer: {:02X?}", &buffer[..n.min(buffer.len())]);

        if n < 1 {
            return Err(Error::InvalidResponse(
                "Board ID response empty".to_string(),
            ));
        }

        // The response might be a single byte, let's handle that
        let id = if n >= 4 {
            u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]])
        } else {
            buffer[0] as u32
        };

        Ok(id)
    }

    /// Get the part IDs and serial number.
    ///
    /// # Returns
    ///
    /// A tuple of (part_id_1, part_id_2, serial_number)
    ///
    /// The serial number is constructed from 4 u32 values as returned by the device.
    /// The format matches libairspy's output (high 32 bits from serial_no[2..3], low from [0..1]).
    pub fn board_partid_serialno(&self) -> Result<(u32, u32, u64)> {
        // Structure from libairspy:
        // typedef struct {
        //     uint32_t part_id[2];   // 8 bytes
        //     uint32_t serial_no[4]; // 16 bytes
        // } airspy_read_partid_serialno_t;
        // Total: 24 bytes
        let mut buffer = [0u8; 24];

        // REQUEST_TYPE_VENDOR | RECIPIENT_DEVICE = 0xC0
        // This is command AIRSPY_BOARD_PARTID_SERIALNO_READ = 11
        match self.control_in(0xC0, AIRSPY_BOARD_PARTID_SERIALNO_READ, 0, 0, &mut buffer) {
            Ok(n) => {
                tracing::debug!("board_partid_serialno response length: {}", n);
                tracing::debug!(
                    "board_partid_serialno buffer: {:02X?}",
                    &buffer[..n.min(buffer.len())]
                );

                if n < 24 {
                    return Err(Error::InvalidResponse(format!(
                        "Board partid/serialno response incomplete: got {} bytes, expected 24",
                        n
                    )));
                }

                let part_id_1 = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
                let part_id_2 = u32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]);

                // Serial number is 4 u32 values. Looking at libairspy's airspy_info output,
                // the serial is displayed as a single 64-bit hex value.
                // The format appears to be: (serial_no[2] << 32) | serial_no[3] for high bits,
                // but we need to check the actual layout.
                // From the C code, it just reads the struct and prints it.
                // Let's combine serial_no[2] and serial_no[3] as high/low parts of a 64-bit serial.
                let serial_no_2 =
                    u32::from_le_bytes([buffer[16], buffer[17], buffer[18], buffer[19]]);
                let serial_no_3 =
                    u32::from_le_bytes([buffer[20], buffer[21], buffer[22], buffer[23]]);

                // Combine into 64-bit serial (serial_no[2] is high, serial_no[3] is low)
                let serial_no = ((serial_no_2 as u64) << 32) | (serial_no_3 as u64);

                Ok((part_id_1, part_id_2, serial_no))
            }
            Err(e) => {
                // The partid_serialno command may not be supported on all devices
                // or may have different behavior, so we provide a more informative error
                tracing::debug!(
                    "Board partid/serialno command failed (may not be supported): {}",
                    e
                );
                Err(e)
            }
        }
    }

    /// Get supported sample rates.
    ///
    /// # Returns
    ///
    /// A vector of sample rates in Hz (e.g., [6000000, 3000000])
    pub fn supported_sample_rates(&self) -> Result<Vec<u32>> {
        // According to libairspy, we need to:
        // 1. First call with wIndex=0 to get the count
        // 2. Then call with wIndex=count to get all rates

        // Step 1: Get the count
        let mut count_buffer = [0u32; 1];
        let n = self.control_in_u32(0xC0, AIRSPY_GET_SAMPLERATES, 0, 0, &mut count_buffer)?;

        if n < 1 {
            return Err(Error::InvalidResponse(
                "Sample rates count response empty".to_string(),
            ));
        }

        let count = count_buffer[0] as usize;
        tracing::debug!("supported_sample_rates count: {}", count);

        if count == 0 {
            return Ok(Vec::new());
        }

        if count > 16 {
            return Err(Error::InvalidResponse(format!(
                "Sample rates count too large: {}",
                count
            )));
        }

        // Step 2: Get all the rates
        let mut rates_buffer = vec![0u32; count];
        let n = self.control_in_u32_slice(
            0xC0,
            AIRSPY_GET_SAMPLERATES,
            0,
            count as u16,
            &mut rates_buffer,
        )?;

        tracing::debug!("supported_sample_rates response length: {}", n);

        if n < count {
            return Err(Error::InvalidResponse(format!(
                "Sample rates response incomplete: expected {} rates, got {}",
                count, n
            )));
        }

        Ok(rates_buffer)
    }

    // ========================================================================
    // Configuration API
    // ========================================================================

    /// Set the tuner frequency in Hz.
    ///
    /// # Arguments
    ///
    /// * `freq_hz` - Frequency in Hz (e.g., 100_000_000 for 100 MHz)
    ///
    /// # Returns
    ///
    /// * `Ok(())` on success
    /// * `Err(Error::ControlTransferFailed)` if the command fails
    ///
    /// # Notes
    ///
    /// The Airspy Mini supports 24 MHz to 1.8 GHz.
    /// The frequency is sent as a little-endian u32 in the data payload.
    pub fn set_freq(&self, freq_hz: u32) -> Result<()> {
        // libairspy sends freq_hz as a 4-byte little-endian payload
        let freq_bytes = freq_hz.to_le_bytes();

        // REQUEST_TYPE_VENDOR | RECIPIENT_DEVICE | OUT = 0x40
        let n = self.control_out(0x40, AIRSPY_SET_FREQ, 0, 0, &freq_bytes)?;

        if n < 4 {
            return Err(Error::ControlTransferFailed(format!(
                "SET_FREQ: expected to write 4 bytes, wrote {}",
                n
            )));
        }

        tracing::debug!("Set frequency to {} Hz", freq_hz);
        Ok(())
    }

    /// Set the sample rate by index.
    ///
    /// # Arguments
    ///
    /// * `rate_index` - Index into the supported sample rates list (0 = first rate)
    ///
    /// Use `supported_sample_rates()` to get the list of available rates.
    ///
    /// # Notes
    ///
    /// libairspy also supports setting sample rate by value (if >= 1_000_000),
    /// but this implementation uses index-based selection for simplicity.
    ///
    /// **Important:** This command may fail with "Pipe error" if called before
    /// streaming is started. In libairspy, sample rate is typically set as part
    /// of the streaming initialization sequence. Use `start_rx()` to begin
    /// streaming, which properly initializes the USB endpoints.
    pub fn set_sample_rate(&self, rate_index: u8) -> Result<()> {
        // libairspy calls libusb_clear_halt before setting sample rate.
        // This requires the endpoint to be valid, which may not be the case
        // before streaming is started. We attempt the command anyway.

        let mut retval = [0u8; 1];

        // REQUEST_TYPE_VENDOR | RECIPIENT_DEVICE | IN = 0xC0
        // wIndex contains the sample rate index
        let n = self.control_in(
            0xC0,
            AIRSPY_SET_SAMPLERATE,
            0,
            rate_index as u16,
            &mut retval,
        )?;

        if n < 1 {
            return Err(Error::ControlTransferFailed(
                "SET_SAMPLERATE: no response".to_string(),
            ));
        }

        tracing::debug!("Set sample rate index to {}", rate_index);
        Ok(())
    }

    /// Set the LNA (Low Noise Amplifier) gain.
    ///
    /// # Arguments
    ///
    /// * `gain` - Gain value 0-14 (clamped if out of range)
    ///
    /// Higher values = more gain. The LNA is the first amplifier in the signal chain.
    pub fn set_lna_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min(14); // Clamp to valid range
        let mut retval = [0u8; 1];

        // wIndex contains the gain value
        let n = self.control_in(0xC0, AIRSPY_SET_LNA_GAIN, 0, gain as u16, &mut retval)?;

        if n < 1 {
            return Err(Error::ControlTransferFailed(
                "SET_LNA_GAIN: no response".to_string(),
            ));
        }

        tracing::debug!("Set LNA gain to {}", gain);
        Ok(())
    }

    /// Set the mixer gain.
    ///
    /// # Arguments
    ///
    /// * `gain` - Gain value 0-15 (clamped if out of range)
    ///
    /// The mixer amplifies the signal after downconversion.
    pub fn set_mixer_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min(15); // Clamp to valid range
        let mut retval = [0u8; 1];

        let n = self.control_in(0xC0, AIRSPY_SET_MIXER_GAIN, 0, gain as u16, &mut retval)?;

        if n < 1 {
            return Err(Error::ControlTransferFailed(
                "SET_MIXER_GAIN: no response".to_string(),
            ));
        }

        tracing::debug!("Set mixer gain to {}", gain);
        Ok(())
    }

    /// Set the VGA (Variable Gain Amplifier) gain.
    ///
    /// # Arguments
    ///
    /// * `gain` - Gain value 0-15 (clamped if out of range)
    ///
    /// The VGA is the final amplifier before the ADC.
    pub fn set_vga_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min(15); // Clamp to valid range
        let mut retval = [0u8; 1];

        let n = self.control_in(0xC0, AIRSPY_SET_VGA_GAIN, 0, gain as u16, &mut retval)?;

        if n < 1 {
            return Err(Error::ControlTransferFailed(
                "SET_VGA_GAIN: no response".to_string(),
            ));
        }

        tracing::debug!("Set VGA gain to {}", gain);
        Ok(())
    }

    /// Enable or disable LNA AGC (Automatic Gain Control).
    ///
    /// # Arguments
    ///
    /// * `enabled` - true to enable AGC, false to disable
    ///
    /// When AGC is enabled, the hardware automatically adjusts LNA gain.
    /// Disable AGC when using manual gain control.
    pub fn set_lna_agc(&self, enabled: bool) -> Result<()> {
        let mut retval = [0u8; 1];
        let value = if enabled { 1u16 } else { 0u16 };

        let n = self.control_in(0xC0, AIRSPY_SET_LNA_AGC, 0, value, &mut retval)?;

        if n < 1 {
            return Err(Error::ControlTransferFailed(
                "SET_LNA_AGC: no response".to_string(),
            ));
        }

        tracing::debug!("Set LNA AGC to {}", enabled);
        Ok(())
    }

    /// Enable or disable mixer AGC (Automatic Gain Control).
    ///
    /// # Arguments
    ///
    /// * `enabled` - true to enable AGC, false to disable
    pub fn set_mixer_agc(&self, enabled: bool) -> Result<()> {
        let mut retval = [0u8; 1];
        let value = if enabled { 1u16 } else { 0u16 };

        let n = self.control_in(0xC0, AIRSPY_SET_MIXER_AGC, 0, value, &mut retval)?;

        if n < 1 {
            return Err(Error::ControlTransferFailed(
                "SET_MIXER_AGC: no response".to_string(),
            ));
        }

        tracing::debug!("Set mixer AGC to {}", enabled);
        Ok(())
    }

    /// Set linearity-optimized gain using a preset value.
    ///
    /// # Arguments
    ///
    /// * `gain` - Gain level 0-21 (0 = minimum, 21 = maximum)
    ///
    /// This sets LNA, mixer, and VGA gains using lookup tables optimized
    /// for linear response (low distortion). Good for strong signals.
    pub fn set_linearity_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min((GAIN_COUNT - 1) as u8);
        // libairspy inverts the index: GAIN_COUNT - 1 - value
        let index = GAIN_COUNT - 1 - gain as usize;

        // Disable AGC first
        self.set_mixer_agc(false)?;
        self.set_lna_agc(false)?;

        // Set individual gains from lookup table
        self.set_vga_gain(LINEARITY_VGA_GAINS[index])?;
        self.set_mixer_gain(LINEARITY_MIXER_GAINS[index])?;
        self.set_lna_gain(LINEARITY_LNA_GAINS[index])?;

        tracing::debug!("Set linearity gain to {} (index {})", gain, index);
        Ok(())
    }

    /// Set sensitivity-optimized gain using a preset value.
    ///
    /// # Arguments
    ///
    /// * `gain` - Gain level 0-21 (0 = minimum, 21 = maximum)
    ///
    /// This sets LNA, mixer, and VGA gains using lookup tables optimized
    /// for sensitivity (low noise). Good for weak signals.
    pub fn set_sensitivity_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min((GAIN_COUNT - 1) as u8);
        let index = GAIN_COUNT - 1 - gain as usize;

        // Disable AGC first
        self.set_mixer_agc(false)?;
        self.set_lna_agc(false)?;

        // Set individual gains from lookup table
        self.set_vga_gain(SENSITIVITY_VGA_GAINS[index])?;
        self.set_mixer_gain(SENSITIVITY_MIXER_GAINS[index])?;
        self.set_lna_gain(SENSITIVITY_LNA_GAINS[index])?;

        tracing::debug!("Set sensitivity gain to {} (index {})", gain, index);
        Ok(())
    }

    /// Enable or disable RF bias (Bias-T).
    ///
    /// # Arguments
    ///
    /// * `enabled` - true to enable Bias-T (DC on antenna port), false to disable
    ///
    /// # Warning
    ///
    /// Enabling Bias-T puts DC voltage on the antenna connector.
    /// Only use with devices that require DC power (like some LNAs or active antennas).
    /// **Do not enable with passive antennas or devices that don't expect DC!**
    pub fn set_rf_bias(&self, enabled: bool) -> Result<()> {
        // RF bias is controlled via GPIO: PORT1, PIN13
        let value = if enabled { 1u8 } else { 0u8 };

        // GPIO port/pin encoding: (port << 5) | pin
        let port_pin = ((GPIO_PORT1 as u16) << 5) | (GPIO_PIN13 as u16);

        // REQUEST_TYPE_VENDOR | RECIPIENT_DEVICE | OUT = 0x40
        // wValue = GPIO value (0 or 1)
        // wIndex = port_pin encoding
        let n = self.control_out(0x40, AIRSPY_GPIO_WRITE, value as u16, port_pin, &[])?;

        // GPIO write returns 0 bytes on success
        if n != 0 {
            tracing::warn!("GPIO_WRITE returned {} bytes (expected 0)", n);
        }

        tracing::debug!("Set RF bias (Bias-T) to {}", enabled);
        Ok(())
    }

    /// Enable or disable sample packing.
    ///
    /// # Arguments
    ///
    /// * `enabled` - true to enable 12-bit packing, false for 16-bit samples
    ///
    /// When packing is enabled, samples are packed as 12-bit values to
    /// reduce USB bandwidth. This increases maximum sample rate but requires
    /// unpacking on the host side.
    ///
    /// # Notes
    ///
    /// Should not be called while streaming is active.
    pub fn set_packing(&self, enabled: bool) -> Result<()> {
        let mut retval = [0u8; 1];
        let value = if enabled { 1u16 } else { 0u16 };

        let n = self.control_in(0xC0, AIRSPY_SET_PACKING, 0, value, &mut retval)?;

        if n < 1 {
            return Err(Error::ControlTransferFailed(
                "SET_PACKING: no response".to_string(),
            ));
        }

        tracing::debug!("Set packing to {}", enabled);
        Ok(())
    }

    // ========================================================================
    // Streaming API
    // ========================================================================

    /// Start receiving samples from the device.
    ///
    /// This enables the receiver and prepares the device for bulk transfers.
    /// After calling this, use `read_sync()` to read samples.
    ///
    /// # Notes
    ///
    /// The streaming sequence follows libairspy:
    /// 1. Set receiver mode to OFF (reset state)
    /// 2. Clear any stale data on the bulk endpoint
    /// 3. Set receiver mode to RX (start streaming)
    pub fn start_rx(&self) -> Result<()> {
        // First, ensure receiver is off
        self.set_receiver_mode(RECEIVER_MODE_OFF)?;

        // Clear the bulk endpoint to remove any stale data
        // This is important for sample rate changes to take effect
        if let Err(e) = self.device.clear_halt(BULK_ENDPOINT_IN) {
            tracing::debug!("Failed to clear halt on bulk endpoint: {}", e);
            // Continue anyway - the endpoint might not have been halted
        }

        // Enable the receiver
        self.set_receiver_mode(RECEIVER_MODE_RX)?;

        tracing::debug!("Started RX streaming");
        Ok(())
    }

    /// Stop receiving samples from the device.
    ///
    /// This disables the receiver. Any in-progress bulk transfers may be aborted.
    pub fn stop_rx(&self) -> Result<()> {
        self.set_receiver_mode(RECEIVER_MODE_OFF)?;
        tracing::debug!("Stopped RX streaming");
        Ok(())
    }

    /// Check if the device is currently streaming.
    ///
    /// Note: This only checks the receiver mode, not whether data is actively
    /// being transferred. For true streaming status, track this in your application.
    pub fn is_streaming(&self) -> bool {
        // We don't have a way to query the device state directly,
        // so this would need to be tracked by the caller.
        // For now, this is a placeholder that always returns false.
        false
    }

    /// Read samples synchronously from the device.
    ///
    /// # Arguments
    ///
    /// * `buf` - Buffer to fill with raw sample data (16-bit samples unless packing is enabled)
    ///
    /// # Returns
    ///
    /// * `Ok(n)` - Number of bytes actually read
    /// * `Err(Error::StreamingError)` - If the bulk transfer fails
    ///
    /// # Notes
    ///
    /// - Call `start_rx()` before reading samples
    /// - The buffer should be at least 16KB for efficient transfers
    /// - Samples are 16-bit unsigned integers (0-4095 in upper 12 bits)
    /// - For best performance, use a buffer size that is a multiple of 512 bytes
    ///
    /// # Example
    ///
    /// ```no_run
    /// use rs_spy::Airspy;
    ///
    /// let airspy = Airspy::open_first().unwrap();
    /// airspy.set_freq(100_000_000).unwrap();
    /// airspy.set_sensitivity_gain(15).unwrap();
    /// airspy.start_rx().unwrap();
    ///
    /// let mut buf = vec![0u8; 262144]; // 256 KB buffer
    /// loop {
    ///     let n = airspy.read_sync(&mut buf).unwrap();
    ///     // Process n bytes of sample data...
    /// }
    /// ```
    pub fn read_sync(&self, buf: &mut [u8]) -> Result<usize> {
        // Bulk read timeout - longer than control transfers for large data
        let timeout = Duration::from_millis(1000);

        match self.device.read_bulk(BULK_ENDPOINT_IN, buf, timeout) {
            Ok(n) => {
                tracing::trace!("Bulk read: {} bytes", n);
                Ok(n)
            }
            Err(e) => {
                tracing::debug!("Bulk read failed: {}", e);
                Err(Error::StreamingError(e.to_string()))
            }
        }
    }

    /// Read samples synchronously with a custom timeout.
    ///
    /// # Arguments
    ///
    /// * `buf` - Buffer to fill with raw sample data
    /// * `timeout` - Maximum time to wait for data
    ///
    /// # Returns
    ///
    /// * `Ok(n)` - Number of bytes actually read
    /// * `Err(Error::StreamingError)` - If the bulk transfer fails or times out
    pub fn read_sync_timeout(&self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        match self.device.read_bulk(BULK_ENDPOINT_IN, buf, timeout) {
            Ok(n) => {
                tracing::trace!("Bulk read: {} bytes", n);
                Ok(n)
            }
            Err(e) => {
                tracing::debug!("Bulk read failed: {}", e);
                Err(Error::StreamingError(e.to_string()))
            }
        }
    }

    /// Set the receiver mode (internal command).
    ///
    /// # Arguments
    ///
    /// * `mode` - RECEIVER_MODE_OFF (0) or RECEIVER_MODE_RX (1)
    fn set_receiver_mode(&self, mode: u16) -> Result<()> {
        // REQUEST_TYPE_VENDOR | RECIPIENT_DEVICE | OUT = 0x40
        // wValue = mode, wIndex = 0, no data payload
        let result =
            self.device
                .write_control(0x40, AIRSPY_RECEIVER_MODE, mode, 0, &[], USB_TIMEOUT);

        match result {
            Ok(_) => Ok(()),
            Err(e) => {
                tracing::debug!("Set receiver mode failed: {}", e);
                Err(Error::ControlTransferFailed(format!(
                    "RECEIVER_MODE({}): {}",
                    mode, e
                )))
            }
        }
    }

    // ========================================================================
    // Private helper methods
    // ========================================================================

    /// Perform a control IN transfer.
    fn control_in(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> Result<usize> {
        match self
            .device
            .read_control(request_type, request, value, index, buf, USB_TIMEOUT)
        {
            Ok(n) => Ok(n),
            Err(e) => {
                tracing::debug!(
                    "Control IN transfer failed: req={}, val={}, idx={}, error={}",
                    request,
                    value,
                    index,
                    e
                );
                Err(Error::ControlTransferFailed(e.to_string()))
            }
        }
    }

    /// Perform a control IN transfer that returns u32 values.
    /// Returns the number of u32 elements read.
    fn control_in_u32(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        buf: &mut [u32],
    ) -> Result<usize> {
        let byte_buf = unsafe {
            std::slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut u8, std::mem::size_of_val(buf))
        };

        let n = self.control_in(request_type, request, value, index, byte_buf)?;
        Ok(n / std::mem::size_of::<u32>())
    }

    /// Perform a control IN transfer that reads multiple u32 values.
    /// Returns the number of u32 elements read.
    fn control_in_u32_slice(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        buf: &mut [u32],
    ) -> Result<usize> {
        let byte_buf = unsafe {
            std::slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut u8, std::mem::size_of_val(buf))
        };

        let n = self.control_in(request_type, request, value, index, byte_buf)?;
        Ok(n / std::mem::size_of::<u32>())
    }

    /// Perform a control OUT transfer.
    #[allow(dead_code)]
    fn control_out(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        buf: &[u8],
    ) -> Result<usize> {
        match self
            .device
            .write_control(request_type, request, value, index, buf, USB_TIMEOUT)
        {
            Ok(n) => Ok(n),
            Err(e) => {
                tracing::debug!(
                    "Control OUT transfer failed: req={}, val={}, idx={}, error={}",
                    request,
                    value,
                    index,
                    e
                );
                Err(Error::ControlTransferFailed(e.to_string()))
            }
        }
    }
}

impl Drop for Airspy {
    fn drop(&mut self) {
        // IMPORTANT: Must release the USB interface and close the device properly.
        // This matches libairspy's airspy_open_exit() cleanup sequence.
        // Without this, the device becomes unavailable until USB reset.

        // Release interface 0 before closing
        if let Err(e) = self.device.release_interface(0) {
            tracing::debug!("Failed to release USB interface: {}", e);
        }

        // Reset the device to ensure clean state for next open
        // This is more aggressive than libairspy but ensures the device
        // is in a known good state.
        if let Err(e) = self.device.reset() {
            tracing::debug!("Failed to reset USB device: {}", e);
        }

        // DeviceHandle will automatically close the device when dropped
        tracing::debug!("Airspy device cleaned up");
    }
}
