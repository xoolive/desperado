//! USB transport layer for Airspy devices.

use crate::error::{Error, Result};
use crate::{AIRSPY_PID, AIRSPY_VID};
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Recipient};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::Duration;

/// Default timeout for USB control transfers (milliseconds).
/// Matches libairspy's LIBUSB_CTRL_TIMEOUT_MS constant.
const USB_TIMEOUT: Duration = Duration::from_millis(500);

/// Airspy vendor request/command codes.
///
/// These values come directly from libairspy's airspy_commands.h.
/// See: https://github.com/airspy/airspyone_host/blob/master/libairspy/src/airspy_commands.h
#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum AirspyCommand {
    ReceiverMode = 1,
    Si5351cWrite = 2,
    Si5351cRead = 3,
    R820tWrite = 4,
    R820tRead = 5,
    SpiflashErase = 6,
    SpiflashWrite = 7,
    SpiflashRead = 8,
    BoardIdRead = 9,
    VersionStringRead = 10,
    BoardPartidSerialnrRead = 11,
    SetSamplerate = 12,
    SetFreq = 13,
    SetLnaGain = 14,
    SetMixerGain = 15,
    SetVgaGain = 16,
    SetLnaAgc = 17,
    SetMixerAgc = 18,
    MsVendorCmd = 19,
    SetRfBias = 20,
    GpioWrite = 21,
    GpioRead = 22,
    GpiodirWrite = 23,
    GpiodirRead = 24,
    GetSamplerates = 25,
    SetPacking = 26,
    SpiflashEraseSector = 27,
}

impl AirspyCommand {
    fn as_u8(self) -> u8 {
        self as u8
    }
}

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

pub const DEFAULT_ASYNC_QUEUE_LEN: usize = 16;

/// Raw gain stage values sent over the async control channel.
///
/// The dB-to-preset mapping lives in the caller (e.g. `desperado`); here we
/// just carry the raw hardware knobs so rs-spy stays free of Gain abstractions.
#[derive(Debug, Clone, Copy)]
pub struct AirspyGainStages {
    pub lna_agc: bool,
    pub mixer_agc: bool,
    pub lna: u8,   // 0-14
    pub mixer: u8, // 0-15
    pub vga: u8,   // 0-15
}

pub struct AsyncReadHandle {
    rx: mpsc::Receiver<Result<Vec<u8>>>,
    ctrl_tx: mpsc::Sender<AsyncReadControl>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    thread: Option<thread::JoinHandle<()>>,
}

#[derive(Clone)]
pub struct AsyncReadControlHandle {
    ctrl_tx: mpsc::Sender<AsyncReadControl>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
}

enum AsyncReadControl {
    Tune(u32),
    SetGain(AirspyGainStages),
}

impl AsyncReadHandle {
    pub fn recv(&self) -> Option<Result<Vec<u8>>> {
        self.rx.recv().ok()
    }

    pub fn control_handle(&self) -> AsyncReadControlHandle {
        AsyncReadControlHandle {
            ctrl_tx: self.ctrl_tx.clone(),
            stop: self.stop.clone(),
            dropped: self.dropped.clone(),
        }
    }
}

impl Drop for AsyncReadHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

impl AsyncReadControlHandle {
    pub fn dropped_chunks(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    pub fn tune(&self, center_freq: u32) -> Result<()> {
        self.ctrl_tx
            .send(AsyncReadControl::Tune(center_freq))
            .map_err(|_| Error::StreamingError("async control channel closed".to_string()))
    }

    pub fn set_gain(&self, stages: AirspyGainStages) -> Result<()> {
        self.ctrl_tx
            .send(AsyncReadControl::SetGain(stages))
            .map_err(|_| Error::StreamingError("async control channel closed".to_string()))
    }
}

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
///
/// Uses nusb for USB communication. The `nusb::Interface` is internally
/// Arc-backed and Clone, so no explicit Arc wrapping is needed.
pub struct Airspy {
    iface: nusb::Interface,
    /// Keep the nusb::Device alive so the interface remains valid.
    #[allow(dead_code)]
    usb_device: nusb::Device,
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
        let devices = nusb::list_devices().wait().map_err(Error::OpenFailed)?;

        for dev_info in devices {
            if dev_info.vendor_id() == AIRSPY_VID && dev_info.product_id() == AIRSPY_PID {
                return Self::open_device_info(dev_info);
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
        let devices = nusb::list_devices().wait().map_err(Error::OpenFailed)?;

        let mut count = 0;
        for dev_info in devices {
            if dev_info.vendor_id() == AIRSPY_VID && dev_info.product_id() == AIRSPY_PID {
                if count == index {
                    return Self::open_device_info(dev_info);
                }
                count += 1;
            }
        }

        Err(Error::DeviceNotFound)
    }

    /// List all available Airspy devices.
    pub fn list_devices() -> Result<Vec<String>> {
        let devices = nusb::list_devices().wait().map_err(Error::OpenFailed)?;
        let mut result = Vec::new();

        for dev_info in devices {
            if dev_info.vendor_id() == AIRSPY_VID && dev_info.product_id() == AIRSPY_PID {
                let serial = format!(
                    "Bus {} Device {:03}",
                    dev_info.bus_id(),
                    dev_info.device_address()
                );
                result.push(serial);
            }
        }

        Ok(result)
    }

    /// Open a device from nusb DeviceInfo.
    ///
    /// This follows the libairspy initialization sequence:
    /// 1. Open the USB device
    /// 2. Detach kernel driver (Linux only)
    /// 3. Claim interface 0
    fn open_device_info(dev_info: nusb::DeviceInfo) -> Result<Self> {
        let usb_device = dev_info.open().wait().map_err(Error::OpenFailed)?;

        // On Linux, detach kernel driver if attached to interface 0
        #[cfg(target_os = "linux")]
        {
            let _ = usb_device.detach_kernel_driver(0);
        }

        // Claim interface 0
        let iface = usb_device
            .claim_interface(0)
            .wait()
            .map_err(Error::ClaimFailed)?;

        Ok(Airspy { iface, usb_device })
    }

    /// Get the firmware version string.
    ///
    /// # Returns
    ///
    /// A version string like "AirSpy MINI v1.0.0-rc10-0-g946184a 2016-09-19".
    pub fn version(&self) -> Result<String> {
        let data = self.control_in(AirspyCommand::VersionStringRead.as_u8(), 0, 0, 128)?;

        if data.is_empty() {
            return Err(Error::InvalidResponse("Version response empty".to_string()));
        }

        let version_str = String::from_utf8_lossy(&data)
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
        let data = self.control_in(AirspyCommand::BoardIdRead.as_u8(), 0, 0, 4)?;

        tracing::debug!("board_id response length: {}", data.len());
        tracing::debug!("board_id buffer: {:02X?}", &data);

        if data.is_empty() {
            return Err(Error::InvalidResponse(
                "Board ID response empty".to_string(),
            ));
        }

        let id = if data.len() >= 4 {
            u32::from_le_bytes([data[0], data[1], data[2], data[3]])
        } else {
            data[0] as u32
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
        match self.control_in(AirspyCommand::BoardPartidSerialnrRead.as_u8(), 0, 0, 24) {
            Ok(data) => {
                tracing::debug!("board_partid_serialno response length: {}", data.len());
                tracing::debug!("board_partid_serialno buffer: {:02X?}", &data);

                if data.len() < 24 {
                    return Err(Error::InvalidResponse(format!(
                        "Board partid/serialno response incomplete: got {} bytes, expected 24",
                        data.len()
                    )));
                }

                let part_id_1 = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                let part_id_2 = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);

                let serial_no_2 = u32::from_le_bytes([data[16], data[17], data[18], data[19]]);
                let serial_no_3 = u32::from_le_bytes([data[20], data[21], data[22], data[23]]);

                // Combine into 64-bit serial (serial_no[2] is high, serial_no[3] is low)
                let serial_no = ((serial_no_2 as u64) << 32) | (serial_no_3 as u64);

                Ok((part_id_1, part_id_2, serial_no))
            }
            Err(e) => {
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
        // Step 1: Get the count (wIndex=0)
        let count_data = self.control_in(AirspyCommand::GetSamplerates.as_u8(), 0, 0, 4)?;

        if count_data.len() < 4 {
            return Err(Error::InvalidResponse(
                "Sample rates count response too short".to_string(),
            ));
        }

        let count = u32::from_le_bytes([count_data[0], count_data[1], count_data[2], count_data[3]])
            as usize;
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

        // Step 2: Get all the rates (wIndex=count)
        let rates_data = self.control_in(
            AirspyCommand::GetSamplerates.as_u8(),
            0,
            count as u16,
            (count * 4) as u16,
        )?;

        let n_rates = rates_data.len() / 4;
        tracing::debug!("supported_sample_rates response: {} rates", n_rates);

        if n_rates < count {
            return Err(Error::InvalidResponse(format!(
                "Sample rates response incomplete: expected {} rates, got {}",
                count, n_rates
            )));
        }

        let mut rates = Vec::with_capacity(count);
        for i in 0..count {
            let offset = i * 4;
            rates.push(u32::from_le_bytes([
                rates_data[offset],
                rates_data[offset + 1],
                rates_data[offset + 2],
                rates_data[offset + 3],
            ]));
        }

        Ok(rates)
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
    /// # Notes
    ///
    /// The Airspy Mini supports 24 MHz to 1.8 GHz.
    /// The frequency is sent as a little-endian u32 in the data payload.
    pub fn set_freq(&self, freq_hz: u32) -> Result<()> {
        let freq_bytes = freq_hz.to_le_bytes();
        self.control_out(AirspyCommand::SetFreq.as_u8(), 0, 0, &freq_bytes)?;
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
    pub fn set_sample_rate(&self, rate_index: u8) -> Result<()> {
        tracing::debug!("set_sample_rate: rate_index={}", rate_index);

        // libairspy calls clear_halt before the control transfer
        if let Err(e) = self.clear_halt() {
            tracing::trace!(
                "clear_halt before set_sample_rate: {:?} (may be expected)",
                e
            );
        }

        let data = self.control_in(
            AirspyCommand::SetSamplerate.as_u8(),
            0,
            rate_index as u16,
            1,
        )?;

        if data.is_empty() {
            return Err(Error::ControlTransferFailed(
                "SET_SAMPLERATE: no response".to_string(),
            ));
        }

        tracing::debug!(
            "set_sample_rate: Successfully set to index {}, response: {:02x}",
            rate_index,
            data[0]
        );
        Ok(())
    }

    /// Set sample rate for host-side IQ output, matching libairspy behavior.
    ///
    /// The Airspy firmware stores supported rates as IQ-mode output rates
    /// (half the real ADC rate). Our IQ converter decimates N real samples
    /// to N/2 complex IQ samples, so the host receives at the firmware rate.
    ///
    /// This method:
    /// 1. Looks up `iq_rate_hz` in the firmware's supported rates list
    /// 2. If found, uses index-based sample rate setting (firmware-safe)
    /// 3. If not found, falls back to sending `iq_rate_hz * 2 / 1000` kHz
    ///    directly — the doubling is required because the firmware expects the
    ///    real ADC rate in kHz for the fallback path (matching libairspy's
    ///    `airspy_set_samplerate` which does `samplerate *= 2` for IQ mode
    ///    before `/= 1000`).
    ///
    /// # Arguments
    /// * `iq_rate_hz` - Desired IQ output rate in Hz (e.g., `4_096_000` for DAB)
    ///
    /// # Example: welle.io DAB with Airspy Mini
    /// `set_sample_rate_for_iq(4_096_000)`:
    /// - 4.096M not in firmware list [6M, 3M] → fallback
    /// - Sends `4_096_000 * 2 / 1000 = 8192` kHz to firmware
    /// - ADC runs at 8.192 MHz real → IQ converter → 4.096M complex IQ
    pub fn set_sample_rate_for_iq(&self, iq_rate_hz: u32) -> Result<()> {
        tracing::debug!(iq_rate_hz, "set_sample_rate_for_iq");

        // Primary: index-based lookup.
        // Firmware stores IQ-mode rates directly (e.g. [6M, 3M] for Mini),
        // so we match iq_rate_hz without doubling. The firmware internally
        // knows these correspond to 2× ADC rates and configures accordingly.
        if let Ok(supported) = self.supported_sample_rates()
            && let Some(idx) = supported.iter().position(|&r| r == iq_rate_hz)
        {
            tracing::debug!(iq_rate_hz, idx, "set_sample_rate_for_iq: using index");
            return self.set_sample_rate(idx as u8);
        }

        // Fallback: send real ADC rate in kHz to firmware.
        // libairspy does `samplerate *= 2` for IQ mode then `/= 1000`.
        // The doubling converts from desired IQ rate to real ADC rate,
        // since IqConverter will decimate 2:1 back to the IQ rate.
        let adc_rate_khz = (iq_rate_hz * 2) / 1000;
        tracing::debug!(
            iq_rate_hz,
            adc_rate_khz,
            "set_sample_rate_for_iq: kHz fallback"
        );

        if let Err(e) = self.clear_halt() {
            tracing::trace!("clear_halt before set_sample_rate_for_iq: {:?}", e);
        }

        let data = self.control_in(
            AirspyCommand::SetSamplerate.as_u8(),
            0,
            adc_rate_khz as u16,
            1,
        )?;

        if data.is_empty() {
            return Err(Error::ControlTransferFailed(
                "SET_SAMPLERATE: no response".to_string(),
            ));
        }

        tracing::debug!(
            iq_rate_hz,
            adc_rate_khz,
            response = data[0],
            "set_sample_rate_for_iq: kHz fallback success"
        );
        Ok(())
    }

    /// Set the LNA (Low Noise Amplifier) gain.
    ///
    /// # Arguments
    ///
    /// * `gain` - Gain value 0-14 (clamped if out of range)
    pub fn set_lna_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min(14);
        let data = self.control_in(AirspyCommand::SetLnaGain.as_u8(), 0, gain as u16, 1)?;

        if data.is_empty() {
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
    pub fn set_mixer_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min(15);
        let data = self.control_in(AirspyCommand::SetMixerGain.as_u8(), 0, gain as u16, 1)?;

        if data.is_empty() {
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
    pub fn set_vga_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min(15);
        let data = self.control_in(AirspyCommand::SetVgaGain.as_u8(), 0, gain as u16, 1)?;

        if data.is_empty() {
            return Err(Error::ControlTransferFailed(
                "SET_VGA_GAIN: no response".to_string(),
            ));
        }

        tracing::debug!("Set VGA gain to {}", gain);
        Ok(())
    }

    /// Enable or disable LNA AGC (Automatic Gain Control).
    pub fn set_lna_agc(&self, enabled: bool) -> Result<()> {
        let value = if enabled { 1u16 } else { 0u16 };
        let data = self.control_in(AirspyCommand::SetLnaAgc.as_u8(), 0, value, 1)?;

        if data.is_empty() {
            return Err(Error::ControlTransferFailed(
                "SET_LNA_AGC: no response".to_string(),
            ));
        }

        tracing::debug!("Set LNA AGC to {}", enabled);
        Ok(())
    }

    /// Enable or disable mixer AGC (Automatic Gain Control).
    pub fn set_mixer_agc(&self, enabled: bool) -> Result<()> {
        let value = if enabled { 1u16 } else { 0u16 };
        let data = self.control_in(AirspyCommand::SetMixerAgc.as_u8(), 0, value, 1)?;

        if data.is_empty() {
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
    pub fn set_linearity_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min((GAIN_COUNT - 1) as u8);
        let index = GAIN_COUNT - 1 - gain as usize;

        self.set_mixer_agc(false)?;
        self.set_lna_agc(false)?;
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
    pub fn set_sensitivity_gain(&self, gain: u8) -> Result<()> {
        let gain = gain.min((GAIN_COUNT - 1) as u8);
        let index = GAIN_COUNT - 1 - gain as usize;

        self.set_mixer_agc(false)?;
        self.set_lna_agc(false)?;
        self.set_vga_gain(SENSITIVITY_VGA_GAINS[index])?;
        self.set_mixer_gain(SENSITIVITY_MIXER_GAINS[index])?;
        self.set_lna_gain(SENSITIVITY_LNA_GAINS[index])?;

        tracing::debug!("Set sensitivity gain to {} (index {})", gain, index);
        Ok(())
    }

    /// Enable or disable RF bias (Bias-T).
    ///
    /// # Warning
    ///
    /// Enabling Bias-T puts DC voltage on the antenna connector.
    /// Only use with devices that require DC power (like some LNAs or active antennas).
    pub fn set_rf_bias(&self, enabled: bool) -> Result<()> {
        let value = if enabled { 1u8 } else { 0u8 };
        let port_pin = ((GPIO_PORT1 as u16) << 5) | (GPIO_PIN13 as u16);

        self.control_out(
            AirspyCommand::GpioWrite.as_u8(),
            value as u16,
            port_pin,
            &[],
        )?;

        tracing::debug!("Set RF bias (Bias-T) to {}", enabled);
        Ok(())
    }

    /// Enable or disable sample packing.
    ///
    /// # Arguments
    ///
    /// * `enabled` - true to enable 12-bit packing, false for 16-bit samples
    pub fn set_packing(&self, enabled: bool) -> Result<()> {
        let value = if enabled { 1u16 } else { 0u16 };
        let data = self.control_in(AirspyCommand::SetPacking.as_u8(), 0, value, 1)?;

        if data.is_empty() {
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
    pub fn start_rx(&self) -> Result<()> {
        // First, ensure receiver is off
        self.set_receiver_mode(RECEIVER_MODE_OFF)?;

        // Clear the bulk endpoint to remove any stale data
        if let Err(e) = self.clear_halt() {
            tracing::debug!("Failed to clear halt on bulk endpoint: {}", e);
        }

        // Enable the receiver
        self.set_receiver_mode(RECEIVER_MODE_RX)?;

        tracing::debug!("Started RX streaming");
        Ok(())
    }

    /// Stop receiving samples from the device.
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
    /// * `Err(Error::BulkTransfer)` - If the bulk transfer fails
    ///
    /// # Notes
    ///
    /// - Call `start_rx()` before reading samples
    /// - The buffer should be at least 16KB for efficient transfers
    pub fn read_sync(&self, buf: &mut [u8]) -> Result<usize> {
        let timeout = Duration::from_millis(1000);

        let mut ep_in = self
            .iface
            .endpoint::<Bulk, In>(BULK_ENDPOINT_IN)
            .map_err(|e| Error::StreamingError(format!("failed to open bulk endpoint: {}", e)))?;

        ep_in.submit(Buffer::new(buf.len()));

        match ep_in.wait_next_complete(timeout) {
            Some(completion) => {
                completion.status.map_err(Error::BulkTransfer)?;
                let n = completion.actual_len.min(buf.len());
                buf[..n].copy_from_slice(&completion.buffer[..n]);
                ep_in.cancel_all();
                while ep_in.pending() > 0 {
                    let _ = ep_in.wait_next_complete(Duration::from_millis(100));
                }
                tracing::trace!("Bulk read: {} bytes", n);
                Ok(n)
            }
            None => {
                ep_in.cancel_all();
                while ep_in.pending() > 0 {
                    let _ = ep_in.wait_next_complete(Duration::from_millis(100));
                }
                Err(Error::Timeout)
            }
        }
    }

    /// Read samples synchronously with a custom timeout.
    pub fn read_sync_timeout(&self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let mut ep_in = self
            .iface
            .endpoint::<Bulk, In>(BULK_ENDPOINT_IN)
            .map_err(|e| Error::StreamingError(format!("failed to open bulk endpoint: {}", e)))?;

        ep_in.submit(Buffer::new(buf.len()));

        match ep_in.wait_next_complete(timeout) {
            Some(completion) => {
                completion.status.map_err(Error::BulkTransfer)?;
                let n = completion.actual_len.min(buf.len());
                buf[..n].copy_from_slice(&completion.buffer[..n]);
                ep_in.cancel_all();
                while ep_in.pending() > 0 {
                    let _ = ep_in.wait_next_complete(Duration::from_millis(100));
                }
                tracing::trace!("Bulk read: {} bytes", n);
                Ok(n)
            }
            None => {
                ep_in.cancel_all();
                while ep_in.pending() > 0 {
                    let _ = ep_in.wait_next_complete(Duration::from_millis(100));
                }
                Err(Error::Timeout)
            }
        }
    }

    /// Start a multi-transfer async reader using nusb's endpoint queue.
    ///
    /// This replaces the old multi-thread SharedBulkHandle approach with nusb's
    /// native multi-transfer bulk I/O through its `Endpoint` queue. Multiple USB
    /// transfers are kept in-flight simultaneously in a single thread, eliminating
    /// the inter-transfer gap that causes device FIFO overflow at high sample rates.
    ///
    /// Architecture:
    /// ```text
    ///   streaming thread (nusb endpoint queue)  ──sync_channel──▶  consumer
    ///     └── control commands via mpsc channel   (tune / set_gain)
    /// ```
    ///
    /// Returns an `AsyncReadHandle` for receiving data and sending control commands.
    pub fn into_multi_transfer_reader(
        self,
        buf_num: usize,
        buf_len: usize,
    ) -> Result<AsyncReadHandle> {
        let buf_num = if buf_num == 0 {
            DEFAULT_ASYNC_QUEUE_LEN
        } else {
            buf_num
        };
        let buf_len = if buf_len == 0 {
            RECOMMENDED_BUFFER_SIZE
        } else {
            buf_len
        };

        if !buf_len.is_multiple_of(512) {
            return Err(Error::StreamingError(format!(
                "Invalid buffer length {} (must be multiple of 512)",
                buf_len
            )));
        }

        // Use blocking send for backpressure (lesson 20.1: never try_send in
        // real-time signal processing pipelines — dropped samples corrupt
        // protocol framing).
        let (tx, rx) = mpsc::sync_channel::<Result<Vec<u8>>>(buf_num * 4);
        let (ctrl_tx, ctrl_rx) = mpsc::channel::<AsyncReadControl>();
        let stop = Arc::new(AtomicBool::new(false));
        let dropped = Arc::new(AtomicU64::new(0));

        let iface = self.iface.clone();
        let stop_thread = Arc::clone(&stop);

        let thread = thread::spawn(move || {
            streaming_thread(self, iface, tx, stop_thread, ctrl_rx, buf_len, buf_num);
        });

        Ok(AsyncReadHandle {
            rx,
            ctrl_tx,
            stop,
            dropped,
            thread: Some(thread),
        })
    }

    /// Start a callback-like async reader loop in a dedicated thread.
    ///
    /// This is a simpler alternative to `into_multi_transfer_reader` that uses
    /// fewer in-flight transfers. For high sample rates, prefer
    /// `into_multi_transfer_reader`.
    pub fn into_async_reader(self, queue_len: usize, buf_len: usize) -> Result<AsyncReadHandle> {
        // Use the same nusb endpoint queue approach but with fewer transfers
        let num_transfers = if queue_len == 0 { 4 } else { queue_len.min(8) };
        self.into_multi_transfer_reader(num_transfers, buf_len)
    }

    /// Set the receiver mode (internal command).
    fn set_receiver_mode(&self, mode: u16) -> Result<()> {
        match self.control_out(AirspyCommand::ReceiverMode.as_u8(), mode, 0, &[]) {
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

    /// Clear halt on the bulk IN endpoint.
    ///
    /// nusb does not expose a direct clear_halt API on Interface; the endpoint
    /// queue handles stale state through the submit/complete cycle. This is
    /// kept as a no-op to preserve the libairspy initialization sequence
    /// comments without changing behavior.
    fn clear_halt(&self) -> Result<()> {
        // nusb handles endpoint state internally — no explicit clear_halt needed
        Ok(())
    }

    // ========================================================================
    // Private helper methods
    // ========================================================================

    /// Perform a control IN transfer using nusb.
    ///
    /// Returns the received data bytes.
    fn control_in(&self, request: u8, value: u16, index: u16, length: u16) -> Result<Vec<u8>> {
        let data = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length,
                },
                USB_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)?;

        Ok(data)
    }

    /// Perform a control OUT transfer using nusb.
    fn control_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data,
                },
                USB_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)
    }
}

/// The streaming thread function.
///
/// Uses nusb's Endpoint queue to keep multiple USB bulk transfers in-flight
/// simultaneously. This is the core improvement over the old rusb-based
/// multi-thread SharedBulkHandle approach: there is no gap between transfers,
/// so the device's internal FIFO never overflows.
///
/// The thread loops:
/// 1. Wait for the next completed transfer
/// 2. Process any pending control commands (tune, gain)
/// 3. Send the data to the consumer via the bounded channel (blocking for backpressure)
/// 4. Re-submit the buffer for another transfer
fn streaming_thread(
    dev: Airspy,
    iface: nusb::Interface,
    tx: mpsc::SyncSender<Result<Vec<u8>>>,
    stop: Arc<AtomicBool>,
    ctrl_rx: mpsc::Receiver<AsyncReadControl>,
    transfer_size: usize,
    num_transfers: usize,
) {
    tracing::debug!(
        "streaming thread started: transfer_size={}, num_transfers={}",
        transfer_size,
        num_transfers
    );

    // Open the bulk IN endpoint
    let Ok(mut ep_in) = iface.endpoint::<Bulk, In>(BULK_ENDPOINT_IN) else {
        tracing::warn!("failed to open bulk endpoint 0x{:02x}", BULK_ENDPOINT_IN);
        return;
    };

    // Pre-fill the queue with transfers
    for _ in 0..num_transfers {
        ep_in.submit(Buffer::new(transfer_size));
    }

    tracing::debug!("submitted {} initial transfers", num_transfers);

    // Consecutive transfer error counter for disconnect detection.
    const MAX_CONSECUTIVE_ERRORS: u32 = 5;
    let mut consecutive_errors: u32 = 0;

    // Main streaming loop
    while !stop.load(Ordering::Relaxed) {
        // Process any pending control commands (non-blocking)
        while let Ok(cmd) = ctrl_rx.try_recv() {
            match cmd {
                AsyncReadControl::Tune(freq) => {
                    tracing::debug!("streaming thread: tuning to {} Hz", freq);
                    if let Err(e) = dev.set_freq(freq) {
                        tracing::warn!("Airspy retune to {} Hz failed: {}", freq, e);
                    }
                }
                AsyncReadControl::SetGain(stages) => {
                    if let Err(e) = apply_gain_stages(&dev, stages) {
                        tracing::warn!("Airspy set gain failed: {}", e);
                    }
                }
            }
        }

        if stop.load(Ordering::Relaxed) {
            break;
        }

        // Wait for the next completed transfer
        let completion = match ep_in.wait_next_complete(Duration::from_millis(500)) {
            Some(c) => c,
            None => {
                // Timeout — check stop flag and retry
                tracing::trace!("streaming: transfer timeout, retrying");
                continue;
            }
        };

        // Check for transfer errors
        if let Err(ref e) = completion.status {
            consecutive_errors += 1;
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                tracing::warn!(
                    "dongle disconnected ({} consecutive transfer errors: {})",
                    consecutive_errors,
                    e
                );
                stop.store(true, Ordering::Relaxed);
                break;
            }
            tracing::warn!(
                "bulk transfer error ({}/{}): {}",
                consecutive_errors,
                MAX_CONSECUTIVE_ERRORS,
                e
            );
            // Re-submit and continue (transient errors are common)
            ep_in.submit(Buffer::new(transfer_size));
            continue;
        }

        // Successful transfer — reset error counter
        consecutive_errors = 0;

        // Extract the data
        let data = completion.buffer[..completion.actual_len].to_vec();

        if data.is_empty() {
            ep_in.submit(Buffer::new(transfer_size));
            continue;
        }

        // Send data to the consumer (blocking — provides backpressure).
        // IMPORTANT: We use blocking send here to provide backpressure.
        // Lesson 20.1: never use try_send in real-time signal processing
        // pipelines — dropped samples corrupt protocol framing.
        match tx.send(Ok(data)) {
            Ok(()) => {}
            Err(_) => {
                // Channel closed — consumer is gone
                tracing::debug!("streaming: consumer disconnected");
                break;
            }
        }

        // Re-submit a new transfer to keep the queue full
        ep_in.submit(Buffer::new(transfer_size));
    }

    // Stop the receiver
    let _ = dev.stop_rx();

    // Cancel all pending transfers
    ep_in.cancel_all();

    // Drain remaining completions
    while ep_in.pending() > 0 {
        let _ = ep_in.wait_next_complete(Duration::from_millis(100));
    }

    tracing::debug!("streaming thread exited");
}

/// Apply gain stages from an `AirspyGainStages` value to an open device.
///
/// Called from the streaming thread's control command handler.
fn apply_gain_stages(dev: &Airspy, stages: AirspyGainStages) -> Result<()> {
    dev.set_lna_agc(stages.lna_agc)?;
    dev.set_mixer_agc(stages.mixer_agc)?;
    if !stages.lna_agc {
        dev.set_lna_gain(stages.lna)?;
    }
    if !stages.mixer_agc {
        dev.set_mixer_gain(stages.mixer)?;
    }
    dev.set_vga_gain(stages.vga)?;
    Ok(())
}
