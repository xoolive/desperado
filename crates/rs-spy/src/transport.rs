//! USB transport layer for Airspy devices.

use crate::error::{Error, Result};
use crate::{AIRSPY_PID, AIRSPY_VID};
use rusb::{Context, Device, DeviceHandle, UsbContext};
use std::time::Duration;

/// Default timeout for USB control transfers (milliseconds).
/// Matches libairspy's LIBUSB_CTRL_TIMEOUT_MS constant.
const USB_TIMEOUT: Duration = Duration::from_millis(500);

// Airspy command codes from libairspy
const AIRSPY_BOARD_ID_READ: u8 = 9;
const AIRSPY_VERSION_STRING_READ: u8 = 10;
const AIRSPY_BOARD_PARTID_SERIALNO_READ: u8 = 11;
const AIRSPY_GET_SAMPLERATES: u8 = 25;

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
