/// Low-level RTL2832U register access and I2C bridge.
///
/// This module handles all USB control transfers to the RTL2832U chip,
/// including direct register read/write, demodulator register access,
/// and I2C communication with the tuner via the built-in I2C bridge.
///
/// # USB Control Transfer Encoding
///
/// The RTL2832U uses vendor-specific control transfers with bRequest=0.
/// Register addresses and blocks are encoded in wValue and wIndex:
///
/// - **Read**: `CTRL_IN` (0xC0), wValue=addr, wIndex=block<<8
/// - **Write**: `CTRL_OUT` (0x40), wValue=addr, wIndex=(block<<8)|0x10
/// - **Demod read**: wValue=(addr<<8)|0x20, wIndex=page
/// - **Demod write**: wValue=(addr<<8)|0x20, wIndex=0x10|page
///
/// Reads are decoded as little-endian, writes are encoded as big-endian.
use std::time::Duration;

use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use nusb::{Interface, MaybeFuture};
use tracing::{debug, trace};

use crate::error::{Error, Result};

// ── USB control transfer constants ──────────────────────────────────────────

/// Timeout for USB control transfers (matches librtlsdr).
const CTRL_TIMEOUT: Duration = Duration::from_millis(300);

// ── Block identifiers for register addressing ───────────────────────────────

/// Demodulator register block.
#[allow(dead_code)]
pub(crate) const BLOCK_DEMOD: u16 = 0;
/// USB controller register block.
pub(crate) const BLOCK_USB: u16 = 1;
/// System register block.
pub(crate) const BLOCK_SYS: u16 = 2;
/// I2C bridge block — used for tuner communication.
pub(crate) const BLOCK_IIC: u16 = 6;

// ── System and USB register addresses ───────────────────────────────────────

/// System control register.
pub const USB_SYSCTL: u16 = 0x2000;
/// Endpoint A maximum packet size.
pub const USB_EPA_MAXPKT: u16 = 0x2158;
/// Endpoint A control register.
pub const USB_EPA_CTL: u16 = 0x2148;
/// Demodulator control register.
pub const DEMOD_CTL: u16 = 0x3000;
/// GPIO output register.
pub const GPO: u16 = 0x3001;
/// GPIO output enable register.
pub const GPOE: u16 = 0x3003;
/// GPIO direction register.
pub const GPD: u16 = 0x3004;
/// Demodulator control 1 register.
pub const DEMOD_CTL_1: u16 = 0x300b;

// ── EEPROM constants ────────────────────────────────────────────────────────

/// I2C address of the EEPROM.
pub const EEPROM_ADDR: u8 = 0xa0;
/// Total EEPROM size in bytes.
pub const EEPROM_SIZE: usize = 256;

/// Low-level RTL2832U USB device handle.
///
/// Wraps a `nusb::Interface` and provides register-level access to the
/// RTL2832U demodulator, system/USB controller, and I2C bridge for tuner
/// communication. All operations are synchronous (blocking).
#[derive(Clone)]
pub struct Device {
    iface: Interface,
}

impl Device {
    /// Create a new `Device` from an already-claimed `nusb::Interface`.
    pub fn new(iface: Interface) -> Self {
        Self { iface }
    }

    /// Get a reference to the underlying nusb interface.
    pub fn interface(&self) -> &Interface {
        &self.iface
    }

    // ── Generic register access (USB/SYS blocks) ───────────────────────────

    /// Read 1 or 2 bytes from a USB/SYS register.
    ///
    /// Returns the value decoded as little-endian u16.
    pub fn read_reg(&self, block: u16, addr: u16, len: u16) -> Result<u16> {
        let data = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: addr,
                    index: block << 8,
                    length: len,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)?;

        let val = match data.len() {
            1 => data[0] as u16,
            2 => u16::from_le_bytes([data[0], data[1]]),
            n => {
                trace!("read_reg: unexpected response length {n}");
                if n >= 2 {
                    u16::from_le_bytes([data[0], data[1]])
                } else if n == 1 {
                    data[0] as u16
                } else {
                    0
                }
            }
        };

        trace!("read_reg block={block} addr=0x{addr:04x} len={len} => 0x{val:04x}");
        Ok(val)
    }

    /// Write 1 or 2 bytes to a USB/SYS register.
    ///
    /// Value is encoded as big-endian. The write bit (0x10) is set in wIndex.
    pub fn write_reg(&self, block: u16, addr: u16, val: u16, len: u8) -> Result<()> {
        let data: Vec<u8> = match len {
            1 => vec![val as u8],
            2 => vec![(val >> 8) as u8, val as u8], // big-endian
            _ => vec![(val >> 8) as u8, val as u8],
        };

        trace!("write_reg block={block} addr=0x{addr:04x} val=0x{val:04x} len={len}");

        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: addr,
                    index: (block << 8) | 0x10,
                    data: &data,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)
    }

    // ── Demodulator register access ─────────────────────────────────────────

    /// Read one byte from a demodulator register.
    ///
    /// Uses the RTL2832U demod page/address encoding:
    /// wValue = (addr << 8) | 0x20, wIndex = page.
    pub fn demod_read_reg(&self, page: u16, addr: u16) -> Result<u8> {
        let data = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: (addr << 8) | 0x20,
                    index: page,
                    length: 1,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)?;

        let val = if data.is_empty() { 0 } else { data[0] };
        trace!("demod_read_reg page={page} addr=0x{addr:02x} => 0x{val:02x}");
        Ok(val)
    }

    /// Write 1 or 2 bytes to a demodulator register.
    ///
    /// Every demod write is followed by a dummy read of page 0x0a, addr 0x01
    /// for synchronization (required by RTL2832U hardware).
    pub fn demod_write_reg(&self, page: u16, addr: u16, val: u16, len: u8) -> Result<()> {
        let data: Vec<u8> = match len {
            1 => vec![val as u8],
            2 => vec![(val >> 8) as u8, val as u8], // big-endian
            _ => vec![(val >> 8) as u8, val as u8],
        };

        trace!("demod_write_reg page={page} addr=0x{addr:02x} val=0x{val:04x} len={len}");

        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: (addr << 8) | 0x20,
                    index: 0x10 | page,
                    data: &data,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)?;

        // Dummy read for synchronization — required by the RTL2832U after
        // every demodulator write. Without this, register writes can be lost.
        let _ = self.demod_read_reg(0x0a, 0x01);

        Ok(())
    }

    // ── I2C bridge ──────────────────────────────────────────────────────────

    /// Write a buffer of bytes to an I2C device via the RTL2832U I2C bridge.
    ///
    /// The I2C bridge is accessed through BLOCK_IIC (block 6). The I2C device
    /// address is placed in wValue, and the data bytes are sent as the control
    /// transfer payload.
    pub fn i2c_write(&self, i2c_addr: u8, data: &[u8]) -> Result<()> {
        trace!("i2c_write addr=0x{i2c_addr:02x} len={}", data.len());

        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: i2c_addr as u16,
                    index: (BLOCK_IIC << 8) | 0x10,
                    data,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)
    }

    /// Read bytes from an I2C device via the RTL2832U I2C bridge.
    pub fn i2c_read(&self, i2c_addr: u8, len: u16) -> Result<Vec<u8>> {
        trace!("i2c_read addr=0x{i2c_addr:02x} len={len}");

        let data = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: i2c_addr as u16,
                    index: BLOCK_IIC << 8,
                    length: len,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)?;

        Ok(data)
    }

    /// Read a single byte from an I2C register.
    ///
    /// First writes the register address to set the I2C pointer,
    /// then reads one byte back.
    pub fn i2c_read_reg(&self, i2c_addr: u8, reg: u8) -> Result<u8> {
        // Set register pointer
        self.i2c_write(i2c_addr, &[reg])?;
        // Read one byte
        let data = self.i2c_read(i2c_addr, 1)?;
        Ok(if data.is_empty() { 0 } else { data[0] })
    }

    // ── EEPROM access ───────────────────────────────────────────────────────

    /// Read the full EEPROM contents (256 bytes).
    ///
    /// The EEPROM is at I2C address 0xA0. Read is done one byte at a time
    /// to avoid I2C length limitations.
    pub fn read_eeprom(&self) -> Result<[u8; EEPROM_SIZE]> {
        let mut buf = [0u8; EEPROM_SIZE];
        for (i, buf_entry) in buf.iter_mut().enumerate().take(EEPROM_SIZE) {
            self.i2c_write(EEPROM_ADDR, &[i as u8])?;
            let data = self.i2c_read(EEPROM_ADDR, 1)?;
            *buf_entry = if data.is_empty() { 0 } else { data[0] };
        }
        debug!("read_eeprom: {} bytes", EEPROM_SIZE);
        Ok(buf)
    }

    // ── I2C repeater (gate) control ─────────────────────────────────────────

    /// Enable or disable the I2C repeater (gate).
    ///
    /// The I2C repeater must be enabled before communicating with the tuner
    /// and disabled afterwards. This gates the I2C bus connection between
    /// the RTL2832U and the external tuner chip.
    ///
    /// - Enabled: demod reg(1, 0x01) = 0x18 (bit 3 set)
    /// - Disabled: demod reg(1, 0x01) = 0x10 (bit 3 clear)
    pub fn set_i2c_repeater(&self, enable: bool) -> Result<()> {
        let val = if enable { 0x18 } else { 0x10 };
        trace!("set_i2c_repeater enable={enable} val=0x{val:02x}");
        self.demod_write_reg(1, 0x01, val, 1)
    }

    // ── GPIO control ────────────────────────────────────────────────────────

    /// Configure a GPIO pin as output.
    ///
    /// Clears the direction bit (GPD) and sets the output enable bit (GPOE).
    pub fn set_gpio_output(&self, gpio: u8) -> Result<()> {
        let mask = 1u16 << gpio;

        // Read current direction, clear our bit (output mode)
        let gpd = self.read_reg(BLOCK_SYS, GPD, 1)?;
        self.write_reg(BLOCK_SYS, GPD, gpd & !mask, 1)?;

        // Read current output enable, set our bit
        let gpoe = self.read_reg(BLOCK_SYS, GPOE, 1)?;
        self.write_reg(BLOCK_SYS, GPOE, gpoe | mask, 1)?;

        Ok(())
    }

    /// Set or clear a GPIO output bit.
    pub fn set_gpio_bit(&self, gpio: u8, on: bool) -> Result<()> {
        let mask = 1u16 << gpio;
        let gpo = self.read_reg(BLOCK_SYS, GPO, 1)?;
        let val = if on { gpo | mask } else { gpo & !mask };
        self.write_reg(BLOCK_SYS, GPO, val, 1)
    }
}
