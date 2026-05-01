/// Main RTL-SDR device driver.
///
/// This module provides the [`RtlSdr`] struct which orchestrates device
/// initialization, tuner detection, frequency/gain/sample-rate configuration,
/// and high-throughput IQ sample streaming via nusb's multi-transfer bulk I/O.
///
/// # Architecture
///
/// ```text
/// RtlSdr (public API)
///   ├── Device (USB register access, I2C bridge)
///   ├── R82xx  (tuner driver: freq, gain, bandwidth)
///   └── streaming (multi-transfer bulk endpoint queue)
/// ```
///
/// # Streaming
///
/// The key improvement over rusb-based implementations is that nusb provides
/// native multi-transfer bulk I/O through its `Endpoint` queue. Multiple USB
/// transfers are kept in-flight simultaneously, eliminating the inter-transfer
/// gap that causes RTL2832U FIFO overflow at high sample rates.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::time::Duration;

use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, In};
use tracing::{debug, info, trace, warn};

use crate::device::{self, Device};
use crate::error::{Error, Result};
use crate::tuner::{self, KNOWN_TUNERS, R82xx, TunerType};

// ── Constants ───────────────────────────────────────────────────────────────

/// Default RTL2832U crystal frequency (Hz).
pub const DEF_RTL_XTAL_FREQ: u32 = 28_800_000;

/// USB Vendor ID for RTL-SDR devices (Realtek).
pub const RTL_USB_VID: u16 = 0x0bda;

/// Bulk IN endpoint for IQ sample data.
const BULK_ENDPOINT: u8 = 0x81;

/// Default FIR filter coefficients for the RTL2832U demodulator.
/// 16 taps: first 8 are i8, last 8 are i12 (packed).
const DEFAULT_FIR: [i16; 16] = [
    -54, -36, -41, -40, -32, -14, 14, 53, 101, 156, 215, 273, 327, 372, 404, 421,
];

/// Recommended buffer size per USB transfer (bytes).
/// 16384 bytes = 16 KB, multiple of USB high-speed max packet size (512).
pub const TRANSFER_BUF_SIZE: usize = 16384;

/// Number of USB transfers to keep in-flight simultaneously.
/// 15 matches librtlsdr's `rtlsdr_read_async` default.
pub const NUM_TRANSFERS: usize = 15;

/// Recommended channel depth for the sample delivery queue.
pub const RECOMMENDED_QUEUE_DEPTH: usize = 32;

// ── Device info ─────────────────────────────────────────────────────────────

/// Metadata captured for a detected RTL-SDR device.
#[derive(Debug, Clone)]
pub struct DeviceDescriptor {
    /// Filtered RTL-SDR device index used by selector-based open paths.
    pub index: usize,
    /// USB bus identifier.
    pub bus: String,
    /// USB device address on the bus.
    pub address: u8,
    /// USB vendor ID.
    pub vendor_id: u16,
    /// USB product ID.
    pub product_id: u16,
    /// Manufacturer string (if available).
    pub manufacturer: Option<String>,
    /// Product string (if available).
    pub product: Option<String>,
    /// Serial number string (if available).
    pub serial: Option<String>,
    /// Board identity derived during enumeration.
    pub board_variant: BoardVariant,
}

/// Board identity derived from USB enumeration metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardVariant {
    Generic,
    RtlSdrBlogV4,
}

struct EnumeratedDevice {
    usb: nusb::DeviceInfo,
    descriptor: DeviceDescriptor,
}

impl EnumeratedDevice {
    fn from_usb(index: usize, usb: nusb::DeviceInfo) -> Self {
        let board_variant = classify_board_variant(usb.manufacturer_string(), usb.product_string());
        let descriptor = DeviceDescriptor {
            index,
            bus: usb.bus_id().to_string(),
            address: usb.device_address(),
            vendor_id: usb.vendor_id(),
            product_id: usb.product_id(),
            manufacturer: usb.manufacturer_string().map(str::to_owned),
            product: usb.product_string().map(str::to_owned),
            serial: usb.serial_number().map(str::to_owned),
            board_variant,
        };

        Self { usb, descriptor }
    }
}

fn is_known_rtl_device(vendor_id: u16, product_id: u16) -> bool {
    matches!((vendor_id, product_id), (RTL_USB_VID, 0x2832 | 0x2838))
}

fn classify_board_variant(manufacturer: Option<&str>, product: Option<&str>) -> BoardVariant {
    match (manufacturer, product) {
        (Some(manufacturer), Some(product))
            if manufacturer.eq_ignore_ascii_case("RTLSDRBlog")
                && product.eq_ignore_ascii_case("Blog V4") =>
        {
            BoardVariant::RtlSdrBlogV4
        }
        _ => BoardVariant::Generic,
    }
}

/// Selector used to open a specific RTL-SDR device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceId<'a> {
    /// Open by filtered RTL-SDR index.
    Index(usize),
    /// Open by exact serial number match.
    Serial(&'a str),
}

/// Collection of enumerated RTL-SDR devices.
pub struct DeviceDescriptors {
    devices: Vec<EnumeratedDevice>,
}

impl DeviceDescriptors {
    /// Enumerate all connected RTL-SDR devices.
    pub fn new() -> Result<Self> {
        let devices = nusb::list_devices()
            .wait()
            .map_err(Error::OpenFailed)?
            .filter(|device| is_known_rtl_device(device.vendor_id(), device.product_id()))
            .enumerate()
            .map(|(index, device)| EnumeratedDevice::from_usb(index, device))
            .collect();

        Ok(Self { devices })
    }

    /// Returns `true` when no supported RTL-SDR devices were found.
    pub fn is_empty(&self) -> bool {
        self.devices.is_empty()
    }

    /// Number of supported RTL-SDR devices discovered during enumeration.
    pub fn len(&self) -> usize {
        self.devices.len()
    }

    /// Iterate over the captured device descriptors.
    pub fn iter(&self) -> impl ExactSizeIterator<Item = &DeviceDescriptor> + '_ {
        self.devices.iter().map(|device| &device.descriptor)
    }

    /// Access a single captured device descriptor by selector.
    pub fn get(&self, device_id: DeviceId<'_>) -> Option<&DeviceDescriptor> {
        self.find(device_id).map(|device| &device.descriptor)
    }

    /// Open an enumerated device without re-running enumeration.
    pub fn open(&self, device_id: DeviceId<'_>) -> Result<RtlSdr> {
        let enumerated = self.find(device_id).ok_or(Error::DeviceNotFound)?;
        RtlSdr::open_enumerated(enumerated)
    }

    fn find(&self, device_id: DeviceId<'_>) -> Option<&EnumeratedDevice> {
        match device_id {
            DeviceId::Index(index) => self.devices.get(index),
            DeviceId::Serial(serial) => self
                .devices
                .iter()
                .find(|device| device.descriptor.serial.as_deref() == Some(serial)),
        }
    }
}

// ── Main RtlSdr struct ──────────────────────────────────────────────────────

/// RTL-SDR device handle.
///
/// Provides complete control over an RTL-SDR dongle including:
/// - Device initialization and tuner detection
/// - Center frequency, gain, sample rate, and bandwidth control
/// - Bias-T (phantom power) control
/// - High-throughput IQ streaming with multi-transfer USB I/O
///
/// # Example
///
/// ```no_run
/// use rs_rtl::{DeviceId, RtlSdr};
///
/// let mut sdr = RtlSdr::open(DeviceId::Index(0))?;
/// sdr.set_center_freq(100_000_000)?;  // 100 MHz
/// sdr.set_sample_rate(2_048_000)?;    // 2.048 MS/s
/// sdr.set_gain_manual(496)?;          // 49.6 dB
///
/// let reader = sdr.start_streaming()?;
/// while let Some(data) = reader.recv() {
///     // process IQ data...
/// }
/// # Ok::<(), rs_rtl::Error>(())
/// ```
pub struct RtlSdr {
    /// Low-level USB device handle.
    dev: Device,
    /// nusb Device (needed for detach/reset on drop).
    #[allow(dead_code)]
    usb_device: nusb::Device,
    /// R82xx tuner driver.
    tuner: R82xx,
    /// Current center frequency (Hz).
    center_freq: u32,
    /// Current sample rate (Hz).
    sample_rate: u32,
    /// RTL2832U crystal frequency (Hz).
    rtl_xtal_freq: u32,
    /// Tuner crystal frequency (Hz).
    tuner_xtal_freq: u32,
    /// Whether bias-T is forced on by EEPROM.
    force_bias_t: bool,
    /// Whether direct sampling is forced by EEPROM.
    force_direct_sampling: bool,
    /// Board identity derived during device enumeration.
    board_variant: BoardVariant,
}

impl RtlSdr {
    /// Open an RTL-SDR device by index (0-based).
    ///
    /// Performs full hardware initialization:
    /// 1. Opens USB device and claims interface
    /// 2. Initializes RTL2832U baseband (demod, FIR, ADC)
    /// 3. Detects tuner type (R820T or R828D) via I2C probing
    /// 4. Initializes tuner (register defaults, filter calibration)
    /// 5. Reads EEPROM for device-specific flags
    pub fn open(device_id: DeviceId<'_>) -> Result<Self> {
        info!("opening RTL-SDR device {device_id:?}");
        DeviceDescriptors::new()?.open(device_id)
    }

    fn open_enumerated(enumerated: &EnumeratedDevice) -> Result<Self> {
        let device_info = &enumerated.descriptor;

        info!(
            "found RTL-SDR: bus={}, addr={}, manufacturer={:?}, product={:?}, variant={:?}",
            device_info.bus,
            device_info.address,
            device_info.manufacturer,
            device_info.product,
            device_info.board_variant,
        );

        // Open the USB device
        let usb_device = enumerated.usb.open().wait().map_err(Error::OpenFailed)?;

        // Detach kernel driver on Linux if needed
        #[cfg(target_os = "linux")]
        {
            let _ = usb_device.detach_kernel_driver(0);
        }

        // Claim interface 0
        let iface = usb_device
            .claim_interface(0)
            .wait()
            .map_err(Error::ClaimFailed)?;
        let dev = Device::new(iface);

        let mut sdr = Self {
            dev,
            usb_device,
            tuner: R82xx::new(
                TunerType::R820T,
                tuner::R820T_I2C_ADDR,
                DEF_RTL_XTAL_FREQ,
                false,
            ),
            center_freq: 0,
            sample_rate: 0,
            rtl_xtal_freq: DEF_RTL_XTAL_FREQ,
            tuner_xtal_freq: DEF_RTL_XTAL_FREQ,
            force_bias_t: false,
            force_direct_sampling: false,
            board_variant: device_info.board_variant,
        };

        sdr.init()?;

        Ok(sdr)
    }

    /// Open the first available RTL-SDR device.
    pub fn open_first() -> Result<Self> {
        Self::open(DeviceId::Index(0))
    }

    /// Full hardware initialization sequence.
    fn init(&mut self) -> Result<()> {
        // Test write: verify USB communication works
        self.dev
            .write_reg(device::BLOCK_USB, device::USB_SYSCTL, 0x09, 1)?;

        // Initialize RTL2832U baseband
        self.init_baseband()?;

        // Enable I2C repeater for tuner communication
        self.dev.set_i2c_repeater(true)?;

        // Detect and initialize tuner
        let (tuner_type, i2c_addr) = self.search_tuner()?;

        let is_blog_v4 = matches!(self.board_variant, BoardVariant::RtlSdrBlogV4);
        // Determine tuner crystal frequency
        let tuner_xtal = if tuner_type == TunerType::R828D && !is_blog_v4 {
            tuner::XTAL_FREQ_16
        } else {
            DEF_RTL_XTAL_FREQ
        };
        self.tuner_xtal_freq = tuner_xtal;

        // Create and initialize the tuner driver
        self.tuner = R82xx::new(
            tuner_type,
            i2c_addr,
            tuner_xtal,
            is_blog_v4,
        );
        self.tuner.init(&self.dev)?;

        // Disable I2C repeater
        self.dev.set_i2c_repeater(false)?;

        // Configure demodulator for SDR mode
        // Disable Zero-IF mode
        self.dev.demod_write_reg(1, 0xb1, 0x1a, 1)?;

        // Enable in-phase ADC only
        self.dev.demod_write_reg(0, 0x08, 0x4d, 1)?;

        // Set IF frequency
        self.set_if_freq(self.tuner.if_freq())?;

        // Enable spectrum inversion
        self.dev.demod_write_reg(1, 0x15, 0x01, 1)?;

        // Read EEPROM for device flags
        match self.dev.read_eeprom() {
            Ok(eeprom) => {
                if eeprom[7] & 0x02 == 0 {
                    self.force_bias_t = true;
                    debug!("EEPROM: force bias-T enabled");
                }
                if eeprom[7] & 0x01 != 0 {
                    self.force_direct_sampling = true;
                    debug!("EEPROM: force direct sampling enabled");
                }
            }
            Err(e) => {
                debug!("failed to read EEPROM: {}, continuing", e);
            }
        }

        info!(
            "RTL-SDR initialized: tuner={:?}, variant={:?}, xtal={}Hz",
            tuner_type, self.board_variant, tuner_xtal,
        );

        Ok(())
    }

    /// Initialize the RTL2832U baseband (demodulator, ADC, FIR).
    fn init_baseband(&self) -> Result<()> {
        debug!("init_baseband");

        // Power on USB/demod
        self.dev
            .write_reg(device::BLOCK_USB, device::USB_SYSCTL, 0x09, 1)?;
        self.dev
            .write_reg(device::BLOCK_USB, device::USB_EPA_MAXPKT, 0x0002, 2)?;
        self.dev
            .write_reg(device::BLOCK_USB, device::USB_EPA_CTL, 0x1002, 2)?;

        // Power on demodulator
        self.dev
            .write_reg(device::BLOCK_SYS, device::DEMOD_CTL_1, 0x22, 1)?;
        self.dev
            .write_reg(device::BLOCK_SYS, device::DEMOD_CTL, 0xe8, 1)?;

        // Reset demodulator (set then clear soft_rst)
        self.dev.demod_write_reg(1, 0x01, 0x14, 1)?;
        self.dev.demod_write_reg(1, 0x01, 0x10, 1)?;

        // Disable spectrum inversion (will be set later)
        self.dev.demod_write_reg(1, 0x15, 0x00, 1)?;

        // Adjust channel rejection
        self.dev.demod_write_reg(1, 0x16, 0x00, 2)?;

        // Clear DDC shift / IF registers (5 regs: 0x16-0x1a)
        for i in 0..5 {
            self.dev.demod_write_reg(1, 0x16 + i, 0x00, 1)?;
        }

        // Set FIR filter coefficients
        self.set_fir(&DEFAULT_FIR)?;

        // Enable SDR mode, disable DAGC
        self.dev.demod_write_reg(0, 0x19, 0x05, 1)?;

        // Init FSM state
        self.dev.demod_write_reg(1, 0x93, 0xf0, 1)?;
        self.dev.demod_write_reg(1, 0x94, 0x0f, 1)?;

        // Disable AGC
        self.dev.demod_write_reg(1, 0x11, 0x00, 1)?;
        self.dev.demod_write_reg(1, 0x04, 0x00, 1)?;

        // Disable PID filter
        self.dev.demod_write_reg(0, 0x61, 0x60, 1)?;

        // Default ADC I/Q datapath
        self.dev.demod_write_reg(0, 0x06, 0x80, 1)?;

        // Enable Zero-IF, DC cancel, IQ comp
        self.dev.demod_write_reg(1, 0xb1, 0x1b, 1)?;

        // Disable 4.096 MHz clock output
        self.dev.demod_write_reg(0, 0x0d, 0x83, 1)?;

        Ok(())
    }

    /// Write FIR filter coefficients to the demodulator.
    ///
    /// First 8 coefficients are i8, last 8 are i12 packed as 12 bytes
    /// (2 coefficients per 3 bytes).
    fn set_fir(&self, fir: &[i16; 16]) -> Result<()> {
        let mut buf = [0u8; 20];

        // First 8 coefficients: direct i8
        for i in 0..8 {
            buf[i] = fir[i] as i8 as u8;
        }

        // Last 8 coefficients: packed i12 (2 per 3 bytes)
        for i in 0..4 {
            let val0 = fir[8 + 2 * i] as u16;
            let val1 = fir[8 + 2 * i + 1] as u16;

            buf[8 + 3 * i] = (val0 >> 4) as u8;
            buf[8 + 3 * i + 1] = ((val0 & 0x0f) << 4 | (val1 >> 8) & 0x0f) as u8;
            buf[8 + 3 * i + 2] = val1 as u8;
        }

        // Write to demod registers starting at page 1, addr 0x1c
        for (i, &byte) in buf.iter().enumerate() {
            self.dev
                .demod_write_reg(1, 0x1c + i as u16, byte as u16, 1)?;
        }

        Ok(())
    }

    pub fn board_variant(&self) -> BoardVariant {
        self.board_variant
    }

    /// Search for a supported tuner on the I2C bus.
    ///
    /// Probes known I2C addresses (R820T at 0x34, R828D at 0x74) by reading
    /// register 0x00. A return value of 0x69 identifies the R82xx silicon.
    fn search_tuner(&self) -> Result<(TunerType, u8)> {
        for &(tuner_type, addr) in KNOWN_TUNERS {
            match self.dev.i2c_read_reg(addr, 0x00) {
                Ok(val) => {
                    debug!("I2C probe addr=0x{:02x}: got 0x{:02x}", addr, val);
                    if val == 0x69 {
                        info!("found tuner {:?} at I2C address 0x{:02x}", tuner_type, addr);
                        return Ok((tuner_type, addr));
                    }
                }
                Err(e) => {
                    trace!("I2C probe addr=0x{:02x}: error {}", addr, e);
                }
            }
        }

        Err(Error::TunerNotFound)
    }

    // ── Configuration API ───────────────────────────────────────────────────

    /// Get the detected tuner type.
    pub fn tuner_type(&self) -> TunerType {
        self.tuner.tuner_type()
    }

    /// Get the current center frequency (Hz).
    pub fn center_freq(&self) -> u32 {
        self.center_freq
    }

    /// Get the current sample rate (Hz).
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Set the center frequency in Hz.
    ///
    /// Configures the tuner PLL and RF frontend filters. Valid range depends
    /// on the tuner: typically 24 MHz to 1766 MHz for R820T, with Blog V4
    /// supporting down to ~500 kHz via upconversion.
    pub fn set_center_freq(&mut self, freq: u32) -> Result<()> {
        self.dev.set_i2c_repeater(true)?;
        self.tuner.set_freq(&self.dev, freq)?;
        self.dev.set_i2c_repeater(false)?;

        // Update IF in case bandwidth changed it
        self.set_if_freq(self.tuner.if_freq())?;

        self.center_freq = freq;
        debug!("center_freq set to {} Hz", freq);
        Ok(())
    }

    /// Set the sample rate in Hz.
    ///
    /// Valid ranges: 225,001–300,000 Hz and 900,001–3,200,000 Hz.
    /// The actual sample rate may differ slightly due to integer division;
    /// use [`Self::sample_rate`] to read back the actual rate.
    ///
    /// This also updates the tuner bandwidth and IF frequency, matching
    /// the behaviour of librtlsdr's `rtlsdr_set_sample_rate`.
    pub fn set_sample_rate(&mut self, rate: u32) -> Result<()> {
        if !(225_001..=300_000).contains(&rate) && !(900_001..=3_200_000).contains(&rate) {
            return Err(Error::InvalidSampleRate { rate });
        }

        // Calculate resampling ratio
        // rsamp_ratio = (rtl_xtal * 2^22) / rate, masked to 28 bits
        let rsamp_ratio =
            ((self.rtl_xtal_freq as u64 * (1u64 << 22)) / rate as u64) as u32 & 0x0fff_fffc;

        // Compute actual rate
        let real_ratio = rsamp_ratio | ((rsamp_ratio & 0x0800_0000) << 1); // sign extension
        let actual_rate = ((self.rtl_xtal_freq as u64 * (1u64 << 22)) / real_ratio as u64) as u32;

        debug!(
            "set_sample_rate: requested={}Hz, actual={}Hz, ratio=0x{:08x}",
            rate, actual_rate, rsamp_ratio
        );

        // Save actual rate BEFORE configuring tuner (tuner needs it for bandwidth calc)
        self.sample_rate = actual_rate;

        // Update tuner bandwidth and IF frequency.
        // librtlsdr does: bw = (self.bw > 0) ? self.bw : self.rate
        // We don't track a separate bw field, so always use the sample rate.
        self.dev.set_i2c_repeater(true)?;
        let if_freq = self
            .tuner
            .set_bandwidth(&self.dev, actual_rate, actual_rate)?;
        self.dev.set_i2c_repeater(false)?;

        // Update IF frequency in demod
        self.set_if_freq(if_freq)?;

        // Re-set center frequency (IF may have changed)
        if self.center_freq != 0 {
            self.set_center_freq(self.center_freq)?;
        }

        // Write resampling ratio to demodulator
        self.dev
            .demod_write_reg(1, 0x9f, (rsamp_ratio >> 16) as u16, 2)?;
        self.dev
            .demod_write_reg(1, 0xa1, (rsamp_ratio & 0xffff) as u16, 2)?;

        // Reset demodulator after rate change
        self.dev.demod_write_reg(1, 0x01, 0x14, 1)?;
        self.dev.demod_write_reg(1, 0x01, 0x10, 1)?;

        Ok(())
    }

    /// Set the IF frequency for the RTL2832U demodulator.
    fn set_if_freq(&self, freq: u32) -> Result<()> {
        // if_freq (signed) = -(freq * 2^22 / rtl_xtal)
        let if_freq = -((freq as i64 * (1i64 << 22)) / self.rtl_xtal_freq as i64) as i32;

        trace!("set_if_freq: {}Hz -> if_reg=0x{:06x}", freq, if_freq as u32);

        self.dev
            .demod_write_reg(1, 0x19, ((if_freq >> 16) & 0x3f) as u16, 1)?;
        self.dev
            .demod_write_reg(1, 0x1a, ((if_freq >> 8) & 0xff) as u16, 1)?;
        self.dev
            .demod_write_reg(1, 0x1b, (if_freq & 0xff) as u16, 1)?;

        Ok(())
    }

    /// Set gain to automatic (LNA + Mixer AGC).
    pub fn set_gain_auto(&mut self) -> Result<()> {
        self.dev.set_i2c_repeater(true)?;
        self.tuner.set_gain_auto(&self.dev)?;
        self.dev.set_i2c_repeater(false)?;
        Ok(())
    }

    /// Set manual gain in tenths of dB (e.g., 496 = 49.6 dB).
    ///
    /// Use [`Self::gains`] to get the list of supported values. The nearest
    /// supported value will be used.
    pub fn set_gain_manual(&mut self, gain_tenth_db: i32) -> Result<()> {
        self.dev.set_i2c_repeater(true)?;
        self.tuner.set_gain_manual(&self.dev, gain_tenth_db)?;
        self.dev.set_i2c_repeater(false)?;
        Ok(())
    }

    /// Get the list of supported gain values (in tenths of dB).
    pub fn gains(&self) -> &[i32] {
        self.tuner.gains()
    }

    /// Read the current gain from the tuner hardware (tenths of dB).
    pub fn read_gain(&self) -> Result<i32> {
        self.dev.set_i2c_repeater(true)?;
        let gain = self.tuner.read_gain(&self.dev)?;
        self.dev.set_i2c_repeater(false)?;
        Ok(gain)
    }

    /// Set the IF bandwidth in Hz.
    ///
    /// Returns the actual IF frequency used (may differ from default 3.57 MHz
    /// for narrow bandwidths). If `bw` is 0, uses automatic bandwidth based
    /// on the current sample rate.
    pub fn set_bandwidth(&mut self, bw: u32) -> Result<u32> {
        let bw = if bw == 0 { self.sample_rate } else { bw };

        self.dev.set_i2c_repeater(true)?;
        let if_freq = self.tuner.set_bandwidth(&self.dev, bw, self.sample_rate)?;
        self.dev.set_i2c_repeater(false)?;

        // Update IF frequency in demod
        self.set_if_freq(if_freq)?;

        Ok(if_freq)
    }

    /// Enable or disable bias-T (phantom power on the antenna port).
    ///
    /// Uses GPIO0 on the RTL2832U. If the EEPROM has force_bias_t set,
    /// bias-T cannot be disabled.
    pub fn set_bias_t(&mut self, enable: bool) -> Result<()> {
        let actual = enable || self.force_bias_t;
        debug!(
            "set_bias_t: requested={}, actual={} (force={})",
            enable, actual, self.force_bias_t
        );

        self.dev.set_gpio_output(0)?;
        self.dev.set_gpio_bit(0, actual)?;

        Ok(())
    }

    // ── Streaming ───────────────────────────────────────────────────────────

    /// Start streaming IQ samples.
    ///
    /// Returns an [`AsyncReadHandle`] that delivers IQ data chunks via a
    /// bounded channel. Internally, this uses nusb's multi-transfer bulk I/O
    /// to keep [`NUM_TRANSFERS`] USB transfers in-flight simultaneously,
    /// eliminating inter-transfer gaps.
    ///
    /// The streaming thread runs until the returned handle is dropped or
    /// [`AsyncReadHandle::stop()`] is called.
    pub fn start_streaming(&mut self) -> Result<AsyncReadHandle> {
        self.start_streaming_with(RECOMMENDED_QUEUE_DEPTH, TRANSFER_BUF_SIZE, NUM_TRANSFERS)
    }

    /// Start streaming with custom parameters.
    ///
    /// - `queue_depth`: bounded channel capacity for sample delivery
    /// - `transfer_size`: bytes per USB transfer (must be multiple of 512)
    /// - `num_transfers`: number of simultaneous in-flight USB transfers
    pub fn start_streaming_with(
        &mut self,
        queue_depth: usize,
        transfer_size: usize,
        num_transfers: usize,
    ) -> Result<AsyncReadHandle> {
        debug!(
            "start_streaming: queue_depth={}, transfer_size={}, num_transfers={}",
            queue_depth, transfer_size, num_transfers
        );

        // Reset endpoint before starting
        self.dev
            .write_reg(device::BLOCK_USB, device::USB_EPA_CTL, 0x1002, 2)?;
        self.dev
            .write_reg(device::BLOCK_USB, device::USB_EPA_CTL, 0x0000, 2)?;

        // Create the sample delivery channel
        let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(queue_depth);

        // Shared stop flag
        let stop = Arc::new(AtomicBool::new(false));
        let stop_clone = Arc::clone(&stop);

        // Dropped chunks counter
        let dropped = Arc::new(AtomicU64::new(0));
        let dropped_clone = Arc::clone(&dropped);

        // Control channel for runtime frequency/gain changes
        let (ctrl_tx, ctrl_rx) = mpsc::channel::<StreamControl>();

        // Clone the device for the streaming thread. The nusb Interface is
        // Clone (Arc-backed), so this is cheap and safe.
        let dev_clone = self.dev.clone();
        let tuner_clone = self.tuner.clone();
        let rtl_xtal_freq = self.rtl_xtal_freq;

        // Spawn the streaming thread
        let handle = std::thread::Builder::new()
            .name("rs-rtl-stream".to_string())
            .spawn(move || {
                streaming_thread(
                    dev_clone,
                    tuner_clone,
                    rtl_xtal_freq,
                    tx,
                    stop_clone,
                    dropped_clone,
                    ctrl_rx,
                    transfer_size,
                    num_transfers,
                );
            })
            .map_err(|e| Error::InvalidParam(format!("failed to spawn streaming thread: {}", e)))?;

        Ok(AsyncReadHandle {
            rx,
            stop,
            dropped,
            ctrl_tx: Some(ctrl_tx),
            thread: Some(handle),
        })
    }
}

impl Drop for RtlSdr {
    fn drop(&mut self) {
        debug!("RtlSdr::drop: putting tuner in standby");
        let _ = self.dev.set_i2c_repeater(true);
        let _ = self.tuner.standby(&self.dev);
        let _ = self.dev.set_i2c_repeater(false);

        // Note: nusb releases the interface automatically when all
        // Interface clones are dropped. No explicit release needed.
    }
}

// ── Streaming internals ─────────────────────────────────────────────────────

/// Commands that can be sent to the streaming thread.
enum StreamControl {
    /// Tune to a new frequency (Hz).
    Tune(u32),
    /// Set manual gain (tenths of dB).
    SetGain(i32),
    /// Enable automatic gain.
    SetGainAuto,
    /// Stop streaming.
    Stop,
}

/// Handle to an active streaming session.
///
/// IQ sample data is delivered as `Vec<u8>` chunks through the internal
/// bounded channel. Each chunk contains interleaved unsigned 8-bit I/Q
/// samples: `[I0, Q0, I1, Q1, ...]`.
///
/// Dropping this handle stops the streaming thread.
pub struct AsyncReadHandle {
    rx: mpsc::Receiver<Vec<u8>>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    ctrl_tx: Option<mpsc::Sender<StreamControl>>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl AsyncReadHandle {
    /// Receive the next chunk of IQ data (blocking).
    ///
    /// Returns `None` if the streaming thread has exited (device disconnected
    /// or stop requested).
    pub fn recv(&self) -> Option<Vec<u8>> {
        self.rx.recv().ok()
    }

    /// Try to receive the next chunk without blocking.
    pub fn try_recv(&self) -> Option<Vec<u8>> {
        self.rx.try_recv().ok()
    }

    /// Get the number of chunks dropped due to channel backpressure.
    pub fn dropped_chunks(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Request stop of the streaming thread.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(ctrl) = &self.ctrl_tx {
            let _ = ctrl.send(StreamControl::Stop);
        }
    }

    /// Get a clonable control handle for runtime configuration changes.
    pub fn control_handle(&self) -> AsyncReadControlHandle {
        AsyncReadControlHandle {
            ctrl_tx: self.ctrl_tx.as_ref().cloned().expect("control channel"),
            dropped: Arc::clone(&self.dropped),
        }
    }
}

impl Drop for AsyncReadHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        // Drop the control sender to unblock the streaming thread
        self.ctrl_tx.take();
        // Wait for the streaming thread to finish
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

/// Clonable handle for controlling an active streaming session.
///
/// Can be used from another thread to change frequency or gain while
/// streaming continues.
#[derive(Clone)]
pub struct AsyncReadControlHandle {
    ctrl_tx: mpsc::Sender<StreamControl>,
    dropped: Arc<AtomicU64>,
}

impl AsyncReadControlHandle {
    /// Request a frequency change during streaming.
    pub fn tune(&self, freq_hz: u32) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::Tune(freq_hz))
            .map_err(|_| Error::NotStreaming)
    }

    /// Request a gain change during streaming.
    pub fn set_gain(&self, gain_tenth_db: i32) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::SetGain(gain_tenth_db))
            .map_err(|_| Error::NotStreaming)
    }

    /// Request automatic gain during streaming.
    pub fn set_gain_auto(&self) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::SetGainAuto)
            .map_err(|_| Error::NotStreaming)
    }

    /// Stop the streaming session.
    pub fn stop(&self) {
        let _ = self.ctrl_tx.send(StreamControl::Stop);
    }

    /// Get the number of dropped chunks.
    pub fn dropped_chunks(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// The streaming thread function.
///
/// Uses nusb's Endpoint queue to keep multiple USB bulk transfers in-flight
/// simultaneously. This is the core innovation over rusb-based implementations:
/// there is no gap between transfers, so the RTL2832U's internal FIFO never
/// overflows.
///
/// The thread loops:
/// 1. Wait for the next completed transfer
/// 2. Send the data to the consumer via the bounded channel
/// 3. Re-submit the buffer for another transfer
///
/// Control commands (tune, gain) are processed between transfer completions.
#[allow(clippy::too_many_arguments)]
fn streaming_thread(
    dev: Device,
    mut tuner: R82xx,
    rtl_xtal_freq: u32,
    tx: mpsc::SyncSender<Vec<u8>>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    ctrl_rx: mpsc::Receiver<StreamControl>,
    transfer_size: usize,
    num_transfers: usize,
) {
    debug!(
        "streaming thread started: transfer_size={}, num_transfers={}",
        transfer_size, num_transfers
    );

    // Open the bulk IN endpoint
    let iface = dev.interface();
    let Ok(mut ep_in) = iface.endpoint::<Bulk, In>(BULK_ENDPOINT) else {
        warn!("failed to open bulk endpoint 0x{:02x}", BULK_ENDPOINT);
        return;
    };

    // Pre-fill the queue with transfers
    for _ in 0..num_transfers {
        ep_in.submit(Buffer::new(transfer_size));
    }

    debug!("submitted {} initial transfers", num_transfers);

    // Consecutive transfer error counter for disconnect detection.
    // After MAX_CONSECUTIVE_ERRORS errors in a row, we assume the dongle
    // has been disconnected and exit cleanly instead of spinning.
    const MAX_CONSECUTIVE_ERRORS: u32 = 5;
    let mut consecutive_errors: u32 = 0;

    // Main streaming loop
    while !stop.load(Ordering::Relaxed) {
        // Process any pending control commands (non-blocking)
        while let Ok(cmd) = ctrl_rx.try_recv() {
            match cmd {
                StreamControl::Stop => {
                    debug!("streaming thread: stop command received");
                    stop.store(true, Ordering::Relaxed);
                    break;
                }
                StreamControl::Tune(freq) => {
                    debug!("streaming thread: tuning to {} Hz", freq);
                    if let Err(e) = (|| -> Result<()> {
                        dev.set_i2c_repeater(true)?;
                        tuner.set_freq(&dev, freq)?;
                        dev.set_i2c_repeater(false)?;
                        // Update IF frequency in demod
                        let if_freq = tuner.if_freq();
                        let if_reg =
                            -((if_freq as i64 * (1i64 << 22)) / rtl_xtal_freq as i64) as i32;
                        dev.demod_write_reg(1, 0x19, ((if_reg >> 16) & 0x3f) as u16, 1)?;
                        dev.demod_write_reg(1, 0x1a, ((if_reg >> 8) & 0xff) as u16, 1)?;
                        dev.demod_write_reg(1, 0x1b, (if_reg & 0xff) as u16, 1)?;
                        Ok(())
                    })() {
                        warn!("runtime tune failed: {}", e);
                    }
                }
                StreamControl::SetGain(gain) => {
                    debug!("streaming thread: setting gain to {} tenths dB", gain);
                    if let Err(e) = (|| -> Result<()> {
                        dev.set_i2c_repeater(true)?;
                        tuner.set_gain_manual(&dev, gain)?;
                        dev.set_i2c_repeater(false)?;
                        Ok(())
                    })() {
                        warn!("runtime gain change failed: {}", e);
                    }
                }
                StreamControl::SetGainAuto => {
                    debug!("streaming thread: setting gain to auto");
                    if let Err(e) = (|| -> Result<()> {
                        dev.set_i2c_repeater(true)?;
                        tuner.set_gain_auto(&dev)?;
                        dev.set_i2c_repeater(false)?;
                        Ok(())
                    })() {
                        warn!("runtime auto gain failed: {}", e);
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
                trace!("streaming: transfer timeout, retrying");
                continue;
            }
        };

        // Check for transfer errors
        if let Err(ref e) = completion.status {
            consecutive_errors += 1;
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                warn!(
                    "dongle disconnected ({} consecutive transfer errors: {})",
                    consecutive_errors, e
                );
                stop.store(true, Ordering::Relaxed);
                break;
            }
            warn!(
                "bulk transfer error ({}/{}): {}",
                consecutive_errors, MAX_CONSECUTIVE_ERRORS, e
            );
            // Re-submit and continue (transient errors are common)
            ep_in.submit(Buffer::new(transfer_size));
            continue;
        }

        // Successful transfer — reset error counter
        consecutive_errors = 0;

        // Extract the data — completion.buffer is a Buffer which derefs to [u8]
        let data = completion.buffer[..completion.actual_len].to_vec();

        if data.is_empty() {
            // Empty transfer — re-submit
            ep_in.submit(Buffer::new(transfer_size));
            continue;
        }

        // Send data to the consumer (blocking — provides backpressure)
        // IMPORTANT: We use blocking send here to provide backpressure.
        // Lesson 20.1: never use try_send in real-time signal processing
        // pipelines — dropped samples corrupt protocol framing.
        match tx.send(data) {
            Ok(()) => {}
            Err(_) => {
                // Channel closed — consumer is gone
                debug!("streaming: consumer disconnected");
                break;
            }
        }

        // Re-submit a new transfer to keep the queue full
        ep_in.submit(Buffer::new(transfer_size));
    }

    // Cancel all pending transfers
    ep_in.cancel_all();

    // Drain remaining completions
    while ep_in.pending() > 0 {
        let _ = ep_in.wait_next_complete(Duration::from_millis(100));
    }

    debug!(
        "streaming thread exited (dropped {} chunks)",
        dropped.load(Ordering::Relaxed)
    );
}
