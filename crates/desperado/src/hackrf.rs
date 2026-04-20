//! HackRF SDR I/Q Data Source Module
//! (requires the `hackrf` feature)
//!
//! Provides synchronous and asynchronous I/Q readers for HackRF devices,
//! using the `rs_hackrf` crate (pure-Rust nusb backend) for hardware access.
//!
//! # Sample Format
//!
//! HackRF outputs interleaved 8-bit signed I/Q samples (Cs8) directly over USB.
//! Each sample pair is `[I, Q]` where both I and Q are `i8` values (-128 to 127).

use futures::Stream;
use num_complex::Complex;

use crate::{Gain, GainElement, GainElementName, IqFormat, error};

/// Tokio-side bridge queue depth between the USB reader thread and the async consumer.
///
/// Sized to absorb bursts when the tokio event loop is busy with DSP work.
/// At 2 MSPS Cs8 = 4 MB/s, each 256 KB chunk is ~64 ms,
/// so 32 chunks ≈ 2 s of headroom.
const BRIDGE_QUEUE_DEPTH: usize = 32;

/// Default LNA gain (dB) when no gain is specified.
const DEFAULT_LNA_GAIN: u32 = 16;
/// Default VGA gain (dB) when no gain is specified.
const DEFAULT_VGA_GAIN: u32 = 20;

/// HackRF device configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct HackRfConfig {
    /// Device index (0 for first device)
    pub device_index: usize,
    /// Center frequency in Hz
    pub center_freq: u64,
    /// Sample rate in Hz
    pub sample_rate: u32,
    /// Gain configuration
    pub gain: Gain,
    /// Enable RF amplifier (+14 dB, before LNA)
    pub amp_enable: bool,
    /// Enable antenna port bias tee
    pub bias_tee: bool,
}

impl HackRfConfig {
    /// Create a new HackRF configuration.
    pub fn new(device_index: usize, center_freq: u64, sample_rate: u32, gain: Gain) -> Self {
        Self {
            device_index,
            center_freq,
            sample_rate,
            gain,
            amp_enable: false,
            bias_tee: false,
        }
    }
}

/// Control message for dynamic HackRF parameter adjustment.
///
/// Sent to the async reader's USB thread to adjust parameters in real-time.
#[derive(Debug, Clone)]
pub enum HackRfMessage {
    /// Retune to center frequency (Hz)
    Frequency(u64),
    /// Change gain
    Gain(Gain),
}

/// Open and configure a HackRF device from a `HackRfConfig`.
fn open_and_configure(config: &HackRfConfig) -> error::Result<rs_hackrf::HackRf> {
    let hackrf = rs_hackrf::HackRf::open_by_index(config.device_index)?;

    hackrf.set_sample_rate(config.sample_rate)?;
    hackrf.set_freq(config.center_freq)?;

    apply_gain(&hackrf, &config.gain)?;

    hackrf.set_amp_enable(config.amp_enable)?;
    hackrf.set_antenna_enable(config.bias_tee)?;

    Ok(hackrf)
}

/// Apply gain settings to an open HackRF device.
fn apply_gain(hackrf: &rs_hackrf::HackRf, gain: &Gain) -> error::Result<()> {
    match gain {
        Gain::Auto => {
            // HackRF has no AGC — use sensible defaults
            tracing::info!(
                "HackRF has no AGC; using default LNA={DEFAULT_LNA_GAIN} VGA={DEFAULT_VGA_GAIN}"
            );
            hackrf.set_lna_gain(DEFAULT_LNA_GAIN)?;
            hackrf.set_vga_gain(DEFAULT_VGA_GAIN)?;
        }
        Gain::Manual(db) => {
            // Split manual gain roughly 40/60 between LNA and VGA
            let total = *db as u32;
            let lna = (total * 2 / 5).min(40); // ~40% to LNA, max 40 dB
            let vga = total.saturating_sub(lna).min(62); // rest to VGA, max 62 dB
            tracing::info!(
                total,
                lna,
                vga,
                "Setting HackRF manual gain (split LNA/VGA)"
            );
            hackrf.set_lna_gain(lna)?;
            hackrf.set_vga_gain(vga)?;
        }
        Gain::Elements(elements) => {
            let mut lna_set = false;
            let mut vga_set = false;
            for GainElement { name, value_db } in elements {
                match name {
                    GainElementName::Lna => {
                        hackrf.set_lna_gain(*value_db as u32)?;
                        lna_set = true;
                    }
                    GainElementName::Vga => {
                        hackrf.set_vga_gain(*value_db as u32)?;
                        vga_set = true;
                    }
                    other => {
                        tracing::warn!(?other, "HackRF does not support gain element; ignoring");
                    }
                }
            }
            if !lna_set {
                hackrf.set_lna_gain(DEFAULT_LNA_GAIN)?;
            }
            if !vga_set {
                hackrf.set_vga_gain(DEFAULT_VGA_GAIN)?;
            }
        }
    }
    Ok(())
}

// ─── Synchronous Reader ──────────────────────────────────────────────────────

/// Synchronous HackRF I/Q Reader.
///
/// Lazily starts a background USB reader thread on the first call to `next()`.
pub struct HackRfReader {
    config: HackRfConfig,
    /// Background streaming handle (lazily initialized on first `next()` call).
    bg_rx: Option<std::sync::mpsc::Receiver<Result<Vec<u8>, String>>>,
}

impl HackRfReader {
    pub fn new(config: &HackRfConfig) -> error::Result<Self> {
        Ok(Self {
            config: config.clone(),
            bg_rx: None,
        })
    }

    fn start_reader_thread(&mut self) -> error::Result<()> {
        let (tx, rx) = std::sync::mpsc::sync_channel::<Result<Vec<u8>, String>>(64);
        let (tx_init, rx_init) = std::sync::mpsc::sync_channel::<Result<(), String>>(1);

        let config = self.config.clone();

        std::thread::Builder::new()
            .name("hackrf-sync".into())
            .spawn(move || {
                let mut hackrf = match open_and_configure(&config) {
                    Ok(dev) => dev,
                    Err(e) => {
                        let _ = tx_init.send(Err(e.to_string()));
                        return;
                    }
                };

                if let Err(e) = hackrf.start_rx() {
                    let _ = tx_init.send(Err(e.to_string()));
                    return;
                }
                let _ = tx_init.send(Ok(()));

                let mut buf = vec![0u8; rs_hackrf::RECOMMENDED_BUFFER_SIZE];
                loop {
                    match hackrf.read_sync(&mut buf) {
                        Ok(n) if n > 0 => {
                            if tx.send(Ok(buf[..n].to_vec())).is_err() {
                                break;
                            }
                        }
                        Ok(_) => continue, // 0 bytes, retry
                        Err(e) => {
                            let _ = tx.send(Err(e.to_string()));
                            break;
                        }
                    }
                }
                // stop_rx is handled by Drop on HackRf
            })
            .map_err(|e| {
                error::Error::device(format!("Failed to spawn HackRF reader thread: {e}"))
            })?;

        match rx_init.recv() {
            Ok(Ok(())) => {
                self.bg_rx = Some(rx);
                Ok(())
            }
            Ok(Err(msg)) => Err(error::Error::Other(msg)),
            Err(_) => Err(error::Error::device(
                "Failed to initialize HackRF reader thread",
            )),
        }
    }

    /// Retune the reader to a new center frequency.
    pub fn tune(&mut self, center_freq: u64) -> error::Result<()> {
        if self.bg_rx.is_none() {
            self.config.center_freq = center_freq;
        }
        Ok(())
    }

    /// Change gain setting.
    pub fn set_gain(&mut self, gain: Gain) -> error::Result<()> {
        if self.bg_rx.is_none() {
            self.config.gain = gain;
        }
        Ok(())
    }
}

impl Iterator for HackRfReader {
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
                    let samples = crate::convert_bytes_to_complex(IqFormat::Cs8, &bytes);
                    Some(Ok(samples))
                }
                Ok(Err(msg)) => Some(Err(error::Error::Other(msg))),
                Err(_) => None,
            };
        }
        None
    }
}

// ─── Asynchronous Reader ─────────────────────────────────────────────────────

/// Asynchronous HackRF I/Q Reader with dynamic control.
///
/// Uses `rs_hackrf::HackRf::into_streaming_reader()` which keeps multiple USB bulk
/// transfers in-flight simultaneously via nusb's endpoint queue, eliminating the
/// inter-transfer gap that caused FIFO overflow and sample loss with single-transfer
/// `read_sync()`.
///
/// Architecture:
/// ```text
///   USB streaming thread  ──sync_channel──▶  bridge thread  ──tokio::mpsc(32)──▶  decode loop
///   (nusb multi-transfer)                    (bytes→complex)                       (IqAsyncSource)
///     └── control commands via mpsc channel   (tune / gain)
/// ```
///
/// Control messages (tune, gain) are sent to the streaming thread via
/// `AsyncReadControlHandle`.
pub struct AsyncHackRfReader {
    /// Control handle for tune/gain on the USB streaming thread.
    control: rs_hackrf::AsyncReadControlHandle,
    /// Async receiver for the decode loop.
    samples_rx: tokio::sync::mpsc::Receiver<error::Result<Vec<Complex<f32>>>>,
}

impl AsyncHackRfReader {
    /// Create a new async HackRF reader.
    ///
    /// Opens and configures the device, then starts multi-transfer streaming.
    /// A bridge thread converts raw Cs8 IQ bytes to `Complex<f32>` and
    /// forwards them to the tokio channel.
    pub fn new(config: &HackRfConfig) -> error::Result<Self> {
        let hackrf = open_and_configure(config)?;

        let reader = hackrf
            .into_streaming_reader(0, 0)
            .map_err(|e| error::Error::device(format!("Failed to start HackRF streaming: {e}")))?;
        let control = reader.control_handle();

        let (samples_tx, samples_rx) = tokio::sync::mpsc::channel(BRIDGE_QUEUE_DEPTH);

        // Bridge thread: receives raw bytes from the USB streaming thread,
        // converts Cs8 to Complex<f32>, sends to tokio channel.
        // Exits when the consumer drops `samples_rx`.
        std::thread::Builder::new()
            .name("hackrf-bridge".into())
            .spawn(move || {
                while let Some(result) = reader.recv() {
                    match result {
                        Ok(bytes) => {
                            if bytes.is_empty() {
                                continue;
                            }
                            let samples =
                                Ok(crate::convert_bytes_to_complex(IqFormat::Cs8, &bytes));
                            if samples_tx.blocking_send(samples).is_err() {
                                break; // consumer dropped
                            }
                        }
                        Err(e) => {
                            tracing::error!("HackRF read error: {e}");
                            let _ = samples_tx.blocking_send(Err(error::Error::device(format!(
                                "HackRF read error: {e}"
                            ))));
                            break;
                        }
                    }
                }
            })
            .map_err(|e| {
                error::Error::device(format!("Failed to spawn HackRF bridge thread: {e}"))
            })?;

        Ok(Self {
            control,
            samples_rx,
        })
    }

    /// Send a control message to the USB streaming thread.
    pub fn adjust(&self, message: HackRfMessage) -> error::Result<()> {
        match &message {
            HackRfMessage::Frequency(freq) => tracing::debug!("HackRF tune -> {freq} Hz"),
            HackRfMessage::Gain(gain) => tracing::debug!("HackRF gain -> {gain:?}"),
        }
        match message {
            HackRfMessage::Frequency(freq) => self
                .control
                .tune(freq)
                .map_err(|e| error::Error::device(format!("HackRF tune failed: {e}"))),
            HackRfMessage::Gain(gain) => apply_gain_via_control(&self.control, &gain),
        }
    }

    /// Retune to a specific center frequency.
    pub fn tune(&self, center_freq: u64) -> error::Result<()> {
        self.adjust(HackRfMessage::Frequency(center_freq))
    }

    /// Change gain setting.
    pub fn set_gain(&self, gain: Gain) -> error::Result<()> {
        self.adjust(HackRfMessage::Gain(gain))
    }
}

/// Apply gain settings via the async control handle.
fn apply_gain_via_control(
    control: &rs_hackrf::AsyncReadControlHandle,
    gain: &Gain,
) -> error::Result<()> {
    match gain {
        Gain::Auto => {
            tracing::info!(
                "HackRF has no AGC; using default LNA={DEFAULT_LNA_GAIN} VGA={DEFAULT_VGA_GAIN}"
            );
            control
                .set_lna_gain(DEFAULT_LNA_GAIN)
                .map_err(|e| error::Error::device(format!("HackRF set LNA gain failed: {e}")))?;
            control
                .set_vga_gain(DEFAULT_VGA_GAIN)
                .map_err(|e| error::Error::device(format!("HackRF set VGA gain failed: {e}")))?;
        }
        Gain::Manual(db) => {
            let total = *db as u32;
            let lna = (total * 2 / 5).min(40);
            let vga = total.saturating_sub(lna).min(62);
            control
                .set_lna_gain(lna)
                .map_err(|e| error::Error::device(format!("HackRF set LNA gain failed: {e}")))?;
            control
                .set_vga_gain(vga)
                .map_err(|e| error::Error::device(format!("HackRF set VGA gain failed: {e}")))?;
        }
        Gain::Elements(elements) => {
            let mut lna_set = false;
            let mut vga_set = false;
            for GainElement { name, value_db } in elements {
                match name {
                    GainElementName::Lna => {
                        control.set_lna_gain(*value_db as u32).map_err(|e| {
                            error::Error::device(format!("HackRF set LNA gain failed: {e}"))
                        })?;
                        lna_set = true;
                    }
                    GainElementName::Vga => {
                        control.set_vga_gain(*value_db as u32).map_err(|e| {
                            error::Error::device(format!("HackRF set VGA gain failed: {e}"))
                        })?;
                        vga_set = true;
                    }
                    other => {
                        tracing::warn!(?other, "HackRF does not support gain element; ignoring");
                    }
                }
            }
            if !lna_set {
                control.set_lna_gain(DEFAULT_LNA_GAIN).map_err(|e| {
                    error::Error::device(format!("HackRF set LNA gain failed: {e}"))
                })?;
            }
            if !vga_set {
                control.set_vga_gain(DEFAULT_VGA_GAIN).map_err(|e| {
                    error::Error::device(format!("HackRF set VGA gain failed: {e}"))
                })?;
            }
        }
    }
    Ok(())
}

impl Stream for AsyncHackRfReader {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        self.samples_rx.poll_recv(cx)
    }
}
