//! dabradio — DAB/DAB+ digital radio decoder.
//!
//! This crate receives raw I/Q samples (via [`desperado`]) from an SDR device or
//! file and decodes a DAB Mode I ensemble, extracting services, labels,
//! programme-associated data (Dynamic Label, MOT slide-show images), and
//! DAB+ HE-AAC audio.
//!
//! # Architecture
//!
//! The processing pipeline is split into the following stages:
//!
//! ```text
//! IQ stream ─► OFDM sync & FFT ─► DQPSK decode ─┬─► FIC ─► ensemble metadata
//!                                               └─► MSC ─► subchannel frames
//!                                                              │
//!                                                     FEC (Viterbi + RS)
//!                                                              │
//!                                                     AAC decode ─► audio
//!                                                              │
//!                                                     PAD ─► DLS / MOT
//! ```
//!
//! # Modules
//!
//! | Module | Purpose |
//! |---|---|
//! | `ofdm` | Frame synchronisation, FFT, DQPSK differential decoding |
//! | `fic` | Fast Information Channel — ensemble & service metadata |
//! | `msc` | Main Service Channel — subchannel extraction |
//! | `fec` | Forward Error Correction (EEP depuncturing, Viterbi, energy dispersal) |
//! | `audio` | DAB+ super-frame assembly, Reed-Solomon, AAC decoding |
//! | `pad` | Programme Associated Data (Dynamic Label Segment, MOT slide-show) |
//! | `constants` | DAB Mode I parameters and Band III channel table |
//! | `charsets` | EBU Latin → UTF-8 conversion for service labels |
//!

mod audio;
mod charsets;
mod constants;
mod dab_resampler;
mod fec;
mod fic;
mod msc;
mod ofdm;
mod pad;

use clap::Parser;
use crossbeam_channel as channel;
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use dab_resampler::DabResampler;
use desperado::DeviceConfig;
use desperado::Gain;
use desperado::IqAsyncSource;
use desperado::dsp::DspBlock;
use desperado::dsp::rotate::Rotate;
use desperado::sdr::{gain_steps_for_source, is_device_uri};
use desperado::{IqFormat, IqSource};
use futures::StreamExt;
use num_complex::Complex;
use pad::PadData;
use ratatui::{
    Terminal, TerminalOptions, Viewport,
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
};
use std::collections::hash_map::DefaultHasher;
use std::fs::File;
use std::hash::{Hash, Hasher};
use std::io::{IsTerminal, Seek, SeekFrom, Write};
use std::path::Path;
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tinyaudio::prelude::*;
use tracing::Level;
use tracing::{debug, error, info, warn};
use tracing_subscriber::prelude::*;

/// Unified IQ chunk iterator that works with both sync and async sources.
///
/// For file/stdin sources, uses synchronous `IqSource` (simple, no threading).
/// For live SDR sources, uses `IqAsyncSource` directly.
enum IqChunkSource {
    Sync(Box<IqSource>),
    Async {
        source: Box<IqAsyncSource>,
        chunk_size: usize,
        pending: Vec<Complex<f32>>,
    },
}

impl IqChunkSource {
    /// Get the next chunk of IQ samples.
    /// For sync sources, this blocks until data is available.
    /// For async sources, this awaits the background reader thread.
    async fn next_chunk(&mut self) -> Option<desperado::error::Result<Vec<Complex<f32>>>> {
        match self {
            IqChunkSource::Sync(source) => source.next(),
            IqChunkSource::Async {
                source,
                chunk_size,
                pending,
            } => {
                while pending.len() < *chunk_size {
                    match source.next().await {
                        Some(Ok(mut samples)) => pending.append(&mut samples),
                        Some(Err(e)) => return Some(Err(e)),
                        None => {
                            if pending.is_empty() {
                                return None;
                            }
                            return Some(Ok(std::mem::take(pending)));
                        }
                    }
                }

                let out: Vec<Complex<f32>> = pending.drain(..*chunk_size).collect();
                Some(Ok(out))
            }
        }
    }

    /// Adjust tuner gain on live SDR sources. No-op for file/stdin sources.
    fn set_gain(&self, gain: Gain) -> desperado::error::Result<()> {
        match self {
            IqChunkSource::Sync(_) => Ok(()),
            IqChunkSource::Async { source, .. } => source.set_gain(gain),
        }
    }
}

const AUDIO_RATE: usize = 48_000;
const AUDIO_QUEUE_SECONDS: usize = 4;
const AUDIO_QUEUE_MAX: usize = AUDIO_RATE * 2 * AUDIO_QUEUE_SECONDS; // stereo buffer in samples

#[derive(Default, Clone)]
struct TuiState {
    // Header
    ensemble_name: String,
    channel: String,
    center_freq_hz: u32,
    // Signal
    input_rate_hz: u32,
    output_rate_hz: u32,
    iq_level_dbfs: f32,
    // Services panel
    services: Vec<(String, Option<u16>)>, // (label, bitrate_kbps)
    selected_service_idx: usize,
    // Decode info
    service_switch_request: Option<String>, // written by TUI, consumed by main loop
    service: String,
    dls: String,
    mot_count: usize,
    mot_info: String,
    mot_filename: Option<String>, // content name for the current preview image
    mot_preview_path: Option<String>,
    clear_mot_image: bool, // request Kitty image erase on next TUI tick
    save_message: String,  // transient feedback after (S) save
    audio_q_fill: usize,
    status: String,
    // Gain
    gain: Option<f64>,                // current gain in dB; None = auto/unknown
    gain_change_request: Option<f64>, // written by TUI thread, consumed by main loop
}

fn should_pause_for_tui_selection(
    tui_enabled: bool,
    is_file_source: bool,
    has_cli_service: bool,
    msc_initialized: bool,
    frame_count: usize,
    has_selectable_service: bool,
) -> bool {
    tui_enabled
        && is_file_source
        && !has_cli_service
        && !msc_initialized
        && frame_count >= 50
        && has_selectable_service
}

struct TuiGuard {
    running: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}

impl Drop for TuiGuard {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn spawn_dab_tui(
    state: Arc<Mutex<TuiState>>,
    running: Arc<AtomicBool>,
    keyboard_available: bool,
    inline_image_support: bool,
    can_adjust_gain: bool,
    gain_steps: Vec<f64>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let _ = enable_raw_mode();
        let backend = CrosstermBackend::new(std::io::stdout());
        let mut terminal = match Terminal::with_options(
            backend,
            TerminalOptions {
                viewport: Viewport::Inline(20),
            },
        ) {
            Ok(t) => t,
            Err(_) => {
                let _ = disable_raw_mode();
                error!("failed to initialize TUI terminal backend");
                return;
            }
        };

        let mut save_message_until: Option<Instant> = None;

        while running.load(Ordering::Relaxed) {
            // Auto-clear save feedback after a few seconds
            if let Some(until) = save_message_until
                && Instant::now() >= until
            {
                save_message_until = None;
                if let Ok(mut s) = state.lock() {
                    s.save_message.clear();
                }
            }

            if let Ok(s) = state.lock() {
                let _ = terminal.draw(|f| {
                    let area = f.area();

                    // Outer vertical split: header (1) / body (fill) / footer (1)
                    let rows = Layout::default()
                        .direction(Direction::Vertical)
                        .constraints([
                            Constraint::Length(1),
                            Constraint::Min(0),
                            Constraint::Length(1),
                        ])
                        .split(area);
                    let header_area = rows[0];
                    let body_area = rows[1];
                    let footer_area = rows[2];

                    // ── Header ──
                    let ensemble_display = if s.ensemble_name.is_empty() {
                        "Scanning…".to_string()
                    } else {
                        s.ensemble_name.clone()
                    };
                    let freq_str = format!("{:.3} MHz", s.center_freq_hz as f64 / 1_000_000.0);
                    let (status_color, status_dot) = if s.status.contains("decoding") {
                        (Color::Green, "●")
                    } else if s.status.contains("sync") || s.status.contains("FIC") {
                        (Color::Yellow, "●")
                    } else {
                        (Color::DarkGray, "○")
                    };
                    let header = Paragraph::new(Line::from(vec![
                        Span::raw("  "),
                        Span::styled(
                            ensemble_display,
                            Style::default()
                                .fg(Color::Blue)
                                .add_modifier(Modifier::BOLD),
                        ),
                        Span::styled("  ·  ", Style::default().add_modifier(Modifier::DIM)),
                        Span::styled(s.channel.clone(), Style::default().fg(Color::Blue)),
                        Span::styled("  ·  ", Style::default().add_modifier(Modifier::DIM)),
                        Span::styled(freq_str, Style::default().fg(Color::Blue)),
                        Span::raw("   "),
                        Span::styled(status_dot, Style::default().fg(status_color)),
                        Span::styled(
                            format!("  {}", s.status),
                            Style::default().add_modifier(Modifier::DIM),
                        ),
                    ]));
                    f.render_widget(header, header_area);

                    // ── Footer ──
                    let controls = if keyboard_available {
                        if can_adjust_gain {
                            "  (↑↓) navigate   (↵) select   (+/-) gain   (S) save image   (q/Esc) quit"
                        } else {
                            "  (↑↓) navigate   (↵) select   (S) save image   (q/Esc) quit"
                        }
                    } else {
                        "  Ctrl-C to quit  (stdin piped)"
                    };
                    let footer = Paragraph::new(Line::from(Span::styled(
                        controls,
                        Style::default().add_modifier(Modifier::DIM),
                    )));
                    f.render_widget(footer, footer_area);

                    // Body: services panel (28%) | info+MOT panel (72%)
                    let cols = Layout::default()
                        .direction(Direction::Horizontal)
                        .constraints([Constraint::Percentage(28), Constraint::Percentage(72)])
                        .split(body_area);
                    let left = cols[0];
                    let right = cols[1];

                    // ── Services panel ──
                    let svc_inner = left.width.saturating_sub(4) as usize;
                    let svc_title = format!(" Services ({}) ", s.services.len());
                    let services_lines: Vec<Line> = if s.services.is_empty() {
                        vec![Line::from("  Scanning…")]
                    } else {
                        s.services
                            .iter()
                            .enumerate()
                            .map(|(i, (label, bitrate))| {
                                let is_active = !s.service.is_empty()
                                    && label.trim().eq_ignore_ascii_case(s.service.trim());
                                let is_cursor = i == s.selected_service_idx;
                                let prefix = if is_active || is_cursor { "► " } else { "  " };
                                let br_str =
                                    bitrate.map(|br| format!(" {}k", br)).unwrap_or_default();
                                let max_label = svc_inner.saturating_sub(2 + br_str.len());
                                let text =
                                    format!("{}{}{}", prefix, one_line(label, max_label), br_str);
                                if is_active {
                                    Line::from(Span::styled(
                                        text,
                                        Style::default()
                                            .fg(Color::Cyan)
                                            .add_modifier(Modifier::BOLD),
                                    ))
                                } else if is_cursor {
                                    Line::from(Span::styled(
                                        text,
                                        Style::default().add_modifier(Modifier::BOLD),
                                    ))
                                } else {
                                    Line::from(text)
                                }
                            })
                            .collect()
                    };
                    let services_panel = Paragraph::new(services_lines)
                        .block(Block::default().title(svc_title).borders(Borders::ALL));
                    f.render_widget(services_panel, left);

                    // ── Info + MOT panel ──
                    let right_inner = right.width.saturating_sub(4) as usize;
                    let gain_str = match s.gain {
                        None => "auto".to_string(),
                        Some(db) => format!("{:.1} dB", db),
                    };
                    let iq_color = if s.iq_level_dbfs > -10.0 {
                        Style::default().fg(Color::Red)
                    } else if s.iq_level_dbfs > -30.0 {
                        Style::default().fg(Color::Green)
                    } else {
                        Style::default().fg(Color::Yellow)
                    };
                    let right_lines = vec![
                        Line::from(vec![
                            Span::raw(format!(
                                " IQ: {}  ",
                                level_bar(s.iq_level_dbfs, -60.0, 0.0, 14),
                            )),
                            Span::styled(format!("{:.1} dBFS", s.iq_level_dbfs), iq_color),
                            Span::raw(format!(
                                "   Audio buffer: {} {}",
                                level_bar(s.audio_q_fill as f32, 0.0, AUDIO_QUEUE_MAX as f32, 14),
                                s.audio_q_fill,
                            )),
                        ]),
                        Line::from(format!(
                            " In: {:.3} MHz  →  {} Hz   Gain: {}",
                            s.input_rate_hz as f64 / 1_000_000.0,
                            s.output_rate_hz,
                            gain_str,
                        )),
                        Line::from(""),
                        Line::from(format!(
                            " DLS: {}",
                            one_line(&s.dls, right_inner.saturating_sub(6))
                        )),
                        Line::from(format!(
                            " MOT: {}  ·  {}",
                            s.mot_count,
                            one_line(&s.mot_info, right_inner.saturating_sub(12))
                        )),
                        if s.save_message.is_empty() {
                            Line::from("")
                        } else {
                            Line::from(Span::styled(
                                format!(" {}", s.save_message),
                                Style::default().fg(Color::Green),
                            ))
                        },
                    ];
                    let right_panel = Paragraph::new(right_lines)
                        .block(Block::default().title(" DAB+ ").borders(Borders::ALL));
                    f.render_widget(right_panel, right);

                    if inline_image_support && let Some(path) = &s.mot_preview_path {
                        draw_tui_image(path, right);
                    }
                });
            }

            // Erase Kitty image when service switches
            if inline_image_support {
                let needs_clear = state
                    .try_lock()
                    .ok()
                    .map(|s| s.clear_mot_image)
                    .unwrap_or(false);
                if needs_clear {
                    let _ = std::io::stdout().write_all(b"\x1b_Ga=d\x1b\\");
                    let _ = std::io::stdout().flush();
                    if let Ok(mut s) = state.lock() {
                        s.clear_mot_image = false;
                    }
                }
            }

            if keyboard_available && event::poll(Duration::from_millis(1)).unwrap_or(false) {
                match event::read() {
                    Ok(Event::Resize(_, _)) => {
                        // Clear Kitty graphics layer images before resize;
                        // they persist at old absolute coordinates otherwise,
                        // creating a "patchwork" of ghost images.
                        if inline_image_support {
                            let _ = std::io::stdout().write_all(b"\x1b_Ga=d\x1b\\");
                            let _ = std::io::stdout().flush();
                        }
                        let _ = terminal.autoresize();
                        let _ = terminal.clear();
                    }
                    Ok(Event::Key(key)) if key.kind == KeyEventKind::Press => match key.code {
                        KeyCode::Esc | KeyCode::Char('q') => {
                            running.store(false, Ordering::Relaxed);
                            break;
                        }
                        KeyCode::Up => {
                            if let Ok(mut s) = state.lock() {
                                s.selected_service_idx = s.selected_service_idx.saturating_sub(1);
                            }
                        }
                        KeyCode::Down => {
                            if let Ok(mut s) = state.lock() {
                                let max = s.services.len().saturating_sub(1);
                                s.selected_service_idx = (s.selected_service_idx + 1).min(max);
                            }
                        }
                        KeyCode::Enter => {
                            if let Ok(mut s) = state.lock()
                                && let Some((label, _)) =
                                    s.services.get(s.selected_service_idx).cloned()
                            {
                                s.service_switch_request = Some(label);
                            }
                        }
                        KeyCode::Char('s') | KeyCode::Char('S') => {
                            if let Ok(mut s) = state.lock() {
                                if let Some(ref src) = s.mot_preview_path.clone() {
                                    let ext = if src.ends_with(".png") { "png" } else { "jpg" };
                                    let dest = s
                                        .mot_filename
                                        .clone()
                                        .unwrap_or_else(|| format!("mot_{}.{}", s.mot_count, ext));
                                    s.save_message = match std::fs::copy(src, &dest) {
                                        Ok(_) => format!("Saved: {}", dest),
                                        Err(e) => format!("Save failed: {}", e),
                                    };
                                } else {
                                    s.save_message = "No image available".to_string();
                                }
                                save_message_until = Some(Instant::now() + Duration::from_secs(4));
                            }
                        }
                        KeyCode::Char('+') if can_adjust_gain => {
                            if let Ok(mut s) = state.lock() {
                                let new_db = next_gain_up(s.gain.unwrap_or(30.0), &gain_steps);
                                s.gain = Some(new_db);
                                s.gain_change_request = Some(new_db);
                            }
                        }
                        KeyCode::Char('-') if can_adjust_gain => {
                            if let Ok(mut s) = state.lock() {
                                let new_db = next_gain_down(s.gain.unwrap_or(30.0), &gain_steps);
                                s.gain = Some(new_db);
                                s.gain_change_request = Some(new_db);
                            }
                        }
                        _ => {}
                    },
                    _ => {}
                }
            }

            thread::sleep(Duration::from_millis(200));
        }

        let _ = terminal.clear();
        let _ = terminal.show_cursor();
        let _ = disable_raw_mode();

        // Clear Kitty graphics protocol images before exiting
        if inline_image_support {
            let _ = std::io::stdout().write_all(b"\x1b_Ga=d\x1b\\");
            let _ = std::io::stdout().flush();
        }
    })
}

fn draw_tui_image(path: &str, area: Rect) {
    // Layout inside right panel (with border):
    //   row 0: top border
    //   row 1: IQ bar
    //   row 2: rates + gain
    //   row 3: blank
    //   row 4: DLS
    //   row 5: blank
    //   row 6: MOT info
    //   row 7+: image area
    //   row last: bottom border
    let image_y_offset = 7u16;
    let overhead = image_y_offset + 1; // +1 for bottom border
    let cfg = viuer::Config {
        absolute_offset: true,
        x: area.x.saturating_add(1),
        y: area.y.saturating_add(image_y_offset) as i16,
        restore_cursor: false,
        width: None, // let viuer calculate width to preserve aspect ratio
        height: Some(area.height.saturating_sub(overhead) as u32),
        transparent: true,
        premultiplied_alpha: false,
        truecolor: true,
        use_kitty: true,
        use_iterm: false,
    };
    let _ = viuer::print_from_file(path, &cfg);
}

fn level_bar(value: f32, min_val: f32, max_val: f32, width: usize) -> String {
    let clamped = value.clamp(min_val, max_val);
    let filled = ((clamped - min_val) / (max_val - min_val) * width as f32) as usize;
    let filled = filled.min(width);
    format!("{}{}", "█".repeat(filled), "░".repeat(width - filled))
}

fn one_line(input: &str, max_len: usize) -> String {
    let mut s = input.replace(['\n', '\r', '\t'], " ");
    if s.len() > max_len {
        s.truncate(max_len.saturating_sub(3));
        s.push_str("...");
    }
    s
}

struct WavWriter {
    file: File,
    data_bytes: u32,
}

impl WavWriter {
    fn create(path: &str, sample_rate: u32, channels: u16) -> std::io::Result<Self> {
        let mut file = File::create(path)?;
        file.write_all(&[0u8; 44])?;
        let mut writer = Self {
            file,
            data_bytes: 0,
        };
        writer.write_header(sample_rate, channels)?;
        Ok(writer)
    }

    fn write_samples_f32(&mut self, pcm: &[f32]) -> std::io::Result<()> {
        for &s in pcm {
            let clamped = s.clamp(-1.0, 1.0);
            let q = (clamped * i16::MAX as f32) as i16;
            self.file.write_all(&q.to_le_bytes())?;
        }
        self.data_bytes = self
            .data_bytes
            .saturating_add((pcm.len() * std::mem::size_of::<i16>()) as u32);
        Ok(())
    }

    fn finalize(&mut self, sample_rate: u32, channels: u16) -> std::io::Result<()> {
        self.write_header(sample_rate, channels)
    }

    fn write_header(&mut self, sample_rate: u32, channels: u16) -> std::io::Result<()> {
        let bits_per_sample: u16 = 16;
        let bytes_per_sample = (bits_per_sample / 8) as u32;
        let byte_rate = sample_rate * channels as u32 * bytes_per_sample;
        let block_align = channels * (bits_per_sample / 8);
        let riff_size = 36u32.saturating_add(self.data_bytes);

        self.file.seek(SeekFrom::Start(0))?;
        self.file.write_all(b"RIFF")?;
        self.file.write_all(&riff_size.to_le_bytes())?;
        self.file.write_all(b"WAVE")?;
        self.file.write_all(b"fmt ")?;
        self.file.write_all(&16u32.to_le_bytes())?;
        self.file.write_all(&1u16.to_le_bytes())?;
        self.file.write_all(&channels.to_le_bytes())?;
        self.file.write_all(&sample_rate.to_le_bytes())?;
        self.file.write_all(&byte_rate.to_le_bytes())?;
        self.file.write_all(&block_align.to_le_bytes())?;
        self.file.write_all(&bits_per_sample.to_le_bytes())?;
        self.file.write_all(b"data")?;
        self.file.write_all(&self.data_bytes.to_le_bytes())?;
        self.file.seek(SeekFrom::End(0))?;
        Ok(())
    }
}

#[derive(Parser)]
#[command(name = "dabradio", about = "DAB/DAB+ digital radio decoder")]
struct Cli {
    /// Source: file path or SDR URI (rtlsdr://, soapy://, airspy://)
    source: Option<String>,

    /// DAB channel (e.g. "12A", "12C")
    #[arg(long)]
    channel: Option<String>,

    /// Center frequency in Hz (alternative to --channel)
    #[arg(short = 'f', long)]
    freq: Option<u32>,

    /// IQ format for file sources: cu8, cs8, cs16, cf32 (auto-detected from GQRX
    /// file names; defaults to cu8)
    #[arg(long)]
    format: Option<String>,

    /// Input sample rate in samples/s (needed for non-2.048MS/s file/stdin sources)
    #[arg(long)]
    sample_rate: Option<u32>,

    /// RF center frequency of the captured signal, in Hz (auto-detected from
    /// GQRX file names; defaults to the selected multiplex, i.e. no downconversion
    /// shift). For live SDR, sets the device tune center when the URI does not
    /// specify one — meaningful for wideband/high-rate sources such as HACKRF
    /// (to dodge a DC spike), not useful for RTL-SDR where bandwidth == sample
    /// rate.
    #[arg(long)]
    center_freq: Option<u32>,

    /// List services and exit (no audio)
    #[arg(long)]
    list: bool,

    /// List all standard DAB Band III channels and exit
    #[arg(long)]
    list_channels: bool,

    /// Output as JSON
    #[arg(long)]
    json: bool,

    /// Maximum number of frames to process (0 = unlimited)
    #[arg(long, default_value = "0")]
    max_frames: usize,

    /// Service to decode (label or hex SId like "0xF201")
    #[arg(long)]
    service: Option<String>,

    /// Output file for decoded MSC logical frames (DAB+ or legacy DAB)
    #[arg(short, long)]
    output: Option<String>,

    /// Output decoded PCM audio to WAV file
    #[arg(long)]
    wav: Option<String>,

    /// Disable audio output
    #[arg(long)]
    no_audio: bool,

    /// Save MOT slideshow images to directory (e.g. --slideshow ./images)
    #[arg(long)]
    slideshow: Option<String>,

    /// MOT inline image width in terminal cells (0 = auto)
    #[arg(long, default_value = "40")]
    mot_width: u32,

    /// MOT inline image height in terminal cells (0 = auto)
    #[arg(long, default_value = "0")]
    mot_height: u32,

    /// Show inline TUI dashboard (non-fullscreen)
    #[arg(long, default_value_t = false)]
    tui: bool,

    /// Enable debug logs for audio pipeline queue/underrun tracking
    #[arg(long, default_value_t = false)]
    debug_audio: bool,

    /// Decode TPEG/TEC from packet-mode components and emit GeoJSON
    #[arg(long, default_value_t = false)]
    traffic: bool,

    /// Emit DAB Traffic Announcement events (FIG 0/18 support + FIG 0/19 switching)
    /// as JSON lines (bearer dab-announcement). Distinct from TPEG GeoJSON.
    #[arg(long, default_value_t = false)]
    announcements: bool,

    /// Dump parsed FIG 0/0–0/3, 0/8, 0/13, 0/18, 0/19 as JSON after processing
    /// the full input (or --max-frames). Accumulates FIGs across the whole run
    /// so late FIG 0/13 / 0/18 announcements are not missed.
    #[arg(long, default_value_t = false)]
    dump_fic: bool,

    /// Decode all packet-mode subchannels and print MSC packet CRC stats
    /// (validates the packet-mode chain even when FIG 0/13 is not TPEG)
    #[arg(long, default_value_t = false)]
    dump_packets: bool,

    /// Optional TMC/GLR location-table CSV or directory (points.csv)
    #[arg(long)]
    location_tables: Option<String>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    let source_is_stdin = cli.source.as_deref() == Some("-");
    let tui_enabled = cli.tui && !source_is_stdin;

    let app_level = if tui_enabled {
        Level::WARN
    } else {
        Level::INFO
    };
    let mut log_filter = tracing_subscriber::filter::Targets::new()
        .with_target("dabradio", app_level)
        .with_target("desperado", app_level)
        .with_target("rtl_sdr_rs", Level::WARN)
        .with_default(Level::WARN);
    if cli.debug_audio {
        log_filter = log_filter
            .with_target("dabradio::audio", Level::DEBUG)
            .with_target("dabradio::audio_pipeline", Level::DEBUG);
    }
    tracing_subscriber::registry()
        .with(log_filter)
        .with(tracing_subscriber::fmt::layer())
        .init();

    if cli.list_channels {
        print_channels(cli.json);
        return Ok(());
    }

    if cli.channel.is_some() && cli.freq.is_some() {
        warn!("Both --channel and --freq were provided; using --channel");
    }

    let source = cli
        .source
        .as_ref()
        .ok_or("Source path or SDR URI is required unless --list-channels is used")?;

    if cli.tui && source_is_stdin {
        warn!("Ignoring --tui because input is piped stdin ('-')");
    }

    if cfg!(debug_assertions) && is_device_uri(source) && !cli.no_audio {
        warn!(
            "Debug build is not real-time for live SDR; use --release for valid audio underrun/hiccup checks"
        );
    }

    // Resolve center frequency
    let center_freq = if let Some(channel) = &cli.channel {
        constants::channel_frequency(channel)
            .ok_or_else(|| format!("Unknown DAB channel: {}", channel))?
    } else if let Some(freq) = cli.freq {
        freq
    } else {
        return Err("Either --channel or --freq must be specified".into());
    };

    if cli.channel.is_none() {
        if constants::is_band_iii_frequency(center_freq) {
            let (nearest_name, nearest_hz, delta_hz) = constants::nearest_channel(center_freq);
            if delta_hz > 0 {
                warn!(
                    "Frequency {} Hz is in Band III but not on a standard DAB channel center; nearest is {} ({} Hz, delta {} Hz)",
                    center_freq, nearest_name, nearest_hz, delta_hz,
                );
            }
        } else {
            let (nearest_name, nearest_hz, delta_hz) = constants::nearest_channel(center_freq);
            warn!(
                "Frequency {} Hz is outside DAB Band III ({}..={} Hz); nearest channel is {} ({} Hz, delta {} Hz)",
                center_freq,
                constants::BAND_III_MIN_HZ,
                constants::BAND_III_MAX_HZ,
                nearest_name,
                nearest_hz,
                delta_hz,
            );
        }
    }

    // Open IQ source (file or live SDR URI).
    //
    // Parameter resolution for file/stdin sources, with precedence:
    //   explicit CLI flag  >  GQRX filename auto-detect  >  existing default
    // For live SDR, GQRX detection is skipped (the source is a device URI).
    //
    // - stdin/file: use --sample-rate / --format when provided
    // - airspy://: 4.096 MS/s IQ (matching welle.io's approach).
    //   4096000 is not in firmware's supported list [6M, 3M], so the kHz fallback
    //   sends 4096000*2/1000 = 8192 kHz to firmware. ADC runs at 8.192 MHz real,
    //   IqConverter decimates 2:1 → 4.096M complex IQ.
    //   DabResampler::Half then does 2:1 pair-averaging → 2.048M (DAB native rate).
    //   This avoids fractional resampling, giving clean OFDM sync.
    // - others default: 2.048 MS/s
    let is_device = is_device_uri(source);
    let gqrx_meta = if !is_device {
        desperado::gqrx::parse_gqrx_filename(source)
    } else {
        None
    };

    // Resolve the actual RF center of the captured signal:
    //   explicit --center-freq  >  GQRX filename  >  selected multiplex (no shift)
    // The default (== multiplex frequency) yields a zero downconversion shift,
    // preserving behavior for narrowband captures centred on the channel. For
    // live SDR the device is tuned to this resolved center; a non-default value
    // (e.g. an off-channel HACKRF tune to dodge a DC spike) is compensated by
    // the rotator below. (Meaningful for wideband/high-rate sources; not useful
    // for RTL-SDR where bandwidth == sample rate.)
    let input_center = if let Some(hz) = cli.center_freq {
        hz
    } else if let Some(meta) = gqrx_meta {
        meta.center_freq_hz
    } else {
        center_freq
    };
    let shift_hz: i32 = center_freq as i32 - input_center as i32;

    let iq_format = if let Some(fmt) = &cli.format {
        match fmt.as_str() {
            "cu8" => IqFormat::Cu8,
            "cs8" => IqFormat::Cs8,
            "cs16" => IqFormat::Cs16,
            "cf32" => IqFormat::Cf32,
            other => return Err(format!("Unknown IQ format: {}", other).into()),
        }
    } else if let Some(meta) = gqrx_meta {
        meta.format
    } else {
        IqFormat::Cu8
    };

    let default_input_rate = if source.starts_with("airspy://") {
        4_096_000
    } else {
        constants::SAMPLE_RATE
    };
    let input_rate = if let Some(r) = cli.sample_rate {
        r
    } else if let Some(meta) = gqrx_meta {
        meta.sample_rate_hz
    } else {
        default_input_rate
    };

    if let Some(meta) = gqrx_meta {
        info!(
            center_freq_hz = meta.center_freq_hz,
            sample_rate_hz = meta.sample_rate_hz,
            format = %meta.format,
            "Auto-detected GQRX capture parameters from file name"
        );
    }
    info!(
        "Opening {} (multiplex={} Hz, input_center={} Hz, shift={} Hz, rate={} Hz, format={})",
        source, center_freq, input_center, shift_hz, input_rate, iq_format
    );

    let is_rtlsdr_uri = source.starts_with("rtlsdr://");
    let source_uri = source.to_string();

    let chunk_size = constants::T_F; // One frame's worth of samples
    let (mut source, is_file_source, input_sample_rate, effective_gain) =
        open_iq_source(source, input_center, input_rate, chunk_size, iq_format).await?;
    // Round startup discard to the nearest whole T_F so the OFDM processor
    // always receives its first chunk on a frame boundary. A non-multiple of
    // T_F causes a partial first chunk that forces the null-symbol detector
    // to search at an off-boundary position, which can degrade initial sync.
    let mut startup_discard_samples = if is_rtlsdr_uri {
        let t_f = constants::T_F;
        let raw = input_sample_rate as usize;
        raw.div_ceil(t_f) * t_f // round up to next whole T_F
    } else {
        0usize
    };

    let mut iq_resampler = if input_sample_rate != constants::SAMPLE_RATE {
        info!(
            input_rate = input_sample_rate,
            output_rate = constants::SAMPLE_RATE,
            "Enabling DAB-optimized I/Q resampler"
        );
        Some(DabResampler::new(input_sample_rate).map_err(std::io::Error::other)?)
    } else {
        None
    };

    let mut ofdm_processor = ofdm::processor::OfdmProcessor::new();
    let mut ensemble = fic::fib::EnsembleInfo::new();
    let mut frame_count = 0usize;
    let mut fib_count = 0usize;

    // MSC decoding state (initialized once we know the service -> subchannel mapping)
    let mut msc_handler: Option<msc::MscHandler> = None;
    let mut dab_plus_decoder: Option<audio::DabPlusDecoder> = None;
    let mut mp2_decoder: Option<audio::mp2::Mp2Decoder> = None;
    let mut msc_output: Vec<Vec<u8>> = Vec::new();
    let mut decoding_service = cli.service.is_some();
    let selected_service_sid = cli.service.as_deref().and_then(parse_service_id_arg);
    let mut announced_service_label = false;
    let mut mot_image_count = 0usize;
    let mut last_mot_hash: Option<u64> = None;
    let keyboard_available = std::io::stdin().is_terminal();
    let inline_image_support = terminal_supports_inline_images();
    let term_image_support = inline_image_support && !tui_enabled;

    let location_table = cli.location_tables.as_ref().and_then(|p| {
        match traffic::LocationTable::load(std::path::Path::new(p)) {
            Ok(t) => {
                info!(entries = t.len(), path = %p, "Loaded location table");
                Some(t)
            }
            Err(e) => {
                warn!(path = %p, error = %e, "Failed to load location table");
                None
            }
        }
    });
    let mut traffic_channels: Vec<TrafficChannel> = Vec::new();
    let mut armed_packet_keys: std::collections::HashSet<(u8, u16)> =
        std::collections::HashSet::new();
    let mut traffic_no_tpeg_warned = false;
    let mut traffic_features: Vec<traffic::TrafficFeature> = Vec::new();
    let mut packet_crc_warned = false;
    let mut announcement_monitor = fic::fib::AnnouncementMonitor::new();
    let mut announcement_events: Vec<traffic::AnnouncementEvent> = Vec::new();
    // All FIG 0/19 entries observed during the run (for --dump-fic).
    let mut announcement_switching_seen: std::collections::HashMap<
        u8,
        fic::fib::AnnouncementSwitch,
    > = std::collections::HashMap::new();

    let app_running = Arc::new(AtomicBool::new(true));
    let tui_state = Arc::new(Mutex::new(TuiState {
        channel: cli
            .channel
            .clone()
            .unwrap_or_else(|| "(custom)".to_string()),
        center_freq_hz: center_freq,
        input_rate_hz: input_sample_rate,
        output_rate_hz: constants::SAMPLE_RATE,
        iq_level_dbfs: -60.0,
        gain: effective_gain,
        status: "starting".to_string(),
        ..Default::default()
    }));
    let _tui_guard = if tui_enabled {
        let gain_steps = gain_steps_for_source(&source_uri);
        Some(TuiGuard {
            running: app_running.clone(),
            handle: Some(spawn_dab_tui(
                tui_state.clone(),
                app_running.clone(),
                keyboard_available,
                inline_image_support,
                !is_file_source,
                gain_steps,
            )),
        })
    } else {
        None
    };

    let sig_running = app_running.clone();
    tokio::spawn(async move {
        // First Ctrl-C: graceful shutdown
        if tokio::signal::ctrl_c().await.is_ok() {
            sig_running.store(false, Ordering::Relaxed);
        }
        // Second Ctrl-C: force exit (audio device drop can hang on stuck ALSA)
        if tokio::signal::ctrl_c().await.is_ok() {
            eprintln!("\nForce exit (second Ctrl-C)");
            restart_audio_server();
            std::process::exit(1);
        }
    });

    let mut iq_rotator = if shift_hz != 0 {
        let angle = -2.0f32 * std::f32::consts::PI * (shift_hz as f32) / (input_sample_rate as f32);
        info!(shift_hz, "Applying input frequency shift");
        Some(Rotate::new(angle))
    } else {
        None
    };
    // Adaptive baseband DC removal: always considered for RTL-SDR-native cu8
    // (direct-conversion LO leakage). When digitally downconverting (shift!=0),
    // force removal before the mix so a capture-center spike cannot become an
    // in-band tone. Clean cf32/USRP captures with near-zero mean stay below the
    // adaptive threshold and are left untouched.
    // Adaptive for cu8 (RTL LO leakage). Force when digitally mixing so a
    // baseband DC spike cannot become an in-band tone after rotation.
    // Thresholded adaptive path leaves clean USRP/cf32 captures alone.
    let mut dc_tracker = AdaptiveDcRemover::new(
        matches!(iq_format, IqFormat::Cu8) || shift_hz != 0,
        shift_hz != 0,
    );

    // Audio output setup (tinyaudio + crossbeam channel)
    // Queue is 4 seconds deep — enough to absorb decode bursts.
    // Prebuffer is small (0.25 s for live, 1 s for file) so audio starts quickly.
    // PCM enqueue is blocking for both paths: no samples are ever silently dropped.
    // The IQ bridge chain (sync_channel(15) + tokio mpsc(4) ≈ 1.2 s deep) absorbs
    // any stall while the audio queue drains to make room.
    let (tx, rx) = channel::bounded::<f32>(AUDIO_QUEUE_MAX);
    let audio_underruns = Arc::new(AtomicU64::new(0));
    let audio_dropped = Arc::new(AtomicU64::new(0));
    let audio_primed = Arc::new(AtomicBool::new(false));
    let audio_measure_active = Arc::new(AtomicBool::new(true));
    // 1 s stereo for both sources. DAB+ decodes audio in superframe bursts (~46k samples
    // every 480 ms); the prebuffer must hold at least 2 bursts so the audio callback
    // doesn't underrun between them. The bounded IQ queue keeps the decoder at hardware
    // rate, so this fills in ~1 s of wall-clock time for live sources.
    let audio_prebuffer_samples = AUDIO_RATE * 2;
    let mut audio_last_dbg = Instant::now();
    let mut audio_last_underruns = 0u64;
    let mut audio_last_dropped = 0u64;
    let mut first_pcm_logged = false;
    let mut audio_primed_logged = false;

    let _device = if !cli.no_audio {
        let config = OutputDeviceParameters {
            channels_count: 2, // DAB+ HE-AAC v2 typically outputs stereo
            sample_rate: AUDIO_RATE,
            channel_sample_count: 1024,
        };
        let make_callback = |rx: channel::Receiver<f32>,
                             audio_underruns_cb: Arc<AtomicU64>,
                             audio_primed_cb: Arc<AtomicBool>,
                             audio_measure_active_cb: Arc<AtomicBool>| {
            move |data: &mut [f32]| {
                if !audio_measure_active_cb.load(Ordering::Relaxed) {
                    data.fill(0.0);
                    return;
                }

                if !audio_primed_cb.load(Ordering::Relaxed) {
                    if rx.len() < audio_prebuffer_samples {
                        data.fill(0.0);
                        return;
                    }
                    audio_primed_cb.store(true, Ordering::Relaxed);
                }

                let mut all_empty = true;
                for sample in data.iter_mut() {
                    match rx.try_recv() {
                        Ok(v) => {
                            *sample = v;
                            all_empty = false;
                        }
                        Err(_) => {
                            *sample = 0.0;
                            audio_underruns_cb.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
                // Full underrun: queue drained completely. Un-prime so we wait
                // for the buffer to refill before playing again, preventing
                // permanent stutter from a single missed superframe burst.
                if all_empty {
                    audio_primed_cb.store(false, Ordering::Relaxed);
                }
            }
        };
        match run_output_device(
            config,
            make_callback(
                rx.clone(),
                Arc::clone(&audio_underruns),
                Arc::clone(&audio_primed),
                Arc::clone(&audio_measure_active),
            ),
        ) {
            Ok(device) => Some(device),
            Err(e) => {
                error!("Failed to open audio output device: {}.", e);
                warn!("Attempting to restart audio server to recover sound card...");
                restart_audio_server();
                // Give the audio server time to reinitialize.
                thread::sleep(Duration::from_millis(500));
                // Retry once after recovery.
                match run_output_device(
                    config,
                    make_callback(
                        rx.clone(),
                        Arc::clone(&audio_underruns),
                        Arc::clone(&audio_primed),
                        Arc::clone(&audio_measure_active),
                    ),
                ) {
                    Ok(device) => {
                        info!("Audio device opened after server restart");
                        Some(device)
                    }
                    Err(e2) => {
                        error!(
                            "Still cannot open audio device after restart: {}. Continuing without audio.",
                            e2
                        );
                        None
                    }
                }
            }
        }
    } else {
        None
    };

    let mut wav_writer = if decoding_service || tui_enabled {
        if let Some(path) = &cli.wav {
            Some(WavWriter::create(path, AUDIO_RATE as u32, 2)?)
        } else {
            None
        }
    } else {
        None
    };

    let mut main_loop_iter = 0usize;
    while app_running.load(Ordering::Relaxed) {
        // For file playback, stop consuming the recording once the TUI has a
        // selectable service list. Without this pause the decoder races to EOF
        // before the user can press Enter, making the TUI appear to crash.
        // This control-flow decision must not use try_lock: the TUI redraw
        // thread can otherwise win the mutex on every fast file-input loop and
        // let the main thread race all the way to EOF.
        let has_selectable_service = tui_state
            .lock()
            .ok()
            .is_some_and(|s| !s.services.is_empty());
        // Let several FIG label cycles pass before pausing; stopping as soon
        // as the first labels arrive leaves an incomplete menu.
        if should_pause_for_tui_selection(
            tui_enabled,
            is_file_source,
            cli.service.is_some(),
            msc_handler.is_some(),
            frame_count,
            has_selectable_service,
        ) {
            if let Ok(mut s) = tui_state.lock() {
                s.status = "select a service".to_string();
            }
            while app_running.load(Ordering::Relaxed) && msc_handler.is_none() {
                if apply_tui_service_switch(
                    &mut ensemble,
                    &tui_state,
                    &mut msc_handler,
                    &mut dab_plus_decoder,
                    &mut mp2_decoder,
                    &audio_primed,
                    &mut announced_service_label,
                ) {
                    decoding_service = true;
                    break;
                }
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
            if !app_running.load(Ordering::Relaxed) {
                break;
            }
        }

        // Use a timeout so we periodically re-check app_running even if the
        // SDR source blocks (device error, USB stall, etc.).
        let chunk_result = loop {
            match tokio::time::timeout(Duration::from_millis(500), source.next_chunk()).await {
                Ok(result) => break result,
                Err(_timeout) => {
                    if !app_running.load(Ordering::Relaxed) {
                        break None;
                    }
                    // Timeout — retry
                    continue;
                }
            }
        };
        let Some(chunk) = chunk_result else {
            info!("stream ended");
            break;
        };
        main_loop_iter += 1;
        if main_loop_iter <= 10 {
            debug!(
                iter = main_loop_iter,
                samples = chunk.as_ref().map(|c| c.len()).unwrap_or(0),
                "got chunk"
            );
        }
        let mut samples = chunk?;
        if startup_discard_samples > 0 {
            if samples.len() <= startup_discard_samples {
                startup_discard_samples -= samples.len();
                continue;
            }
            let keep_from = startup_discard_samples;
            startup_discard_samples = 0;
            samples = samples.split_off(keep_from);
        }
        // Adaptive DC removal before any digital mix. When shift!=0 the spike
        // would otherwise become an in-band tone after rotation; for cu8 it
        // also covers on-channel RTL-SDR LO leakage. Thresholded so clean
        // USRP/cf32 captures are not disturbed.
        dc_tracker.process(&mut samples);
        let samples = if let Some(ref mut rotator) = iq_rotator {
            rotator.process(&samples)
        } else {
            samples
        };
        let samples = {
            let mut samples = samples;
            if let Some(ref mut resampler) = iq_resampler {
                samples = resampler.process(&samples);
                if samples.is_empty() {
                    continue;
                }
            }
            samples
        };
        if main_loop_iter <= 10 {
            debug!(n = samples.len(), "ofdm processing");
        }

        // Update IQ level (mean power → dBFS, smoothed with EMA)
        if tui_enabled && !samples.is_empty() {
            let mean_power =
                samples.iter().map(|s| s.norm_sqr()).sum::<f32>() / samples.len() as f32;
            let dbfs = if mean_power > 0.0 {
                10.0 * mean_power.log10()
            } else {
                -120.0
            };
            if let Ok(mut s) = tui_state.try_lock() {
                s.iq_level_dbfs = s.iq_level_dbfs * 0.7 + dbfs * 0.3;
            }
        }

        let frames = ofdm_processor.process(&samples);

        for frame in &frames {
            if !app_running.load(Ordering::Relaxed) {
                break;
            }
            frame_count += 1;

            // DQPSK decode all symbols
            let soft_bits = ofdm::decoder::dqpsk_decode(&frame.symbols);

            // Debug: compare per-symbol soft bit statistics across the frame
            // This helps identify if later symbols (MSC) degrade compared to earlier ones (FIC)
            if frame_count <= 5 {
                // Compute per-symbol mean absolute soft bit value
                let mut sym_stats: Vec<(usize, f64)> = Vec::new();
                for (idx, sym) in soft_bits.iter().enumerate() {
                    let mean_abs: f64 = sym.iter().map(|&x| (x as f64).abs()).sum::<f64>()
                        / (constants::K * 2) as f64;
                    sym_stats.push((idx, mean_abs));
                }

                // Sample 8 symbols spread across the frame: indices 0,1,2 (FIC), 10, 25, 40, 55, 70 (MSC)
                let sample_indices = [0, 1, 2, 10, 25, 40, 55, 70];
                let samples: Vec<String> = sample_indices
                    .iter()
                    .filter_map(|&i| {
                        sym_stats
                            .get(i)
                            .map(|(idx, val)| format!("{}:{:.1}", idx, val))
                    })
                    .collect();

                debug!(
                    frame = frame_count,
                    per_symbol = samples.join(" "),
                    "Symbol quality across frame"
                );
            }

            // Extract FIC data from symbols 0..2 (first 3 data symbols after PRS)
            if soft_bits.len() >= constants::FIC_SYMBOLS {
                let fic_symbols: Vec<Vec<i8>> = soft_bits[..constants::FIC_SYMBOLS].to_vec();
                let fibs = fic::handler::process_fic(&fic_symbols);

                // Feed FIC decode ratio back to OFDM processor for coarse freq gating.
                // Each frame has 4 subblocks × 3 FIBs = 12 expected FIBs.
                // Matches welle.io's ficHandler.getFicDecodeRatioPercent() feedback loop.
                let fic_ratio = ((fibs.len() as f32 / 12.0) * 100.0).round().min(100.0) as u8;
                ofdm_processor.set_fic_decode_ratio(fic_ratio);

                // FIG 0/19 is only present while an announcement is active; clear
                // the prior frame's switching table so absence can be detected.
                if cli.announcements {
                    for (k, v) in ensemble.announcement_switching.drain() {
                        announcement_switching_seen.insert(k, v);
                    }
                }

                for fib in &fibs {
                    fib_count += 1;
                    ensemble.parse_fib(fib);
                }
                if !ensemble.announcement_switching.is_empty() {
                    for (k, v) in &ensemble.announcement_switching {
                        announcement_switching_seen.insert(*k, v.clone());
                    }
                }
                if cli.announcements {
                    let ts = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .ok()
                        .map(|d| d.as_millis() as u64);
                    for ev in announcement_monitor.observe(&ensemble, ts) {
                        if !cli.json {
                            info!(
                                phase = ?ev.phase,
                                cluster = ev.cluster_id,
                                subch = ev.subchannel_id,
                                types = ?ev.announcement_types,
                                "Announcement"
                            );
                        }
                        println!("{}", ev.to_json());
                        announcement_events.push(ev);
                    }
                    for ev in announcement_monitor.end_missing(&ensemble, ts) {
                        if !cli.json {
                            info!(
                                phase = ?ev.phase,
                                cluster = ev.cluster_id,
                                subch = ev.subchannel_id,
                                "Announcement ended"
                            );
                        }
                        println!("{}", ev.to_json());
                        announcement_events.push(ev);
                    }
                }
                if frame_count <= 5 || frame_count.is_multiple_of(50) {
                    debug!(
                        frame = frame_count,
                        fibs_total = fib_count,
                        fibs_this = fibs.len(),
                        "FIC"
                    );
                }
                if let Ok(mut s) = tui_state.try_lock() {
                    if s.service.is_empty() {
                        s.status = if fib_count > 0 {
                            "sync + FIC OK".to_string()
                        } else {
                            "OFDM sync only".to_string()
                        };
                    }
                    // Ensemble name
                    if s.ensemble_name.is_empty()
                        && let Some(name) = &ensemble.ensemble_label
                    {
                        s.ensemble_name = name.clone();
                    }
                    // Services list: rebuild when the labeled-service count changes.
                    // Compare against labeled services only (same filter as the list
                    // itself) so unlabeled entries don't cause a perpetual rebuild
                    // that would override the user's cursor position every frame.
                    let labeled_count = ensemble
                        .services
                        .values()
                        .filter(|svc| svc.label.is_some())
                        .count();
                    if s.services.len() != labeled_count {
                        let mut list: Vec<(String, Option<u16>)> = ensemble
                            .services
                            .values()
                            .filter_map(|svc| {
                                let label = svc.label.clone()?;
                                let bitrate = svc
                                    .subchannel_id
                                    .and_then(|sid| ensemble.subchannels.get(&sid))
                                    .map(|sub| sub.bitrate);
                                Some((label.trim().to_string(), bitrate))
                            })
                            .collect();
                        list.sort_by(|a, b| a.0.cmp(&b.0));
                        s.services = list;
                        // Update cursor position to match the currently playing service
                        // (by label, not by stale index) to handle list rebuilds
                        if !s.service.is_empty()
                            && let Some(idx) = s
                                .services
                                .iter()
                                .position(|(l, _)| l.eq_ignore_ascii_case(&s.service))
                        {
                            s.selected_service_idx = idx;
                        }
                    }
                }
            }

            if msc_handler.is_some()
                && !announced_service_label
                && let Some(sid) = selected_service_sid
                && let Some(label) = ensemble
                    .services
                    .get(&sid)
                    .and_then(|service| service.label.as_deref())
                && !label.trim().is_empty()
            {
                info!(label = %label.trim(), "Resolved service label");
                announced_service_label = true;
                if let Ok(mut s) = tui_state.try_lock() {
                    let trimmed = label.trim().to_string();
                    if let Some(idx) = s
                        .services
                        .iter()
                        .position(|(l, _)| l.eq_ignore_ascii_case(&trimmed))
                    {
                        s.selected_service_idx = idx;
                    }
                    s.service = trimmed;
                }
            }

            // Check if we have enough info to list services
            if cli.list && ensemble.is_complete() {
                ensemble.resolve_services();
                print_services(&ensemble, cli.json);
                return Ok(());
            }

            // Fallback for file sources: if --list and we've processed enough frames,
            // print what we have even if labels are still incomplete.
            // For live SDR sources, keep waiting for full FIC so service labels can arrive.
            if cli.list && is_file_source && frame_count >= 50 && ensemble.has_services() {
                ensemble.resolve_services();
                print_services(&ensemble, cli.json);
                return Ok(());
            }

            // --dump-fic waits until EOF / --max-frames so FIG 0/13 from the
            // full FIC repetition cycle is accumulated before printing.
            // Packet/TPEG targets are re-evaluated as FIG 0/3 / 0/8 / 0/13
            // arrive; already-armed MSC handlers are kept.
            if cli.traffic || cli.dump_packets {
                ensemble.resolve_services();
                merge_traffic_channels(
                    &mut traffic_channels,
                    &mut armed_packet_keys,
                    &ensemble,
                    &mut traffic_features,
                    cli.dump_packets,
                    cli.traffic,
                );
                if cli.traffic
                    && !traffic_no_tpeg_warned
                    && ensemble.is_complete()
                    && ensemble.tpeg_targets().is_empty()
                    && traffic_features.is_empty()
                {
                    // Advisory only — keep listening; FIG 0/13 may still arrive.
                    warn!(
                        "No TPEG (UAtype 0x004) packet-mode component found in FIC yet; \
                         continuing to watch for late FIG 0/13"
                    );
                    traffic_no_tpeg_warned = true;
                }
            }

            // Try to initialize MSC handler if we have service info but no handler yet
            if decoding_service && msc_handler.is_none() && ensemble.has_services() {
                ensemble.resolve_services();
                if let Some(ref service_arg) = cli.service
                    && let Some((handler, bitrate, coding)) = try_init_msc(&ensemble, service_arg)
                {
                    msc_handler = Some(handler);
                    match coding {
                        fic::fib::AudioCoding::DabPlus => {
                            dab_plus_decoder = Some(audio::DabPlusDecoder::new(bitrate));
                            mp2_decoder = None;
                        }
                        fic::fib::AudioCoding::MpegLayer2 => {
                            dab_plus_decoder = None;
                            mp2_decoder = Some(audio::mp2::Mp2Decoder::new());
                        }
                        fic::fib::AudioCoding::Other(_) => unreachable!(),
                    }
                    if let Ok(mut s) = tui_state.try_lock() {
                        s.status = format!("decoding @ {} kbps", bitrate);
                        if s.service.is_empty() {
                            s.service = service_arg.clone();
                        }
                    }
                }
            }

            // Service switch request from TUI Enter key
            if tui_enabled
                && ensemble.has_services()
                && apply_tui_service_switch(
                    &mut ensemble,
                    &tui_state,
                    &mut msc_handler,
                    &mut dab_plus_decoder,
                    &mut mp2_decoder,
                    &audio_primed,
                    &mut announced_service_label,
                )
            {
                decoding_service = true;
            }

            // Gain change request from TUI +/- keys
            if tui_enabled {
                let gain_req = tui_state
                    .try_lock()
                    .ok()
                    .and_then(|mut s| s.gain_change_request.take());
                if let Some(db) = gain_req
                    && let Err(e) = source.set_gain(Gain::Manual(db))
                {
                    warn!(gain = db, error = %e, "Failed to set gain");
                }
            }

            // Feed MSC symbols to packet-mode traffic decoders
            if !traffic_channels.is_empty() {
                let msc_start = constants::FIC_SYMBOLS;
                let msc_end = soft_bits.len().min(75);
                for ch in &mut traffic_channels {
                    for sym in &soft_bits[msc_start..msc_end] {
                        if let Some(decoded) = ch.msc.feed_symbol(sym) {
                            for (assembler, target) in &mut ch.assemblers {
                                for group in assembler.feed_bytes(&decoded) {
                                    let payload = group.payload_for_dg_flag(target.no_data_groups);
                                    debug!(
                                        address = group.address,
                                        bytes = payload.len(),
                                        no_data_groups = target.no_data_groups,
                                        "MSC data group"
                                    );
                                    if !cli.traffic || !target.is_tpeg() {
                                        continue;
                                    }
                                    let sid = format_service_id(target.service_id);
                                    let events = traffic::decode_payload(
                                        payload,
                                        location_table.as_ref(),
                                        Some(&sid),
                                    );
                                    for event in events {
                                        if cli.json {
                                            println!("{}", event.to_json());
                                        }
                                        traffic_features.push(event);
                                    }
                                }
                                if !packet_crc_warned && assembler.stats.is_chance_level() {
                                    packet_crc_warned = true;
                                    warn!(
                                        packets = assembler.stats.packets_seen,
                                        crc_ok = assembler.stats.crc_ok,
                                        pass_rate = format!(
                                            "{:.4}%",
                                            assembler.stats.crc_pass_rate() * 100.0
                                        ),
                                        "Packet CRC pass rate is chance-level; MSC/packet decode is not validated"
                                    );
                                }
                            }
                        }
                    }
                }
            }

            // Feed MSC symbols to handler (symbols 3..74 are MSC, i.e. soft_bits[3..75])
            if let Some(ref mut handler) = msc_handler {
                let msc_start = constants::FIC_SYMBOLS; // 3
                let msc_end = soft_bits.len().min(75); // 75 data symbols total
                for sym in &soft_bits[msc_start..msc_end] {
                    if let Some(decoded) = handler.feed_symbol(sym) {
                        debug!(
                            "MSC frame {}: {} bytes, first 8: {:02X?}",
                            handler.frames_decoded,
                            decoded.len(),
                            &decoded[..decoded.len().min(8)]
                        );

                        // Feed to DAB+ decoder for audio
                        if let Some(ref mut dab_dec) = dab_plus_decoder {
                            let decoded_out = dab_dec.feed_frame_with_metadata(&decoded);

                            // Handle PAD metadata (DLS text and MOT slideshows)
                            for item in &decoded_out.metadata {
                                match item {
                                    PadData::Dls(dls) => {
                                        info!(text = %dls.text, "DLS");
                                        if let Ok(mut s) = tui_state.try_lock() {
                                            s.dls = dls.text.clone();
                                        }
                                    }
                                    PadData::Mot(mot) => {
                                        mot_image_count += 1;
                                        let ext = if mot.content_type.contains("png") {
                                            "png"
                                        } else {
                                            "jpg"
                                        };
                                        info!(
                                            bytes = mot.data.len(),
                                            content_type = %mot.content_type,
                                            content_name = ?mot.content_name,
                                            "MOT"
                                        );
                                        if let Ok(mut s) = tui_state.try_lock() {
                                            s.mot_count = mot_image_count;
                                            s.mot_info =
                                                mot.content_name.clone().unwrap_or_else(|| {
                                                    format!("{} bytes", mot.data.len())
                                                });
                                            s.mot_filename = Some({
                                                let name =
                                                    mot.content_name.clone().unwrap_or_else(|| {
                                                        format!("mot_{}", mot_image_count)
                                                    });
                                                if name.contains('.') {
                                                    name
                                                } else {
                                                    format!("{}.{}", name, ext)
                                                }
                                            });
                                            if tui_enabled {
                                                let mut path = std::env::temp_dir();
                                                path.push(format!(
                                                    "dabradio_tui_mot_preview.{}",
                                                    ext
                                                ));
                                                if std::fs::write(&path, &mot.data).is_ok() {
                                                    s.mot_preview_path =
                                                        Some(path.to_string_lossy().to_string());
                                                }
                                            }
                                        }
                                        // Save to slideshow directory if specified
                                        if let Some(ref dir) = cli.slideshow {
                                            let filename = format!(
                                                "{}/slide_{:03}.{}",
                                                dir, mot_image_count, ext
                                            );
                                            if let Err(e) = std::fs::write(&filename, &mot.data) {
                                                warn!("Failed to write {}: {}", filename, e);
                                            } else {
                                                info!("Saved slideshow image: {}", filename);
                                            }
                                        }

                                        let mot_hash = hash_bytes(&mot.data);
                                        if term_image_support && Some(mot_hash) != last_mot_hash {
                                            match show_image_in_terminal(
                                                &mot.data,
                                                ext,
                                                cli.mot_width,
                                                cli.mot_height,
                                            ) {
                                                Ok(()) => {
                                                    info!(
                                                        bytes = mot.data.len(),
                                                        content_type = %mot.content_type,
                                                        "Displayed MOT image in terminal"
                                                    );
                                                }
                                                Err(e) => {
                                                    warn!(error = %e, "Failed to display MOT image in terminal");
                                                }
                                            }
                                        }
                                        last_mot_hash = Some(mot_hash);
                                    }
                                }
                            }

                            if !decoded_out.pcm.is_empty() && _device.is_some() {
                                if !first_pcm_logged {
                                    first_pcm_logged = true;
                                    info!(
                                        prebuffer_samples = audio_prebuffer_samples,
                                        "First PCM samples from DAB+ decoder — filling prebuffer"
                                    );
                                }

                                // Send with timeout so we can check app_running and avoid
                                // blocking forever if the audio callback stops consuming
                                // (device error, etc.). The IQ bridge chain (~1.2 s deep)
                                // absorbs any stall while the audio callback drains the queue.
                                // block_in_place lets tokio know this thread will block briefly.
                                tokio::task::block_in_place(|| {
                                    for sample in &decoded_out.pcm {
                                        loop {
                                            match tx
                                                .send_timeout(*sample, Duration::from_millis(100))
                                            {
                                                Ok(()) => break,
                                                Err(channel::SendTimeoutError::Timeout(_)) => {
                                                    if !app_running.load(Ordering::Relaxed) {
                                                        return;
                                                    }
                                                    // Queue full, retry after timeout
                                                }
                                                Err(channel::SendTimeoutError::Disconnected(_)) => {
                                                    return;
                                                }
                                            }
                                        }
                                    }
                                });

                                if !audio_primed_logged && audio_primed.load(Ordering::Relaxed) {
                                    audio_primed_logged = true;
                                    info!(queue_fill = tx.len(), "Audio primed — playback started");
                                }

                                if let Ok(mut s) = tui_state.try_lock() {
                                    s.audio_q_fill = tx.len();
                                }
                                if audio_last_dbg.elapsed() >= Duration::from_secs(1) {
                                    let underruns = audio_underruns.load(Ordering::Relaxed);
                                    let delta_underruns =
                                        underruns.saturating_sub(audio_last_underruns);
                                    let dropped = audio_dropped.load(Ordering::Relaxed);
                                    let _delta_dropped = dropped.saturating_sub(audio_last_dropped);
                                    let sf_attempted = dab_dec.superframe.superframes_attempted;
                                    let sf_decoded = dab_dec.superframe.superframes_decoded;
                                    let sf_fc_fail = dab_dec.superframe.fire_code_failures;
                                    let sf_rs_err = dab_dec.superframe.rs_errors;
                                    let sf_ok_pct = sf_decoded
                                        .saturating_mul(100)
                                        .checked_div(sf_attempted)
                                        .unwrap_or(0);
                                    debug!(target: "dabradio::audio_pipeline",
                                        queue_fill = tx.len(),
                                        queue_capacity = tx.capacity().unwrap_or(0),
                                        primed = audio_primed.load(Ordering::Relaxed),
                                        underruns_total = underruns,
                                        underruns_delta = delta_underruns,
                                        superframes_attempted = sf_attempted,
                                        superframes_decoded = sf_decoded,
                                        superframe_success_pct = sf_ok_pct,
                                        fire_code_failures = sf_fc_fail,
                                        rs_errors = sf_rs_err,
                                        rs_corrections = dab_dec.superframe.rs_corrections,
                                        au_crc_errors = dab_dec.superframe.au_crc_errors,
                                        "Audio pipeline status"
                                    );
                                    audio_last_dbg = Instant::now();
                                    audio_last_underruns = underruns;
                                    audio_last_dropped = dropped;

                                    // Warn at info level if superframe success rate is poor,
                                    // so it's visible without RUST_LOG=debug.
                                    if sf_attempted >= 5 && sf_ok_pct < 80 {
                                        tracing::info!(
                                            superframes_attempted = sf_attempted,
                                            superframes_decoded = sf_decoded,
                                            fire_code_failures = sf_fc_fail,
                                            rs_errors = sf_rs_err,
                                            success_pct = sf_ok_pct,
                                            "Poor superframe decode rate — \
                                             high fire_code_failures = OFDM/IQ data quality; \
                                             high rs_errors = RF/SNR"
                                        );
                                    }
                                }
                            }

                            if let Some(writer) = wav_writer.as_mut()
                                && !decoded_out.pcm.is_empty()
                            {
                                writer.write_samples_f32(&decoded_out.pcm)?;
                            }
                        }

                        // Legacy DAB: each UEP-decoded logical frame carries
                        // MPEG Audio Layer II bytes rather than a DAB+ superframe.
                        if let Some(ref mut mp2_dec) = mp2_decoder {
                            let decoded_out = mp2_dec.feed_frame(&decoded);
                            let sample_rate = decoded_out.sample_rate;
                            let channels = decoded_out.channels;
                            let pcm = match decoded_out.into_stereo_48k() {
                                Some(pcm) => pcm,
                                None => {
                                    warn!(
                                        sample_rate,
                                        channels,
                                        "Legacy DAB PCM format is not supported by the 48 kHz output path"
                                    );
                                    Vec::new()
                                }
                            };

                            if !pcm.is_empty() && _device.is_some() {
                                if !first_pcm_logged {
                                    first_pcm_logged = true;
                                    info!(
                                        prebuffer_samples = audio_prebuffer_samples,
                                        "First PCM samples from MP2 decoder — filling prebuffer"
                                    );
                                }
                                tokio::task::block_in_place(|| {
                                    for sample in &pcm {
                                        loop {
                                            match tx
                                                .send_timeout(*sample, Duration::from_millis(100))
                                            {
                                                Ok(()) => break,
                                                Err(channel::SendTimeoutError::Timeout(_)) => {
                                                    if !app_running.load(Ordering::Relaxed) {
                                                        return;
                                                    }
                                                }
                                                Err(channel::SendTimeoutError::Disconnected(_)) => {
                                                    return;
                                                }
                                            }
                                        }
                                    }
                                });
                                if !audio_primed_logged && audio_primed.load(Ordering::Relaxed) {
                                    audio_primed_logged = true;
                                    info!(queue_fill = tx.len(), "Audio primed — playback started");
                                }
                                if let Ok(mut s) = tui_state.try_lock() {
                                    s.audio_q_fill = tx.len();
                                }
                            }

                            if let Some(writer) = wav_writer.as_mut()
                                && !pcm.is_empty()
                            {
                                writer.write_samples_f32(&pcm)?;
                            }
                        }

                        // Optionally collect raw frames for file output
                        if cli.output.is_some() {
                            msc_output.push(decoded);
                        }
                    }
                }
            }

            if cli.max_frames > 0 && frame_count >= cli.max_frames {
                break;
            }
        }

        if cli.max_frames > 0 && frame_count >= cli.max_frames {
            break;
        }
    }

    audio_measure_active.store(false, Ordering::Relaxed);

    if cli.dump_fic {
        ensemble.resolve_services();
        // Prefer the accumulated switching history when --announcements cleared
        // the live table each frame.
        if ensemble.announcement_switching.is_empty() && !announcement_switching_seen.is_empty() {
            ensemble.announcement_switching = announcement_switching_seen.clone();
        }
        print_fic_dump(&ensemble, frame_count, fib_count);
        if !cli.traffic && !cli.dump_packets && !cli.announcements {
            return Ok(());
        }
    }

    if cli.announcements && !cli.json {
        info!(
            events = announcement_events.len(),
            support_services = ensemble.announcement_support.len(),
            "Announcement summary"
        );
        for (sid, s) in &ensemble.announcement_support {
            if traffic::is_traffic_relevant(s.asu_flags) {
                info!(
                    service = %format_service_id(*sid),
                    label = ensemble.services.get(sid).and_then(|x| x.label.as_deref()).unwrap_or(""),
                    asu = format!("0x{:04X}", s.asu_flags),
                    types = ?traffic::announcement_type_names(s.asu_flags),
                    clusters = ?s.cluster_ids,
                    "Traffic-relevant FIG 0/18 support"
                );
            }
        }
    }

    if cli.traffic || cli.dump_packets {
        if cli.traffic && armed_packet_keys.is_empty() && traffic_features.is_empty() {
            warn!(
                "No TPEG (UAtype 0x004) packet-mode component found in FIC; \
                 FIG 0/13 did not confirm the TPEG hypothesis"
            );
        }
        let mut stats_rows = Vec::new();
        for ch in &traffic_channels {
            for (assembler, target) in &ch.assemblers {
                info!(
                    service = %format_service_id(target.service_id),
                    address = target.packet_address,
                    packets = assembler.stats.packets_seen,
                    crc_ok = assembler.stats.crc_ok,
                    groups = assembler.stats.groups_complete,
                    pass_rate = format!("{:.2}%", assembler.stats.crc_pass_rate() * 100.0),
                    "packet-mode stats"
                );
                stats_rows.push(serde_json::json!({
                    "service_id": format_service_id(target.service_id),
                    "label": target.label,
                    "subchannel_id": target.subchannel.id,
                    "start_addr": target.subchannel.start_addr,
                    "packet_address": target.packet_address,
                    "dscty": target.dscty,
                    "ua_types": target.ua_types.iter().map(|t| format!("0x{t:03X}")).collect::<Vec<_>>(),
                    "is_tpeg": target.is_tpeg(),
                    "packets_seen": assembler.stats.packets_seen,
                    "crc_ok": assembler.stats.crc_ok,
                    "crc_fail": assembler.stats.crc_fail,
                    "address_mismatch": assembler.stats.address_mismatch,
                    "groups_complete": assembler.stats.groups_complete,
                    "pass_rate": assembler.stats.crc_pass_rate(),
                    "chance_level": assembler.stats.is_chance_level(),
                    "first_last_hist": assembler.stats.first_last_hist,
                    "continuity_gaps": assembler.stats.continuity_gaps,
                    "useful_zero": assembler.stats.useful_zero,
                    "command_flag": assembler.stats.command_flag,
                    "address_match": assembler.stats.address_match,
                    "seen_addresses": assembler.stats.seen_addresses.iter().zip(assembler.stats.seen_address_counts.iter()).filter(|(_,c)| **c>0).map(|(a,c)| serde_json::json!({"addr": a, "count": c})).collect::<Vec<_>>(),
                    "crc_ok_addresses": assembler.stats.crc_ok_addresses.iter().zip(assembler.stats.crc_ok_address_counts.iter()).filter(|(_,c)| **c>0).map(|(a,c)| serde_json::json!({"addr": a, "count": c})).collect::<Vec<_>>(),
                    "fec_frames": assembler.stats.fec_frames,
                    "fec_desync": assembler.stats.fec_desync,
                    "fec_rs_fail_rows": assembler.stats.fec_rs_fail_rows,
                }));
            }
        }
        if cli.dump_packets {
            println!(
                "{}",
                serde_json::to_string_pretty(&stats_rows).unwrap_or_default()
            );
        }
        if cli.traffic && (!cli.json || traffic_features.is_empty()) {
            emit_traffic_output(&traffic_features, true);
        }
    }

    // Drop the audio device with a timeout. tinyaudio's ALSA backend calls
    // thread::join() inside Drop, which hangs if snd_pcm_writei is blocked.
    // If the drop doesn't complete in 2 seconds, force-exit to avoid
    // leaving a zombie process that locks the sound card.
    if _device.is_some() {
        let drop_done = Arc::new(AtomicBool::new(false));
        let drop_done2 = drop_done.clone();
        let drop_handle = thread::spawn(move || {
            drop(_device);
            drop_done2.store(true, Ordering::Relaxed);
        });
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline && !drop_done.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(50));
        }
        if drop_done.load(Ordering::Relaxed) {
            let _ = drop_handle.join();
        } else {
            warn!("Audio device drop timed out (ALSA likely stuck) — forcing exit");
            // Try to restart PipeWire so the sound card isn't left locked.
            // This is best-effort; process::exit below will run regardless.
            restart_audio_server();
            std::process::exit(1);
        }
    }

    // Print ensemble info
    ensemble.resolve_services();

    if decoding_service {
        // Print decode summary
        if let Some(ref dab_dec) = dab_plus_decoder {
            let sf = &dab_dec.superframe;
            info!(
                superframes = sf.superframes_decoded,
                rs_corrections = sf.rs_corrections,
                rs_errors = sf.rs_errors,
                au_crc_errors = sf.au_crc_errors,
                "DAB+ decode summary"
            );
        }
        if let Some(ref mp2_dec) = mp2_decoder {
            info!(
                frames = mp2_dec.frames_decoded,
                decode_errors = mp2_dec.decode_errors,
                sync_bytes_skipped = mp2_dec.sync_bytes_skipped,
                sample_rate = mp2_dec.sample_rate,
                channels = mp2_dec.channels,
                "Legacy DAB MP2 decode summary"
            );
        }

        if let Some(ref handler) = msc_handler {
            info!(
                logical_frames = handler.frames_decoded,
                ofdm_frames = frame_count,
                fibs = fib_count,
                "MSC summary"
            );
        } else {
            info!(
                ofdm_frames = frame_count,
                fibs = fib_count,
                "No MSC frames decoded"
            );
            info!("MSC handler never initialized (service not found?)");
        }

        let underruns = audio_underruns.load(Ordering::Relaxed);
        if underruns > 0 {
            warn!(underruns, "Audio callback underruns detected");
        }

        let dropped = audio_dropped.load(Ordering::Relaxed);
        if dropped > 0 {
            warn!(dropped, "Dropped audio samples to avoid SDR backpressure");
        }

        // Write raw frames to output file if specified
        if let Some(ref output_path) = cli.output
            && !msc_output.is_empty()
        {
            let bytes_per_frame = msc_output[0].len();
            let mut all_bytes = Vec::with_capacity(msc_output.len() * bytes_per_frame);
            for frame_data in &msc_output {
                all_bytes.extend_from_slice(frame_data);
            }
            std::fs::write(output_path, &all_bytes)?;
            info!(bytes = all_bytes.len(), path = %output_path, "Wrote decoded MSC output");
        }

        if let Some(writer) = wav_writer.as_mut() {
            writer.finalize(AUDIO_RATE as u32, 2)?;
            if let Some(path) = &cli.wav {
                info!(path = %path, "Wrote decoded WAV audio");
            }
        }
    } else if !cli.traffic && !cli.dump_packets && !cli.announcements {
        // Just print service listing
        let output = ensemble.to_output();

        if cli.json {
            println!("{}", serde_json::to_string_pretty(&output)?);
        } else {
            info!(
                frames = frame_count,
                fibs = fib_count,
                "Processed OFDM/FIC frames"
            );
            print_services(&ensemble, false);
        }

        if frame_count == 0 {
            warn!("No OFDM frames decoded -- check input file and frequency");
        } else if fib_count == 0 {
            warn!(
                "Decoded {} OFDM frames but no valid FIBs -- sync or FEC issue",
                frame_count
            );
        }
    }

    Ok(())
}

async fn open_iq_source(
    input: &str,
    center_freq: u32,
    sample_rate: u32,
    chunk_size: usize,
    iq_format: IqFormat,
) -> Result<(IqChunkSource, bool, u32, Option<f64>), Box<dyn std::error::Error>> {
    // File and stdin: use synchronous IqSource (simple, reliable)
    if input == "-" {
        let source = IqSource::from_stdin(center_freq, sample_rate, chunk_size, iq_format)?;
        return Ok((
            IqChunkSource::Sync(Box::new(source)),
            true,
            sample_rate,
            None,
        ));
    }

    if !is_device_uri(input) {
        if Path::new(input)
            .extension()
            .and_then(|extension| extension.to_str())
            .is_some_and(|extension| extension.eq_ignore_ascii_case("wav"))
        {
            let source = IqSource::from_wav_iq_file(input, chunk_size)?;
            let wav_sample_rate = source
                .wav_iq_sample_rate()
                .expect("WAV-IQ source must expose its header sample rate");
            info!(
                path = input,
                sample_rate = wav_sample_rate,
                "Opening stereo PCM16 WAV-IQ capture"
            );
            return Ok((
                IqChunkSource::Sync(Box::new(source)),
                true,
                wav_sample_rate,
                None,
            ));
        }

        let source = IqSource::from_file(input, center_freq, sample_rate, chunk_size, iq_format)?;
        return Ok((
            IqChunkSource::Sync(Box::new(source)),
            true,
            sample_rate,
            None,
        ));
    }

    // Live SDR: use sync IqSource in a dedicated reader thread.
    // This architecture was historically smoother for DAB under heavy OFDM load.
    if input.starts_with("soapy://") {
        #[cfg(not(feature = "soapy"))]
        {
            return Err("soapy:// is not enabled. Rebuild with --features soapy".into());
        }
    }

    let (configured_uri, effective_gain) = ensure_tuning_query(input, center_freq, sample_rate);
    info!("Opening live SDR source: {}", configured_uri);
    let input_rate = detect_device_sample_rate(&configured_uri)?;

    let config = DeviceConfig::from_str(&configured_uri)?;
    let source = IqAsyncSource::from_device_config(&config).await?;
    Ok((
        IqChunkSource::Async {
            source: Box::new(source),
            chunk_size,
            pending: Vec::with_capacity(chunk_size * 2),
        },
        false,
        input_rate,
        effective_gain,
    ))
}

fn detect_device_sample_rate(configured_uri: &str) -> Result<u32, Box<dyn std::error::Error>> {
    let config = DeviceConfig::from_str(configured_uri)?;
    if let DeviceConfig::RtlSdr(cfg) = &config {
        return Ok(cfg.sample_rate);
    }
    #[cfg(feature = "soapy")]
    if let DeviceConfig::Soapy(cfg) = &config {
        return Ok(cfg.sample_rate as u32);
    }
    if let DeviceConfig::Airspy(cfg) = &config {
        return Ok(cfg.sample_rate);
    }
    if let DeviceConfig::HackRf(cfg) = &config {
        return Ok(cfg.sample_rate);
    }

    Err(std::io::Error::other(format!(
        "Unsupported SDR backend for sample-rate detection: {configured_uri}"
    ))
    .into())
}

fn ensure_tuning_query(uri: &str, center_freq_hz: u32, sample_rate: u32) -> (String, Option<f64>) {
    let has_query = uri.contains('?');
    let has_freq = uri.contains("freq=") || uri.contains("frequency=");
    let has_rate = uri.contains("rate=") || uri.contains("sample_rate=");
    let has_gain = uri.contains("gain=");
    let has_gain_mode = uri.contains("gain_mode=") || uri.contains("gain-mode=");
    let has_amp = uri.contains("amp=") || uri.contains("amp_enable=");

    let mut out = uri.to_string();
    if !has_query {
        out.push('?');
    }
    if !has_freq {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        out.push_str(&format!("freq={center_freq_hz}"));
    }
    if !has_rate {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        out.push_str(&format!("rate={sample_rate}"));
    }

    // For Airspy devices doing DAB, default to linearity gain mode with gain=40
    // Linearity mode is critical for OFDM: it optimizes for low intermodulation
    // distortion across all sub-carriers, whereas sensitivity mode optimizes for
    // weak-signal reception and can cause OFDM sync failures.
    if uri.starts_with("airspy://") {
        if !has_gain_mode {
            if !out.ends_with('?') && !out.ends_with('&') {
                out.push('&');
            }
            out.push_str("gain_mode=linearity");
        }
        if !has_gain {
            if !out.ends_with('?') && !out.ends_with('&') {
                out.push('&');
            }
            out.push_str("gain=40");
        }
    }

    // For RTL-SDR devices, default to gain=29.7 for balanced reception
    if uri.starts_with("rtlsdr://") && !has_gain {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        out.push_str("gain=29.7");
    }

    // For HackRF devices, default to gain=72 and enable the RF amp for reception.
    if uri.starts_with("hackrf://") {
        if !has_gain {
            if !out.ends_with('?') && !out.ends_with('&') {
                out.push('&');
            }
            out.push_str("gain=72");
        }
        if !has_amp {
            if !out.ends_with('?') && !out.ends_with('&') {
                out.push('&');
            }
            out.push_str("amp=true");
        }
    }

    // Extract effective gain from the final URI
    let effective_gain = out.split('?').nth(1).and_then(|qs| {
        qs.split('&')
            .find(|p| p.starts_with("gain="))
            .and_then(|p| p.strip_prefix("gain="))
            .and_then(|v| v.parse::<f64>().ok())
    });

    (out, effective_gain)
}

fn print_channels(json: bool) {
    let channels = constants::band_iii_channels();

    if json {
        let rows: Vec<serde_json::Value> = channels
            .iter()
            .map(|(name, hz)| {
                serde_json::json!({
                    "channel": name,
                    "frequency_hz": hz,
                    "frequency_mhz": (*hz as f64) / 1_000_000.0,
                })
            })
            .collect();
        if let Ok(s) = serde_json::to_string_pretty(&rows) {
            println!("{}", s);
        }
        return;
    }

    println!("DAB Band III channels:");
    println!("{:<8} {:<12} Frequency", "Channel", "Hz");
    println!("{}", "-".repeat(40));
    for (name, hz) in channels {
        println!(
            "{:<8} {:<12} {:.3} MHz",
            name,
            hz,
            (*hz as f64) / 1_000_000.0
        );
    }
}

fn parse_service_id_arg(service_arg: &str) -> Option<u32> {
    let hex = service_arg
        .strip_prefix("0x")
        .or_else(|| service_arg.strip_prefix("0X"))?;
    u32::from_str_radix(hex, 16).ok()
}

fn hash_bytes(data: &[u8]) -> u64 {
    let mut hasher = DefaultHasher::new();
    data.hash(&mut hasher);
    hasher.finish()
}

fn terminal_supports_inline_images() -> bool {
    if !std::io::stdout().is_terminal() {
        return false;
    }

    let term = std::env::var("TERM").unwrap_or_default().to_lowercase();
    let term_program = std::env::var("TERM_PROGRAM")
        .unwrap_or_default()
        .to_lowercase();

    std::env::var_os("KITTY_WINDOW_ID").is_some()
        || term.contains("kitty")
        || term.contains("ghostty")
        || term_program.contains("ghostty")
}

fn show_image_in_terminal(
    data: &[u8],
    ext: &str,
    mot_width: u32,
    mot_height: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut path = std::env::temp_dir();
    path.push(format!("dabradio_terminal_mot.{}", ext));
    std::fs::write(&path, data)?;

    let cfg = viuer::Config {
        absolute_offset: false,
        x: 0,
        y: 0,
        restore_cursor: false,
        width: (mot_width > 0).then_some(mot_width),
        height: (mot_height > 0).then_some(mot_height),
        transparent: true,
        premultiplied_alpha: false,
        truecolor: true,
        use_kitty: true,
        use_iterm: false,
    };

    viuer::print_from_file(&path, &cfg)?;
    std::io::stdout().flush()?;

    Ok(())
}

fn format_service_id(sid: u32) -> String {
    if sid > 0xFFFF {
        format!("0x{sid:08X}")
    } else {
        format!("0x{sid:04X}")
    }
}

/// Slow IIR DC tracker for direct-conversion captures.
///
/// In adaptive mode, engages only once `|mean|` exceeds `threshold` so clean
/// USRP/cf32 inputs are not altered. In force mode (digital downconvert),
/// always subtracts the running mean before the mix — matching the previous
/// shift!=0 behaviour, with a smoother estimate than per-chunk mean.
struct AdaptiveDcRemover {
    enabled: bool,
    force: bool,
    mean: Complex<f32>,
    alpha: f32,
    threshold: f32,
    engaged: bool,
}

impl AdaptiveDcRemover {
    fn new(enabled: bool, force: bool) -> Self {
        Self {
            enabled,
            force,
            mean: Complex::new(0.0, 0.0),
            alpha: 0.05,
            // ~2% of full-scale after cu8 normalization. The Belgian sample's
            // residual mean is ~0.001 (no spectral DC spike), so adaptive mode
            // stays off there while still catching real RTL LO leakage.
            threshold: 0.02,
            engaged: false,
        }
    }

    fn process(&mut self, samples: &mut [Complex<f32>]) {
        if !self.enabled || samples.is_empty() {
            return;
        }
        let chunk_mean = samples
            .iter()
            .fold(Complex::new(0.0f32, 0.0f32), |acc, s| acc + *s)
            / samples.len() as f32;
        self.mean = self.mean * (1.0 - self.alpha) + chunk_mean * self.alpha;
        if self.force || self.mean.norm() >= self.threshold {
            self.engaged = true;
        }
        if self.engaged {
            for s in samples.iter_mut() {
                *s -= self.mean;
            }
        }
    }
}

struct TrafficChannel {
    msc: msc::MscHandler,
    assemblers: Vec<(msc::packet::PacketAssembler, fic::fib::TpegTarget)>,
}

fn traffic_target_key(target: &fic::fib::TpegTarget) -> (u8, u16) {
    (target.subchannel.id, target.packet_address)
}

/// Arm new packet-mode targets as FIG information arrives, without resetting
/// MSC handlers already established for previously seen (subch, address) keys.
fn merge_traffic_channels(
    channels: &mut Vec<TrafficChannel>,
    armed: &mut std::collections::HashSet<(u8, u16)>,
    ensemble: &fic::fib::EnsembleInfo,
    features: &mut Vec<traffic::TrafficFeature>,
    dump_packets: bool,
    decode_tpeg: bool,
) {
    let targets = if dump_packets {
        ensemble.packet_mode_targets()
    } else {
        ensemble.tpeg_targets()
    };

    // Refresh metadata (e.g. late FIG 0/13 UAtypes) on already-armed targets.
    for ch in channels.iter_mut() {
        for (_, target) in &mut ch.assemblers {
            if let Some(updated) = targets
                .iter()
                .find(|t| traffic_target_key(t) == traffic_target_key(target))
            {
                *target = updated.clone();
            }
        }
    }

    for target in targets {
        let key = traffic_target_key(&target);
        if armed.contains(&key) {
            continue;
        }
        if decode_tpeg && target.is_tpeg() && (target.ca_flag || target.ca_org.is_some()) {
            let mut props = traffic::TrafficProperties::for_bearer(traffic::Bearer::DabTpeg);
            props.unsupported_ca = Some(true);
            props.encrypted = Some(true);
            props.service_id = Some(format_service_id(target.service_id));
            props.description =
                Some("Packet-mode component is CA-flagged; descrambling is not supported".into());
            features.push(traffic::TrafficFeature::new(props, None));
            info!(
                service = %format_service_id(target.service_id),
                "Skipping CA-protected TPEG component"
            );
            armed.insert(key);
            continue;
        }
        if target.is_tpeg() {
            info!(
                service = %format_service_id(target.service_id),
                label = target.label.as_deref().unwrap_or("(unlabelled)"),
                subchannel = target.subchannel.id,
                address = target.packet_address,
                bitrate = target.subchannel.bitrate,
                dscty = ?target.dscty,
                no_data_groups = target.no_data_groups,
                "TPEG component confirmed (UAtype 0x004)"
            );
        } else if dump_packets {
            info!(
                service = %format_service_id(target.service_id),
                label = target.label.as_deref().unwrap_or("(unlabelled)"),
                subchannel = target.subchannel.id,
                start_addr = target.subchannel.start_addr,
                address = target.packet_address,
                ua_types = ?target.ua_types,
                "Packet-mode component (not TPEG)"
            );
        }

        let subch_id = target.subchannel.id;
        let addr = target.packet_address;
        if let Some(ch) = channels.iter_mut().find(|c| {
            c.assemblers
                .first()
                .is_some_and(|(_, t)| t.subchannel.id == subch_id)
        }) {
            ch.assemblers
                .push((msc::packet::PacketAssembler::new(addr), target));
            armed.insert(key);
            continue;
        }

        let Some(msc_handler) = msc::MscHandler::new(&target.subchannel) else {
            warn!(
                subchannel = subch_id,
                "Failed to initialize packet-mode MSC handler"
            );
            continue;
        };
        channels.push(TrafficChannel {
            msc: msc_handler,
            assemblers: vec![(msc::packet::PacketAssembler::new(addr), target)],
        });
        armed.insert(key);
    }
}

fn print_fic_dump(ensemble: &fic::fib::EnsembleInfo, frame_count: usize, fib_count: usize) {
    let output = ensemble.to_output();
    let mut fig013_flat: Vec<serde_json::Value> = Vec::new();
    for ((sid, scids), apps) in &ensemble.user_applications {
        for ua in apps {
            fig013_flat.push(serde_json::json!({
                "sid": format_service_id(*sid),
                "scids": scids,
                "ua_type": format!("0x{:03X}", ua.ua_type),
                "ua_type_dec": ua.ua_type,
                "name": ua.name(),
                "data": ua.data,
            }));
        }
    }
    fig013_flat.sort_by(|a, b| {
        let sa = a["sid"].as_str().unwrap_or("");
        let sb = b["sid"].as_str().unwrap_or("");
        sa.cmp(sb)
            .then_with(|| {
                a["scids"]
                    .as_u64()
                    .unwrap_or(0)
                    .cmp(&b["scids"].as_u64().unwrap_or(0))
            })
            .then_with(|| {
                a["ua_type"]
                    .as_str()
                    .unwrap_or("")
                    .cmp(b["ua_type"].as_str().unwrap_or(""))
            })
    });
    let tpeg_hits: Vec<_> = fig013_flat
        .iter()
        .filter(|e| e["ua_type_dec"].as_u64() == Some(0x004))
        .cloned()
        .collect();
    let dump = serde_json::json!({
        "ensemble_id": ensemble.ensemble_id,
        "ensemble_id_hex": ensemble.ensemble_id.map(|e| format!("0x{e:04X}")),
        "ensemble_label": ensemble.ensemble_label,
        "ofdm_frames": frame_count,
        "fibs_decoded": fib_count,
        "subchannels": ensemble.subchannels,
        "packet_components": ensemble.packet_components,
        "scids_bindings": ensemble.scids_bindings.iter().map(|((sid, scids), b)| {
            serde_json::json!({
                "sid": format_service_id(*sid),
                "scids": scids,
                "binding": b,
            })
        }).collect::<Vec<_>>(),
        "fig_0_13_flat": fig013_flat,
        "tpeg_ua_type_0x004": tpeg_hits,
        "announcement_support": ensemble.announcement_support.iter().map(|(sid, s)| {
            serde_json::json!({
                "sid": format_service_id(*sid),
                "asu_flags": format!("0x{:04X}", s.asu_flags),
                "asu_flags_dec": s.asu_flags,
                "announcement_types": traffic::announcement_type_names(s.asu_flags),
                "traffic_relevant": traffic::is_traffic_relevant(s.asu_flags),
                "cluster_ids": s.cluster_ids,
            })
        }).collect::<Vec<_>>(),
        "announcement_switching": ensemble.announcement_switching.values().map(|sw| {
            serde_json::json!({
                "cluster_id": sw.cluster_id,
                "asw_flags": format!("0x{:04X}", sw.asw_flags),
                "asw_flags_dec": sw.asw_flags,
                "announcement_types": traffic::announcement_type_names(sw.asw_flags),
                "traffic_relevant": traffic::is_traffic_relevant(sw.asw_flags),
                "new_flag": sw.new_flag,
                "region_flag": sw.region_flag,
                "subchannel_id": sw.subchannel_id,
                "region_id": sw.region_id,
            })
        }).collect::<Vec<_>>(),
        "user_applications": ensemble.user_applications.iter().map(|((sid, scids), apps)| {
            serde_json::json!({
                "sid": format_service_id(*sid),
                "scids": scids,
                "apps": apps.iter().map(|ua| serde_json::json!({
                    "ua_type": format!("0x{:03X}", ua.ua_type),
                    "name": ua.name(),
                    "data": ua.data,
                })).collect::<Vec<_>>(),
            })
        }).collect::<Vec<_>>(),
        "services": output.services,
    });
    println!(
        "{}",
        serde_json::to_string_pretty(&dump).unwrap_or_default()
    );
}

fn emit_traffic_output(features: &[traffic::TrafficFeature], as_collection: bool) {
    if as_collection {
        println!(
            "{}",
            traffic::FeatureCollection::new(features.to_vec()).to_json()
        );
    } else {
        for f in features {
            println!("{}", f.to_json());
        }
    }
}

/// Try to find the requested service and initialize an MSC handler for it.
/// Returns the handler and the service bitrate.
fn try_init_msc(
    ensemble: &fic::fib::EnsembleInfo,
    service_arg: &str,
) -> Option<(msc::MscHandler, u16, fic::fib::AudioCoding)> {
    // Try to match by hex SId (e.g. "0xF201") or by label (case-insensitive)
    let target_sid = parse_service_id_arg(service_arg);

    let service = if let Some(sid) = target_sid {
        ensemble.services.get(&sid)
    } else {
        // Match by label (case-insensitive, trimmed)
        let arg_lower = service_arg.to_lowercase();
        ensemble.services.values().find(|s| {
            s.label
                .as_ref()
                .is_some_and(|l| l.trim().to_lowercase() == arg_lower)
        })
    };

    let service = service?;
    let subch_id = service.subchannel_id?;
    let subch = ensemble.subchannels.get(&subch_id)?;
    let bitrate = subch.bitrate;
    let coding = service.audio_coding?;
    if matches!(coding, fic::fib::AudioCoding::Other(_)) {
        warn!(?coding, "Unsupported DAB audio component type");
        return None;
    }

    info!(
        label = %service.label.as_deref().unwrap_or("(label pending)"),
        service_id = format_args!("0x{:04X}", service.service_id),
        subchannel_id = subch_id,
        subchannel_size_cu = subch.sub_size,
        uep_table_index = ?subch.uep_table_index,
        bitrate_kbps = bitrate,
        protection = %if subch.is_eep {
            let opt = if subch.eep_option == 0 { "A" } else { "B" };
            format!("EEP {}-{}", subch.protection_level + 1, opt)
        } else {
            format!("UEP {}", subch.protection_level)
        },
        ?coding,
        "Decoding service"
    );

    let handler = msc::MscHandler::new(subch);
    if handler.is_none() {
        warn!(subchannel_id = subch_id, "Failed to initialize MSC handler");
    }
    handler.map(|h| (h, bitrate, coding))
}

/// Consume and apply one pending TUI service selection.
fn apply_tui_service_switch(
    ensemble: &mut fic::fib::EnsembleInfo,
    tui_state: &Arc<Mutex<TuiState>>,
    msc_handler: &mut Option<msc::MscHandler>,
    dab_plus_decoder: &mut Option<audio::DabPlusDecoder>,
    mp2_decoder: &mut Option<audio::mp2::Mp2Decoder>,
    audio_primed: &AtomicBool,
    announced_service_label: &mut bool,
) -> bool {
    let switch_to = tui_state
        .try_lock()
        .ok()
        .and_then(|mut state| state.service_switch_request.take());
    let Some(label) = switch_to else {
        return false;
    };

    ensemble.resolve_services();
    let Some((new_handler, bitrate, coding)) = try_init_msc(ensemble, &label) else {
        warn!(service = %label, "Service switch failed: subchannel not ready yet");
        return false;
    };

    *msc_handler = Some(new_handler);
    match coding {
        fic::fib::AudioCoding::DabPlus => {
            *dab_plus_decoder = Some(audio::DabPlusDecoder::new(bitrate));
            *mp2_decoder = None;
        }
        fic::fib::AudioCoding::MpegLayer2 => {
            *dab_plus_decoder = None;
            *mp2_decoder = Some(audio::mp2::Mp2Decoder::new());
        }
        fic::fib::AudioCoding::Other(_) => unreachable!(),
    }
    audio_primed.store(false, Ordering::Relaxed);
    *announced_service_label = true;
    info!(service = %label, bitrate, "Switching service");
    if let Ok(mut state) = tui_state.try_lock() {
        state.service = label;
        state.status = format!("decoding @ {} kbps", bitrate);
        state.dls.clear();
        state.mot_count = 0;
        state.mot_info.clear();
        state.mot_filename = None;
        state.mot_preview_path = None;
        state.clear_mot_image = true;
    }
    true
}

fn print_services(ensemble: &fic::fib::EnsembleInfo, json: bool) {
    let output = ensemble.to_output();

    if json {
        if let Ok(s) = serde_json::to_string_pretty(&output) {
            println!("{}", s);
        }
        return;
    }

    if let Some(label) = &output.ensemble_label {
        println!(
            "Ensemble: {} (EId: 0x{:04X})",
            label,
            output.ensemble_id.unwrap_or(0)
        );
    } else if let Some(eid) = output.ensemble_id {
        println!("Ensemble: EId 0x{:04X}", eid);
    }

    if output.services.is_empty() {
        println!("No services found.");
        return;
    }

    println!("\nServices:");
    println!(
        "{:<8} {:<20} {:<6} {:<10} Protection",
        "SId", "Label", "SubCh", "Bitrate"
    );
    println!("{}", "-".repeat(60));
    for svc in &output.services {
        println!(
            "{:<8} {:<20} {:<6} {:<10} {}",
            svc.service_id,
            svc.label.as_deref().unwrap_or("(unknown)"),
            svc.subchannel_id
                .map(|id| format!("{}", id))
                .unwrap_or_default(),
            svc.bitrate
                .map(|br| format!("{} kbps", br))
                .unwrap_or_default(),
            svc.protection.as_deref().unwrap_or(""),
        );
        for c in &svc.components {
            if c.tmid == 3 {
                let apps = if c.user_applications.is_empty() {
                    String::new()
                } else {
                    format!(" {}", c.user_applications.join(", "))
                };
                println!(
                    "         packet SCId={:?} addr={:?} subch={:?} CA={}{}",
                    c.scid, c.packet_address, c.subchannel_id, c.ca, apps
                );
            }
        }
    }
}

/// Find the next gain step above `current` (or the max if already at/above the top).
fn next_gain_up(current: f64, steps: &[f64]) -> f64 {
    const EPS: f64 = 0.05;
    for &s in steps {
        if s > current + EPS {
            return s;
        }
    }
    steps.last().copied().unwrap_or(current)
}

/// Find the next gain step below `current` (or the min if already at/below the bottom).
fn next_gain_down(current: f64, steps: &[f64]) -> f64 {
    const EPS: f64 = 0.05;
    for &s in steps.iter().rev() {
        if s < current - EPS {
            return s;
        }
    }
    steps.first().copied().unwrap_or(current)
}

/// Restart the PipeWire/PulseAudio audio server to release a stuck sound card.
///
/// tinyaudio opens ALSA directly, which can grab the hardware device exclusively.
/// When the process is force-killed (or Drop hangs and we call process::exit),
/// the ALSA handle is never closed and the sound card stays locked until the
/// audio server reclaims it. This function does that automatically.
fn restart_audio_server() {
    // Try PipeWire first (modern distros), then PulseAudio as fallback.
    let commands: &[&[&str]] = &[
        &[
            "systemctl",
            "--user",
            "restart",
            "pipewire",
            "wireplumber",
            "pipewire-pulse",
        ],
        &["systemctl", "--user", "restart", "pulseaudio"],
    ];
    for cmd in commands {
        match std::process::Command::new(cmd[0])
            .args(&cmd[1..])
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .status()
        {
            Ok(status) if status.success() => {
                eprintln!("Audio server restarted ({})", cmd.join(" "));
                return;
            }
            _ => continue,
        }
    }
    eprintln!(
        "Could not restart audio server automatically. Try: systemctl --user restart pipewire wireplumber pipewire-pulse"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hackrf_default_uri_enables_rf_amp() {
        let (uri, effective_gain) = ensure_tuning_query("hackrf://", 223_936_000, 2_048_000);

        assert_eq!(
            uri,
            "hackrf://?freq=223936000&rate=2048000&gain=72&amp=true"
        );
        assert_eq!(effective_gain, Some(72.0));
    }

    #[test]
    fn hackrf_explicit_amp_is_preserved() {
        let (uri, effective_gain) =
            ensure_tuning_query("hackrf://?amp=false", 223_936_000, 2_048_000);

        assert_eq!(
            uri,
            "hackrf://?amp=false&freq=223936000&rate=2048000&gain=72"
        );
        assert_eq!(effective_gain, Some(72.0));
    }

    #[test]
    fn adaptive_dc_remover_ignores_clean_signal_below_threshold() {
        let mut dc = AdaptiveDcRemover::new(true, false);
        let mut samples = vec![Complex::new(0.001, -0.0005); 256];
        let before = samples[0];
        dc.process(&mut samples);
        assert!(!dc.engaged);
        assert_eq!(samples[0], before);
    }

    #[test]
    fn adaptive_dc_remover_engages_on_large_offset() {
        let mut dc = AdaptiveDcRemover::new(true, false);
        let mut samples = vec![Complex::new(0.2, 0.0); 256];
        // alpha=0.05 → need ~100 chunks to converge the running mean.
        for _ in 0..120 {
            dc.process(&mut samples);
            samples = vec![Complex::new(0.2, 0.0); 256];
        }
        assert!(dc.engaged);
        dc.process(&mut samples);
        let mean_re = samples.iter().map(|s| s.re).sum::<f32>() / samples.len() as f32;
        assert!(
            mean_re.abs() < 0.01,
            "expected near-zero residual after DC removal, got {mean_re}"
        );
    }

    #[test]
    fn adaptive_dc_remover_force_mode_always_subtracts() {
        let mut dc = AdaptiveDcRemover::new(true, true);
        let mut samples = vec![Complex::new(0.001, 0.0); 128];
        dc.process(&mut samples);
        assert!(dc.engaged);
    }

    #[test]
    fn file_tui_pauses_for_service_selection_after_label_cycles() {
        assert!(!should_pause_for_tui_selection(
            true, true, false, false, 49, true
        ));
        assert!(should_pause_for_tui_selection(
            true, true, false, false, 50, true
        ));
    }

    #[test]
    fn tui_does_not_pause_live_or_preselected_decoding() {
        assert!(!should_pause_for_tui_selection(
            true, false, false, false, 50, true
        ));
        assert!(!should_pause_for_tui_selection(
            true, true, true, false, 50, true
        ));
        assert!(!should_pause_for_tui_selection(
            true, true, false, true, 50, true
        ));
    }

    fn test_packet_subch(id: u8) -> fic::fib::SubchannelInfo {
        fic::fib::SubchannelInfo {
            id,
            start_addr: 84,
            sub_size: 6,
            protection_level: 2,
            is_eep: true,
            eep_option: 0,
            uep_table_index: None,
            bitrate: 8,
        }
    }

    fn ensemble_with_packet_tpeg(with_ua: bool) -> fic::fib::EnsembleInfo {
        let mut decoder = fic::fib::EnsembleInfo::new();
        let subch = test_packet_subch(12);
        decoder.subchannels.insert(12, subch.clone());
        decoder.packet_components.insert(
            0x001,
            fic::fib::PacketComponent {
                scid: 0x001,
                subchannel_id: 12,
                packet_address: 852,
                dscty: 5,
                no_data_groups: false,
                ca_org: None,
            },
        );
        let mut service = fic::fib::ServiceInfo {
            service_id: 0xF201,
            label: Some("Pkt".into()),
            ..Default::default()
        };
        let mut component = fic::fib::ServiceComponent {
            tmid: 3,
            scid: Some(0x001),
            subchannel_id: Some(12),
            packet_address: Some(852),
            dscty: Some(5),
            ps_flag: true,
            ..Default::default()
        };
        if with_ua {
            component.scids = Some(0);
            component.user_applications = vec![fic::fib::UserApplication {
                ua_type: fic::fib::UserApplication::TPEG,
                data: vec![],
            }];
            decoder
                .user_applications
                .insert((0xF201, 0), component.user_applications.clone());
        }
        service.components.push(component);
        decoder.services.insert(0xF201, service);
        decoder.resolve_services();
        decoder
    }

    #[test]
    fn merge_traffic_channels_arms_late_tpeg_without_resetting() {
        let mut channels = Vec::new();
        let mut armed = std::collections::HashSet::new();
        let mut features = Vec::new();

        // Completeness without FIG 0/13: nothing to arm for --traffic.
        let early = ensemble_with_packet_tpeg(false);
        merge_traffic_channels(
            &mut channels,
            &mut armed,
            &early,
            &mut features,
            false,
            true,
        );
        assert!(channels.is_empty());
        assert!(armed.is_empty());

        // Late FIG 0/13: arm once.
        let late = ensemble_with_packet_tpeg(true);
        merge_traffic_channels(&mut channels, &mut armed, &late, &mut features, false, true);
        assert_eq!(channels.len(), 1);
        assert_eq!(channels[0].assemblers.len(), 1);
        let keys_after_first = armed.clone();

        // Re-merge must not reset or duplicate MSC handlers.
        merge_traffic_channels(&mut channels, &mut armed, &late, &mut features, false, true);
        assert_eq!(channels.len(), 1);
        assert_eq!(channels[0].assemblers.len(), 1);
        assert_eq!(armed, keys_after_first);
        assert!(channels[0].assemblers[0].1.is_tpeg());
    }
}
