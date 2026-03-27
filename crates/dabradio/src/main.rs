//! dabradio — DAB/DAB+ digital radio decoder.
//!
//! Reads IQ samples and decodes DAB ensemble information (services, labels, subchannels).

mod audio;
mod charsets;
mod constants;
mod fec;
mod fic;
mod msc;
mod ofdm;
mod pad;

use clap::Parser;
use crossbeam_channel as channel;
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
use desperado::DeviceConfig;
#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
use desperado::IqAsyncSource;
use desperado::dsp::DspBlock;
#[cfg(feature = "resampler")]
use desperado::dsp::dab_resampler::DabResampler;
use desperado::dsp::rotate::Rotate;
use desperado::{IqFormat, IqSource};
#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
use futures::StreamExt;
use num_complex::Complex;
use pad::PadData;
use ratatui::{
    Terminal, TerminalOptions, Viewport,
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout, Rect},
    text::Line,
    widgets::{Block, Borders, Paragraph},
};
use std::collections::hash_map::DefaultHasher;
use std::fs::File;
use std::hash::{Hash, Hasher};
use std::io::{IsTerminal, Seek, SeekFrom, Write};
#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tinyaudio::prelude::*;
use tracing::Level;
use tracing::{debug, info, warn};
use tracing_subscriber::prelude::*;

/// Unified IQ chunk iterator that works with both sync and async sources.
///
/// For file/stdin sources, uses synchronous `IqSource` (simple, no threading).
/// For live SDR sources, uses `IqAsyncSource` directly.
enum IqChunkSource {
    Sync(Box<IqSource>),
    #[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
    Async {
        source: IqAsyncSource,
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
            #[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
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
}

const AUDIO_RATE: usize = 48_000;

#[derive(Default, Clone)]
struct TuiState {
    source: String,
    channel: String,
    center_freq_hz: u32,
    input_rate_hz: u32,
    output_rate_hz: u32,
    ofdm_frames: usize,
    fibs: usize,
    msc_frames: usize,
    service: String,
    dls: String,
    mot_count: usize,
    mot_info: String,
    mot_preview_path: Option<String>,
    audio_q_fill: usize,
    status: String,
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
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let _ = enable_raw_mode();
        let backend = CrosstermBackend::new(std::io::stdout());
        let mut terminal = match Terminal::with_options(
            backend,
            TerminalOptions {
                viewport: Viewport::Inline(18),
            },
        ) {
            Ok(t) => t,
            Err(_) => {
                let _ = disable_raw_mode();
                eprintln!("dabradio: failed to initialize TUI terminal backend");
                return;
            }
        };

        while running.load(Ordering::Relaxed) {
            if let Ok(s) = state.lock() {
                let _ = terminal.draw(|f| {
                    let area = f.area();
                    let cols = Layout::default()
                        .direction(Direction::Horizontal)
                        .constraints([Constraint::Percentage(66), Constraint::Percentage(34)])
                        .split(area);
                    let left = cols[0];
                    let right = cols[1];
                    let inner = left.width.saturating_sub(4) as usize;
                    let controls = if keyboard_available {
                        " Controls: q/Esc/Ctrl-C quit"
                    } else {
                        " Controls: Ctrl-C quit (stdin is piped)"
                    };
                    let left_lines = vec![
                        Line::from(format!(
                            " Source: {}",
                            one_line(&s.source, inner.saturating_sub(9))
                        )),
                        Line::from(format!(
                            " Channel: {}   Freq: {:.3} MHz",
                            one_line(&s.channel, 8),
                            s.center_freq_hz as f64 / 1_000_000.0
                        )),
                        Line::from(format!(
                            " Input: {} Hz   Output: {} Hz",
                            s.input_rate_hz, s.output_rate_hz
                        )),
                        Line::from(""),
                        Line::from(format!(
                            " OFDM frames: {}   FIBs: {}   MSC frames: {}",
                            s.ofdm_frames, s.fibs, s.msc_frames
                        )),
                        Line::from(format!(
                            " Service: {}",
                            one_line(&s.service, inner.saturating_sub(10))
                        )),
                        Line::from(format!(
                            " DLS: {}",
                            one_line(&s.dls, inner.saturating_sub(6))
                        )),
                        Line::from(format!(
                            " MOT: {}   {}",
                            s.mot_count,
                            one_line(&s.mot_info, inner.saturating_sub(15))
                        )),
                        Line::from(format!(" Audio queue: {} samples", s.audio_q_fill)),
                        Line::from(""),
                        Line::from(format!(
                            " Status: {}",
                            one_line(&s.status, inner.saturating_sub(9))
                        )),
                        Line::from(""),
                        Line::from(controls),
                    ];

                    let left_panel = Paragraph::new(left_lines).block(
                        Block::default()
                            .title(" DAB Radio Dashboard ")
                            .borders(Borders::ALL),
                    );
                    f.render_widget(left_panel, left);

                    let right_lines = vec![
                        Line::from(format!(" MOT count: {}", s.mot_count)),
                        Line::from(format!(
                            " {}",
                            one_line(&s.mot_info, right.width.saturating_sub(4) as usize)
                        )),
                        Line::from(""),
                    ];

                    let right_panel = Paragraph::new(right_lines).block(
                        Block::default()
                            .title(" MOT Preview ")
                            .borders(Borders::ALL),
                    );
                    f.render_widget(right_panel, right);

                    if inline_image_support && let Some(path) = &s.mot_preview_path {
                        draw_tui_image(path, right);
                    }
                });
            }

            if keyboard_available && event::poll(Duration::from_millis(1)).unwrap_or(false) {
                match event::read() {
                    Ok(Event::Resize(_, _)) => {
                        let _ = terminal.autoresize();
                        let _ = terminal.clear();
                    }
                    Ok(Event::Key(key)) if key.kind == KeyEventKind::Press => {
                        if matches!(key.code, KeyCode::Esc | KeyCode::Char('q')) {
                            running.store(false, Ordering::Relaxed);
                            break;
                        }
                    }
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
    let cfg = viuer::Config {
        absolute_offset: true,
        x: area.x.saturating_add(1),
        y: area.y.saturating_add(4) as i16,
        restore_cursor: false,
        width: Some(area.width.saturating_sub(2) as u32),
        height: Some(area.height.saturating_sub(5) as u32),
        transparent: true,
        premultiplied_alpha: false,
        truecolor: true,
        use_kitty: true,
        use_iterm: false,
    };
    let _ = viuer::print_from_file(path, &cfg);
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

    /// IQ format for file sources: cu8, cs8, cs16, cf32
    #[arg(long, default_value = "cu8")]
    format: String,

    /// Input sample rate in samples/s (needed for non-2.048MS/s file/stdin sources)
    #[arg(long)]
    sample_rate: Option<u32>,

    /// Shift input IQ by this frequency before demodulation (Hz)
    #[arg(long, default_value = "0")]
    freq_shift_hz: i32,

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

    /// Output file for decoded data (raw DAB+ logical frames)
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

    // Parse IQ format
    let iq_format = match cli.format.as_str() {
        "cu8" => IqFormat::Cu8,
        "cs8" => IqFormat::Cs8,
        "cs16" => IqFormat::Cs16,
        "cf32" => IqFormat::Cf32,
        other => return Err(format!("Unknown IQ format: {}", other).into()),
    };

    info!(
        "Opening {} (freq={} Hz, format={})",
        source, center_freq, cli.format
    );

    // Open IQ source (file or live SDR URI)
    // - stdin/file: use --sample-rate when provided
    // - airspy:// default: 6.0 MS/s (then resample to 2.048 MS/s)
    // - others default: 2.048 MS/s
    let default_input_rate = cli.sample_rate.unwrap_or_else(|| {
        if source.starts_with("airspy://") {
            6_000_000
        } else {
            constants::SAMPLE_RATE
        }
    });
    let is_rtlsdr_uri = source.starts_with("rtlsdr://");

    let chunk_size = constants::T_F; // One frame's worth of samples
    let (mut source, is_file_source, input_sample_rate) = open_iq_source(
        source,
        center_freq,
        default_input_rate,
        chunk_size,
        iq_format,
    )
    .await?;
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

    #[cfg(feature = "resampler")]
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

    #[cfg(not(feature = "resampler"))]
    if input_sample_rate != constants::SAMPLE_RATE {
        return Err(format!(
            "Input sample rate is {} but DAB requires {}; rebuild with --features resampler (or airspy)",
            input_sample_rate,
            constants::SAMPLE_RATE
        )
        .into());
    }

    let mut ofdm_processor = ofdm::processor::OfdmProcessor::new();
    let mut ensemble = fic::fib::EnsembleInfo::new();
    let mut frame_count = 0usize;
    let mut fib_count = 0usize;

    // MSC decoding state (initialized once we know the service -> subchannel mapping)
    let mut msc_handler: Option<msc::MscHandler> = None;
    let mut dab_plus_decoder: Option<audio::DabPlusDecoder> = None;
    let mut msc_output: Vec<Vec<u8>> = Vec::new();
    let decoding_service = cli.service.is_some();
    let selected_service_sid = cli.service.as_deref().and_then(parse_service_id_arg);
    let mut announced_service_label = false;
    let mut mot_image_count = 0usize;
    let mut last_mot_hash: Option<u64> = None;
    let keyboard_available = std::io::stdin().is_terminal();
    let inline_image_support = terminal_supports_inline_images();
    let term_image_support = inline_image_support && !tui_enabled;

    let app_running = Arc::new(AtomicBool::new(true));
    let tui_state = Arc::new(Mutex::new(TuiState {
        source: cli.source.clone().unwrap_or_default(),
        channel: cli
            .channel
            .clone()
            .unwrap_or_else(|| "(custom)".to_string()),
        center_freq_hz: center_freq,
        input_rate_hz: input_sample_rate,
        output_rate_hz: constants::SAMPLE_RATE,
        status: if tui_enabled && !keyboard_available {
            "stdin piped: q/esc disabled; use Ctrl-C".to_string()
        } else {
            "starting".to_string()
        },
        ..Default::default()
    }));
    let _tui_guard = if tui_enabled {
        Some(TuiGuard {
            running: app_running.clone(),
            handle: Some(spawn_dab_tui(
                tui_state.clone(),
                app_running.clone(),
                keyboard_available,
                inline_image_support,
            )),
        })
    } else {
        None
    };

    let sig_running = app_running.clone();
    tokio::spawn(async move {
        if tokio::signal::ctrl_c().await.is_ok() {
            sig_running.store(false, Ordering::Relaxed);
        }
    });

    let mut iq_rotator = if cli.freq_shift_hz != 0 {
        let angle = -2.0f32 * std::f32::consts::PI * (cli.freq_shift_hz as f32)
            / (input_sample_rate as f32);
        info!(
            shift_hz = cli.freq_shift_hz,
            "Applying input frequency shift"
        );
        Some(Rotate::new(angle))
    } else {
        None
    };

    // Audio output setup (tinyaudio + crossbeam channel)
    // Queue is 4 seconds deep — enough to absorb decode bursts.
    // Prebuffer is small (0.25 s for live, 1 s for file) so audio starts quickly.
    // PCM enqueue is blocking for both paths: no samples are ever silently dropped.
    // The IQ bridge chain (sync_channel(15) + tokio mpsc(4) ≈ 1.2 s deep) absorbs
    // any stall while the audio queue drains to make room.
    let audio_queue_seconds = 4;
    let (tx, rx) = channel::bounded::<f32>(AUDIO_RATE * 2 * audio_queue_seconds);
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

    let _device = if decoding_service && !cli.no_audio {
        let config = OutputDeviceParameters {
            channels_count: 2, // DAB+ HE-AAC v2 typically outputs stereo
            sample_rate: AUDIO_RATE,
            channel_sample_count: 1024,
        };
        let audio_underruns_cb = Arc::clone(&audio_underruns);
        let audio_primed_cb = Arc::clone(&audio_primed);
        let audio_measure_active_cb = Arc::clone(&audio_measure_active);
        Some(
            run_output_device(config, move |data| {
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
            })
            .expect("Failed to open audio output device"),
        )
    } else {
        None
    };

    let mut wav_writer = if decoding_service {
        if let Some(path) = &cli.wav {
            Some(WavWriter::create(path, AUDIO_RATE as u32, 2)?)
        } else {
            None
        }
    } else {
        None
    };

    while app_running.load(Ordering::Relaxed) {
        let Some(chunk) = source.next_chunk().await else {
            break;
        };
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
        let samples = if let Some(ref mut rotator) = iq_rotator {
            rotator.process(&samples)
        } else {
            samples
        };
        #[cfg(feature = "resampler")]
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
        let frames = ofdm_processor.process(&samples);

        for frame in &frames {
            if !app_running.load(Ordering::Relaxed) {
                break;
            }
            frame_count += 1;
            if let Ok(mut s) = tui_state.try_lock() {
                s.ofdm_frames = frame_count;
            }

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

                for fib in &fibs {
                    fib_count += 1;
                    ensemble.parse_fib(fib);
                }
                if let Ok(mut s) = tui_state.try_lock() {
                    s.fibs = fib_count;
                    s.status = if fib_count > 0 {
                        "sync + FIC OK".to_string()
                    } else {
                        "OFDM sync only".to_string()
                    };
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
                    s.service = label.trim().to_string();
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

            // Try to initialize MSC handler if we have service info but no handler yet
            if decoding_service && msc_handler.is_none() && ensemble.has_services() {
                ensemble.resolve_services();
                if let Some(ref service_arg) = cli.service
                    && let Some((handler, bitrate)) = try_init_msc(&ensemble, service_arg)
                {
                    msc_handler = Some(handler);
                    dab_plus_decoder = Some(audio::DabPlusDecoder::new(bitrate));
                    if let Ok(mut s) = tui_state.try_lock() {
                        s.status = format!("decoding @ {} kbps", bitrate);
                        if s.service.is_empty() {
                            s.service = service_arg.clone();
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
                        if let Ok(mut s) = tui_state.try_lock() {
                            s.msc_frames = handler.frames_decoded;
                        }
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
                                            s.mot_info = format!(
                                                "{} bytes {}",
                                                mot.data.len(),
                                                mot.content_type
                                            );
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

                                // Blocking send for both live and file sources: no samples are
                                // ever silently dropped. The IQ bridge chain (~1.2 s deep) absorbs
                                // any stall while the audio callback drains the queue.
                                // block_in_place lets tokio know this thread will block briefly.
                                tokio::task::block_in_place(|| {
                                    for sample in &decoded_out.pcm {
                                        if tx.send(*sample).is_err() {
                                            break;
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
                                    let sf_ok_pct = if sf_attempted > 0 {
                                        sf_decoded * 100 / sf_attempted
                                    } else {
                                        0
                                    };
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
                                        tracing::warn!(
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
    } else {
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
) -> Result<(IqChunkSource, bool, u32), Box<dyn std::error::Error>> {
    // File and stdin: use synchronous IqSource (simple, reliable)
    if input == "-" {
        let source = IqSource::from_stdin(center_freq, sample_rate, chunk_size, iq_format)?;
        return Ok((IqChunkSource::Sync(Box::new(source)), true, sample_rate));
    }

    if !is_device_uri(input) {
        let source = IqSource::from_file(input, center_freq, sample_rate, chunk_size, iq_format)?;
        return Ok((IqChunkSource::Sync(Box::new(source)), true, sample_rate));
    }

    // Live SDR: use sync IqSource in a dedicated reader thread.
    // This architecture was historically smoother for DAB under heavy OFDM load.
    if input.starts_with("rtlsdr://") {
        #[cfg(not(feature = "rtlsdr"))]
        {
            return Err("rtlsdr:// is not enabled. Rebuild with --features rtlsdr, \
                 or pipe: rtl_sdr -f FREQ -s 2048000 - | dabradio - --freq FREQ --format cu8"
                .into());
        }
    }
    if input.starts_with("soapy://") {
        #[cfg(not(feature = "soapy"))]
        {
            return Err("soapy:// is not enabled. Rebuild with --features soapy".into());
        }
    }
    if input.starts_with("airspy://") {
        #[cfg(not(feature = "airspy"))]
        {
            return Err("airspy:// is not enabled. Rebuild with --features airspy".into());
        }
    }

    let configured_uri = ensure_tuning_query(input, center_freq, sample_rate);
    info!("Opening live SDR source: {}", configured_uri);
    let input_rate = detect_device_sample_rate(&configured_uri)?;
    #[cfg(not(any(feature = "rtlsdr", feature = "soapy", feature = "airspy")))]
    let _ = input_rate;

    #[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
    {
        let config = DeviceConfig::from_str(&configured_uri)?;
        let source = IqAsyncSource::from_device_config(&config).await?;
        Ok((
            IqChunkSource::Async {
                source,
                chunk_size,
                pending: Vec::with_capacity(chunk_size * 2),
            },
            false,
            input_rate,
        ))
    }

    #[cfg(not(any(feature = "rtlsdr", feature = "soapy", feature = "airspy")))]
    Err(
        format!("No SDR backend enabled for URI: {configured_uri}. Rebuild with --features rtlsdr")
            .into(),
    )
}

#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
fn detect_device_sample_rate(configured_uri: &str) -> Result<u32, Box<dyn std::error::Error>> {
    let config = DeviceConfig::from_str(configured_uri)?;
    #[cfg(feature = "rtlsdr")]
    if let DeviceConfig::RtlSdr(cfg) = &config {
        return Ok(cfg.sample_rate);
    }
    #[cfg(feature = "soapy")]
    if let DeviceConfig::Soapy(cfg) = &config {
        return Ok(cfg.sample_rate as u32);
    }
    #[cfg(feature = "airspy")]
    if let DeviceConfig::Airspy(cfg) = &config {
        return Ok(cfg.sample_rate);
    }

    Err(std::io::Error::other(format!(
        "Unsupported SDR backend for sample-rate detection: {configured_uri}"
    ))
    .into())
}

#[cfg(not(any(feature = "rtlsdr", feature = "soapy", feature = "airspy")))]
fn detect_device_sample_rate(_configured_uri: &str) -> Result<u32, Box<dyn std::error::Error>> {
    Err(std::io::Error::other("No SDR backend enabled").into())
}

fn is_device_uri(input: &str) -> bool {
    input.starts_with("rtlsdr://")
        || input.starts_with("soapy://")
        || input.starts_with("airspy://")
}

fn ensure_tuning_query(uri: &str, center_freq_hz: u32, sample_rate: u32) -> String {
    let has_query = uri.contains('?');
    let has_freq = uri.contains("freq=") || uri.contains("frequency=");
    let has_rate = uri.contains("rate=") || uri.contains("sample_rate=");

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
    out
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

/// Try to find the requested service and initialize an MSC handler for it.
/// Returns the handler and the service bitrate.
fn try_init_msc(
    ensemble: &fic::fib::EnsembleInfo,
    service_arg: &str,
) -> Option<(msc::MscHandler, u16)> {
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

    info!(
        label = %service.label.as_deref().unwrap_or("(label pending)"),
        service_id = format_args!("0x{:04X}", service.service_id),
        subchannel_id = subch_id,
        bitrate_kbps = bitrate,
        protection = %if subch.is_eep {
            let opt = if subch.eep_option == 0 { "A" } else { "B" };
            format!("EEP {}-{}", subch.protection_level + 1, opt)
        } else {
            format!("UEP {}", subch.protection_level)
        },
        "Decoding service"
    );

    let handler = msc::MscHandler::new(subch);
    if handler.is_none() {
        warn!(subchannel_id = subch_id, "Failed to initialize MSC handler");
    }
    handler.map(|h| (h, bitrate))
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
        eprintln!(
            "Ensemble: {} (EId: 0x{:04X})",
            label,
            output.ensemble_id.unwrap_or(0)
        );
    } else if let Some(eid) = output.ensemble_id {
        eprintln!("Ensemble: EId 0x{:04X}", eid);
    }

    if output.services.is_empty() {
        eprintln!("No services found.");
        return;
    }

    eprintln!("\nServices:");
    eprintln!(
        "{:<8} {:<20} {:<6} {:<10} Protection",
        "SId", "Label", "SubCh", "Bitrate"
    );
    eprintln!("{}", "-".repeat(60));
    for svc in &output.services {
        eprintln!(
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
    }
}
