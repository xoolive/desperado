//! FM Demodulator for various IQ sources (mono or stereo with RDS)
//!
//! This application demonstrates FM demodulation with support for:
//! - RTL-SDR devices
//! - Airspy devices
//! - SoapySDR-compatible devices
//! - IQ file playback
//! - Mono or stereo decoding
//! - RDS (Radio Data System) decoding
//!
//! # Usage Examples
//!
//! ## RTL-SDR (mono)
//! ```bash
//! fmradio rtlsdr:// -c 105.1M
//! ```
//!
//! ## Airspy (mono)
//! ```bash
//! fmradio airspy:// -c 105.1M
//! ```
//!
//! ## RTL-SDR (stereo with RDS)
//! ```bash
//! fmradio rtlsdr:// -c 105.1M -v
//! ```
//!
//! ## SoapySDR
//! ```bash
//! fmradio 'soapy://?driver=hackrf' -c 105.1M
//! ```
//!
//! ## IQ File Playback
//! ```bash
//! fmradio samples.iq -c 105.1M --format cu8
//! ```

use crossbeam::channel;
use desperado::dsp::{
    DspBlock, afc::SquareFreqOffsetCorrection, decimator::Decimator, filters::LowPassFir,
    rotate::Rotate,
};
use fmradio::fm::{DeemphasisFilter, PhaseExtractor};
use fmradio::rds::{RdsDecoder, RdsResamplerCustom};
use futures::StreamExt;
use ratatui::style::{Color, Modifier, Style};
use std::f32::consts::PI;
use std::io::{Write, stderr, stdout};
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tracing::{Level, debug, info, warn};
use tracing_subscriber::prelude::*;

use clap::{ArgAction, Parser};
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use desperado::rtlsdr::RtlSdrMessage;
use desperado::{DeviceConfig, IqAsyncSource, IqFormat, iq_level_dbfs};

use ratatui::{
    Terminal, TerminalOptions, Viewport,
    backend::CrosstermBackend,
    layout::{Constraint, Layout},
    text::Line,
    widgets::{Block, Borders, Paragraph},
};
use rubato::{
    Resampler, SincFixedOut, SincInterpolationParameters, SincInterpolationType, WindowFunction,
};
use tinyaudio::prelude::*;

#[derive(Debug, Clone, Copy)]
struct Frequency(u32);

#[derive(Parser, Debug)]
#[command(author, version, about = "FM demodulator from various IQ sources (mono or stereo)", long_about = None)]
struct Args {
    /// Source: file path, "-" (stdin), or SDR URI (rtlsdr://, airspy://, soapy://)
    source: String,

    /// Center frequency in Hz (accepts k/M suffix, e.g. 105.1M)
    #[arg(short, long, value_parser = Frequency::from_str)]
    center_freq: Frequency,

    /// Sample rate in Hz
    #[arg(short, long, default_value_t = 2_000_000)]
    sample_rate: u32,

    /// Tuner gain (None for AGC, Some(gain) for manual)
    #[arg(short, long, default_value = None)]
    gain: Option<i32>,

    /// Frequency offset in Hz (can be negative)
    #[arg(short, long, default_value_t = 200_000, allow_hyphen_values = true)]
    offset_freq: i32,

    /// Enable automatic frequency correction
    #[arg(short, long, default_value_t = false)]
    afc: bool,

    /// Force mono decoding (default is stereo)
    #[arg(short = 'm', long, default_value_t = false)]
    mono: bool,

    /// Verbosity level (-v=info, -vv=debug, -vvv=trace)
    #[arg(short, long, action = ArgAction::Count)]
    verbose: u8,

    /// Disable audio output (for SSH/headless operation)
    #[arg(long, default_value_t = false)]
    no_audio: bool,

    /// Show inline text UI (non-fullscreen) with live FM/RDS stats
    #[arg(long, default_value_t = false)]
    tui: bool,

    /// IQ format for file input (cu8, cs8, cs16, cf32)
    #[arg(long, default_value = "cu8")]
    format: String,

    /// Output raw FM-demodulated MPX signal to stdout (for piping to redsea)
    /// Format: signed 16-bit PCM at native MPX rate (use --resample-out for redsea)
    #[arg(long, default_value_t = false)]
    raw_out: bool,

    /// Resample raw output to this rate. Recommended: 171000 (3×57kHz) or 228000 (4×57kHz)
    /// for optimal RDS decoding with redsea
    #[arg(long)]
    resample_out: Option<u32>,
}

const FM_BANDWIDTH: f32 = 240_000.0;
const MONO_SIGNAL_BW: f32 = 15_000.0;
const AUDIO_RATE: usize = 48_000;

#[derive(Default, Clone)]
struct TuiState {
    mode: &'static str,
    source: String,
    center_freq_hz: u32,
    alt_freq_hz: Option<u32>,
    sample_rate_hz: u32,
    mpx_rate_hz: f32,
    iq_level_dbfs: f32,
    audio_buf_fill: usize,
    rds_ps: String,
    rds_rt: String,
    rds_total_blocks: u64,
    rds_valid_blocks: u64,
    rds_groups: u64,
    rds_language: String,
    rds_ct: String,
    program_type: String,
}

fn spawn_inline_tui(
    state: Arc<Mutex<TuiState>>,
    running: Arc<AtomicBool>,
    stereo_enabled: Arc<AtomicBool>,
    adjust_tx: tokio::sync::mpsc::UnboundedSender<RtlSdrMessage>,
    can_hard_retune: bool,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let _ = enable_raw_mode();
        let backend = CrosstermBackend::new(stderr());
        let mut terminal = match Terminal::with_options(
            backend,
            TerminalOptions {
                viewport: Viewport::Inline(16),
            },
        ) {
            Ok(t) => t,
            Err(_) => {
                let _ = disable_raw_mode();
                return;
            }
        };

        while running.load(Ordering::Relaxed) {
            if let Ok(s) = state.lock() {
                let rds_bler = if s.rds_total_blocks > 0 {
                    100.0 * (1.0 - s.rds_valid_blocks as f64 / s.rds_total_blocks as f64)
                } else {
                    0.0
                };

                let bar = level_bar(s.iq_level_dbfs);
                let title = format!(
                    " fmradio, tuned on {} MHz: {} ",
                    s.center_freq_hz as f32 / 1_000_000.0,
                    s.rds_ps
                );
                let _ = terminal.draw(|f| {
                    let area = f.area();
                    let chunks =
                        Layout::vertical([Constraint::Min(5), Constraint::Length(1)]).split(area);

                    let body = chunks[0];
                    let footer = chunks[1];
                    let inner = body.width.saturating_sub(4) as usize;
                    let src = one_line(&s.source, inner.saturating_sub(9));
                    let ps = one_line(&s.rds_ps, inner.saturating_sub(10));
                    let rt = one_line(&s.rds_rt, inner.saturating_sub(10));
                    let time = one_line(&s.rds_ct, inner.saturating_sub(10));
                    let language = one_line(&s.rds_language, inner.saturating_sub(10));
                    let program_type = one_line(&s.program_type, inner.saturating_sub(10));
                    let alt_freq_str = s
                        .alt_freq_hz
                        .map(|f| format!("{:.1} MHz", f as f64 / 1_000_000.0))
                        .unwrap_or_else(|| "—".to_string());
                    let lines = vec![
                        Line::from(format!(" Source: {}", src)),
                        Line::from(format!(
                            " In: {} Hz   MPX: {:.0} Hz",
                            s.sample_rate_hz, s.mpx_rate_hz
                        )),
                        Line::from(""),
                        Line::from(format!(" RDS PS: {}", ps)),
                        Line::from(format!(" RDS RT: {}", rt)),
                        Line::from(format!(" RDS Clock: {}", time)),
                        Line::from(format!(" RDS Program Type: {}", program_type)),
                        Line::from(format!(" RDS Language: {}", language)),
                        Line::from(format!(" RDS Alt Freq: {}", alt_freq_str)),
                        Line::from(format!(" Audio: {}", s.mode)),
                        Line::from(""),
                        Line::from(" Metrics:"),
                        Line::from(format!(" IQ Level: {:+6.1} dBFS  {}", s.iq_level_dbfs, bar)),
                        Line::from(format!(" Audio Buffer: {} samples", s.audio_buf_fill)),
                        Line::from(format!(
                            " RDS Blocks: valid={} total={}  BLER={:5.1}%",
                            s.rds_valid_blocks, s.rds_total_blocks, rds_bler
                        )),
                        Line::from(format!(" RDS Groups: {}", s.rds_groups)),
                    ];
                    let p = Paragraph::new(lines).block(
                        Block::default()
                            .title(title)
                            .title_style(Style::new().fg(Color::Blue).add_modifier(Modifier::BOLD))
                            .borders(Borders::ALL),
                    );
                    f.render_widget(p, body);

                    let controls = if can_hard_retune {
                        "(Esc/Q) quit | (←/→) 100kHz | (↑/↓) 1MHz | (S) stereo"
                    } else {
                        "(Esc/Q) quit | (S) stereo"
                    };
                    let footer_widget = Paragraph::new(Line::from(controls)).centered();
                    f.render_widget(footer_widget, footer);
                });
            }

            if event::poll(Duration::from_millis(1)).unwrap_or(false) {
                match event::read() {
                    Ok(Event::Resize(_, _)) => {
                        let _ = terminal.autoresize();
                        let _ = terminal.clear();
                    }
                    Ok(Event::Key(key)) if key.kind == KeyEventKind::Press => match key.code {
                        KeyCode::Esc | KeyCode::Char('q') => {
                            running.store(false, Ordering::Relaxed);
                            break;
                        }
                        KeyCode::Char('s') | KeyCode::Char('S') => {
                            let now = stereo_enabled.load(Ordering::Relaxed);
                            stereo_enabled.store(!now, Ordering::Relaxed);
                        }
                        KeyCode::Left if can_hard_retune => {
                            if let Ok(s) = state.lock() {
                                let next = s.center_freq_hz.saturating_sub(100_000).max(100_000);
                                drop(s);
                                if let Ok(mut s) = state.lock() {
                                    s.center_freq_hz = next;
                                }
                                let _ = adjust_tx.send(RtlSdrMessage::Frequency(next));
                            }
                        }
                        KeyCode::Right if can_hard_retune => {
                            if let Ok(s) = state.lock() {
                                let next = s.center_freq_hz.saturating_add(100_000);
                                drop(s);
                                if let Ok(mut s) = state.lock() {
                                    s.center_freq_hz = next;
                                }
                                let _ = adjust_tx.send(RtlSdrMessage::Frequency(next));
                            }
                        }
                        KeyCode::Up if can_hard_retune => {
                            if let Ok(s) = state.lock() {
                                let next = s.center_freq_hz.saturating_add(1_000_000);
                                drop(s);
                                if let Ok(mut s) = state.lock() {
                                    s.center_freq_hz = next;
                                }
                                let _ = adjust_tx.send(RtlSdrMessage::Frequency(next));
                            }
                        }
                        KeyCode::Down if can_hard_retune => {
                            if let Ok(s) = state.lock() {
                                let next = s.center_freq_hz.saturating_sub(1_000_000).max(100_000);
                                drop(s);
                                if let Ok(mut s) = state.lock() {
                                    s.center_freq_hz = next;
                                }
                                let _ = adjust_tx.send(RtlSdrMessage::Frequency(next));
                            }
                        }
                        _ => {}
                    },
                    _ => {}
                }
            }

            thread::sleep(Duration::from_millis(250));
        }

        let _ = terminal.clear();
        let _ = terminal.show_cursor();
        let _ = disable_raw_mode();
    })
}

fn one_line(input: &str, max_len: usize) -> String {
    let mut s = input.replace(['\n', '\r', '\t'], " ");
    if s.len() > max_len {
        s.truncate(max_len.saturating_sub(3));
        s.push_str("...");
    }
    s
}

fn level_bar(dbfs: f32) -> String {
    let n = (((dbfs + 60.0) / 3.0).clamp(0.0, 20.0)) as usize;
    let mut s = String::from("[");
    s.push_str(&"#".repeat(n));
    s.push_str(&".".repeat(20usize.saturating_sub(n)));
    s.push(']');
    s
}

fn tuning_freq_from_center(center_freq_hz: u32, offset_freq_hz: i32) -> u32 {
    if offset_freq_hz >= 0 {
        center_freq_hz.saturating_sub(offset_freq_hz as u32)
    } else {
        center_freq_hz.saturating_add((-offset_freq_hz) as u32)
    }
}

#[tokio::main]
async fn main() -> desperado::Result<()> {
    let args = Args::parse();

    // Initialize tracing with verbosity level
    // 0 = WARN (quiet), 1 = INFO, 2 = DEBUG, 3+ = TRACE
    let app_level = if args.tui {
        // Keep inline dashboard stable (avoid interleaved logs).
        Level::WARN
    } else {
        match args.verbose {
            0 => Level::WARN,
            1 => Level::INFO,
            2 => Level::DEBUG,
            _ => Level::TRACE,
        }
    };

    let log_filter = tracing_subscriber::filter::Targets::new()
        .with_target("fmradio", app_level)
        .with_target("desperado", app_level)
        .with_target("rtl_sdr_rs", Level::WARN)
        .with_default(Level::WARN);
    let _ = tracing_subscriber::registry()
        .with(log_filter)
        .with(
            tracing_subscriber::fmt::layer()
                .with_writer(std::io::stderr)
                .with_target(false),
        )
        .try_init();

    // Create IQ source based on selected type
    let mut current_center_hz = args.center_freq.0;
    let mut iq_source = build_iq_source(
        &args,
        tuning_freq_from_center(current_center_hz, args.offset_freq),
    )
    .await?;

    let initial_stereo = !args.mono;
    let stereo_enabled = Arc::new(AtomicBool::new(initial_stereo));
    let (adjust_tx, mut adjust_rx) = tokio::sync::mpsc::unbounded_channel::<RtlSdrMessage>();

    info!(
        "FM demodulator: {} mode, source: {}",
        if initial_stereo { "stereo" } else { "mono" },
        args.source
    );

    let channels = if args.tui || initial_stereo { 2 } else { 1 };

    // Setup audio output - larger buffer for file playback to prevent underruns
    let is_file_source = is_file_like_source(&args.source);
    let buffer_size = if is_file_source {
        AUDIO_RATE * 4 // 4 seconds for file playback
    } else {
        AUDIO_RATE * 2 // 2 seconds for live sources
    };
    let (tx, rx) = channel::bounded::<f32>(buffer_size);

    let _device = if !args.no_audio {
        let config = OutputDeviceParameters {
            channels_count: channels,
            sample_rate: AUDIO_RATE,
            channel_sample_count: 1024,
        };
        Some(
            run_output_device(config, move |data| {
                for sample in data.iter_mut() {
                    *sample = rx.try_recv().unwrap_or(0.0);
                }
            })
            .unwrap(),
        )
    } else {
        info!("Audio output disabled (--no-audio)");
        None
    };

    let tui_state = Arc::new(Mutex::new(TuiState {
        mode: if initial_stereo { "stereo" } else { "mono" },
        source: args.source.clone(),
        center_freq_hz: args.center_freq.0,
        alt_freq_hz: None,
        sample_rate_hz: args.sample_rate,
        mpx_rate_hz: 0.0,
        iq_level_dbfs: -120.0,
        audio_buf_fill: 0,
        rds_ps: String::new(),
        rds_rt: String::new(),
        rds_total_blocks: 0,
        rds_valid_blocks: 0,
        rds_groups: 0,
        program_type: String::new(),
        rds_ct: String::new(),
        rds_language: String::new(),
    }));
    let app_running = Arc::new(AtomicBool::new(true));
    let tui_thread = if args.tui {
        let can_hard_retune = args.source.starts_with("rtlsdr://");
        Some(spawn_inline_tui(
            tui_state.clone(),
            app_running.clone(),
            stereo_enabled.clone(),
            adjust_tx.clone(),
            can_hard_retune,
        ))
    } else {
        None
    };

    let sig_running = app_running.clone();
    tokio::spawn(async move {
        if tokio::signal::ctrl_c().await.is_ok() {
            sig_running.store(false, Ordering::Relaxed);
        }
    });

    let mut rotate = Rotate::new(-2.0 * PI * args.offset_freq as f32 / args.sample_rate as f32);
    let mut phase_extractor = PhaseExtractor::new();
    let factor = (args.sample_rate as f32 / FM_BANDWIDTH).round() as usize;
    let mut decimator = Decimator::new(factor);

    // IMPORTANT: Calculate actual decimated sample rate
    // factor = round(2000000 / 240000) = 8, so actual rate = 2000000 / 8 = 250000
    let actual_mpx_rate = args.sample_rate as f32 / factor as f32;
    info!(
        "[DSP] Decimation factor: {}, actual MPX rate: {} Hz (target was {} Hz)",
        factor, actual_mpx_rate, FM_BANDWIDTH
    );
    if let Ok(mut s) = tui_state.lock() {
        s.mpx_rate_hz = actual_mpx_rate;
    }

    let n = 2048;
    let window = 64;
    let mut afc = SquareFreqOffsetCorrection::with_params(n, window, false);

    let lowpass_fir = LowPassFir::new(MONO_SIGNAL_BW, actual_mpx_rate, 256);

    let run_result = if args.tui || initial_stereo {
        info!("Running stereo FM demodulator with RDS...");
        run_stereo(
            &mut iq_source,
            &args,
            &mut rotate,
            &mut phase_extractor,
            &mut decimator,
            &mut afc,
            &lowpass_fir,
            actual_mpx_rate,
            tx.clone(),
            if args.tui {
                Some(tui_state.clone())
            } else {
                None
            },
            args.tui,
            stereo_enabled.clone(),
            &mut adjust_rx,
            &mut current_center_hz,
            app_running.clone(),
        )
        .await
    } else {
        info!("Running mono FM demodulator...");
        run_mono(
            &mut iq_source,
            &args,
            &mut rotate,
            &mut phase_extractor,
            &mut decimator,
            &mut afc,
            &lowpass_fir,
            actual_mpx_rate,
            tx.clone(),
            if args.tui {
                Some(tui_state.clone())
            } else {
                None
            },
            args.tui,
            &mut adjust_rx,
            &mut current_center_hz,
            app_running.clone(),
        )
        .await
    };

    if is_file_source && !args.no_audio && app_running.load(Ordering::Relaxed) {
        let drain_start = Instant::now();
        let max_drain = Duration::from_secs(10);
        while !tx.is_empty() && drain_start.elapsed() < max_drain {
            thread::sleep(Duration::from_millis(20));
        }
    }

    // Always restore TUI terminal state, including on I/O errors.
    app_running.store(false, Ordering::Relaxed);
    if let Some(handle) = tui_thread {
        let _ = handle.join();
    }

    run_result
}

async fn build_iq_source(args: &Args, tuning_freq: u32) -> desperado::Result<IqAsyncSource> {
    if args.source == "-" {
        let format = IqFormat::from_str(&args.format)
            .map_err(|e| std::io::Error::other(format!("Invalid format: {}", e)))?;
        return Ok(IqAsyncSource::from_stdin(
            tuning_freq,
            args.sample_rate,
            16384,
            format,
        ));
    }

    if !is_device_uri(&args.source) {
        let format = IqFormat::from_str(&args.format)
            .map_err(|e| std::io::Error::other(format!("Invalid format: {}", e)))?;
        return IqAsyncSource::from_file(
            &args.source,
            tuning_freq,
            args.sample_rate,
            16384,
            format,
        )
        .await;
    }

    let configured_uri =
        ensure_tuning_query(&args.source, tuning_freq, args.sample_rate, args.gain);
    let config = DeviceConfig::from_str(&configured_uri)
        .map_err(|e| std::io::Error::other(format!("Invalid SDR URI: {}", e)))?;

    IqAsyncSource::from_device_config(&config).await
}

fn is_device_uri(input: &str) -> bool {
    input.starts_with("rtlsdr://")
        || input.starts_with("soapy://")
        || input.starts_with("airspy://")
}

fn is_file_like_source(input: &str) -> bool {
    input == "-" || !is_device_uri(input)
}

fn ensure_tuning_query(
    uri: &str,
    center_freq_hz: u32,
    sample_rate: u32,
    gain: Option<i32>,
) -> String {
    let has_query = uri.contains('?');
    let has_freq = uri.contains("freq=") || uri.contains("frequency=");
    let has_rate = uri.contains("rate=") || uri.contains("sample_rate=");
    let has_gain = uri.contains("gain=");

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
    if let Some(g) = gain
        && !has_gain
    {
        if !out.ends_with('?') && !out.ends_with('&') {
            out.push('&');
        }
        // Backward compatibility: fmradio historically treated RTL-SDR gain
        // CLI values as tenths of dB (e.g. -g 300 => 30.0 dB).
        // Keep that behavior for rtlsdr:// while preserving raw values for
        // other backends.
        if uri.starts_with("rtlsdr://") {
            out.push_str(&format!("gain={:.1}", f64::from(g) / 10.0));
        } else {
            out.push_str(&format!("gain={g}"));
        }
    }

    out
}

#[allow(clippy::too_many_arguments)]
async fn run_mono(
    iq_source: &mut IqAsyncSource,
    args: &Args,
    rotate: &mut Rotate,
    phase_extractor: &mut PhaseExtractor,
    decimator: &mut Decimator,
    afc: &mut SquareFreqOffsetCorrection,
    lowpass_fir: &LowPassFir,
    mpx_sample_rate: f32,
    tx: channel::Sender<f32>,
    tui_state: Option<Arc<Mutex<TuiState>>>,
    tui_mode: bool,
    adjust_rx: &mut tokio::sync::mpsc::UnboundedReceiver<RtlSdrMessage>,
    current_center_hz: &mut u32,
    running: Arc<AtomicBool>,
) -> desperado::Result<()> {
    let mut deemphasis = DeemphasisFilter::new(mpx_sample_rate, 50e-6);
    let mut audio_resample =
        AudioAdaptiveResampler::new(AUDIO_RATE as f64 / mpx_sample_rate as f64, 1, 1, !tui_mode);

    let mut agc_gain = 0.5f32;
    let mut settle_drop_chunks = 0usize;
    const AGC_ATTACK: f32 = 0.999;
    const AGC_RELEASE: f32 = 0.9999;

    let is_file_source = is_file_like_source(&args.source);
    while let Some(chunk) = iq_source.next().await {
        if !running.load(Ordering::Relaxed) {
            break;
        }

        // Check for control messages (non-blocking)
        while let Ok(msg) = adjust_rx.try_recv() {
            if let RtlSdrMessage::Frequency(desired_center) = msg
                && desired_center != *current_center_hz
            {
                let tuning_freq = tuning_freq_from_center(desired_center, args.offset_freq);
                match iq_source.tune(tuning_freq) {
                    Ok(()) => {
                        *current_center_hz = desired_center;
                        // Reset DSP state so demod/RDS reacquires on the new station.
                        *rotate = Rotate::new(
                            -2.0 * PI * args.offset_freq as f32 / args.sample_rate as f32,
                        );
                        phase_extractor.reset();
                        decimator.reset();
                        deemphasis.reset();
                        audio_resample = AudioAdaptiveResampler::new(
                            AUDIO_RATE as f64 / mpx_sample_rate as f64,
                            1,
                            1,
                            !tui_mode,
                        );
                        agc_gain = 0.5;
                        if let Some(state) = &tui_state
                            && let Ok(mut s) = state.lock()
                        {
                            s.center_freq_hz = desired_center;
                        }
                        settle_drop_chunks = 4;
                    }
                    Err(e) => {
                        warn!("Retune failed: {}", e);
                    }
                }
            }
        }

        let chunk = chunk.map_err(|e| std::io::Error::other(format!("{}", e)))?;

        if settle_drop_chunks > 0 {
            settle_drop_chunks -= 1;
            continue;
        }

        let shifted = rotate.process(&chunk);
        let decimated = decimator.process(&shifted);
        if let Some(state) = &tui_state
            && let Ok(mut s) = state.lock()
        {
            s.iq_level_dbfs = iq_level_dbfs(&decimated) as f32;
            s.audio_buf_fill = tx.len();
        }
        let afc_corrected = if args.afc {
            afc.process(&decimated)
        } else {
            decimated
        };
        let phase = phase_extractor.process(&afc_corrected);
        let filtered = lowpass_fir.process(&phase);
        let deemphasized = deemphasis.process(&filtered);

        let audio = audio_resample.process(&deemphasized);

        let chunk_max = audio.iter().fold(0.0_f32, |a, &b| a.max(b.abs()));

        if chunk_max > 0.0001 {
            let target_gain = 0.5 / chunk_max;
            if target_gain < agc_gain {
                agc_gain = agc_gain * AGC_ATTACK + target_gain * (1.0 - AGC_ATTACK);
            } else {
                agc_gain = agc_gain * AGC_RELEASE + target_gain * (1.0 - AGC_RELEASE);
            }
            agc_gain = agc_gain.clamp(0.01, 2.0);
        }

        let processed: Vec<f32> = audio
            .iter()
            .map(|&s| {
                let scaled = s * agc_gain;
                if scaled > 1.0 {
                    1.0 - (-10.0 * (scaled - 1.0)).exp()
                } else if scaled < -1.0 {
                    -1.0 + (-10.0 * (-scaled - 1.0)).exp()
                } else {
                    scaled
                }
            })
            .collect();

        if !args.no_audio {
            // Only do adaptive resampling for live sources (not files)
            if !is_file_source {
                audio_resample.adjust_ratio(tx.len() as f64 / (AUDIO_RATE * 2) as f64);
            }

            // For file sources, use blocking send to let audio buffer control pacing
            // For live sources, use non-blocking to avoid dropping samples from hardware
            if is_file_source {
                // Blocking send - wait for buffer space (natural pacing)
                for sample in processed {
                    if tx.send(sample).is_err() {
                        break;
                    }
                }
            } else {
                // Non-blocking for live sources
                if tx.len() > (AUDIO_RATE * 2) * 9 / 10 {
                    println!(
                        "\n[DROP] Buffer >90% full, dropping {} samples",
                        processed.len()
                    );
                    continue;
                }

                for sample in processed {
                    if tx.try_send(sample).is_err() {
                        println!("\n[WARNING] Audio buffer full");
                        break;
                    }
                }
            }
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_stereo(
    iq_source: &mut IqAsyncSource,
    args: &Args,
    rotate: &mut Rotate,
    phase_extractor: &mut PhaseExtractor,
    decimator: &mut Decimator,
    afc: &mut SquareFreqOffsetCorrection,
    _lowpass_fir: &LowPassFir,
    mpx_sample_rate: f32,
    tx: channel::Sender<f32>,
    tui_state: Option<Arc<Mutex<TuiState>>>,
    tui_mode: bool,
    stereo_enabled: Arc<AtomicBool>,
    adjust_rx: &mut tokio::sync::mpsc::UnboundedReceiver<RtlSdrMessage>,
    current_center_hz: &mut u32,
    running: Arc<AtomicBool>,
) -> desperado::Result<()> {
    let mut stereo = StereoDecoderPLL::new(mpx_sample_rate);
    let mut deemphasis_l = DeemphasisFilter::new(mpx_sample_rate, 50e-6);
    let mut deemphasis_r = DeemphasisFilter::new(mpx_sample_rate, 50e-6);

    // RDS resampler: actual MPX rate → 171 kHz (matches redsea's preferred rate)
    // Note: 171 kHz gives exactly 3 samples per RDS symbol (171000 / 57000 = 3)
    let rds_target_rate = 171_000.0_f32;
    let mut rds_resampler = RdsResamplerCustom::new(mpx_sample_rate, rds_target_rate);
    // Always decode RDS in JSON mode. In TUI, printing is disabled and values are
    // consumed by the dashboard. Outside TUI, JSON lines are emitted via tracing logs.
    let mut rds = RdsDecoder::new(rds_target_rate, args.verbose >= 2);
    if tui_mode {
        rds.set_print_json_output(false);
    }

    let mut audio_resample =
        AudioAdaptiveResampler::new(AUDIO_RATE as f64 / mpx_sample_rate as f64, 5, 2, !tui_mode);
    let mut settle_drop_chunks = 0usize;

    let is_file_source = is_file_like_source(&args.source);

    // Raw output mode: write MPX samples to stdout for piping to redsea
    let raw_out = args.raw_out;
    let mut stdout = std::io::stdout().lock();

    // Optional resampler for raw output (actual_mpx_rate -> target rate)
    let mpx_rate_u32 = mpx_sample_rate as u32;
    let mut raw_resampler: Option<rubato::SincFixedOut<f32>> = if raw_out {
        if let Some(target_rate) = args.resample_out {
            if target_rate != mpx_rate_u32 {
                let ratio = target_rate as f64 / mpx_sample_rate as f64;
                let params = SincInterpolationParameters {
                    sinc_len: 128,
                    f_cutoff: 0.9,
                    interpolation: SincInterpolationType::Cubic,
                    oversampling_factor: 128,
                    window: WindowFunction::BlackmanHarris2,
                };
                info!(
                    "[RAW-OUT] Resampling from {} Hz to {} Hz",
                    mpx_rate_u32, target_rate
                );
                Some(SincFixedOut::<f32>::new(ratio, 1.1, params, 1024, 1).unwrap())
            } else {
                None
            }
        } else {
            info!(
                "[RAW-OUT] Outputting at {} Hz (use --resample-out to change)",
                mpx_rate_u32
            );
            None
        }
    } else {
        None
    };
    let mut raw_leftover: Vec<f32> = Vec::new();

    while let Some(chunk) = iq_source.next().await {
        if !running.load(Ordering::Relaxed) {
            break;
        }

        // Check for control messages (non-blocking)
        while let Ok(msg) = adjust_rx.try_recv() {
            if let RtlSdrMessage::Frequency(desired_center) = msg
                && desired_center != *current_center_hz
            {
                let tuning_freq = tuning_freq_from_center(desired_center, args.offset_freq);
                match iq_source.tune(tuning_freq) {
                    Ok(()) => {
                        *current_center_hz = desired_center;
                        // Reset DSP state so stereo/RDS lock is reacquired for new station.
                        *rotate = Rotate::new(
                            -2.0 * PI * args.offset_freq as f32 / args.sample_rate as f32,
                        );
                        phase_extractor.reset();
                        decimator.reset();
                        stereo = StereoDecoderPLL::new(mpx_sample_rate);
                        deemphasis_l = DeemphasisFilter::new(mpx_sample_rate, 50e-6);
                        deemphasis_r = DeemphasisFilter::new(mpx_sample_rate, 50e-6);
                        rds_resampler = RdsResamplerCustom::new(mpx_sample_rate, rds_target_rate);
                        rds = RdsDecoder::new(rds_target_rate, args.verbose >= 2);
                        if tui_mode {
                            rds.set_print_json_output(false);
                        }
                        audio_resample = AudioAdaptiveResampler::new(
                            AUDIO_RATE as f64 / mpx_sample_rate as f64,
                            5,
                            2,
                            !tui_mode,
                        );
                        if let Some(state) = &tui_state
                            && let Ok(mut s) = state.lock()
                        {
                            s.center_freq_hz = desired_center;
                            s.rds_ps.clear();
                            s.rds_rt.clear();
                            s.rds_total_blocks = 0;
                            s.rds_valid_blocks = 0;
                            s.rds_groups = 0;
                        }
                        settle_drop_chunks = 12;
                    }
                    Err(e) => {
                        warn!("Retune failed: {}", e);
                    }
                }
            }
        }

        let chunk = chunk.map_err(|e| std::io::Error::other(format!("{}", e)))?;

        if settle_drop_chunks > 0 {
            settle_drop_chunks -= 1;
            continue;
        }

        let shifted = rotate.process(&chunk);
        let decimated = decimator.process(&shifted);
        if let Some(state) = &tui_state
            && let Ok(mut s) = state.lock()
        {
            s.iq_level_dbfs = iq_level_dbfs(&decimated) as f32;
            s.audio_buf_fill = tx.len();
        }

        let afc_corrected = if args.afc {
            afc.process(&decimated)
        } else {
            decimated
        };
        let phase = phase_extractor.process(&afc_corrected);

        // Raw output mode: output MPX samples to stdout for piping to redsea
        if raw_out {
            use std::io::Write;

            // Optionally resample
            let samples_to_output = if let Some(ref mut resampler) = raw_resampler {
                // Add to leftover buffer
                raw_leftover.extend_from_slice(&phase);

                let mut output = Vec::new();
                loop {
                    let needed = resampler.input_frames_next();
                    if raw_leftover.len() < needed {
                        break;
                    }
                    let input_chunk: Vec<Vec<f32>> = vec![raw_leftover[..needed].to_vec()];
                    raw_leftover.drain(0..needed);

                    if let Ok(resampled) = resampler.process(&input_chunk, None)
                        && !resampled.is_empty()
                    {
                        output.extend_from_slice(&resampled[0]);
                    }
                }
                output
            } else {
                phase.clone()
            };

            // Convert f32 to signed 16-bit PCM and write to stdout
            // rtl_fm outputs samples scaled similarly to audio - we need to find the right scale
            // Debug: print min/max of first batch
            static DBG_RAW: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let dbg_cnt = DBG_RAW.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if dbg_cnt == 0 && !samples_to_output.is_empty() {
                let min = samples_to_output
                    .iter()
                    .cloned()
                    .fold(f32::INFINITY, f32::min);
                let max = samples_to_output
                    .iter()
                    .cloned()
                    .fold(f32::NEG_INFINITY, f32::max);
                let mean = samples_to_output.iter().sum::<f32>() / samples_to_output.len() as f32;
                debug!(
                    "[RAW-DBG] Phase signal: min={:.4}, max={:.4}, mean={:.4}, len={}",
                    min,
                    max,
                    mean,
                    samples_to_output.len()
                );
            }

            // Scale: Phase extractor outputs ~±π for FM deviation.
            // rtl_fm seems to scale to small i16 values (most samples near 0)
            // Scale so that full deviation maps to ~25000 (leaving headroom)
            for sample in samples_to_output {
                let scaled = (sample * 5000.0).clamp(-32768.0, 32767.0) as i16;
                let bytes = scaled.to_le_bytes();
                let _ = stdout.write_all(&bytes);
            }
            let _ = stdout.flush();

            // In raw output mode, skip normal processing
            continue;
        }

        // Stereo decoder needs raw phase signal (contains 19 kHz pilot and 38 kHz L-R subcarrier)
        // The stereo decoder has its own pilot bandpass filter (18.3-19.7 kHz)
        let (left, right, pilot_phases) = stereo.process(&phase);
        let stereo_on = stereo_enabled.load(Ordering::Relaxed);
        let deem_l = deemphasis_l.process(&left);
        let deem_r = deemphasis_r.process(&right);

        // RDS: Resample MPX to 171 kHz using pilot-coherent carrier mixing
        // This uses the 19 kHz pilot phase × 3 for perfect 57 kHz carrier lock
        let (rds_i, rds_q) = rds_resampler.process_with_pilot(&phase, &pilot_phases);
        if !rds_i.is_empty() {
            rds.process_iq(&rds_i, &rds_q);
            if let Some(state) = &tui_state
                && let Ok(mut s) = state.lock()
            {
                let (total, valid, groups) = rds.stats();
                s.rds_total_blocks = total;
                s.rds_valid_blocks = u64::from(valid);
                s.rds_groups = u64::from(groups);
                s.rds_ps = rds.station_name().unwrap_or_default();
                s.rds_rt = rds.radio_text().unwrap_or_default();
                s.rds_ct = rds
                    .clock_time()
                    .map(|t| format!("{:}", t))
                    .unwrap_or_default();
                s.rds_language = rds.language().unwrap_or_default();
                s.program_type = rds.program_type();
                s.alt_freq_hz = rds.alt_frequency();
                s.mode = if stereo_on { "stereo" } else { "mono" };
            }
        }

        // Interleave stereo
        let mut interleaved = Vec::with_capacity(deem_l.len() * 2);
        if stereo_on {
            for i in 0..deem_l.len() {
                interleaved.push(deem_l[i]);
                interleaved.push(deem_r[i]);
            }
        } else {
            for i in 0..deem_l.len() {
                let m = 0.5 * (deem_l[i] + deem_r[i]);
                interleaved.push(m);
                interleaved.push(m);
            }
        }

        let audio = audio_resample.process(&interleaved);

        if !args.no_audio {
            // Only do adaptive resampling for live sources (not files)
            if !is_file_source {
                audio_resample.adjust_ratio(tx.len() as f64 / (AUDIO_RATE * 2) as f64);
            }

            // For file sources, use blocking send to let audio buffer control pacing
            // For live sources, use non-blocking to avoid dropouts
            if is_file_source {
                for sample in audio {
                    if tx.send(sample).is_err() {
                        break;
                    }
                }
            } else {
                for sample in audio {
                    if tx.try_send(sample).is_err() {
                        break;
                    }
                }
            }
        }
    }

    Ok(())
}

impl FromStr for Frequency {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        if let Some(stripped) = s.strip_suffix('M') {
            let val: f32 = stripped.trim().parse().map_err(|_| "Invalid MHz value")?;
            Ok(Frequency((val * 1_000_000.0) as u32))
        } else if let Some(stripped) = s.strip_suffix('k') {
            let val: f32 = stripped.trim().parse().map_err(|_| "Invalid kHz value")?;
            Ok(Frequency((val * 1_000.0) as u32))
        } else {
            let val: u32 = s.parse().map_err(|_| "Invalid Hz value")?;
            Ok(Frequency(val))
        }
    }
}

struct AudioAdaptiveResampler {
    resampler: SincFixedOut<f32>,
    target_fill: f64,
    alpha: f64,
    k_p: f64,
    k_i: f64,
    smoothed_error: f64,
    integral_error: f64,
    adjustment_interval: usize,
    adjustment_counter: usize,
    leftover: Vec<f32>,
    resample_ratio: f64,
    channels: usize,
    show_status_meter: bool,
}

impl AudioAdaptiveResampler {
    fn new(
        initial_ratio: f64,
        adjustment_interval: usize,
        channels: usize,
        show_status_meter: bool,
    ) -> Self {
        let params = SincInterpolationParameters {
            sinc_len: 256,
            f_cutoff: 0.95,
            interpolation: SincInterpolationType::Cubic,
            oversampling_factor: 160,
            window: WindowFunction::BlackmanHarris2,
        };

        let output_frames = 1024;

        let resampler =
            SincFixedOut::<f32>::new(initial_ratio, 2.0, params, output_frames, channels).unwrap();

        Self {
            resampler,
            target_fill: if channels == 1 { 0.3 } else { 0.4 },
            alpha: if channels == 1 { 0.95 } else { 0.9 },
            k_p: if channels == 1 { 0.0001 } else { 0.002 },
            k_i: if channels == 1 { 1e-7 } else { 5e-6 },
            smoothed_error: 0.0,
            integral_error: 0.0,
            adjustment_interval,
            adjustment_counter: 0,
            leftover: Vec::new(),
            resample_ratio: initial_ratio,
            channels,
            show_status_meter,
        }
    }

    fn adjust_ratio(&mut self, buffer_fill: f64) {
        self.adjustment_counter += 1;
        if self.adjustment_counter >= self.adjustment_interval {
            self.adjustment_counter = 0;

            let error = self.target_fill - buffer_fill;

            self.smoothed_error = self.alpha * self.smoothed_error + (1.0 - self.alpha) * error;

            if self.smoothed_error.abs() < 0.01 {
                self.smoothed_error = 0.0;
            }

            if self.smoothed_error.abs() < 0.15 {
                self.integral_error += self.smoothed_error;
                self.integral_error = self.integral_error.clamp(-100.0, 100.0);
            }

            self.resample_ratio += self.k_p * self.smoothed_error + self.k_i * self.integral_error;

            let nominal = AUDIO_RATE as f64 / FM_BANDWIDTH as f64;
            let min_ratio = nominal * 0.97;
            let max_ratio = nominal * 1.02;

            self.resample_ratio = self.resample_ratio.clamp(min_ratio, max_ratio);

            self.resampler
                .set_resample_ratio(self.resample_ratio, true)
                .unwrap();

            if self.channels == 1 && self.show_status_meter {
                print!(
                    "\rBuf: {:.1}% | Ratio: {:.6} (nom: {:.6}) | Err: {:.3} | smooth: {:.3} | integral: {:.3}      ",
                    buffer_fill * 100.0,
                    self.resample_ratio,
                    nominal,
                    error,
                    self.smoothed_error,
                    self.integral_error
                );
                stdout().flush().unwrap();
            }
        }
    }

    fn process(&mut self, input: &[f32]) -> Vec<f32> {
        self.leftover.extend_from_slice(input);

        let mut output = Vec::new();

        loop {
            let input_frames_needed = self.resampler.input_frames_next();
            let samples_needed = input_frames_needed * self.channels;

            if self.leftover.len() < samples_needed {
                break;
            }

            let chunk: Vec<f32> = self.leftover.drain(..samples_needed).collect();

            // For mono: simple vector wrapping
            // For stereo: deinterleave
            let input_block = if self.channels == 1 {
                vec![chunk]
            } else {
                let mut chs: Vec<Vec<f32>> =
                    vec![Vec::with_capacity(input_frames_needed); self.channels];
                for frame_idx in 0..input_frames_needed {
                    #[allow(clippy::needless_range_loop)]
                    for ch in 0..self.channels {
                        let idx = frame_idx * self.channels + ch;
                        chs[ch].push(chunk[idx]);
                    }
                }
                chs
            };

            match self.resampler.process(&input_block, None) {
                Ok(output_block) => {
                    if self.channels == 1 {
                        output.extend_from_slice(&output_block[0]);
                    } else {
                        // Re-interleave stereo
                        let out_frames = output_block[0].len();
                        #[allow(clippy::needless_range_loop)]
                        for i in 0..out_frames {
                            for ch in 0..self.channels {
                                output.push(output_block[ch][i]);
                            }
                        }
                    }
                }
                Err(e) => {
                    warn!("Resampler error: {:?}", e);
                    break;
                }
            }
        }

        output
    }
}

/// Stereo decoder using a complex PLL locked to the 19 kHz pilot tone.
pub struct StereoDecoderPLL {
    sample_rate: f32,
    pll_phase: f64,
    pll_freq: f64,
    nominal_freq: f64,
    kp: f64,
    ki: f64,
    error_lpf_state: f64, // IIR lowpass state for phase error filtering
    error_lpf_alpha: f64, // IIR lowpass coefficient
    pilot_hi: LowPassFir,
    pilot_lo: LowPassFir,
    diff_lowpass: LowPassFir,
}

impl StereoDecoderPLL {
    pub fn new(sample_rate: f32) -> Self {
        let nominal = 19_000.0_f64;
        // IIR lowpass for phase error: fc ~ 50 Hz, alpha = 2*pi*fc / (fs + 2*pi*fc)
        // This filters out the 2*f_pilot component from the multiplier phase detector
        let fc = 50.0; // Phase error filter cutoff (Hz)
        let error_lpf_alpha = 2.0 * std::f64::consts::PI * fc
            / (sample_rate as f64 + 2.0 * std::f64::consts::PI * fc);

        Self {
            sample_rate,
            pll_phase: 0.0,
            pll_freq: nominal,
            nominal_freq: nominal,
            // PLL gains: kp for proportional (phase), ki for integral (frequency)
            // These are tuned for tracking a stable 19 kHz pilot
            kp: 0.01, // Proportional gain - controls phase response speed
            ki: 5e-6, // Integral gain - controls frequency acquisition
            error_lpf_state: 0.0,
            error_lpf_alpha,
            pilot_hi: LowPassFir::new(19_700.0, sample_rate, 257),
            pilot_lo: LowPassFir::new(18_300.0, sample_rate, 257),
            diff_lowpass: LowPassFir::new(15_000.0, sample_rate, 257),
        }
    }

    /// Process input samples and return (left, right, pilot_phases)
    /// pilot_phases contains the 19 kHz pilot PLL phase at each sample (for RDS carrier recovery)
    pub fn process(&mut self, input: &[f32]) -> (Vec<f32>, Vec<f32>, Vec<f64>) {
        let n = input.len();
        let pilot_hi = self.pilot_hi.process(input);
        let pilot_lo = self.pilot_lo.process(input);
        let mut pilot_band = Vec::with_capacity(n);
        for i in 0..n {
            pilot_band.push(pilot_hi[i] - pilot_lo[i]);
        }

        let mut lmr_raw = Vec::with_capacity(n);
        let mut pilot_phases = Vec::with_capacity(n);
        let phase_inc = 2.0 * std::f64::consts::PI / self.sample_rate as f64;

        for &p in &pilot_band {
            let p = p as f64;

            // Multiplier phase detector for real sinusoid:
            // If pilot = A*cos(ω*t + φ) and NCO = sin(pll_phase), then
            // product = A*cos(ω*t + φ) * sin(pll_phase)
            //         = A/2 * [sin(pll_phase - ω*t - φ) + sin(pll_phase + ω*t + φ)]
            // After lowpass filtering, we get: A/2 * sin(phase_error)
            // where phase_error = pll_phase - (ω*t + φ)
            // When locked: phase_error → 0, so error → 0
            let raw_error = p * self.pll_phase.sin();

            // IIR lowpass filter to remove 2*f_pilot component
            self.error_lpf_state += self.error_lpf_alpha * (raw_error - self.error_lpf_state);
            let error = self.error_lpf_state;

            // Second-order PLL update: frequency integrator + phase
            self.pll_freq += self.ki * error;
            self.pll_freq = self
                .pll_freq
                .clamp(self.nominal_freq * 0.99, self.nominal_freq * 1.01);

            // Phase update: NCO advance + proportional correction
            self.pll_phase += phase_inc * self.pll_freq + self.kp * error;

            // Keep phase in [-π, π]
            while self.pll_phase > std::f64::consts::PI {
                self.pll_phase -= 2.0 * std::f64::consts::PI;
            }
            while self.pll_phase < -std::f64::consts::PI {
                self.pll_phase += 2.0 * std::f64::consts::PI;
            }

            // Store the pilot phase at this sample AFTER the PLL update
            pilot_phases.push(self.pll_phase);

            let carrier_38 = (2.0 * self.pll_phase).sin();
            lmr_raw.push(input[lmr_raw.len()] as f64 * carrier_38);
        }

        let lmr_f32: Vec<f32> = lmr_raw.iter().map(|&v| v as f32).collect();
        let lmr = self.diff_lowpass.process(&lmr_f32);

        let mut left = Vec::with_capacity(n);
        let mut right = Vec::with_capacity(n);
        for i in 0..n {
            let lpr = input[i];
            let lmr_val = lmr[i];
            left.push(0.5 * (lpr + lmr_val));
            right.push(0.5 * (lpr - lmr_val));
        }
        (left, right, pilot_phases)
    }

    /// Get the current PLL phase (for 19 kHz pilot)
    /// Multiply by 3 to get phase for 57 kHz RDS carrier
    pub fn pilot_phase(&self) -> f64 {
        self.pll_phase
    }

    /// Get the current PLL frequency (should be ~19 kHz when locked)
    pub fn pilot_freq(&self) -> f64 {
        self.pll_freq
    }
}
