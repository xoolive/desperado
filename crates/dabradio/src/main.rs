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
#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
use desperado::DeviceConfig;
use desperado::{IqFormat, IqSource};
use num_complex::Complex;
use pad::PadData;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::io::{IsTerminal, Write};
#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
use std::str::FromStr;
use std::time::Duration;
use tinyaudio::prelude::*;
use tracing::Level;
use tracing::{debug, info, warn};
use tracing_subscriber::prelude::*;

/// Unified IQ chunk iterator that works with both sync and async sources.
///
/// For file/stdin sources, uses synchronous `IqSource` (simple, no threading).
/// For live SDR sources, uses sync `IqSource` inside a dedicated reader thread,
/// then consumes IQ chunks through an async channel.
enum IqChunkSource {
    Sync(Box<IqSource>),
    #[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
    Threaded(tokio::sync::mpsc::Receiver<desperado::error::Result<Vec<Complex<f32>>>>),
}

#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
type IqChunkResult = desperado::error::Result<Vec<Complex<f32>>>;

#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
type IqChunkRx = tokio::sync::mpsc::Receiver<IqChunkResult>;

impl IqChunkSource {
    /// Get the next chunk of IQ samples.
    /// For sync sources, this blocks until data is available.
    /// For async sources, this awaits the background reader thread.
    async fn next_chunk(&mut self) -> Option<desperado::error::Result<Vec<Complex<f32>>>> {
        match self {
            IqChunkSource::Sync(source) => source.next(),
            #[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
            IqChunkSource::Threaded(rx) => rx.recv().await,
        }
    }
}

#[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
fn spawn_sync_reader_thread(
    configured_uri: String,
    queue_chunks: usize,
) -> Result<IqChunkRx, Box<dyn std::error::Error>> {
    let (tx, rx) = tokio::sync::mpsc::channel(queue_chunks);
    let (tx_init, rx_init) = std::sync::mpsc::channel::<Result<(), String>>();
    std::thread::spawn(move || {
        let source_res = (|| -> Result<IqSource, String> {
            let config = DeviceConfig::from_str(&configured_uri).map_err(|e| e.to_string())?;
            IqSource::from_device_config(config).map_err(|e| e.to_string())
        })();

        let source = match source_res {
            Ok(source) => {
                let _ = tx_init.send(Ok(()));
                source
            }
            Err(e) => {
                let _ = tx_init.send(Err(e));
                return;
            }
        };

        for chunk in source {
            if tx.blocking_send(chunk).is_err() {
                break;
            }
        }
    });

    match rx_init.recv() {
        Ok(Ok(())) => Ok(rx),
        Ok(Err(e)) => Err(std::io::Error::other(e).into()),
        Err(_) => Err(std::io::Error::other("Reader thread failed to initialize").into()),
    }
}

const AUDIO_RATE: usize = 48_000;

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
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let log_filter = tracing_subscriber::filter::Targets::new()
        .with_target("dabradio", Level::INFO)
        .with_target("desperado", Level::INFO)
        .with_target("rtl_sdr_rs", Level::WARN)
        .with_default(Level::WARN);
    tracing_subscriber::registry()
        .with(log_filter)
        .with(tracing_subscriber::fmt::layer())
        .init();

    let cli = Cli::parse();

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
    let chunk_size = constants::T_F; // One frame's worth of samples
    let (mut source, is_file_source) = open_iq_source(
        source,
        center_freq,
        constants::SAMPLE_RATE,
        chunk_size,
        iq_format,
    )
    .await?;

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
    let term_image_support = terminal_supports_inline_images();

    // Audio output setup (tinyaudio + crossbeam channel)
    // Buffer sizing:
    // - File/stdin: blocking sender naturally paces to playback.
    // - Live SDR: larger queue absorbs decode jitter without stalling RF processing.
    let audio_queue_seconds = if is_file_source { 4 } else { 8 };
    let (tx, rx) = channel::bounded::<f32>(AUDIO_RATE * 2 * audio_queue_seconds);

    let _device = if decoding_service && !cli.no_audio {
        let config = OutputDeviceParameters {
            channels_count: 2, // DAB+ HE-AAC v2 typically outputs stereo
            sample_rate: AUDIO_RATE,
            channel_sample_count: 1024,
        };
        Some(
            run_output_device(config, move |data| {
                for sample in data.iter_mut() {
                    *sample = rx.try_recv().unwrap_or(0.0);
                }
            })
            .expect("Failed to open audio output device"),
        )
    } else {
        None
    };

    while let Some(chunk) = source.next_chunk().await {
        let samples = chunk?;
        let frames = ofdm_processor.process(&samples);

        for frame in &frames {
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

                for fib in &fibs {
                    fib_count += 1;
                    ensemble.parse_fib(fib);
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
                                if is_file_source {
                                    for sample in &decoded_out.pcm {
                                        if tx.send(*sample).is_err() {
                                            break;
                                        }
                                    }
                                } else {
                                    for sample in &decoded_out.pcm {
                                        if tx.try_send(*sample).is_err()
                                            && tx
                                                .send_timeout(*sample, Duration::from_millis(1))
                                                .is_err()
                                        {
                                            break;
                                        }
                                    }
                                }
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
) -> Result<(IqChunkSource, bool), Box<dyn std::error::Error>> {
    // File and stdin: use synchronous IqSource (simple, reliable)
    if input == "-" {
        let source = IqSource::from_stdin(center_freq, sample_rate, chunk_size, iq_format)?;
        return Ok((IqChunkSource::Sync(Box::new(source)), true));
    }

    if !is_device_uri(input) {
        let source = IqSource::from_file(input, center_freq, sample_rate, chunk_size, iq_format)?;
        return Ok((IqChunkSource::Sync(Box::new(source)), true));
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

    #[cfg(any(feature = "rtlsdr", feature = "soapy", feature = "airspy"))]
    {
        const LIVE_QUEUE_CHUNKS: usize = 512;

        let rx = spawn_sync_reader_thread(configured_uri.clone(), LIVE_QUEUE_CHUNKS)?;
        Ok((IqChunkSource::Threaded(rx), false))
    }

    #[cfg(not(any(feature = "rtlsdr", feature = "soapy", feature = "airspy")))]
    Err(
        format!("No SDR backend enabled for URI: {configured_uri}. Rebuild with --features rtlsdr")
            .into(),
    )
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
        truecolor: true,
        use_kitty: true,
        use_iterm: false,
        ..Default::default()
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
