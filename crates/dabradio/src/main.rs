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
use desperado::IqFormat;
use pad::PadData;
use tinyaudio::prelude::*;
use tracing::{debug, info, warn};

const AUDIO_RATE: usize = 48_000;

#[derive(Parser)]
#[command(name = "dabradio", about = "DAB/DAB+ digital radio decoder")]
struct Cli {
    /// Source: file path to IQ recording
    source: Option<String>,

    /// DAB channel (e.g. "12A", "12C")
    #[arg(long)]
    channel: Option<String>,

    /// Center frequency in Hz (alternative to --channel)
    #[arg(short = 'f', long)]
    freq: Option<u32>,

    /// IQ format: cu8, cs8, cs16, cf32
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

    /// Print extracted PAD/DLS metadata candidates while decoding
    #[arg(long)]
    metadata: bool,

    /// Save MOT slideshow images to directory (e.g. --slideshow ./images)
    #[arg(long)]
    slideshow: Option<String>,

    /// Debug: bypass time de-interleaving in MSC
    #[arg(long)]
    bypass_deinterleave: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

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
        .ok_or("Source file path is required unless --list-channels is used")?;

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

    // Open IQ source using desperado's synchronous reader
    let chunk_size = constants::T_F; // One frame's worth of samples
    let source = desperado::iqread::IqRead::from_file(
        source,
        center_freq,
        constants::SAMPLE_RATE,
        chunk_size,
        iq_format,
    )?;

    let mut ofdm_processor = ofdm::processor::OfdmProcessor::new();
    let mut ensemble = fic::fib::EnsembleInfo::new();
    let mut frame_count = 0usize;
    let mut fib_count = 0usize;

    // MSC decoding state (initialized once we know the service -> subchannel mapping)
    let mut msc_handler: Option<msc::MscHandler> = None;
    let mut dab_plus_decoder: Option<audio::DabPlusDecoder> = None;
    let mut msc_output: Vec<Vec<u8>> = Vec::new();
    let decoding_service = cli.service.is_some();
    let mut mot_image_count = 0usize;

    // Audio output setup (tinyaudio + crossbeam channel)
    let (tx, rx) = channel::bounded::<f32>(AUDIO_RATE * 4); // 4 seconds buffer

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

    let is_file_source = true; // Currently only file sources are supported

    for chunk in source {
        let samples = chunk?;
        let frames = ofdm_processor.process(&samples);

        for frame in &frames {
            frame_count += 1;

            // DQPSK decode all symbols
            let soft_bits = ofdm::decoder::dqpsk_decode(&frame.symbols);

            // Extract FIC data from symbols 0..2 (first 3 data symbols after PRS)
            if soft_bits.len() >= constants::FIC_SYMBOLS {
                let fic_symbols: Vec<Vec<i8>> = soft_bits[..constants::FIC_SYMBOLS].to_vec();
                let fibs = fic::handler::process_fic(&fic_symbols);

                for fib in &fibs {
                    fib_count += 1;
                    ensemble.parse_fib(fib);
                }
            }

            // Check if we have enough info to list services
            if cli.list && ensemble.is_complete() {
                ensemble.resolve_services();
                print_services(&ensemble, cli.json);
                return Ok(());
            }

            // Fallback: if --list and we've processed enough frames, print what we have
            if cli.list && frame_count >= 50 && ensemble.has_services() {
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
                    let mut handler = handler;
                    if cli.bypass_deinterleave {
                        handler.set_bypass_deinterleave(true);
                        debug!("Time de-interleaving BYPASSED");
                    }
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
                                        if cli.metadata {
                                            eprintln!("DLS: {}", dls.text);
                                        }
                                    }
                                    PadData::Mot(mot) => {
                                        mot_image_count += 1;
                                        let ext = if mot.content_type.contains("png") {
                                            "png"
                                        } else {
                                            "jpg"
                                        };
                                        if cli.metadata {
                                            eprintln!(
                                                "MOT: {} bytes, type={}, name={:?}",
                                                mot.data.len(),
                                                mot.content_type,
                                                mot.content_name
                                            );
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
                                    }
                                }
                            }

                            if !decoded_out.pcm.is_empty() && _device.is_some() {
                                // Send PCM samples to audio output
                                if is_file_source {
                                    // Blocking send — audio buffer controls pacing
                                    for sample in &decoded_out.pcm {
                                        if tx.send(*sample).is_err() {
                                            break;
                                        }
                                    }
                                } else {
                                    for sample in &decoded_out.pcm {
                                        if tx.try_send(*sample).is_err() {
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
            eprintln!(
                "DAB+ decode: {} superframes, {} RS corrections, {} RS errors, {} AU CRC errors",
                sf.superframes_decoded, sf.rs_corrections, sf.rs_errors, sf.au_crc_errors,
            );
        }

        if let Some(ref handler) = msc_handler {
            eprintln!(
                "MSC: {} logical frames decoded ({} OFDM frames, {} FIBs)",
                handler.frames_decoded, frame_count, fib_count,
            );
        } else {
            eprintln!(
                "No MSC frames decoded (processed {} OFDM frames, {} FIBs)",
                frame_count, fib_count
            );
            eprintln!("  (MSC handler never initialized -- service not found?)");
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
            eprintln!("Wrote {} bytes to {}", all_bytes.len(), output_path);
        }
    } else {
        // Just print service listing
        let output = ensemble.to_output();

        if cli.json {
            println!("{}", serde_json::to_string_pretty(&output)?);
        } else {
            eprintln!("Processed {} frames, {} valid FIBs", frame_count, fib_count);
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

/// Try to find the requested service and initialize an MSC handler for it.
/// Returns the handler and the service bitrate.
fn try_init_msc(
    ensemble: &fic::fib::EnsembleInfo,
    service_arg: &str,
) -> Option<(msc::MscHandler, u16)> {
    // Try to match by hex SId (e.g. "0xF201") or by label (case-insensitive)
    let target_sid = if let Some(hex) = service_arg
        .strip_prefix("0x")
        .or_else(|| service_arg.strip_prefix("0X"))
    {
        u32::from_str_radix(hex, 16).ok()
    } else {
        None
    };

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

    eprintln!(
        "Decoding service: {} (SId: 0x{:04X}, SubCh: {}, {} kbps, {})",
        service.label.as_deref().unwrap_or("?"),
        service.service_id,
        subch_id,
        bitrate,
        if subch.is_eep {
            let opt = if subch.eep_option == 0 { "A" } else { "B" };
            format!("EEP {}-{}", subch.protection_level + 1, opt)
        } else {
            format!("UEP {}", subch.protection_level)
        }
    );

    let handler = msc::MscHandler::new(subch);
    if handler.is_none() {
        eprintln!(
            "Failed to initialize MSC handler for subchannel {}",
            subch_id
        );
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
