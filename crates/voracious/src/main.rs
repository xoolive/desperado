//! VOR/ILS signal decoder CLI

use clap::Parser;
use std::path::PathBuf;
use voracious::{IqFormat, VorSource, WavIlsSource, WavVorSource};

#[derive(Parser)]
#[command(name = "voracious")]
#[command(about = "VOR/ILS signal decoder for aviation navigation", long_about = None)]
struct Cli {
    /// I/Q sample file path or SDR URI (rtlsdr://, airspy://, soapy://)
    input: PathBuf,

    /// Sample rate in Hz (auto-detected from gqrx filename/URI when possible)
    #[arg(short, long)]
    sample_rate: Option<u32>,

    /// I/Q format for file inputs (cu8, cs8, cs16, cf32)
    /// gqrx files are always cf32, rtlsdr files are always cu8
    #[arg(short, long, default_value = "cf32")]
    format: String,

    /// VOR frequency in MHz (required unless --ils-freq is specified)
    #[arg(short, long)]
    vor_freq: Option<f64>,

    /// ILS frequency in MHz (required unless --vor-freq is specified)
    #[arg(long)]
    ils_freq: Option<f64>,

    /// SDR center frequency in MHz (auto-detected from gqrx filename/URI when possible)
    #[arg(short, long)]
    center_freq: Option<f64>,

    /// Window size in seconds for radial calculation (VOR only)
    #[arg(short, long, default_value = "3.0")]
    window: f64,

    /// Window size in seconds for Morse decoding (should be longer to capture full ident cycle)
    #[arg(long, default_value = "15.0")]
    morse_window: f64,

    /// Enable debug output for Morse decoding
    #[arg(long)]
    debug_morse: bool,

    /// Enable audio output to soundcard (streams demodulated audio)
    #[arg(long)]
    audio: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    // Validate that exactly one of --vor-freq or --ils-freq is specified
    match (cli.vor_freq, cli.ils_freq) {
        (Some(_), Some(_)) => {
            eprintln!("Error: specify either --vor-freq or --ils-freq, not both");
            std::process::exit(1);
        }
        (None, None) => {
            eprintln!("Error: specify either --vor-freq or --ils-freq");
            std::process::exit(1);
        }
        _ => {}
    }

    let input_str = cli.input.to_string_lossy();
    let is_wav = input_str.ends_with(".wav") || input_str.ends_with(".WAV");
    let is_device_uri = input_str.starts_with("rtlsdr://")
        || input_str.starts_with("airspy://")
        || input_str.starts_with("soapy://");

    // Setup audio output if enabled via --audio flag
    let audio_output = if cli.audio {
        let is_file_source = !is_device_uri;
        let buffer_size = if is_file_source {
            voracious::audio::AUDIO_RATE * 4 // 4 seconds for file playback
        } else {
            voracious::audio::AUDIO_RATE * 2 // 2 seconds for live sources
        };

        match voracious::audio::AudioOutput::new(buffer_size) {
            Some(audio) => {
                eprintln!(
                    "Audio output enabled at {} Hz (mono)",
                    voracious::audio::AUDIO_RATE
                );
                Some(audio)
            }
            None => {
                eprintln!(
                    "Warning: Could not initialize audio output (no soundcard or permission denied)"
                );
                None
            }
        }
    } else {
        None
    };

    // Handle WAV files separately (no need for sample-rate or center-freq)
    if is_wav {
        if let Some(ils_freq) = cli.ils_freq {
            let source = WavIlsSource::new(
                &cli.input,
                ils_freq,
                cli.window,
                cli.morse_window,
                cli.debug_morse,
            )?;

            for result in source {
                match result {
                    Ok(frame) => {
                        let json = serde_json::to_string(&frame)?;
                        println!("{}", json);
                    }
                    Err(e) => {
                        eprintln!("Error reading WAV: {}", e);
                        break;
                    }
                }
            }
            return Ok(());
        } else if let Some(vor_freq) = cli.vor_freq {
            let source = WavVorSource::new(
                &cli.input,
                vor_freq,
                cli.window,
                cli.morse_window,
                cli.debug_morse,
            )?;

            for result in source {
                match result {
                    Ok(radial) => {
                        let json = serde_json::to_string(&radial)?;
                        println!("{}", json);
                    }
                    Err(e) => {
                        eprintln!("Error reading WAV: {}", e);
                        break;
                    }
                }
            }
            return Ok(());
        }
    }

    // Can be convenient to infer sample rate and center frequency from gqrx filenames
    let inferred = infer_gqrx_metadata(&cli.input);

    // For SDR URIs, also attempt to infer from URI parameters
    let sample_rate = cli
        .sample_rate
        .or_else(|| {
            if is_device_uri {
                infer_uri_u32_param(&input_str, &["rate", "sample_rate"])
            } else {
                None
            }
        })
        .or_else(|| inferred.as_ref().map(|m| m.sample_rate_hz))
        .unwrap_or(1_800_000);

    // For center frequency, we prioritize CLI > URI > gqrx filename > default.
    // This is because the center frequency is critical for correct decoding,
    // and it's safer to require explicit specification if it can't be reliably
    // inferred.
    let nav_freq = cli.vor_freq.or(cli.ils_freq).unwrap(); // Already validated above
    let center_freq = match cli
        .center_freq
        .or_else(|| {
            if is_device_uri {
                infer_uri_f64_param(&input_str, &["freq", "frequency"])
            } else {
                None
            }
        })
        .or_else(|| inferred.as_ref().map(|m| m.center_freq_mhz))
    {
        Some(v) => v,
        None if is_device_uri => nav_freq,
        None => {
            eprintln!(
                "Missing --center-freq and could not infer from filename. \
Use --center-freq explicitly or provide a gqrx file named like gqrx_*_<centerHz>_<sampleRate>_fc.raw"
            );
            std::process::exit(1);
        }
    };

    // gqrx files are always complex float
    // rtlsdr files are always complex uint8 (cu8)
    let iq_format = match cli.format.parse::<IqFormat>() {
        Ok(v) => v,
        Err(_) => {
            eprintln!(
                "Invalid format: {}. Use cu8, cs8, cs16, or cf32",
                cli.format
            );
            std::process::exit(1);
        }
    };

    // Route to VOR or ILS decoder
    if let Some(vor_freq) = cli.vor_freq {
        // VOR I/Q mode
        let mut source = VorSource::new(
            cli.input,
            sample_rate,
            iq_format,
            vor_freq,
            center_freq,
            cli.window,
            cli.morse_window,
            cli.debug_morse,
        )?;

        // Pass audio output to source for streaming
        if let Some(audio) = audio_output {
            source.set_audio_output(audio);
        }

        for result in source {
            match result {
                Ok(radial) => {
                    let json = serde_json::to_string(&radial)?;
                    println!("{}", json);
                }
                Err(e) => {
                    eprintln!("Error reading samples: {}", e);
                    break;
                }
            }
        }
    } else {
        // ILS I/Q mode
        let ils_freq = cli.ils_freq.unwrap(); // Already validated above
        use voracious::IlsSource;
        let mut source = IlsSource::new(
            cli.input,
            sample_rate,
            iq_format,
            ils_freq,
            center_freq,
            cli.window,
            cli.morse_window,
            cli.debug_morse,
        )?;

        // Pass audio output to source for streaming
        if let Some(audio) = audio_output {
            source.set_audio_output(audio);
        }

        for result in source {
            match result {
                Ok(frame) => {
                    let json = serde_json::to_string(&frame)?;
                    println!("{}", json);
                }
                Err(e) => {
                    eprintln!("Error reading samples: {}", e);
                    break;
                }
            }
        }
    }

    Ok(())
}

fn infer_uri_param<'a>(uri: &'a str, keys: &[&str]) -> Option<&'a str> {
    let query = uri.split_once('?')?.1;
    for param in query.split('&') {
        let (k, v) = param.split_once('=')?;
        if keys.contains(&k) {
            return Some(v);
        }
    }
    None
}

fn infer_uri_u32_param(uri: &str, keys: &[&str]) -> Option<u32> {
    let raw = infer_uri_param(uri, keys)?;
    desperado::parse_si_value::<u32>(raw).ok()
}

fn infer_uri_f64_param(uri: &str, keys: &[&str]) -> Option<f64> {
    let raw = infer_uri_param(uri, keys)?;
    desperado::parse_si_value::<f64>(raw)
        .ok()
        .map(|hz| hz / 1e6)
}

#[derive(Debug, Clone, Copy)]
struct GqrxMeta {
    center_freq_mhz: f64,
    sample_rate_hz: u32,
}

fn infer_gqrx_metadata(path: &std::path::Path) -> Option<GqrxMeta> {
    let name = path.file_name()?.to_str()?;
    if !name.starts_with("gqrx_") || !name.ends_with("_fc.raw") {
        return None;
    }

    let stem = name.strip_suffix("_fc.raw")?;
    let parts: Vec<&str> = stem.split('_').collect();
    if parts.len() < 5 {
        return None;
    }

    let center_hz: u64 = parts.get(parts.len() - 2)?.parse().ok()?;
    let sample_rate_hz: u32 = parts.last()?.parse().ok()?;

    Some(GqrxMeta {
        center_freq_mhz: center_hz as f64 / 1e6,
        sample_rate_hz,
    })
}
