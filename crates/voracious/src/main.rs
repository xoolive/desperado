//! VOR signal decoder CLI

use clap::Parser;
use std::path::PathBuf;
use voracious::sources::{IqFormat, IqSource};

/*#[cfg(feature = "rtlsdr")]
use voracious::sources::RtlSdrSource;*/

#[derive(Parser)]
#[command(name = "voracious")]
#[command(about = "VOR signal decoder for aviation navigation", long_about = None)]
struct Cli {
    /// I/Q sample file path or SDR URI (rtlsdr://, airspy://, soapy://)
    input: PathBuf,

    /// Sample rate in Hz (auto-detected from gqrx filename/URI when possible)
    #[arg(short, long)]
    sample_rate: Option<u32>,

    /// I/Q format for file inputs (cu8, cs8, cs16, cf32)
    #[arg(short, long, default_value = "cf32")]
    format: String,

    /// VOR frequency in MHz
    #[arg(short, long)]
    vor_freq: f64,

    /// SDR center frequency in MHz (auto-detected from gqrx filename/URI when possible)
    #[arg(short, long)]
    center_freq: Option<f64>,

    /// Window size in seconds for radial calculation
    #[arg(short, long, default_value = "3.0")]
    window: f64,

    /// Window size in seconds for Morse decoding (should be longer to capture full ident cycle)
    #[arg(long, default_value = "15.0")]
    morse_window: f64,

    /// Enable debug output for Morse decoding
    #[arg(long)]
    debug_morse: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    let input_str = cli.input.to_string_lossy();
    let is_device_uri = input_str.starts_with("rtlsdr://")
        || input_str.starts_with("airspy://")
        || input_str.starts_with("soapy://");

    let inferred = infer_gqrx_metadata(&cli.input);

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
        None if is_device_uri => cli.vor_freq,
        None => {
            eprintln!(
                "Missing --center-freq and could not infer from filename. \
Use --center-freq explicitly or provide a gqrx file named like gqrx_*_<centerHz>_<sampleRate>_fc.raw"
            );
            std::process::exit(1);
        }
    };

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

    let source = IqSource::new(
        cli.input,
        sample_rate,
        iq_format,
        cli.vor_freq,
        center_freq,
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
                eprintln!("Error reading samples: {}", e);
                break;
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
