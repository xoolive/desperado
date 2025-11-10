use clap::Parser;
use desperado::{IqFormat, IqSource};
use std::io;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the IQ binary file
    #[arg(short, long)]
    bin_file: String,

    /// Sample rate in Hz
    #[arg(short, long)]
    sample_rate: u32,

    /// Center frequency in Hz
    #[arg(short, long)]
    center_freq: u32,

    /// IQ format: cu8, cs8, cs16 or cf32
    #[arg(short, long, default_value = "cu8", value_parser = parse_iq_format)]
    iq_format: IqFormat,
}

fn parse_iq_format(fmt: &str) -> Result<IqFormat, std::io::Error> {
    match fmt.to_lowercase().as_str() {
        "cu8" => Ok(IqFormat::Cu8),
        "cs8" => Ok(IqFormat::Cs8),
        "cs16" => Ok(IqFormat::Cs16),
        "cf32" => Ok(IqFormat::Cf32),
        _ => Err(std::io::Error::other(format!(
            "Unsupported IQ format: {}",
            fmt
        ))),
    }
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    let iq_file = IqSource::from_file(
        &args.bin_file,
        args.center_freq,
        args.sample_rate,
        8132,
        args.iq_format,
    )?;

    let mut num_samples = 0;

    for chunk in iq_file {
        if let Ok(samples) = chunk {
            num_samples += samples.len();
        } else {
            eprintln!("Error reading samples: {}", chunk.unwrap_err());
        }
    }

    println!(
        "Read {} I/Q samples ({} seconds) from '{}'",
        num_samples,
        num_samples as f32 / args.sample_rate as f32,
        args.bin_file
    );
    Ok(())
}
