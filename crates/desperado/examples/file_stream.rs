use clap::Parser;
use desperado::{IqAsyncSource, IqFormat};
use futures::StreamExt;

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
    #[arg(short, long, default_value = "cu8")]
    iq_format: IqFormat,
}

#[tokio::main]
async fn main() -> desperado::Result<()> {
    let args = Args::parse();
    let mut iq_file = IqAsyncSource::from_file(
        &args.bin_file,
        args.center_freq,
        args.sample_rate,
        8136,
        args.iq_format,
    )
    .await?;

    let mut num_samples = 0;

    while let Some(chunk) = iq_file.next().await {
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
