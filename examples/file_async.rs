use desperado::{IqAsyncSource, IqFormat};
use futures::StreamExt;

#[tokio::main]
async fn main() -> tokio::io::Result<()> {
    let path = "data/ais_96k.bin";
    let sample_rate = 96000;
    let mut iq_file =
        IqAsyncSource::from_file(path, 162000, sample_rate, 8136, IqFormat::Cu8).await?;

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
        num_samples as f32 / sample_rate as f32,
        path
    );
    Ok(())
}
