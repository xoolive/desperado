use desperado::{IqFormat, IqSource};
use std::io::{self};

fn main() -> io::Result<()> {
    let path = "data/ais_96k.bin";
    let sample_rate = 96000;
    let iq_file = IqSource::from_file(path, 162000, sample_rate, 8136, IqFormat::Cu8)?;

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
        num_samples as f32 / sample_rate as f32,
        path
    );
    Ok(())
}
