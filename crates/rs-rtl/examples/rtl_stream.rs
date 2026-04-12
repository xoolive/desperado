/// Quick streaming test — read a few chunks of IQ data and print stats.
use std::time::Instant;

use rs_rtl::RtlSdr;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let mut sdr = RtlSdr::open(0).expect("failed to open RTL-SDR");

    sdr.set_center_freq(100_000_000).expect("set freq");
    sdr.set_sample_rate(2_048_000).expect("set rate");
    sdr.set_gain_manual(400).expect("set gain");

    println!(
        "Tuned to {} MHz, {} kS/s\n",
        sdr.center_freq() as f64 / 1e6,
        sdr.sample_rate() as f64 / 1e3,
    );

    println!("Starting streaming (5 seconds)...\n");
    let reader = sdr.start_streaming().expect("start streaming");

    let start = Instant::now();
    let mut total_bytes: u64 = 0;
    let mut chunk_count: u64 = 0;

    while start.elapsed().as_secs() < 5 {
        match reader.recv() {
            Some(data) => {
                total_bytes += data.len() as u64;
                chunk_count += 1;

                // Print first few IQ values of first chunk
                if chunk_count == 1 {
                    let n = data.len().min(20);
                    print!("First IQ bytes: ");
                    for b in &data[..n] {
                        print!("{:3} ", b);
                    }
                    println!("...\n");
                }
            }
            None => {
                eprintln!("Stream ended unexpectedly");
                break;
            }
        }
    }

    reader.stop();
    let elapsed = start.elapsed().as_secs_f64();

    println!("Results:");
    println!("  Duration:   {:.2} s", elapsed);
    println!("  Chunks:     {}", chunk_count);
    println!("  Total data: {:.2} MB", total_bytes as f64 / 1e6);
    println!(
        "  Throughput: {:.2} MB/s",
        total_bytes as f64 / 1e6 / elapsed
    );
    println!("  Dropped:    {}", reader.dropped_chunks());

    let expected_bytes = (sdr.sample_rate() as f64 * 2.0 * elapsed) as u64;
    let ratio = total_bytes as f64 / expected_bytes as f64;
    println!(
        "  Expected:   {:.2} MB ({:.1}% captured)",
        expected_bytes as f64 / 1e6,
        ratio * 100.0,
    );
}
