//! Extract the AM envelope at ILS audio rate from a gqrx cf32 IQ file.
//!
//! Usage:
//! ```sh
//! cargo run --release --example extract_ils -- \
//!   <input.raw> <ils_freq_mhz> <center_freq_mhz> <out_stem> [max_seconds]
//! ```
//!
//! Produces one file:
//! - `<out_stem>_envelope.f32`: AM envelope at 9 kHz, f32 little-endian, no header.
//!
//! This file serves as the test fixture for `tests/ils_decoding.rs`.
//!
//! # Example
//!
//! ```sh
//! cargo run --release -p voracious --example extract_ils -- \
//!   samples_ils/gqrx_20251107_215806_110700000_1800000_fc.raw \
//!   110.695 110.7 tests/data/gqrx_20251107_215806_110700000_1800000_fc_ils
//! ```

use std::io::Read;
use voracious::decoders::IlsLocalizerDemodulator;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 5 {
        eprintln!(
            "Usage: {} <input.raw> <ils_freq_mhz> <center_freq_mhz> <out_stem> [max_seconds]",
            args[0]
        );
        std::process::exit(1);
    }

    let input_path = &args[1];
    let ils_freq_mhz: f64 = args[2].parse()?;
    let center_freq_mhz: f64 = args[3].parse()?;
    let out_stem = &args[4];
    let max_seconds: Option<f64> = args.get(5).and_then(|s| s.parse().ok());

    let sample_rate: u32 = 1_800_000;
    let freq_offset = (ils_freq_mhz - center_freq_mhz) * 1e6;

    let mut demod = IlsLocalizerDemodulator::new(sample_rate);
    let chunk_samples = 262_144usize;
    let chunk_bytes = chunk_samples * 8; // cf32 = 8 bytes/sample

    let mut input = std::fs::File::open(input_path)?;
    let mut envelope_out: Vec<f32> = Vec::new();
    let max_iq_samples = max_seconds.map(|s| (s * sample_rate as f64) as usize);
    let mut total_iq_samples = 0usize;

    loop {
        if let Some(max) = max_iq_samples
            && total_iq_samples >= max
        {
            break;
        }

        let mut buf = vec![0u8; chunk_bytes];
        let n = input.read(&mut buf)?;
        if n == 0 {
            break;
        }
        buf.truncate(n);

        // Parse cf32 (complex float32, little-endian interleaved)
        let samples: Vec<num_complex::Complex<f32>> = buf
            .chunks_exact(8)
            .map(|b| {
                let re = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                let im = f32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                num_complex::Complex::new(re, im)
            })
            .collect();

        total_iq_samples += samples.len();

        let (_env_90, _env_150, audio) = demod.demodulate(&samples, freq_offset);
        envelope_out.extend(audio.iter().map(|&x| x as f32));
    }

    let audio_rate = demod.audio_rate();
    eprintln!(
        "Processed {} IQ samples ({:.1}s) → {} envelope samples ({:.1}s) at {:.0} Hz",
        total_iq_samples,
        total_iq_samples as f64 / sample_rate as f64,
        envelope_out.len(),
        envelope_out.len() as f64 / audio_rate,
        audio_rate,
    );

    let out_path = format!("{out_stem}_envelope.f32");
    let bytes: Vec<u8> = envelope_out.iter().flat_map(|&f| f.to_le_bytes()).collect();
    std::fs::write(&out_path, &bytes)?;
    eprintln!(
        "  {out_path}: {} samples, {:.1} KB",
        envelope_out.len(),
        bytes.len() as f64 / 1024.0
    );

    Ok(())
}
