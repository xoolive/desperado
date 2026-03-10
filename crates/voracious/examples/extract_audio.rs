//! Extract demodulated audio and 30 Hz signals from a gqrx IQ file to f32 binary fixtures.
//!
//! Usage: extract_audio <input.raw> <vor_freq_mhz> <center_freq_mhz> <out_stem> [max_seconds]
//!
//! Produces three files:
//! - `<out_stem>_audio.f32`: mono audio at ~47368 Hz (f32 LE)
//! - `<out_stem>_var30.f32`: 30 Hz variable signal (f32 LE)
//! - `<out_stem>_ref30.f32`: 30 Hz reference signal (f32 LE)
//!
//! These serve as test fixtures for vor_decoding integration tests.

use std::io::Read;
use voracious::decoders::VorDemodulator;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 5 {
        eprintln!(
            "Usage: {} <input.raw> <vor_freq_mhz> <center_freq_mhz> <out_stem> [max_seconds]",
            args[0]
        );
        std::process::exit(1);
    }

    let input_path = &args[1];
    let vor_freq_mhz: f64 = args[2].parse()?;
    let center_freq_mhz: f64 = args[3].parse()?;
    let out_stem = &args[4];
    let max_seconds: Option<f64> = args.get(5).and_then(|s| s.parse().ok());

    let sample_rate: u32 = 1_800_000;
    let freq_offset = (vor_freq_mhz - center_freq_mhz) * 1e6;

    let mut demod = VorDemodulator::new(sample_rate);
    let chunk_samples = 262_144usize;
    let chunk_bytes = chunk_samples * 8; // cf32 = 8 bytes/sample

    let mut input = std::fs::File::open(input_path)?;
    let mut audio_out: Vec<f32> = Vec::new();
    let mut var30_out: Vec<f32> = Vec::new();
    let mut ref30_out: Vec<f32> = Vec::new();
    let max_samples = max_seconds.map(|s| (s * sample_rate as f64) as usize);
    let mut total_iq_samples = 0usize;

    loop {
        if let Some(max) = max_samples
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

        // Parse cf32 (complex float32, little-endian)
        let samples: Vec<num_complex::Complex<f32>> = buf
            .chunks_exact(8)
            .map(|b| {
                let re = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                let im = f32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                num_complex::Complex::new(re, im)
            })
            .collect();

        total_iq_samples += samples.len();

        let (var30, ref30, audio) = demod.demodulate(&samples, freq_offset);
        audio_out.extend(audio.iter().map(|&x| x as f32));
        var30_out.extend(var30.iter().map(|&x| x as f32));
        ref30_out.extend(ref30.iter().map(|&x| x as f32));
    }

    let audio_rate = demod.audio_rate();
    eprintln!(
        "Processed {} IQ samples ({:.1}s) → {} audio samples ({:.1}s) at {:.0} Hz",
        total_iq_samples,
        total_iq_samples as f64 / sample_rate as f64,
        audio_out.len(),
        audio_out.len() as f64 / audio_rate,
        audio_rate,
    );

    let write_f32 = |path: &str, data: &[f32]| -> std::io::Result<()> {
        let bytes: Vec<u8> = data.iter().flat_map(|&f| f.to_le_bytes()).collect();
        std::fs::write(path, &bytes)?;
        eprintln!(
            "  {path}: {} samples, {:.1} KB",
            data.len(),
            bytes.len() as f64 / 1024.0
        );
        Ok(())
    };

    write_f32(&format!("{out_stem}_audio.f32"), &audio_out)?;
    write_f32(&format!("{out_stem}_var30.f32"), &var30_out)?;
    write_f32(&format!("{out_stem}_ref30.f32"), &ref30_out)?;

    Ok(())
}
