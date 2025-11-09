use clap::Parser;
use desperado::{IqFormat, IqSource};
use num_complex::Complex;
use rustfft::FftPlanner;

use plotters::prelude::*;

/// Waterfall plot generator for IQ files
#[derive(Parser, Debug)]
#[command(author, version, about = "A waterfall plot generator from IQ files", long_about = None)]
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

    /// FFT size
    #[arg(short, long, default_value_t = 1024)]
    fft_size: usize,
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

fn fftshift(data: &mut [Complex<f32>]) {
    let n = data.len();
    let half = n / 2;
    data.rotate_left(half);
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let iq = IqSource::from_file(
        args.bin_file,
        args.center_freq,
        args.sample_rate,
        args.fft_size,
        args.iq_format,
    )?;

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(args.fft_size);
    let mut waterfall: Vec<Vec<f32>> = Vec::new();
    let mut line = vec![0f32; args.fft_size];

    for chunk in iq {
        let samples = chunk?;

        let mut buffer = samples.clone();

        fft.process(&mut buffer);
        fftshift(&mut buffer);

        let scale = 1.0 / args.fft_size as f32;
        for v in buffer.iter_mut() {
            *v *= scale;
        }

        for (j, c) in buffer.iter().enumerate() {
            line[j] = 10.0 * c.norm_sqr().log10(); // better for power in dB
        }

        waterfall.push(line.clone());
    }

    let max_db = 0.0; // set 0 dB as max
    let min_db = -100.0; // show last 100 dB of dynamic range

    // Create drawing area
    let root = BitMapBackend::new("waterfall.png", (args.fft_size as u32, 512)).into_drawing_area();
    root.fill(&BLACK)?;

    // Draw waterfall from bottom up
    let height = waterfall.len();
    for (row, spectrum) in waterfall.iter().enumerate() {
        let y = height - 1 - row; // newest at bottom
        for (col, &db) in spectrum.iter().enumerate() {
            let norm = ((db - min_db) / (max_db - min_db + 1e-6)).clamp(0.0, 1.0);

            // Gqrx-style color map: dark blue → yellow → white
            let color = if norm < 0.25 {
                RGBColor(0, (norm * 4.0 * 64.0) as u8, 128)
            } else if norm < 0.5 {
                RGBColor(
                    ((norm - 0.25) * 4.0 * 255.0) as u8,
                    255,
                    (128.0 - (norm - 0.25) * 4.0 * 128.0) as u8,
                )
            } else if norm < 0.75 {
                RGBColor(255, (255.0 - (norm - 0.5) * 4.0 * 128.0) as u8, 0)
            } else {
                RGBColor(255, 255, ((norm - 0.75) * 4.0 * 255.0) as u8)
            };

            root.draw_pixel((col as i32, y as i32), &color)?;
        }
    }

    // Frequency axis
    let freq_start = args.center_freq as f32 - (args.sample_rate as f32) / 2.0;
    let freq_end = args.center_freq as f32 + (args.sample_rate as f32) / 2.0;

    let mut chart = ChartBuilder::on(&root)
        .margin(10)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(freq_start * 1e-6..freq_end * 1e-6, 0..height)?;

    // Axis styling
    chart
        .configure_mesh()
        .x_desc("Frequency (MHz)")
        .axis_desc_style(("Roboto Condensed", 18).into_font().color(&BLACK))
        .label_style(("Roboto Condensaed", 18).into_font().color(&BLACK))
        .draw()?;

    root.present()?;
    println!("Waterfall plot saved to waterfall.png");

    Ok(())
}
