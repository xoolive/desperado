use clap::Parser;
use desperado::dsp::DspBlock;
use desperado::dsp::decimator::Decimator;
use desperado::dsp::rotate::Rotate;
use desperado::rtlsdr::{DeviceSelector, RtlSdrConfig, RtlSdrReader};
use desperado::{Gain, RfMetricsCalculator, parse_si_value};
use fmradio::PhaseExtractor;
use fmradio::rds::{RdsDecoder, RdsResamplerCustom, StereoDecoderPLL};
use futures::StreamExt;
use rayon::prelude::*;
use std::io::Write;
use std::time::{Duration, Instant};

const FM_CHANNEL_CUTOFF_HZ: f64 = 120_000.0;

#[derive(Debug, Clone, Copy)]
struct Frequency(u32);

impl std::str::FromStr for Frequency {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_si_value::<u32>(s)
            .map(Frequency)
            .map_err(|e| e.to_string())
    }
}

#[derive(Parser, Debug)]
#[command(name = "fm_scan", about = "Naive FM band scanner using RF metrics")]
struct Args {
    /// SDR source URL-like selector, e.g. rtlsdr:// or rtlsdr://1
    #[arg(long, default_value = "rtlsdr://")]
    source: String,

    /// Scan start frequency, e.g. 88M
    #[arg(long, default_value = "87.5M")]
    start: Frequency,

    /// Scan end frequency, e.g. 108M
    #[arg(long, default_value = "108M")]
    end: Frequency,

    /// Scan step, e.g. 200k
    #[arg(long, default_value = "100k")]
    step: Frequency,

    /// Sample rate in Hz
    #[arg(long, default_value_t = 2_400_000)]
    sample_rate: u32,

    /// Dwell per frequency in milliseconds
    #[arg(long, default_value_t = 90)]
    dwell_ms: u64,

    /// Estimated tuner ppm error (applied as center of digital peak search)
    #[arg(long, default_value_t = 0.0)]
    ppm_estimate: f64,

    /// +/- ppm search window around --ppm-estimate
    #[arg(long, default_value_t = 40.0)]
    ppm_search: f64,

    /// Number of frequency hypotheses in peak search (odd recommended)
    #[arg(long, default_value_t = 11)]
    ppm_steps: usize,

    /// Optional tuner gain in tenths of dB (RTL-SDR), omit for auto
    #[arg(long)]
    gain: Option<i32>,

    /// Disable startup gain calibration when --gain is not provided
    #[arg(long, default_value_t = false)]
    no_calibrate_gain: bool,

    /// Dwell per probe during gain calibration in milliseconds
    #[arg(long, default_value_t = 120)]
    calibration_dwell_ms: u64,

    /// Verify RDS for candidates with RF score >= threshold
    #[arg(long, default_value_t = 2.0)]
    rds_score_threshold: f64,

    /// Disable RDS verification stage
    #[arg(long, default_value_t = false)]
    no_rds_verify: bool,

    /// RDS verification dwell time per candidate in seconds
    #[arg(long, default_value_t = 5)]
    rds_verify_seconds: u64,

    /// Minimum frequency separation for consecutive RDS verifications (kHz)
    #[arg(long, default_value_t = 250)]
    rds_min_separation_khz: u32,

    /// Enable debug output for RDS verification decisions
    #[arg(long, default_value_t = false)]
    debug_rds_selection: bool,
}

#[derive(Debug, Clone)]
struct ScanResult {
    freq_hz: u32,
    clipping_ratio: f64,
    noise_floor_db: f64,
    channel_power_db: f64,
    peak_offset_hz: f64,
    kurtosis: f64,
    prominence_db: f64,
    rf_score: f64,
    final_score: f64,
    rds_ps: Option<String>,
    rds_groups: usize,
    rds_note: Option<String>,
}

struct RdsHit {
    ps: String,
    groups: usize,
    af_mhz: Vec<f32>,
}

#[derive(Clone, Copy)]
struct VerifyRdsConfig {
    device_index: usize,
    sample_rate: u32,
    timeout_secs: u64,
    gain_tenths_db: Option<i32>,
    debug: bool,
}

#[derive(Clone, Copy)]
struct CalibrationConfig {
    sample_rate: u32,
    dwell: Duration,
    ppm_estimate: f64,
    ppm_search: f64,
    ppm_steps: usize,
}

fn verify_rds(
    device_index: usize,
    freq_hz: u32,
    sample_rate: u32,
    timeout_secs: u64,
    gain_tenths_db: Option<i32>,
) -> Option<RdsHit> {
    let offset_freq = 200_000u32;
    let tuning_freq = freq_hz.saturating_sub(offset_freq);
    let mpx_target = 240_000.0_f32;
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .ok()?;

    runtime.block_on(async move {
        let mut iq = {
            let mut attempts = 0usize;
            loop {
                match desperado::IqAsyncSource::from_rtlsdr(
                    device_index,
                    tuning_freq,
                    sample_rate,
                    gain_tenths_db,
                )
                .await
                {
                    Ok(src) => break src,
                    Err(e) => {
                        let msg = e.to_string();
                        if msg.contains("Usb(Busy)") && attempts < 120 {
                            attempts += 1;
                            tokio::time::sleep(Duration::from_millis(40)).await;
                            continue;
                        }
                        return None;
                    }
                }
            }
        };

        let mut rotate =
            Rotate::new(-2.0 * std::f32::consts::PI * offset_freq as f32 / sample_rate as f32);
        let mut phase_extractor = PhaseExtractor::new();
        let factor = (sample_rate as f32 / mpx_target).round() as usize;
        let mut decimator = Decimator::new(factor);
        let mpx_rate = sample_rate as f32 / factor as f32;
        let mut stereo = StereoDecoderPLL::new(mpx_rate);
        let mut rds_resampler = RdsResamplerCustom::new(mpx_rate, 171_000.0);
        let mut rds = RdsDecoder::new(171_000.0, false);
        rds.set_print_json_output(false);

        let mut warmup_chunks = 4usize;
        let deadline = Instant::now() + Duration::from_secs(timeout_secs.max(2));
        while Instant::now() < deadline {
            let Some(Ok(chunk)) = iq.next().await else {
                break;
            };
            let shifted = rotate.process(&chunk);
            let decimated = decimator.process(&shifted);
            let phase = phase_extractor.process(&decimated);
            let (_l, _r, pilot_phases) = stereo.process(&phase);
            let (rds_i, rds_q) = rds_resampler.process_with_pilot(&phase, &pilot_phases);
            if rds_i.is_empty() {
                continue;
            }
            rds.process_iq(&rds_i, &rds_q);

            if warmup_chunks > 0 {
                warmup_chunks -= 1;
                continue;
            }

            for (groups, json_out) in rds.take_json_outputs().into_iter().enumerate() {
                if let Some(ps) = json_out
                    .ps
                    .as_deref()
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
                {
                    return Some(RdsHit {
                        ps: ps.to_string(),
                        groups,
                        af_mhz: json_out
                            .af
                            .as_ref()
                            .map(|v| {
                                v.iter()
                                    .filter_map(|s| s.parse::<f32>().ok())
                                    .collect::<Vec<f32>>()
                            })
                            .unwrap_or_default(),
                    });
                }
            }

            if let Some(ps) = rds
                .station_name()
                .as_deref()
                .map(str::trim)
                .filter(|s| !s.is_empty())
            {
                let (_, _, decoded_groups) = rds.stats();
                return Some(RdsHit {
                    ps: ps.to_string(),
                    groups: decoded_groups as usize,
                    af_mhz: Vec::new(),
                });
            }
        }
        None
    })
}

fn gain_from_tenths_db(gain_tenths_db: Option<i32>) -> Gain {
    match gain_tenths_db {
        Some(g) => Gain::Manual(f64::from(g) / 10.0),
        None => Gain::Auto,
    }
}

fn open_scan_reader(
    device_index: usize,
    center_freq: u32,
    sample_rate: u32,
    gain_tenths_db: Option<i32>,
) -> desperado::Result<RtlSdrReader> {
    let mut last_err: Option<desperado::Error> = None;
    let mut retry_count = 0;
    for _ in 0..150 {
        match RtlSdrReader::new(&RtlSdrConfig {
            device: DeviceSelector::Index(device_index),
            center_freq,
            sample_rate,
            gain: gain_from_tenths_db(gain_tenths_db),
            bias_tee: false,
            freq_correction_ppm: 0,
        }) {
            Ok(reader) => {
                if retry_count > 0 {
                    println!("RTL-SDR opened after {} retries", retry_count);
                }
                return Ok(reader);
            }
            Err(e) => {
                let msg = e.to_string();
                if msg.contains("Usb(Busy)") {
                    if retry_count == 0 {
                        print!("RTL-SDR device busy, waiting...");
                        std::io::stdout().flush().unwrap();
                    } else if retry_count % 10 == 0 {
                        print!(".");
                        std::io::stdout().flush().unwrap();
                    }
                    retry_count += 1;
                    std::thread::sleep(Duration::from_millis(40));
                    last_err = Some(e);
                    continue;
                }
                return Err(e);
            }
        }
    }
    if retry_count > 0 {
        println!(); // Clear the waiting line
    }
    Err(last_err.unwrap_or_else(|| std::io::Error::other("failed to open RTL-SDR device").into()))
}

fn median(values: &mut [f64]) -> f64 {
    if values.is_empty() {
        return f64::NEG_INFINITY;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    let mid = values.len() / 2;
    if values.len().is_multiple_of(2) {
        (values[mid - 1] + values[mid]) / 2.0
    } else {
        values[mid]
    }
}

fn compute_rf_score_at(results: &[ScanResult], i: usize) -> (f64, f64) {
    let radius = 4usize;
    let start = i.saturating_sub(radius);
    let end = (i + radius + 1).min(results.len());

    let mut window: Vec<f64> = results[start..end]
        .iter()
        .enumerate()
        .filter_map(|(j, row)| {
            let idx = start + j;
            (idx != i).then_some(row.channel_power_db)
        })
        .collect();

    let baseline = median(&mut window);
    let prominence = results[i].channel_power_db - baseline;

    let left = i
        .checked_sub(1)
        .map(|j| results[j].channel_power_db)
        .unwrap_or(results[i].channel_power_db);
    let right = results
        .get(i + 1)
        .map(|r| r.channel_power_db)
        .unwrap_or(results[i].channel_power_db);
    let local_peakness = results[i].channel_power_db - left.max(right);
    let offset_penalty = (results[i].peak_offset_hz.abs() / 1000.0) * 0.03;

    let rf_score =
        prominence + (0.4 * local_peakness) - (80.0 * results[i].clipping_ratio) - offset_penalty;
    (rf_score, prominence)
}

fn should_verify_rds(
    results: &[ScanResult],
    idx: usize,
    threshold: f64,
    last_verified_freq: Option<u32>,
    min_separation_hz: u32,
) -> bool {
    if results[idx].rf_score < threshold {
        return false;
    }

    if let Some(left) = idx.checked_sub(1).map(|j| results[j].rf_score)
        && results[idx].rf_score < left
    {
        return false;
    }
    if let Some(right) = results.get(idx + 1).map(|r| r.rf_score)
        && results[idx].rf_score < right
    {
        return false;
    }

    if let Some(last) = last_verified_freq
        && results[idx].freq_hz.abs_diff(last) < min_separation_hz
    {
        return false;
    }

    true
}

fn pick_verify_freq(results: &[ScanResult], idx: usize) -> u32 {
    let mut best_idx = idx;
    let mut best_power = results[idx].channel_power_db;
    let mut best_rf = results[idx].rf_score;

    let start = idx.saturating_sub(1);
    let end = (idx + 2).min(results.len());

    for (j, result) in results.iter().enumerate().take(end).skip(start) {
        let power = result.channel_power_db;
        let rf = result.rf_score;

        // Pick strongest channel_power_db first, rf_score as tiebreaker
        if power > best_power || (power == best_power && rf > best_rf) {
            best_idx = j;
            best_power = power;
            best_rf = rf;
        }
    }

    results[best_idx].freq_hz
}

fn af_contains(af: &[f32], freq_mhz: f32) -> bool {
    af.iter().any(|f| (f - freq_mhz).abs() <= 0.06)
}

fn verify_rds_with_fallback(
    results: &[ScanResult],
    idx: usize,
    primary_freq: u32,
    cfg: VerifyRdsConfig,
) -> Option<RdsHit> {
    // Build candidate list: primary + adjacent bins within ±100k, sorted by channel_power
    let mut candidates = Vec::new();
    let step_hz = 100_000u32;

    // Add primary
    candidates.push((
        primary_freq,
        results
            .iter()
            .find(|r| r.freq_hz == primary_freq)?
            .channel_power_db,
    ));

    // Add ±100k if they exist in results and different from primary
    if let Some(lower) = primary_freq.checked_sub(step_hz)
        && let Some(r) = results.iter().find(|r| r.freq_hz == lower)
        && lower != primary_freq
    {
        candidates.push((lower, r.channel_power_db));
    }
    if let Some(upper) = primary_freq.checked_add(step_hz)
        && let Some(r) = results.iter().find(|r| r.freq_hz == upper)
        && upper != primary_freq
    {
        candidates.push((upper, r.channel_power_db));
    }

    // Sort by descending power
    candidates.sort_by(|a, b| b.1.total_cmp(&a.1));

    if cfg.debug {
        eprintln!(
            "[DEBUG] RDS verify @ idx {} ({:.3} MHz): candidates {:?}",
            idx,
            results[idx].freq_hz as f64 / 1e6,
            candidates
                .iter()
                .map(|(f, p)| format!("{:.3}M ({:.1}dB)", *f as f64 / 1e6, p))
                .collect::<Vec<_>>()
        );
    }

    // Try each candidate with reduced timeout
    let timeout_per_attempt = if candidates.len() == 1 {
        cfg.timeout_secs
    } else {
        (cfg.timeout_secs / 2).max(3)
    };

    for (attempt, (freq, _power)) in candidates.iter().enumerate() {
        if cfg.debug {
            eprintln!(
                "[DEBUG]   attempt {}: {:.3} MHz",
                attempt + 1,
                *freq as f64 / 1e6
            );
        }

        if let Some(hit) = verify_rds(
            cfg.device_index,
            *freq,
            cfg.sample_rate,
            timeout_per_attempt,
            cfg.gain_tenths_db,
        ) {
            if cfg.debug {
                eprintln!("[DEBUG]   SUCCESS: {} ({} groups)", hit.ps, hit.groups);
            }
            return Some(hit);
        }

        // If first attempt fails and we have more candidates, try one retry on same freq
        if attempt == 0
            && candidates.len() > 1
            && let Some(hit) = verify_rds(
                cfg.device_index,
                *freq,
                cfg.sample_rate,
                timeout_per_attempt,
                cfg.gain_tenths_db,
            )
        {
            if cfg.debug {
                eprintln!(
                    "[DEBUG]   SUCCESS (retry): {} ({} groups)",
                    hit.ps, hit.groups
                );
            }
            return Some(hit);
        }
    }

    if cfg.debug {
        eprintln!("[DEBUG]   FAILED all attempts");
    }
    None
}

fn format_sweep_row(row: &ScanResult) -> String {
    format!(
        "{:>8.3} MHz | rf {:>7.2} | chan {:>7.2} dBFS | wide {:>7.2} dBFS | peak {:+6.1} kHz | clip {:>6.3}% | kurt {:>5.2}",
        row.freq_hz as f64 / 1e6,
        row.rf_score,
        row.channel_power_db,
        row.noise_floor_db,
        row.peak_offset_hz / 1000.0,
        row.clipping_ratio * 100.0,
        row.kurtosis
    )
}

fn collect_rf_metrics(
    reader: &mut RtlSdrReader,
    freq: u32,
    sample_rate: u32,
    dwell: Duration,
    ppm_estimate: f64,
    ppm_search: f64,
    ppm_steps: usize,
) -> Option<(desperado::RfMetrics, f64, f64)> {
    if reader.tune(freq).is_err() {
        return None;
    }

    let mut calc =
        RfMetricsCalculator::with_config(sample_rate as f64, dwell.as_secs_f64(), 20.0, 0.98);
    let steps = ppm_steps.max(1);
    let base_offset_hz = freq as f64 * ppm_estimate / 1e6;
    let ppm_span_hz = freq as f64 * ppm_search.abs() / 1e6;
    let offsets_hz: Vec<f64> = if steps == 1 {
        vec![base_offset_hz]
    } else {
        (0..steps)
            .map(|i| {
                let frac = i as f64 / (steps as f64 - 1.0);
                let delta = (frac * 2.0 - 1.0) * ppm_span_hz;
                base_offset_hz + delta
            })
            .collect()
    };

    let dt = 1.0 / sample_rate as f64;
    let rc = 1.0 / (2.0 * std::f64::consts::PI * FM_CHANNEL_CUTOFF_HZ);
    let alpha = (dt / (rc + dt)).clamp(0.0, 1.0);
    let mut all_samples = Vec::new();
    let started = Instant::now();
    let mut processed = false;
    while started.elapsed() < dwell {
        match reader.next() {
            Some(Ok(samples)) => {
                processed = true;
                let t = started.elapsed().as_secs_f64();
                let _ = calc.push_chunk(&samples, t);
                all_samples.extend_from_slice(&samples);
            }
            Some(Err(_)) | None => break,
        }
    }

    if processed {
        let m = calc.snapshot(started.elapsed().as_secs_f64());
        let (best_db, best_off) = offsets_hz
            .par_iter()
            .map(|off| {
                let phase_inc = -2.0 * std::f64::consts::PI * *off / sample_rate as f64;
                let mut phase = 0.0f64;
                let mut ch_filter_i = 0.0f64;
                let mut ch_filter_q = 0.0f64;
                let mut ch_power_sum = 0.0f64;

                for s in &all_samples {
                    let i = s.re as f64;
                    let q = s.im as f64;
                    let c = phase.cos();
                    let sn = phase.sin();
                    let mixed_i = i * c + q * sn;
                    let mixed_q = q * c - i * sn;

                    ch_filter_i += alpha * (mixed_i - ch_filter_i);
                    ch_filter_q += alpha * (mixed_q - ch_filter_q);
                    ch_power_sum += ch_filter_i * ch_filter_i + ch_filter_q * ch_filter_q;

                    phase += phase_inc;
                    if phase > std::f64::consts::PI {
                        phase -= 2.0 * std::f64::consts::PI;
                    } else if phase < -std::f64::consts::PI {
                        phase += 2.0 * std::f64::consts::PI;
                    }
                }

                let ch_avg = if all_samples.is_empty() {
                    1e-12
                } else {
                    ch_power_sum / all_samples.len() as f64
                };
                let db = 10.0 * ch_avg.max(1e-12).log10();
                (db, *off)
            })
            .max_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap_or((f64::NEG_INFINITY, base_offset_hz));

        Some((m, best_db, best_off))
    } else {
        None
    }
}

fn calibrate_gain(
    reader: &mut RtlSdrReader,
    start_hz: u32,
    end_hz: u32,
    cfg: CalibrationConfig,
) -> Option<i32> {
    let candidates_tenths = [0, 90, 170, 280, 370, 420, 496];
    let span = end_hz - start_hz;
    let probe_freqs: Vec<u32> = (0..=7)
        .map(|i| start_hz + ((span as u64 * i as u64 / 7) as u32))
        .collect();

    let mut best: Option<(i32, f64)> = None;

    print!(
        "Calibrating gain (testing {} settings × {} frequencies)...",
        candidates_tenths.len(),
        probe_freqs.len()
    );
    std::io::stdout().flush().unwrap();

    for (gain_idx, gain_tenths) in candidates_tenths.iter().enumerate() {
        print!(
            "\rCalibrating gain ({}/{})...",
            gain_idx + 1,
            candidates_tenths.len()
        );
        std::io::stdout().flush().unwrap();

        let gain_tenths = *gain_tenths;
        let gain_db = f64::from(gain_tenths) / 10.0;
        if reader.set_gain(Gain::Manual(gain_db)).is_err() {
            continue;
        }

        let mut clips = Vec::<f64>::new();
        let mut powers = Vec::<f64>::new();
        for &freq in &probe_freqs {
            if let Some((m, channel_power_db, _)) = collect_rf_metrics(
                reader,
                freq,
                cfg.sample_rate,
                cfg.dwell,
                cfg.ppm_estimate,
                cfg.ppm_search,
                cfg.ppm_steps,
            ) {
                clips.push(m.clipping_ratio);
                powers.push(channel_power_db);
            }
        }

        if clips.is_empty() || powers.is_empty() {
            continue;
        }

        let avg_clip = clips.iter().sum::<f64>() / clips.len() as f64;
        let avg_power = powers.iter().sum::<f64>() / powers.len() as f64;
        let max_clip = clips.iter().copied().fold(0.0_f64, f64::max);

        // Skip clearly overloaded settings.
        if max_clip > 0.25 {
            continue;
        }

        // Prefer low clipping first, then stronger power.
        let score =
            avg_power - (500.0 * (max_clip - 0.03).max(0.0)) - (80.0 * (avg_clip - 0.01).abs());

        if best.is_none_or(|(_, best_score)| score > best_score) {
            best = Some((gain_tenths, score));
        }
    }

    println!(); // Clear the progress line
    best.map(|(gain, _)| gain)
}

fn parse_rtlsdr_device_index(source: &str) -> Result<usize, String> {
    if !source.starts_with("rtlsdr://") {
        return Err("only rtlsdr:// sources are currently supported".to_string());
    }

    let device_part = &source[9..];
    if device_part.is_empty() {
        return Ok(0);
    }
    device_part.parse::<usize>().map_err(|_| {
        format!(
            "invalid RTL-SDR device index in source '{}': expected integer",
            source
        )
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    println!("FM Scanner starting...");

    if args.start.0 > args.end.0 {
        return Err("--start must be <= --end".into());
    }
    if args.step.0 == 0 {
        return Err("--step must be > 0".into());
    }

    let device_index = parse_rtlsdr_device_index(&args.source)?;
    let dwell = Duration::from_millis(args.dwell_ms.max(50));
    let mut results = Vec::<ScanResult>::new();

    println!("Opening RTL-SDR device {}...", device_index);
    let open_start = Instant::now();
    let mut selected_gain_tenths = args.gain;
    let mut reader = open_scan_reader(
        device_index,
        args.start.0,
        args.sample_rate,
        selected_gain_tenths,
    )?;
    let open_elapsed = open_start.elapsed();
    if open_elapsed.as_millis() > 100 {
        println!("RTL-SDR opened (took {:.1}s)", open_elapsed.as_secs_f64());
    } else {
        println!("RTL-SDR opened");
    }

    if args.gain.is_none() && !args.no_calibrate_gain {
        println!("Running gain calibration...");
        let calibration_start = Instant::now();
        let calibration_dwell = Duration::from_millis(args.calibration_dwell_ms.max(60));
        if let Some(best_gain_tenths) = calibrate_gain(
            &mut reader,
            args.start.0,
            args.end.0,
            CalibrationConfig {
                sample_rate: args.sample_rate,
                dwell: calibration_dwell,
                ppm_estimate: args.ppm_estimate,
                ppm_search: args.ppm_search,
                ppm_steps: args.ppm_steps,
            },
        ) {
            let best_gain_db = f64::from(best_gain_tenths) / 10.0;
            reader.set_gain(Gain::Manual(best_gain_db))?;
            selected_gain_tenths = Some(best_gain_tenths);
            println!(
                "Selected gain {:.1} dB ({} tenths) from startup calibration (took {:.1}s)",
                best_gain_db,
                best_gain_tenths,
                calibration_start.elapsed().as_secs_f64()
            );
        } else {
            println!(
                "Gain calibration did not converge, keeping auto gain (took {:.1}s)",
                calibration_start.elapsed().as_secs_f64()
            );
        }
    }

    println!(
        "Scanning {:.3} MHz to {:.3} MHz (step {:.0} kHz, dwell {} ms) via {}",
        args.start.0 as f64 / 1e6,
        args.end.0 as f64 / 1e6,
        args.step.0 as f64 / 1e3,
        dwell.as_millis(),
        args.source
    );

    let mut printed = Vec::<bool>::new();
    let mut transient_visible = false;
    let mut last_verified_freq: Option<u32> = None;
    let rds_min_sep_hz = args.rds_min_separation_khz.max(50) * 1000;
    let mut freq = args.start.0;
    while freq <= args.end.0 {
        if let Some((m, channel_power_db, peak_offset_hz)) = collect_rf_metrics(
            &mut reader,
            freq,
            args.sample_rate,
            dwell,
            args.ppm_estimate,
            args.ppm_search,
            args.ppm_steps,
        ) {
            let score = channel_power_db - (60.0 * m.clipping_ratio);
            let row = ScanResult {
                freq_hz: freq,
                clipping_ratio: m.clipping_ratio,
                noise_floor_db: m.noise_floor_db,
                channel_power_db,
                peak_offset_hz,
                kurtosis: m.kurtosis,
                prominence_db: 0.0,
                rf_score: score,
                final_score: score,
                rds_ps: None,
                rds_groups: 0,
                rds_note: None,
            };
            results.push(row);
            printed.push(false);

            if results.len() >= 5 {
                let idx = results.len() - 5;
                // CRITICAL: Compute rf_score FIRST before picking verify frequency
                let (rf_score, prominence) = compute_rf_score_at(&results, idx);
                results[idx].rf_score = rf_score;
                results[idx].prominence_db = prominence;
                results[idx].final_score = rf_score;

                if !printed[idx] {
                    if should_verify_rds(
                        &results,
                        idx,
                        args.rds_score_threshold,
                        last_verified_freq,
                        rds_min_sep_hz,
                    ) {
                        // NOW pick verify frequency with updated rf_score
                        let verify_freq = pick_verify_freq(&results, idx);

                        if transient_visible {
                            print!("\r\x1b[2K");
                            transient_visible = false;
                        }
                        if !args.no_rds_verify {
                            drop(reader);
                            let rds_result = verify_rds_with_fallback(
                                &results,
                                idx,
                                verify_freq,
                                VerifyRdsConfig {
                                    device_index,
                                    sample_rate: args.sample_rate,
                                    timeout_secs: args.rds_verify_seconds,
                                    gain_tenths_db: selected_gain_tenths,
                                    debug: args.debug_rds_selection,
                                },
                            );
                            std::thread::sleep(Duration::from_millis(40));
                            reader = open_scan_reader(
                                device_index,
                                verify_freq,
                                args.sample_rate,
                                selected_gain_tenths,
                            )?;

                            // Always attach RDS to the trigger idx, add note if verified elsewhere
                            let entry = &mut results[idx];
                            if let Some(hit) = rds_result {
                                entry.rds_ps = Some(hit.ps);
                                entry.rds_groups = hit.groups;
                                entry.final_score = entry.rf_score
                                    + 20.0
                                    + (entry.rds_groups.min(150) as f64 * 0.05);

                                if verify_freq != entry.freq_hz {
                                    let verify_mhz = verify_freq as f32 / 1e6;
                                    entry.rds_note = Some(format!("verified @ {:.1}", verify_mhz));

                                    // Also check AF
                                    let entry_mhz = entry.freq_hz as f32 / 1e6;
                                    if af_contains(&hit.af_mhz, entry_mhz) {
                                        entry.rds_note = Some(format!(
                                            "verified @ {:.1}, AF confirms {:.1}",
                                            verify_mhz, entry_mhz
                                        ));
                                    } else if af_contains(&hit.af_mhz, verify_mhz + 0.1)
                                        || af_contains(&hit.af_mhz, verify_mhz - 0.1)
                                    {
                                        entry.rds_note = Some(format!(
                                            "verified @ {:.1}, WARN: AF includes +/-100k",
                                            verify_mhz
                                        ));
                                    }
                                }
                            }
                            last_verified_freq = Some(verify_freq);
                        }

                        let entry = &results[idx];
                        let suffix = match &entry.rds_ps {
                            Some(ps) => format!(
                                " | RDS: {} ({} groups){}",
                                ps,
                                entry.rds_groups,
                                entry
                                    .rds_note
                                    .as_ref()
                                    .map(|n| format!(" | {}", n))
                                    .unwrap_or_default()
                            ),
                            None => " | RDS: none".to_string(),
                        };
                        println!("{}{}", format_sweep_row(entry), suffix);
                        printed[idx] = true;
                    } else {
                        let entry = &results[idx];
                        print!("\r\x1b[2K{}", format_sweep_row(entry));
                        let _ = std::io::stdout().flush();
                        transient_visible = true;
                    }
                    printed[idx] = true;
                }
            }
        }

        match freq.checked_add(args.step.0) {
            Some(next) => freq = next,
            None => break,
        }
    }

    drop(reader);

    for idx in 0..results.len() {
        if printed[idx] {
            continue;
        }
        // CRITICAL: Compute rf_score FIRST before picking verify frequency
        let (rf_score, prominence) = compute_rf_score_at(&results, idx);
        results[idx].rf_score = rf_score;
        results[idx].prominence_db = prominence;
        results[idx].final_score = rf_score;

        if should_verify_rds(
            &results,
            idx,
            args.rds_score_threshold,
            last_verified_freq,
            rds_min_sep_hz,
        ) {
            // NOW pick verify frequency with updated rf_score
            let verify_freq = pick_verify_freq(&results, idx);

            if transient_visible {
                print!("\r\x1b[2K");
                transient_visible = false;
            }
            if !args.no_rds_verify {
                let rds_result = verify_rds_with_fallback(
                    &results,
                    idx,
                    verify_freq,
                    VerifyRdsConfig {
                        device_index,
                        sample_rate: args.sample_rate,
                        timeout_secs: args.rds_verify_seconds,
                        gain_tenths_db: selected_gain_tenths,
                        debug: args.debug_rds_selection,
                    },
                );

                // Always attach RDS to the trigger idx, add note if verified elsewhere
                let entry = &mut results[idx];
                if let Some(hit) = rds_result {
                    entry.rds_ps = Some(hit.ps);
                    entry.rds_groups = hit.groups;
                    entry.final_score =
                        entry.rf_score + 20.0 + (entry.rds_groups.min(150) as f64 * 0.05);

                    if verify_freq != entry.freq_hz {
                        let verify_mhz = verify_freq as f32 / 1e6;
                        entry.rds_note = Some(format!("verified @ {:.1}", verify_mhz));

                        // Also check AF
                        let entry_mhz = entry.freq_hz as f32 / 1e6;
                        if af_contains(&hit.af_mhz, entry_mhz) {
                            entry.rds_note = Some(format!(
                                "verified @ {:.1}, AF confirms {:.1}",
                                verify_mhz, entry_mhz
                            ));
                        } else if af_contains(&hit.af_mhz, verify_mhz + 0.1)
                            || af_contains(&hit.af_mhz, verify_mhz - 0.1)
                        {
                            entry.rds_note = Some(format!(
                                "verified @ {:.1}, WARN: AF includes +/-100k",
                                verify_mhz
                            ));
                        }
                    }
                }
                last_verified_freq = Some(verify_freq);
            }

            let entry = &results[idx];
            let suffix = match &entry.rds_ps {
                Some(ps) => format!(
                    " | RDS: {} ({} groups){}",
                    ps,
                    entry.rds_groups,
                    entry
                        .rds_note
                        .as_ref()
                        .map(|n| format!(" | {}", n))
                        .unwrap_or_default()
                ),
                None => " | RDS: none".to_string(),
            };
            println!("{}{}", format_sweep_row(entry), suffix);
            printed[idx] = true;
        } else {
            let entry = &results[idx];
            print!("\r\x1b[2K{}", format_sweep_row(entry));
            let _ = std::io::stdout().flush();
            transient_visible = true;
        }
        printed[idx] = true;
    }

    if transient_visible {
        print!("\r\x1b[2K");
        let _ = std::io::stdout().flush();
    }

    if results.is_empty() {
        println!("No IQ samples captured during scan.");
        return Ok(());
    }

    Ok(())
}
