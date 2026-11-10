//! Command-line entry point for the pure-Rust NRSC-5 receiver.

use std::{
    io::Write,
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};

use clap::Parser;
use crossbeam_channel as channel;
use desperado::IqSource;
use hdradio::{
    FrontEnd, FrontEndReport, InputConfig, ReceiverConfig,
    audio::AudioTransport,
    audio::HdcCoreAudio,
    metadata::aas::AasDecoder,
    metadata::model::{
        FrameSyncReport, FrontEndRfReport, NowPlaying, PhyReport, Program, SisAudioServiceReport,
        SisDataServiceReport, SisReport,
    },
    metadata::sis::SisDecoder,
    ofdm::OfdmReceiver,
    ofdm::carriers::PrimaryMainExtractor,
    ofdm::frame_sync::FrameSynchronizer,
    p1::P1LockMonitor,
    protocol::config::{NRSC5_FM_SAMPLE_RATE_HZ, parse_frequency_hz},
    protocol::pids::PidsDecoder,
};
use tinyaudio::prelude::*;
use tracing::{info, warn};

/// Pure-Rust FM NRSC-5 / HD Radio receiver.
#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Args {
    /// I/Q file source. GQRX filenames provide format, rate, and capture centre.
    source: PathBuf,

    /// FM station frequency, e.g. 106.7M. Defaults to capture centre.
    #[arg(short = 'f', long, value_parser = parse_frequency_hz)]
    frequency: Option<u32>,

    /// Override I/Q sample rate in samples/s.
    #[arg(short = 's', long)]
    sample_rate: Option<u32>,

    /// Override I/Q format (cu8, cs8, cs16, cf32).
    #[arg(long)]
    format: Option<String>,

    /// HD program number: 0 for HD1, 1 for HD2, and so on.
    #[arg(short, long, default_value_t = 0)]
    program: u8,

    /// Stop after this much source time; useful for deterministic file tests.
    #[arg(long)]
    max_seconds: Option<f64>,

    /// Write pre-FEC primary-main soft bits for local oracle-stage comparison.
    #[arg(long, hide = true)]
    dump_primary_main: Option<PathBuf>,

    /// Write raw FFT symbols as interleaved little-endian f32 I/Q.
    #[arg(long, hide = true)]
    dump_fft: Option<PathBuf>,

    /// Write CRC-valid normalized 80-bit PIDS frames for local stage checks.
    #[arg(long, hide = true)]
    dump_pids: Option<PathBuf>,

    /// Dump PCI-valid P1 frame bytes for stage comparison.
    #[arg(long, hide = true)]
    dump_p1: Option<PathBuf>,

    /// Dump length-delimited CRC-valid HDC PDUs for stage comparison.
    #[arg(long, hide = true)]
    dump_hdc: Option<PathBuf>,

    /// Render the selected program's pure-Rust HDC audio to a 44.1 kHz stereo
    /// WAV file. A small number of unavailable SBR frames fall back to core PCM.
    #[arg(long)]
    wav: Option<PathBuf>,

    /// Disable 44.1 kHz stereo soundcard playback (useful for headless runs).
    #[arg(long)]
    no_audio: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_target(false)
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();
    let source = args.source.to_string_lossy().into_owned();
    let config = ReceiverConfig::resolve(
        source,
        args.frequency,
        args.sample_rate,
        args.format.as_deref(),
        args.program,
    )?;
    info!(
        capture_center_hz = config.input.center_frequency_hz,
        station_frequency_hz = config.station_frequency_hz,
        station_offset_hz = config.station_offset_hz(),
        input_sample_rate_hz = config.input.sample_rate_hz,
        format = %config.input.format,
        "configured NRSC-5 front end"
    );

    let mut source = open_file_source(&config)?;
    let (audio_tx, audio_rx) = channel::bounded::<f32>(44_100 * 2 * 4);
    let _audio_device = if args.no_audio {
        info!("audio output disabled (--no-audio)");
        None
    } else {
        let output = OutputDeviceParameters {
            channels_count: 2,
            sample_rate: 44_100,
            channel_sample_count: 1024,
        };
        Some(run_output_device(output, move |samples| {
            for sample in samples {
                *sample = audio_rx.try_recv().unwrap_or(0.0);
            }
        })?)
    };
    let mut front_end = FrontEnd::new(&config)?;
    let mut phy = OfdmReceiver::new();
    let mut frame_synchronizer = FrameSynchronizer::new();
    let mut primary_main_extractor = PrimaryMainExtractor::new();
    let mut coarse_aligned = false;
    let mut coarse_wait = 0u8;
    let mut current_service_mode = 1u8;
    let mut next_block_counter = None;
    let mut pids_decoder = PidsDecoder::default();
    let mut p1_monitor = P1LockMonitor::default();
    let mut aas_decoder = AasDecoder::default();
    let mut audio_transport = AudioTransport::default();
    let mut hdc_core_audio = HdcCoreAudio::new(args.program);
    let mut hdc_pdus = 0u64;
    let mut sis_decoder = SisDecoder::default();
    let mut valid_pids_frames = 0u64;
    let mut frame_sync: Option<hdradio::ofdm::frame_sync::FrameSync> = None;
    let mut primary_main_dump = args
        .dump_primary_main
        .as_ref()
        .map(std::fs::File::create)
        .transpose()?;
    let mut fft_dump = args
        .dump_fft
        .as_ref()
        .map(std::fs::File::create)
        .transpose()?;
    let mut pids_dump = args
        .dump_pids
        .as_ref()
        .map(std::fs::File::create)
        .transpose()?;
    let mut p1_dump = args
        .dump_p1
        .as_ref()
        .map(std::fs::File::create)
        .transpose()?;
    let mut hdc_dump = args
        .dump_hdc
        .as_ref()
        .map(std::fs::File::create)
        .transpose()?;
    let max_input_samples = args
        .max_seconds
        .map(|seconds| (seconds.max(0.0) * config.input.sample_rate_hz as f64) as u64);

    for chunk in &mut source {
        let chunk = chunk?;
        if max_input_samples.is_some_and(|limit| front_end.input_samples() >= limit) {
            break;
        }
        let remaining = max_input_samples
            .map(|limit| limit.saturating_sub(front_end.input_samples()) as usize)
            .unwrap_or(chunk.len());
        if remaining == 0 {
            break;
        }
        let station_samples = front_end.process(&chunk[..chunk.len().min(remaining)]);
        for block in phy.process(&station_samples) {
            if let Some(file) = &mut fft_dump {
                for symbol in &block.symbols {
                    for sample in symbol {
                        file.write_all(&sample.re.to_le_bytes())?;
                        file.write_all(&sample.im.to_le_bytes())?;
                    }
                }
            }
            if next_block_counter.is_none() && coarse_wait > 0 {
                let previous = frame_sync.expect("coarse holdoff requires prior synchronization");
                primary_main_extractor.warm_nominal_references(
                    &block,
                    previous.coarse_frequency_bins,
                    current_service_mode,
                );
                coarse_wait -= 1;
                continue;
            }
            let (sync, transition) = if let Some(counter) = next_block_counter {
                let previous = frame_sync.expect("counter requires coarse synchronization");
                let sync = hdradio::ofdm::frame_sync::FrameSync {
                    coarse_frequency_bins: previous.coarse_frequency_bins,
                    block_offset: 0,
                    primary_service_mode: previous.primary_service_mode,
                    block_counter: counter,
                    reference_matches: 0,
                };
                next_block_counter = Some((counter + 1) % 16);
                (sync, false)
            } else {
                let Some(sync) = frame_synchronizer.synchronize(&block) else {
                    continue;
                };
                tracing::info!(
                    ?sync,
                    timing = block.timing_offset_samples,
                    cfo_hz = block.frequency_offset_hz,
                    "coarse frame synchronization"
                );
                if !coarse_aligned {
                    primary_main_extractor.warm_references(&block, sync, current_service_mode);
                    phy.align_logical_block(sync.block_offset);
                    coarse_aligned = true;
                    frame_sync = Some(sync);
                    continue;
                }
                phy.enter_fine_sync();
                pids_decoder = PidsDecoder::default();
                p1_monitor = P1LockMonitor::default();
                aas_decoder.reset_stream();
                next_block_counter = Some((sync.block_counter + 1) % 16);
                (sync, true)
            };

            let block_service_mode = current_service_mode;
            let equalized = if transition {
                primary_main_extractor.equalize_transition(&block, sync, block_service_mode)
            } else {
                primary_main_extractor.equalize(&block, sync)
            };
            let (sampling_error, common_angle) = primary_main_extractor.tracking_feedback();
            tracing::debug!(sampling_error, common_angle, "reference tracking feedback");
            phy.adjust_tracking(sampling_error, common_angle);
            let primary_main = primary_main_extractor.extract(&equalized, block_service_mode);
            if let Some(file) = &mut primary_main_dump {
                file.write_all(&[sync.block_counter])?;
                file.write_all(
                    &primary_main
                        .bits
                        .iter()
                        .map(|bit| *bit as u8)
                        .collect::<Vec<_>>(),
                )?;
            }
            let p1_lock = p1_monitor.push(sync.block_counter, &primary_main);
            if let Some(frame) = p1_monitor.last_frame() {
                aas_decoder.push_p1(frame);
                for pdu in audio_transport.push_p1(frame) {
                    hdc_pdus += 1;
                    hdc_core_audio.push(&pdu);
                    for sample in hdc_core_audio.take_pending_pcm() {
                        if !args.no_audio && audio_tx.send(sample as f32 / i16::MAX as f32).is_err()
                        {
                            break;
                        }
                    }
                    if let Some(file) = &mut hdc_dump {
                        file.write_all(&[pdu.program, pdu.stream_id, pdu.sequence])?;
                        file.write_all(&(pdu.payload.len() as u32).to_le_bytes())?;
                        file.write_all(&pdu.payload)?;
                    }
                }
                if let Some(file) = &mut p1_dump {
                    file.write_all(frame)?;
                }
            }
            let pids_frame = pids_decoder.push(sync.block_counter, primary_main);
            if let (Some(file), Some(bits)) = (&mut pids_dump, pids_decoder.last_decoded()) {
                file.write_all(&[sync.block_counter])?;
                file.write_all(bits)?;
            }
            if let Some(frame) = pids_frame {
                valid_pids_frames += 1;
                let update = sis_decoder.push(&frame);
                if update != Default::default() {
                    tracing::info!(?update, "decoded SIS update");
                }
            }
            if p1_lock == Some(false) {
                tracing::info!("P1 program-control indicator lost; restarting acquisition");
                phy.lose_fine_sync();
                frame_synchronizer = FrameSynchronizer::default();
                coarse_aligned = false;
                coarse_wait = 8;
                next_block_counter = None;
                frame_sync = Some(sync);
                continue;
            }
            current_service_mode = sync.primary_service_mode;
            frame_sync = Some(sync);
        }
    }

    if hdc_core_audio.decoded_frames() == 0 {
        warn!(
            program = args.program,
            "no decodable HDC audio frames were present for the selected program"
        );
    }
    if let Some(path) = &args.wav {
        hdc_core_audio.write_wav(path)?;
    }
    if !args.no_audio {
        let deadline = Instant::now() + Duration::from_secs(10);
        while !audio_tx.is_empty() && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(20));
        }
    }
    let sis = if sis_decoder.country().is_some()
        || sis_decoder.fcc_facility_id().is_some()
        || sis_decoder.short_name().is_some()
        || sis_decoder.audio_services().next().is_some()
        || !sis_decoder.data_services().is_empty()
    {
        Some(SisReport {
            country: sis_decoder.country().map(str::to_owned),
            fcc_facility_id: sis_decoder.fcc_facility_id(),
            station_name: sis_decoder.short_name().map(str::to_owned),
            audio_services: sis_decoder
                .audio_services()
                .map(|service| SisAudioServiceReport {
                    program: service.program,
                    access: service.access,
                    service_type: service.service_type,
                    sound_experience: service.sound_experience,
                })
                .collect(),
            data_services: sis_decoder
                .data_services()
                .iter()
                .map(|service| SisDataServiceReport {
                    access: service.access,
                    service_type: service.service_type,
                    mime_type: service.mime_type,
                })
                .collect(),
        })
    } else {
        None
    };
    let programs = sis_decoder
        .audio_services()
        .map(|service| Program {
            index: service.program,
            name: aas_decoder
                .program_names()
                .get(&service.program)
                .cloned()
                .unwrap_or_else(|| format!("HD{}", service.program + 1)),
            program_type: format!("type-{}", service.service_type),
            codec: "hdc".to_owned(),
        })
        .collect();
    let now_playing = aas_decoder.metadata().get(&0).and_then(|metadata| {
        Some(NowPlaying {
            program: 0,
            title: metadata.title.clone()?,
            artist: metadata.artist.clone()?,
            genre: metadata.genre.clone().unwrap_or_default(),
        })
    });
    let report = FrontEndReport {
        schema_version: 1,
        rf: FrontEndRfReport {
            mode: "fm-hybrid",
            station_frequency_hz: config.station_frequency_hz,
            capture_center_hz: config.input.center_frequency_hz,
            input_sample_rate_hz: config.input.sample_rate_hz,
            ofdm_sample_rate_hz: NRSC5_FM_SAMPLE_RATE_HZ,
            station_offset_hz: config.station_offset_hz(),
        },
        input_samples: front_end.input_samples(),
        output_samples: front_end.output_samples(),
        phy: PhyReport {
            locked_blocks: phy.metrics().locked_blocks,
            frequency_offset_hz: phy.metrics().frequency_offset_hz,
            timing_metric: phy.metrics().timing_metric,
            valid_pids_frames,
            hdc_pdus,
            hdc_core_frames: hdc_core_audio.decoded_frames(),
            hdc_core_rejected_frames: hdc_core_audio.rejected_frames(),
        },
        frame_sync: frame_sync.map(|sync| FrameSyncReport {
            coarse_frequency_bins: sync.coarse_frequency_bins,
            block_offset: sync.block_offset,
            primary_service_mode: sync.primary_service_mode,
            block_counter: sync.block_counter,
            reference_matches: sync.reference_matches,
        }),
        sis,
        programs,
        now_playing,
    };
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn open_file_source(config: &ReceiverConfig) -> desperado::Result<IqSource> {
    IqSource::from_file(
        &config.source,
        config.input.center_frequency_hz,
        config.input.sample_rate_hz,
        32_768,
        config.input.format,
    )
}

#[allow(dead_code)]
fn _input_config(config: &ReceiverConfig) -> &InputConfig {
    &config.input
}
