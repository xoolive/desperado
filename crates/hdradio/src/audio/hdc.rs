//! Pure-Rust NRSC-5 HDC AAC-core decoding and WAV rendering.

use std::{collections::HashMap, io::Write, path::Path};

use syom::HdcCoreDecoder;

use crate::audio::transport::HdcPdu;

const OUTPUT_SAMPLE_RATE_HZ: u32 = 44_100;
const SAMPLES_PER_FRAME: usize = 2_048;

/// HDC audio decoder for one selected HD program.
///
/// The decoder renders HDC AAC-LC plus its SBR enhancement at 44.1 kHz.
/// Frames whose SBR side data is unavailable are duplicated from the 22.05
/// kHz core so the WAV timeline remains continuous.
#[derive(Debug)]
pub struct HdcCoreAudio {
    program: u8,
    decoders: HashMap<u8, HdcCoreDecoder>,
    pcm: Vec<i16>,
    pending_pcm: Vec<i16>,
    decoded_frames: u64,
    rejected_frames: u64,
}

impl HdcCoreAudio {
    /// Construct a decoder for `program` (zero-based HD program index).
    pub fn new(program: u8) -> Self {
        Self {
            program,
            decoders: HashMap::new(),
            pcm: Vec::new(),
            pending_pcm: Vec::new(),
            decoded_frames: 0,
            rejected_frames: 0,
        }
    }

    /// Decode one complete HDC PDU if it belongs to the selected main stream.
    pub fn push(&mut self, pdu: &HdcPdu) {
        if pdu.program != self.program || pdu.stream_id != 0 {
            return;
        }
        let decoder = self.decoders.entry(pdu.program).or_default();
        match decoder.decode(&pdu.payload) {
            Ok(frame) => {
                let left = frame.channels.first().map(Vec::as_slice).unwrap_or(&[]);
                let right = frame.channels.get(1).map(Vec::as_slice).unwrap_or(left);
                let upsample_core = frame.sample_rate == OUTPUT_SAMPLE_RATE_HZ / 2;
                let mut pcm = Vec::with_capacity(left.len() * if upsample_core { 4 } else { 2 });
                for index in 0..left.len().min(right.len()) {
                    let left = float_to_i16(left[index]);
                    let right = float_to_i16(right[index]);
                    pcm.extend([left, right]);
                    if upsample_core {
                        pcm.extend([left, right]);
                    }
                }
                self.pcm.extend_from_slice(&pcm);
                self.pending_pcm.extend(pcm);
                self.decoded_frames += 1;
            }
            Err(_) => {
                self.pcm.resize(self.pcm.len() + SAMPLES_PER_FRAME * 2, 0);
                self.pending_pcm
                    .resize(self.pending_pcm.len() + SAMPLES_PER_FRAME * 2, 0);
                self.rejected_frames += 1;
            }
        }
    }

    /// Drain interleaved stereo PCM produced by newly decoded frames.
    ///
    /// This is intended for real-time soundcard output; [`Self::write_wav`]
    /// retains the full capture independently.
    pub fn take_pending_pcm(&mut self) -> Vec<i16> {
        std::mem::take(&mut self.pending_pcm)
    }

    /// Number of decoded HDC core frames.
    pub fn decoded_frames(&self) -> u64 {
        self.decoded_frames
    }

    /// Frames rendered as silence because they are structurally invalid.
    pub fn rejected_frames(&self) -> u64 {
        self.rejected_frames
    }

    /// Write interleaved stereo 16-bit PCM to a standard WAV file.
    pub fn write_wav(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let data_bytes = u32::try_from(self.pcm.len() * std::mem::size_of::<i16>())
            .expect("capture PCM must fit RIFF WAV length");
        let mut file = std::fs::File::create(path)?;
        file.write_all(b"RIFF")?;
        file.write_all(&(36 + data_bytes).to_le_bytes())?;
        file.write_all(b"WAVEfmt ")?;
        file.write_all(&16u32.to_le_bytes())?;
        file.write_all(&1u16.to_le_bytes())?;
        file.write_all(&2u16.to_le_bytes())?;
        file.write_all(&OUTPUT_SAMPLE_RATE_HZ.to_le_bytes())?;
        file.write_all(&(OUTPUT_SAMPLE_RATE_HZ * 4).to_le_bytes())?;
        file.write_all(&4u16.to_le_bytes())?;
        file.write_all(&16u16.to_le_bytes())?;
        file.write_all(b"data")?;
        file.write_all(&data_bytes.to_le_bytes())?;
        for sample in &self.pcm {
            file.write_all(&sample.to_le_bytes())?;
        }
        Ok(())
    }
}

fn float_to_i16(sample: f32) -> i16 {
    (sample.clamp(-1.0, 1.0) * i16::MAX as f32).round() as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_a_standard_stereo_wav_header() {
        let decoder = HdcCoreAudio::new(0);
        let path = std::env::temp_dir().join("hdradio-hdc-core-test.wav");
        decoder.write_wav(&path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(&bytes[..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        assert_eq!(
            u32::from_le_bytes(bytes[24..28].try_into().unwrap()),
            44_100
        );
        std::fs::remove_file(path).unwrap();
    }
}
