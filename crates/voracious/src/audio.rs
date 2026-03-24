//! Audio output for Morse ident and signal playback.
//!
//! This module provides real-time audio output via the system soundcard.
//! Uses tinyaudio with a bounded channel for thread-safe sample delivery.

use crossbeam_channel::{Sender, TrySendError, bounded};
use tinyaudio::prelude::*;

/// Audio output sample rate in Hz
pub const AUDIO_RATE: usize = 48_000;

/// Audio output device wrapper with sample sender
pub struct AudioOutput {
    /// Sender for audio samples (f32 in range -1.0..1.0)
    pub tx: Sender<f32>,
    /// Device guard (keeps audio device alive while held)
    _device: Option<Box<dyn std::any::Any>>,
}

impl AudioOutput {
    /// Create a new audio output device with the given buffer size.
    ///
    /// # Arguments
    ///
    /// - `buffer_size`: Number of samples to buffer (e.g., 48000 for 1 second at 48 kHz)
    ///
    /// # Returns
    ///
    /// An `AudioOutput` that can send samples via `tx.send(sample)`.
    /// If audio cannot be initialized (e.g., no soundcard), returns None.
    pub fn new(buffer_size: usize) -> Option<Self> {
        let (tx, rx) = bounded::<f32>(buffer_size);

        let device = {
            let config = OutputDeviceParameters {
                channels_count: 1, // mono for Morse
                sample_rate: AUDIO_RATE,
                channel_sample_count: 1024,
            };

            match run_output_device(config, move |output| {
                for sample in output.iter_mut() {
                    // Non-blocking receive: if no sample available, output silence
                    *sample = rx.try_recv().unwrap_or(0.0);
                }
            }) {
                Ok(device) => Some(Box::new(device) as Box<dyn std::any::Any>),
                Err(_) => None, // Audio initialization failed (e.g., no soundcard)
            }
        };

        Some(AudioOutput {
            tx,
            _device: device,
        })
    }

    /// Send a single sample to the audio output.
    ///
    /// Sample should be in range [-1.0, 1.0]. Values outside this range will be clipped.
    /// Returns Err if the buffer is full (non-blocking).
    pub fn send(&self, sample: f32) -> Result<(), TrySendError<f32>> {
        // Clamp sample to prevent clipping distortion
        let clamped = sample.clamp(-1.0, 1.0);
        self.tx.try_send(clamped)
    }

    /// Send multiple samples to the audio output.
    ///
    /// Returns the number of samples successfully sent.
    pub fn send_batch(&self, samples: &[f32]) -> usize {
        let mut sent = 0;
        for &sample in samples {
            if self.send(sample).is_ok() {
                sent += 1;
            } else {
                // Buffer full, stop sending for now
                break;
            }
        }
        sent
    }

    /// Get the current buffer fill level (number of samples waiting in queue)
    pub fn buffer_fill(&self) -> usize {
        self.tx.len()
    }

    /// Check if the output buffer is critically full (>80% capacity)
    pub fn is_buffer_full(&self) -> bool {
        let capacity = self.tx.capacity().unwrap_or(1);
        self.buffer_fill() > (capacity * 4 / 5)
    }
}
