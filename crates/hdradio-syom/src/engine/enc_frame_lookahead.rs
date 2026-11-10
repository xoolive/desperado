//! One-frame attack lookahead for [`LcEncoder`] — split from
//! `enc_frame.rs` for the line cap. A child module of `enc_frame`, so the
//! `impl` below keeps using the encoder's private fields (including the
//! private `detect` / `encode_with_attack` / `check_shape` methods); no
//! new abstraction boundary.
//!
//! Semantics (see the `enc_frame` module docs for the rationale):
//! `push_frame` runs the attack detectors on the NEW frame, then encodes
//! the previously HELD frame with `held.attack || new.attack` — an attack
//! anywhere in frame N+1 therefore makes frame N a LongStart (its
//! start-window slope covers the pre-attack tail) and frame N+1 an
//! EightShort, so early-in-frame onsets are coded entirely on short
//! windows. The `held.attack` half of the OR is unreachable mid-stream
//! (attack(N) already forced frame N short via N−1's LongStart); it keeps
//! the stream-start frame and the final flushed frame on the causal
//! decision. `flush` encodes the held frame with its own attack flag —
//! there is no next frame to see. The held planes are a private copy: the
//! caller's buffers are reused between pushes.

use super::LcEncoder;
use crate::engine::error::Result;
use crate::engine::swb::LONG_WINDOW_LEN;

/// A frame held for one lookahead step: a private copy of the planes plus
/// the attack flag the detectors raised on them.
pub(super) struct HeldFrame {
    pub pcm: [[f32; LONG_WINDOW_LEN]; 2],
    pub attack: bool,
}

impl HeldFrame {
    /// Borrow the planes in the shape `encode_with_attack` expects.
    fn planes(&self, channels: usize) -> Vec<&[f32]> {
        self.pcm[..channels].iter().map(|p| &p[..]).collect()
    }
}

impl LcEncoder {
    /// Lookahead path: run the attack detectors on `pcm`, encode the
    /// previously held frame (if any) with the OR of both frames' attack
    /// decisions, and hold `pcm` for the next push / [`Self::flush`].
    /// Returns `None` for the very first frame — one frame of latency.
    pub fn push_frame(&mut self, pcm: &[&[f32]]) -> Result<Option<Vec<u8>>> {
        self.check_shape(pcm)?;
        let attack = self.detect(pcm);
        let out = match self.held.take() {
            Some(held) => {
                Some(self.encode_with_attack(&held.planes(self.channels), held.attack || attack)?)
            }
            None => None,
        };
        let mut frame = Box::new(HeldFrame {
            pcm: [[0.0; LONG_WINDOW_LEN]; 2],
            attack,
        });
        for (dst, src) in frame.pcm.iter_mut().zip(pcm.iter()).take(self.channels) {
            dst.copy_from_slice(src);
        }
        self.held = Some(frame);
        Ok(out)
    }

    /// Lookahead path, end of input: encode the held frame with its own
    /// (causal) attack decision — there is no next frame to see. `None`
    /// when nothing is held (every pushed frame already emitted).
    pub fn flush(&mut self) -> Result<Option<Vec<u8>>> {
        let Some(held) = self.held.take() else {
            return Ok(None);
        };
        Ok(Some(self.encode_with_attack(
            &held.planes(self.channels),
            held.attack,
        )?))
    }
}
