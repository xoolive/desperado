//! Perceptual Noise Substitution — ISO/IEC 14496-3 §4.6.13.

use super::error::{Error, Result};
use super::ics::IcsInfo;
use super::section::{SectionData, is_noise};
use super::sf::ScaleFactors;
use super::swb::{long_offsets, short_offsets};

/// LCG suggested by §4.6.13.3 ("one multiply-accumulate per random value").
///
/// ISO leaves the generator non-normative. Seed and int-as-float mapping
/// match FFmpeg libavcodec (`ac->random_state = 0x1f2e3d4c`).
#[derive(Clone, Copy, Debug)]
pub struct Lcg {
    state: u32,
}

impl Lcg {
    /// lavc `ff_aac_decode_init` seed.
    #[must_use]
    pub fn new() -> Self {
        Self { state: 0x1f2e_3d4c }
    }

    fn next_i32(&mut self) -> i32 {
        self.state = self
            .state
            .wrapping_mul(1_664_525)
            .wrapping_add(1_013_904_223);
        self.state as i32
    }
}

impl Default for Lcg {
    fn default() -> Self {
        Self::new()
    }
}

/// Pair-channel PNS correlation inputs (§4.6.13.3).
pub struct PairPns<'a> {
    /// `ms_used[g][sfb]` (true when the band is correlated).
    pub ms_used: Option<&'a [Vec<bool>]>,
    /// The other channel's `sfb_cb`.
    pub other_cb: Option<&'a [Vec<u8>]>,
    /// Shared random vector for a correlated band (pre-normalise).
    pub shared: &'a mut Option<Vec<f32>>,
}

/// Fill NOISE_HCB bands. When the pair is noise on both sides and `ms_used`,
/// reuse `shared` so the pair is correlated (§4.6.13.3).
pub fn apply(
    spec: &mut [f32],
    ics: &IcsInfo,
    sections: &SectionData,
    sf: &ScaleFactors,
    fs_index: u8,
    rng: &mut Lcg,
    mut pair: Option<&mut PairPns<'_>>,
) -> Result<()> {
    let win_len = ics.window_len();
    let offsets = if ics.window_sequence.is_eight_short() {
        short_offsets(fs_index)?
    } else {
        long_offsets(fs_index)?
    };
    let mut wbase = 0usize;
    for g in 0..ics.num_window_groups as usize {
        let glen = ics.window_group_length[g] as usize;
        let cbs = sections.sfb_cb.get(g).ok_or(Error::SpectrumInvalid)?;
        for sfb in 0..ics.max_sfb as usize {
            let cb = *cbs.get(sfb).unwrap_or(&0);
            if !is_noise(cb) {
                continue;
            }
            let start = *offsets.get(sfb).ok_or(Error::SpectrumInvalid)? as usize;
            let end = *offsets.get(sfb + 1).ok_or(Error::SpectrumInvalid)? as usize;
            let width = end.saturating_sub(start);
            if width == 0 {
                continue;
            }
            let nrg = *sf.noise_nrg.get(g).and_then(|v| v.get(sfb)).unwrap_or(&0);
            // ISO §4.6.13 energy. lavc stores `-2^(sfo/4)` because its IMDCT
            // polarity is inverted; ours is the ISO sum, so the scale is positive
            // and PCM matches lavc (PNS-only corr was −1 with the extra minus).
            let target = (0.25 * nrg as f32).exp2();
            let correlated = pair.as_ref().is_some_and(|p| {
                p.ms_used
                    .and_then(|ms| ms.get(g).and_then(|v| v.get(sfb)).copied())
                    .unwrap_or(false)
                    && p.other_cb
                        .and_then(|o| o.get(g).and_then(|v| v.get(sfb)).copied())
                        .map(is_noise)
                        .unwrap_or(false)
            });
            for b in 0..glen {
                let w = wbase + b;
                let vec = if correlated {
                    if let Some(p) = pair.as_mut() {
                        if let Some(s) = p.shared.as_ref() {
                            s.clone()
                        } else {
                            let v = rand_vec(rng, width);
                            *p.shared = Some(v.clone());
                            v
                        }
                    } else {
                        rand_vec(rng, width)
                    }
                } else {
                    rand_vec(rng, width)
                };
                let mut energy = 0.0f32;
                for &x in &vec {
                    energy += x * x;
                }
                let scale = if energy > 0.0 {
                    target / energy.sqrt()
                } else {
                    0.0
                };
                for (i, &x) in vec.iter().enumerate() {
                    spec[w * win_len + start + i] = x * scale;
                }
            }
            if !correlated && let Some(p) = pair.as_mut() {
                *p.shared = None;
            }
        }
        wbase += glen;
    }
    Ok(())
}

fn rand_vec(rng: &mut Lcg, n: usize) -> Vec<f32> {
    // lavc float decoder: `cfo[k] = ac->random_state` (int → float).
    (0..n).map(|_| rng.next_i32() as f32).collect()
}
