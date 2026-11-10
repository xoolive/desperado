//! Channel element → output plane mapping.
//!
//! Two sources of truth, PCE winning when present:
//!
//! - `program_config_element()` (ISO/IEC 14496-3 §4.4.2.4): explicit element
//!   tags in front / side / back / LFE lists. Output planes follow PCE
//!   declaration order — front, then side, then back (a CPE emits L then R),
//!   then LFEs.
//! - ADTS/ASC `channel_configuration` (Table 4.1): default element→channel
//!   assignment. Output planes follow the lavc default layouts, proven by the
//!   committed goldens (`src/decode_mc_tests.rs`):
//!   cfg 3 = FL FR FC, cfg 4 = FL FR FC BC, cfg 5 = FL FR FC BL BR,
//!   cfg 6 = FL FR FC LFE BL BR.
//!
//! Speech mono (one rule): arithmetic mean of the decoded non-LFE planes;
//! LFE planes are mixed only when nothing else exists. For stereo this
//! reduces to the historical `0.5·(L+R)`.

use super::bits::BitReader;
use super::error::Result;

/// Decoded syntactic element kind relevant to channel mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ElemKind {
    Sce,
    Cpe,
    Lfe,
}

/// One decoded element; `plane` is its first plane in decode (element) order.
/// A CPE spans `plane` and `plane + 1`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Element {
    pub kind: ElemKind,
    pub tag: u8,
    pub plane: usize,
}

impl Element {
    fn planes(&self) -> usize {
        match self.kind {
            ElemKind::Cpe => 2,
            ElemKind::Sce | ElemKind::Lfe => 1,
        }
    }
}

/// Parsed `program_config_element()` channel lists, in declaration order.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct PceChannelMap {
    /// `(is_cpe, element_instance_tag)`.
    pub front: Vec<(bool, u8)>,
    pub side: Vec<(bool, u8)>,
    pub back: Vec<(bool, u8)>,
    pub lfe: Vec<u8>,
}

/// `program_config_element()` — parsed and applied to channel ordering.
pub(crate) fn parse_pce(br: &mut BitReader<'_>) -> Result<PceChannelMap> {
    let _tag = br.read(4)?;
    let _object_type = br.read(2)?;
    let _sf_index = br.read(4)?;
    let num_front = br.read(4)? as usize;
    let num_side = br.read(4)? as usize;
    let num_back = br.read(4)? as usize;
    let num_lfe = br.read(2)? as usize;
    let num_assoc = br.read(3)? as usize;
    let num_cc = br.read(4)? as usize;
    if br.read_bit()? {
        let _mono_mixdown = br.read(4)?;
    }
    if br.read_bit()? {
        let _stereo_mixdown = br.read(4)?;
    }
    if br.read_bit()? {
        let _matrix_mixdown_idx = br.read(3)?;
        let _pseudo_surround = br.read_bit()?;
    }
    let front = read_element_list(br, num_front)?;
    let side = read_element_list(br, num_side)?;
    let back = read_element_list(br, num_back)?;
    let mut lfe = Vec::with_capacity(num_lfe);
    for _ in 0..num_lfe {
        lfe.push(br.read(4)? as u8);
    }
    for _ in 0..num_assoc {
        let _assoc_data_tag = br.read(4)?;
    }
    for _ in 0..num_cc {
        let _is_ind_sw = br.read_bit()?;
        let _cc_tag = br.read(4)?;
    }
    br.byte_align()?;
    let comment_bytes = br.read(8)?;
    br.skip(comment_bytes.saturating_mul(8))?;
    Ok(PceChannelMap {
        front,
        side,
        back,
        lfe,
    })
}

fn read_element_list(br: &mut BitReader<'_>, n: usize) -> Result<Vec<(bool, u8)>> {
    let mut list = Vec::with_capacity(n);
    for _ in 0..n {
        let is_cpe = br.read_bit()?;
        let tag = br.read(4)? as u8;
        list.push((is_cpe, tag));
    }
    Ok(list)
}

/// One output plane: which decode-order plane feeds it (`None` → silence)
/// and whether it is LFE (excluded from the speech mono mix).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PlaneMap {
    pub src: Option<usize>,
    pub lfe: bool,
}

/// Map decode-order element planes to output planes (see module docs).
/// Elements the config/PCE does not expect keep their audio, appended in
/// decode order; expected elements missing from the frame map to silence.
pub(crate) fn map_planes(
    elems: &[Element],
    pce: Option<&PceChannelMap>,
    channel_configuration: u8,
) -> Vec<PlaneMap> {
    let mut used = vec![false; elems.len()];
    let mut out = match pce {
        Some(p) => pce_slots(p, elems, &mut used),
        None => default_slots(channel_configuration, elems, &mut used),
    };
    for (i, e) in elems.iter().enumerate() {
        if !used[i] {
            for p in 0..e.planes() {
                out.push(PlaneMap {
                    src: Some(e.plane + p),
                    lfe: e.kind == ElemKind::Lfe,
                });
            }
        }
    }
    out
}

/// `(kind, occurrence among elements of that kind, channel of pair)` per
/// output plane, in lavc layout order.
fn default_slots(config: u8, elems: &[Element], used: &mut [bool]) -> Vec<PlaneMap> {
    use ElemKind::{Cpe, Lfe, Sce};
    const L: usize = 0;
    const R: usize = 1;
    let slots: &[(ElemKind, usize, usize)] = match config {
        1 => &[(Sce, 0, L)],
        2 => &[(Cpe, 0, L), (Cpe, 0, R)],
        3 => &[(Cpe, 0, L), (Cpe, 0, R), (Sce, 0, L)],
        4 => &[(Cpe, 0, L), (Cpe, 0, R), (Sce, 0, L), (Sce, 1, L)],
        5 => &[
            (Cpe, 0, L),
            (Cpe, 0, R),
            (Sce, 0, L),
            (Cpe, 1, L),
            (Cpe, 1, R),
        ],
        6 => &[
            (Cpe, 0, L),
            (Cpe, 0, R),
            (Sce, 0, L),
            (Lfe, 0, L),
            (Cpe, 1, L),
            (Cpe, 1, R),
        ],
        // 0 (PCE expected) and 7+ (not v1): keep element order.
        _ => return identity(elems, used),
    };
    let mut out = Vec::with_capacity(slots.len());
    for &(kind, occ, part) in slots {
        let hit = elems
            .iter()
            .enumerate()
            .filter(|(_, e)| e.kind == kind)
            .nth(occ);
        match hit {
            Some((i, e)) => {
                used[i] = true;
                out.push(PlaneMap {
                    src: Some(e.plane + part),
                    lfe: kind == Lfe,
                });
            }
            None => out.push(PlaneMap {
                src: None,
                lfe: kind == Lfe,
            }),
        }
    }
    out
}

/// PCE declaration order: front, side, back (CPE as L then R), then LFEs.
fn pce_slots(pce: &PceChannelMap, elems: &[Element], used: &mut [bool]) -> Vec<PlaneMap> {
    let mut out = Vec::new();
    for list in [&pce.front, &pce.side, &pce.back] {
        for &(is_cpe, tag) in list {
            let kind = if is_cpe { ElemKind::Cpe } else { ElemKind::Sce };
            push_tagged(&mut out, elems, used, kind, tag);
        }
    }
    for &tag in &pce.lfe {
        push_tagged(&mut out, elems, used, ElemKind::Lfe, tag);
    }
    out
}

fn push_tagged(
    out: &mut Vec<PlaneMap>,
    elems: &[Element],
    used: &mut [bool],
    kind: ElemKind,
    tag: u8,
) {
    let hit = elems
        .iter()
        .enumerate()
        .find(|(i, e)| !used[*i] && e.kind == kind && e.tag == tag);
    let n = match kind {
        ElemKind::Cpe => 2,
        ElemKind::Sce | ElemKind::Lfe => 1,
    };
    match hit {
        Some((i, e)) => {
            used[i] = true;
            for p in 0..n {
                out.push(PlaneMap {
                    src: Some(e.plane + p),
                    lfe: kind == ElemKind::Lfe,
                });
            }
        }
        None => {
            for _ in 0..n {
                out.push(PlaneMap {
                    src: None,
                    lfe: kind == ElemKind::Lfe,
                });
            }
        }
    }
}

fn identity(elems: &[Element], used: &mut [bool]) -> Vec<PlaneMap> {
    let mut out = Vec::new();
    for (i, e) in elems.iter().enumerate() {
        used[i] = true;
        for p in 0..e.planes() {
            out.push(PlaneMap {
                src: Some(e.plane + p),
                lfe: e.kind == ElemKind::Lfe,
            });
        }
    }
    out
}

/// Reorder `planes` per `order`, filling `None` slots with silence.
pub(crate) fn reorder(planes: &mut Vec<Vec<f32>>, order: &[PlaneMap]) {
    let len = planes.first().map_or(0, Vec::len);
    let mut old: Vec<Option<Vec<f32>>> = std::mem::take(planes).into_iter().map(Some).collect();
    for slot in order {
        let plane = slot
            .src
            .and_then(|i| old.get_mut(i).and_then(Option::take))
            .unwrap_or_else(|| vec![0.0; len]);
        planes.push(plane);
    }
}

/// Speech mono: arithmetic mean of the non-LFE planes (all planes when every
/// plane is LFE). Stereo reduces to `0.5·(L+R)`.
pub(crate) fn mono_mix(planes: &[Vec<f32>], order: &[PlaneMap]) -> Vec<f32> {
    let len = planes.first().map_or(0, Vec::len);
    let non_lfe: Vec<usize> = (0..planes.len()).filter(|&i| !order[i].lfe).collect();
    let idx: Vec<usize> = if non_lfe.is_empty() {
        (0..planes.len()).collect()
    } else {
        non_lfe
    };
    let mut mono = vec![0.0f32; len];
    for &i in &idx {
        for (m, &s) in mono.iter_mut().zip(planes[i].iter()) {
            *m += s;
        }
    }
    if !idx.is_empty() {
        let inv = 1.0 / idx.len() as f32;
        for m in &mut mono {
            *m *= inv;
        }
    }
    mono
}
