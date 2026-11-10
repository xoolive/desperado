//! SCE finish and CPE pair for [`super::decode::StreamDecoder`].

use super::bits::BitReader;
use super::decode::StreamDecoder;
use super::error::Result;
use super::ics::IcsInfo;
use super::ics_body::parse_ics_into;
use super::pns;
use super::stereo::{self, MsInfo};
use super::tns;

impl StreamDecoder {
    pub(crate) fn finish_sce(
        &mut self,
        ics: IcsInfo,
        tns: Option<&tns::TnsData>,
        fs_index: u8,
    ) -> Result<()> {
        pns::apply(
            &mut self.spec_l,
            &ics,
            &self.sections_l,
            &self.sf_l,
            fs_index,
            &mut self.rng,
            None,
        )?;
        if let Some(t) = tns {
            tns::apply(&mut self.spec_l, t, &ics, fs_index)?;
        }
        self.fb_l
            .synthesize_into(&self.spec_l, &ics, &mut self.pcm_l)
    }

    pub(crate) fn decode_cpe(
        &mut self,
        br: &mut BitReader<'_>,
        fs_index: u8,
        aot: u8,
        _tag: u8,
    ) -> Result<()> {
        let common_window = br.read_bit()?;
        let (ics_common, ms) = if common_window {
            let ics = super::ics::IcsInfo::parse(br, fs_index, true)?;
            let ms = MsInfo::parse(br, &ics)?;
            (Some(ics), Some(ms))
        } else {
            (None, None)
        };
        let (ics_l, tns_l) = parse_ics_into(
            br,
            fs_index,
            aot,
            ics_common.as_ref(),
            &mut self.quant,
            &mut self.spec_l,
            &mut self.sections_l,
            &mut self.sf_l,
        )?;
        let (ics_r, tns_r) = parse_ics_into(
            br,
            fs_index,
            aot,
            ics_common.as_ref(),
            &mut self.quant,
            &mut self.spec_r,
            &mut self.sections_r,
            &mut self.sf_r,
        )?;
        if let Some(ms) = ms.as_ref() {
            stereo::apply_ms(
                &mut self.spec_l,
                &mut self.spec_r,
                &ics_l,
                &self.sections_l,
                &self.sections_r,
                ms,
                fs_index,
            )?;
        }
        let ms_used = ms.as_ref().map(|m| m.used.clone());
        let mut shared = None;
        {
            let mut pair = pns::PairPns {
                ms_used: ms_used.as_deref(),
                other_cb: Some(&self.sections_r.sfb_cb),
                shared: &mut shared,
            };
            pns::apply(
                &mut self.spec_l,
                &ics_l,
                &self.sections_l,
                &self.sf_l,
                fs_index,
                &mut self.rng,
                Some(&mut pair),
            )?;
        }
        {
            let mut pair = pns::PairPns {
                ms_used: ms_used.as_deref(),
                other_cb: Some(&self.sections_l.sfb_cb),
                shared: &mut shared,
            };
            pns::apply(
                &mut self.spec_r,
                &ics_r,
                &self.sections_r,
                &self.sf_r,
                fs_index,
                &mut self.rng,
                Some(&mut pair),
            )?;
        }
        if let Some(ms) = ms.as_ref() {
            stereo::apply_intensity(
                &self.spec_l,
                &mut self.spec_r,
                &ics_r,
                &self.sections_r,
                &self.sf_r,
                ms,
                fs_index,
            )?;
        }
        if let Some(t) = tns_l.as_ref() {
            tns::apply(&mut self.spec_l, t, &ics_l, fs_index)?;
        }
        if let Some(t) = tns_r.as_ref() {
            tns::apply(&mut self.spec_r, t, &ics_r, fs_index)?;
        }
        self.fb_l
            .synthesize_into(&self.spec_l, &ics_l, &mut self.pcm_l)?;
        self.fb_r
            .synthesize_into(&self.spec_r, &ics_r, &mut self.pcm_r)?;
        Ok(())
    }
}
