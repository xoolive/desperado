//! `id_syn_ele` — ISO/IEC 14496-3 Table 4.71.

/// Syntactic element id.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IdSynEle {
    /// Single channel.
    Sce = 0,
    /// Channel pair.
    Cpe = 1,
    /// Coupling channel.
    Cce = 2,
    /// LFE.
    Lfe = 3,
    /// Data stream.
    Dse = 4,
    /// Program config.
    Pce = 5,
    /// Fill.
    Fil = 6,
    /// End.
    End = 7,
}

impl IdSynEle {
    /// Map a 3-bit wire value.
    #[must_use]
    pub fn from_bits(bits: u8) -> Self {
        match bits & 0b111 {
            0 => Self::Sce,
            1 => Self::Cpe,
            2 => Self::Cce,
            3 => Self::Lfe,
            4 => Self::Dse,
            5 => Self::Pce,
            6 => Self::Fil,
            _ => Self::End,
        }
    }
}
