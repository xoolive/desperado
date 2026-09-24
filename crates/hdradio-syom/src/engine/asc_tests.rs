//! ASC `write_lc` ↔ parse roundtrip.

use super::super::error::Result;
use super::{AudioSpecificConfig, write_lc};

#[test]
fn write_lc_roundtrip() -> Result<()> {
    for (fs_index, ch) in [(3u8, 1u8), (3, 2), (0, 2), (11, 1)] {
        let bytes = write_lc(fs_index, ch);
        assert_eq!(bytes.len(), 2, "bare LC ASC is 16 bits");
        let (asc, bits) = AudioSpecificConfig::parse(&bytes)?;
        assert_eq!(asc.aot, 2);
        assert_eq!(asc.sampling_frequency_index, fs_index);
        assert_eq!(asc.channel_configuration, ch);
        assert!(!asc.sbr_present);
        assert!(!asc.ps_present);
        assert_eq!(asc.sample_rate, asc.output_sample_rate);
        assert_eq!(bits, 16);
    }
    Ok(())
}
