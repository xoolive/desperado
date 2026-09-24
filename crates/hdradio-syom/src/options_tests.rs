//! DecodeOptions defaults.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;

#[test]
fn speech_defaults() {
    let o = DecodeOptions::speech();
    assert_eq!(o.channel_mode, ChannelMode::Mono);
    assert_eq!(o.max_frames(16_000), 16_000 * 7200);
    assert_eq!(o.with_max_duration_secs(1.0).max_frames(16_000), 16_000);
}
