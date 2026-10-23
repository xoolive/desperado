# Changelog

## [0.5.1] - 2026-10-23

### Fixed

- Stop and join live SDR readers when FM and DAB receivers exit, including decoding-error paths and quit requests while input is idle. ([#58])
- Power down the RTL-SDR demodulator and ADCs after USB transfers finish, in addition to placing the tuner in standby. Cleanup attempts all shutdown steps even if an earlier step fails. ([#58])
- Prevent RTL-SDR, Airspy, and HackRF shutdown deadlocks when their sample queues are full. Normal reception retains lossless backpressure. ([#59])
- Make DAB service switching interrupt obsolete audio enqueueing, flush the previous station's playback queue, and re-prime playback for the selected service. Re-selecting the active service no longer resets playback. ([#59])
- Improve DAB TUI responsiveness by reducing shared-state lock contention, draining queued keyboard events, and avoiding unnecessary slideshow redraws. ([#59])

### Dependencies

- Update `zstd` to 0.14, `dirs` to 7, `oxideav-mp2` to 0.0.10, `async-compression` to 0.4.48, and `thiserror` to 2.0.21.

This patch release does not include HD Radio, FM WAV export, or the upcoming radio-library refactor.

[0.5.1]: https://github.com/xoolive/desperado/compare/v0.5.0...v0.5.1
[#58]: https://github.com/xoolive/desperado/pull/58
[#59]: https://github.com/xoolive/desperado/pull/59
