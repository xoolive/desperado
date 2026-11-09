# Changelog

## [0.5.2] - 2026-11-09

### Added

- `desperado::capture::parse_capture_filename` recognises GQRX, SDRuno and rtl_sdr capture file names, including `.zst`-compressed ones, and returns the center frequency, sample rate, I/Q format and, when the name carries a full date and time, the UTC capture start. ([#68])
- `IqFormat` parsing accepts the spellings used by rtl_sdr, acarsdec and dumpvdl2 (`U8`, `S16`, `S16_LE`), GQRX (`fc`) and SoapySDR, case-insensitively. ([#68])
- `parse_si_value` accepts a trailing `Hz` unit, e.g. `131.725 MHz` or `12.5kHz`. ([#68])

### Changed

- fmradio and dabradio detect capture parameters through `desperado::capture`. Undated `rtlsdr_<freq>_<rate>_...` recordings are now auto-configured, and dabradio's `--format` accepts the same aliases. ([#68])

### Dependencies

- Update `tokio` to 1.54.1, `serde_json` to 1.0.154, `async-compression` to 0.4.51, `rubato` to 5.0.1, and `nusb` to 0.2.9.

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

[0.5.2]: https://github.com/xoolive/desperado/compare/v0.5.1...v0.5.2
[0.5.1]: https://github.com/xoolive/desperado/compare/v0.5.0...v0.5.1
[#58]: https://github.com/xoolive/desperado/pull/58
[#59]: https://github.com/xoolive/desperado/pull/59
[#68]: https://github.com/xoolive/desperado/pull/68
