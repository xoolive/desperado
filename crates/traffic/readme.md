# traffic

Application-layer decoding for free, unencrypted road-traffic data:

- **TPEG/TEC** (ISO 21219-5 transport, ISO 21219-15 TEC) carried in DAB packet mode
- **RDS-TMC ALERT-C** (ISO 14819-1) carried in FM RDS groups 3A and 8A
- **DAB Traffic Announcements** (FIG 0/18 support + FIG 0/19 switching) — audio-side
  stream switching with no location/event payload; emitted as
  `AnnouncementEvent` JSON (`bearer: dab-announcement`), not TEC GeoJSON

This crate does not parse DAB FIGs or FM RDS groups. `dabradio` and `fmradio`
call it once they have a validated payload. TPEG/TMC output is GeoJSON
(`FeatureCollection`) with `properties.bearer` set to `dab-tpeg` or
`fm-rds-tmc`. Announcement events use a separate shape.

Conditional-access and ISO 14819-6 encrypted streams are detected and reported;
they are not decrypted. Location tables (RDS-TMC / GLR) are optional and loaded
from a directory or CSV supplied by the caller (`--location-tables` on the
radio CLIs). Without tables, geometry is omitted and the location code is left
in properties.

```bash
# DAB: only runs TPEG decode when FIG 0/13 UAtype 0x004 is present
cargo run --release -p dabradio -- recording.cf32.iq --channel 12D --format cf32 --traffic

# FM: ALERT-C after group 3A registers AID CD46/CD47
cargo run --release -p fmradio -- capture.cf32 --freq 96.9M --format cf32 --no-audio --traffic
```
