# hdradio-syom

A pure-Rust AAC-LC/HE-AAC decoder core adapted for NRSC-5 HDC transport.

This crate is derived from [syom](https://github.com/ekhodzitsky/syom),
licensed under the MIT license. The HDC adapter adds NRSC-5-specific framing,
side-information, TNS, and DRM-derived SBR handling used by the `hdradio`
receiver. See `LICENSE` and `NOTICE` for provenance.

The package is currently maintained as an internal workspace dependency while
HDC compatibility is stabilized.
