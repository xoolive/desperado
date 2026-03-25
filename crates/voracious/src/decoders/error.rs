//! Error types for voracious decoders.

use std::fmt;

/// Result type alias for voracious operations.
pub type Result<T> = std::result::Result<T, Error>;

/// Error types for radio navigation signal decoding.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// Input signal is empty
    EmptyInput,
    /// Input signal is too short for the requested operation
    InsufficientData { required: usize, provided: usize },
    /// Signal contains non-finite values (NaN or infinity)
    NonFiniteSignal,
    /// Signal RMS is below minimum detectable threshold
    SignalTooWeak { rms: f64, threshold: f64 },
    /// Failed to decode Morse ident (insufficient SNR or signal quality)
    MorseDecodeFailed { reason: String },
    /// ILS DDM computation failed due to signal conditions
    DdmComputationFailed { reason: String },
    /// Generic signal processing error
    SignalProcessing { reason: String },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::EmptyInput => write!(f, "input signal is empty"),
            Error::InsufficientData { required, provided } => {
                write!(
                    f,
                    "insufficient data: need {required} samples, got {provided}"
                )
            }
            Error::NonFiniteSignal => write!(f, "signal contains NaN or infinity"),
            Error::SignalTooWeak { rms, threshold } => {
                write!(
                    f,
                    "signal too weak (RMS {rms:.2e} < threshold {threshold:.2e})"
                )
            }
            Error::MorseDecodeFailed { reason } => write!(f, "Morse decode failed: {reason}"),
            Error::DdmComputationFailed { reason } => write!(f, "DDM computation failed: {reason}"),
            Error::SignalProcessing { reason } => write!(f, "signal processing error: {reason}"),
        }
    }
}

impl std::error::Error for Error {}

// ── Structured result types for major operations ────────────────────────────

/// Result of VOR radial calculation.
#[derive(Debug, Clone, PartialEq)]
pub struct VorRadialResult {
    /// Calculated radial bearing (0°-360°) to the station
    pub radial: f64,
    /// Estimated radial accuracy/confidence (0-100%)
    pub confidence: f64,
    /// Whether this is FROM or TO the station
    pub to_from: Option<VorToFrom>,
}

/// TO/FROM indicator for VOR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VorToFrom {
    /// Radial leads TO the station
    To,
    /// Radial leads FROM the station
    From,
}

/// Result of VOR track calculation.
#[derive(Debug, Clone)]
pub struct VorTrackResult {
    /// Calculated radial (0°-360°)
    pub radial: f64,
    /// Course deviation indicator (CDI) in dots (-5 to +5, 0 = on course)
    pub cdi: f64,
    /// To/From flag
    pub to_from: Option<VorToFrom>,
    /// Signal quality metrics
    pub signal_quality: VorSignalQuality,
}

/// VOR signal quality metrics.
#[derive(Debug, Clone)]
pub struct VorSignalQuality {
    /// 30 Hz reference signal RMS level
    pub ref_level: f64,
    /// 30 Hz variable signal RMS level
    pub var_level: f64,
    /// Modulation index (0-1, ideally ~0.3)
    pub modulation_index: f64,
}

/// Result of ILS DDM calculation.
#[derive(Debug, Clone)]
pub struct IlsDdmResult {
    /// Difference in Depth of Modulation (-1.0 to +1.0)
    /// Negative = right of center, Positive = left of center
    pub ddm: f64,
    /// 90 Hz modulation depth (0-100%)
    pub mod_90_hz: f64,
    /// 150 Hz modulation depth (0-100%)
    pub mod_150_hz: f64,
    /// Carrier signal strength (RMS level)
    pub carrier_strength: f64,
}

/// Result of Morse ident decoding.
#[derive(Debug, Clone)]
pub struct MorseDecodeResult {
    /// Decoded ident (typically 2-4 letters)
    pub ident: String,
    /// Confidence level (0-100%)
    pub confidence: f64,
    /// Number of times this ident was detected
    pub decode_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = Error::EmptyInput;
        assert_eq!(err.to_string(), "input signal is empty");

        let err = Error::InsufficientData {
            required: 1000,
            provided: 500,
        };
        assert_eq!(
            err.to_string(),
            "insufficient data: need 1000 samples, got 500"
        );

        let err = Error::SignalTooWeak {
            rms: 1e-3,
            threshold: 1e-2,
        };
        assert!(err.to_string().contains("signal too weak"));
    }

    #[test]
    fn test_error_eq() {
        let err1 = Error::EmptyInput;
        let err2 = Error::EmptyInput;
        assert_eq!(err1, err2);
    }
}
