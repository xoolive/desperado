#![doc = include_str!("../readme.md")]

use std::{
    pin::Pin,
    task::{Context, Poll},
};

use futures::Stream;
use num_complex::Complex;

pub mod iqread;
#[cfg(feature = "rtlsdr")]
pub mod rtlsdr;

/**
 * I/Q Data Format
 */
#[derive(Debug, Copy, Clone)]
pub enum IqFormat {
    /// Complex unsigned 8-bit (Cu8)
    Cu8,
    /// Complex signed 8-bit (Cs8)
    Cs8,
    /// Complex signed 16-bit (Cs16)
    Cs16,
    /// Complex 32-bit float (Cf32)
    Cf32,
}

/**
 * Synchronous I/Q Data Source (iterable)
 */
pub enum IqSource {
    /// File-based IQ source
    IqFile(iqread::IqRead<std::io::BufReader<std::fs::File>>),
    #[cfg(feature = "rtlsdr")]
    /// RTL-SDR-based IQ source (requires "rtlsdr" feature)
    RtlSdr(rtlsdr::RtlSdrReader),
}

impl Iterator for IqSource {
    type Item = Result<Vec<Complex<f32>>, std::io::Error>;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            IqSource::IqFile(source) => source.next(),
            #[cfg(feature = "rtlsdr")]
            IqSource::RtlSdr(_source) => todo!(), /*source.next()*/
        }
    }
}
impl IqSource {
    /// Create a new file-based I/Q source
    pub fn from_file<P: AsRef<std::path::Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Result<Self, std::io::Error> {
        let source =
            iqread::IqRead::from_file(path, center_freq, sample_rate, chunk_size, iq_format)?;
        Ok(IqSource::IqFile(source))
    }

    #[cfg(feature = "rtlsdr")]
    /// Create a new RTL-SDR-based I/Q source
    pub fn from_rtlsdr(
        device_index: usize,
        center_freq: u32,
        sample_rate: u32,
        gain: Option<i32>,
    ) -> Result<Self, rtl_sdr_rs::error::RtlsdrError> {
        let config = rtlsdr::RtlSdrConfig {
            device_index,
            center_freq,
            sample_rate,
            gain,
        };
        let source = rtlsdr::RtlSdrReader::new(&config)?;
        Ok(IqSource::RtlSdr(source))
    }
}

/**
 * Asynchronous I/Q Data Source (streamable)
 */
pub enum IqAsyncSource {
    /// File-based IQ source
    IqAsyncFile(iqread::IqAsyncRead<tokio::io::BufReader<tokio::fs::File>>),
    /// Stdin-based IQ source
    IqAsyncStdin(iqread::IqAsyncRead<tokio::io::BufReader<tokio::io::Stdin>>),
    /// TCP-based IQ source
    IqAsyncTcp(iqread::IqAsyncRead<tokio::io::BufReader<tokio::net::TcpStream>>),
    #[cfg(feature = "rtlsdr")]
    /// RTL-SDR-based IQ source (requires "rtlsdr" feature)
    RtlSdr(rtlsdr::AsyncRtlSdrReader),
}

impl IqAsyncSource {
    /// Create a new file-based asynchronous I/Q source
    pub async fn from_file<P: AsRef<std::path::Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Result<Self, std::io::Error> {
        let source =
            iqread::IqAsyncRead::from_file(path, center_freq, sample_rate, chunk_size, iq_format)
                .await?;
        Ok(IqAsyncSource::IqAsyncFile(source))
    }

    /// Create a new stdin-based asynchronous I/Q source
    pub fn from_stdin(
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Self {
        let source =
            iqread::IqAsyncRead::from_stdin(center_freq, sample_rate, chunk_size, iq_format);
        IqAsyncSource::IqAsyncStdin(source)
    }

    /// Create a new TCP-based asynchronous I/Q source
    pub async fn from_tcp(
        addr: &str,
        port: u16,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Result<Self, std::io::Error> {
        let source = iqread::IqAsyncRead::from_tcp(
            addr,
            port,
            center_freq,
            sample_rate,
            chunk_size,
            iq_format,
        )
        .await?;
        Ok(IqAsyncSource::IqAsyncTcp(source))
    }

    #[cfg(feature = "rtlsdr")]
    /// Create a new RTL-SDR-based asynchronous I/Q source
    pub async fn from_rtlsdr(
        device_index: usize,
        center_freq: u32,
        sample_rate: u32,
        gain: Option<i32>,
    ) -> Result<Self, rtl_sdr_rs::error::RtlsdrError> {
        let config = rtlsdr::RtlSdrConfig {
            device_index,
            center_freq,
            sample_rate,
            gain,
        };
        let async_reader = rtlsdr::AsyncRtlSdrReader::new(&config)?;
        Ok(IqAsyncSource::RtlSdr(async_reader))
    }
}

impl Stream for IqAsyncSource {
    type Item = Result<Vec<Complex<f32>>, std::io::Error>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        match self.get_mut() {
            IqAsyncSource::IqAsyncFile(source) => Pin::new(source).poll_next(cx),
            IqAsyncSource::IqAsyncStdin(source) => Pin::new(source).poll_next(cx),
            IqAsyncSource::IqAsyncTcp(source) => Pin::new(source).poll_next(cx),
            #[cfg(feature = "rtlsdr")]
            IqAsyncSource::RtlSdr(_source) => todo!(), /*Pin::new(source).poll_next(cx)*/
        }
    }
}

fn convert_bytes_to_complex(format: IqFormat, buffer: &[u8]) -> Vec<Complex<f32>> {
    match format {
        IqFormat::Cu8 => buffer
            .chunks_exact(2)
            .map(|c| Complex::new((c[0] as f32 - 127.5) / 128.0, (c[1] as f32 - 127.5) / 128.0))
            .collect(),
        IqFormat::Cs8 => buffer
            .chunks_exact(2)
            .map(|c| Complex::new((c[0] as i8) as f32 / 128.0, (c[1] as i8) as f32 / 128.0))
            .collect(),
        IqFormat::Cs16 => buffer
            .chunks_exact(4)
            .map(|c| {
                Complex::new(
                    i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                    i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
                )
            })
            .collect(),
        IqFormat::Cf32 => buffer
            .chunks_exact(8)
            .map(|c| {
                Complex::new(
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                    f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
                )
            })
            .collect(),
    }
}
