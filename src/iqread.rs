//! I/Q Data Reading Module
//!
//! This module provides functionality to read I/Q samples from various sources,
//! including files, standard input, and TCP streams. It supports different I/Q
//! data formats and provides both synchronous and asynchronous interfaces for
//! reading I/Q samples.
use std::io::Read;
use std::path::{Path, PathBuf};
use std::pin::Pin;
use std::task::{Context, Poll};

use futures::Stream;
use num_complex::Complex;
use tokio::io::AsyncBufRead;

use crate::IqFormat;

/**
 * I/Q Data Source Configuration
 */
pub struct IqConfig {
    pub iq_format: IqFormat,
    pub center_freq: u32,
    pub sample_rate: u32,
    pub chunk_size: usize,
}

impl IqConfig {
    pub fn new(center_freq: u32, sample_rate: u32, chunk_size: usize, iq_format: IqFormat) -> Self {
        Self {
            iq_format,
            center_freq,
            sample_rate,
            chunk_size,
        }
    }
}

/**
 * Synchronous I/Q Reader
 */
pub struct IqRead<R: Read> {
    config: IqConfig,
    reader: R,
}

impl IqRead<std::io::BufReader<std::fs::File>> {
    pub fn from_file<P: AsRef<Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Result<Self, std::io::Error> {
        let path = expanduser(path.as_ref().to_path_buf());
        let file = std::fs::File::open(path)?;
        let reader = std::io::BufReader::new(file);
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Ok(Self { config, reader })
    }

    fn read_samples(&mut self) -> Result<Vec<Complex<f32>>, std::io::Error> {
        let bytes_per_sample = self.config.iq_format.bytes_per_sample();
        let mut buffer = vec![0u8; self.config.chunk_size * bytes_per_sample];
        self.reader.read_exact(&mut buffer)?;
        let samples = crate::convert_bytes_to_complex(self.config.iq_format, &buffer);
        Ok(samples)
    }
}

impl Iterator for IqRead<std::io::BufReader<std::fs::File>> {
    type Item = Result<Vec<Complex<f32>>, std::io::Error>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.read_samples() {
            Ok(samples) => Some(Ok(samples)),
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => None,
            Err(e) => Some(Err(e)),
        }
    }
}

/**
 * Asynchronous I/Q Reader
 */
pub struct IqAsyncRead<R: tokio::io::AsyncBufRead + Unpin> {
    config: IqConfig,
    reader: R,
}

impl IqAsyncRead<tokio::io::BufReader<tokio::fs::File>> {
    pub fn from_file<P: AsRef<Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> impl std::future::Future<
        Output = Result<IqAsyncRead<tokio::io::BufReader<tokio::fs::File>>, std::io::Error>,
    > {
        let path = expanduser(path.as_ref().to_path_buf());
        async move {
            let file = tokio::fs::File::open(path).await?;
            let reader = tokio::io::BufReader::new(file);
            let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
            Ok(IqAsyncRead { config, reader })
        }
    }
}

impl IqAsyncRead<tokio::io::BufReader<tokio::io::Stdin>> {
    pub fn from_stdin(
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Self {
        let reader = tokio::io::BufReader::new(tokio::io::stdin());
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Self { config, reader }
    }
}

impl IqAsyncRead<tokio::io::BufReader<tokio::net::TcpStream>> {
    pub async fn from_tcp(
        address: &str,
        port: u16,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Result<Self, std::io::Error> {
        let stream = tokio::net::TcpStream::connect((address, port)).await?;
        let reader = tokio::io::BufReader::new(stream);
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Ok(Self { config, reader })
    }
}

impl<R: AsyncBufRead + Unpin + Send + 'static> Stream for IqAsyncRead<R> {
    type Item = Result<Vec<Complex<f32>>, std::io::Error>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();
        let bytes_per_sample = this.config.iq_format.bytes_per_sample();
        let mut buffer = vec![0u8; this.config.chunk_size * bytes_per_sample];
        let mut total_read = 0;

        while total_read < buffer.len() {
            let mut read_buf = tokio::io::ReadBuf::new(&mut buffer[total_read..]);
            match Pin::new(&mut this.reader).poll_read(cx, &mut read_buf) {
                Poll::Ready(Ok(())) => {
                    let filled = read_buf.filled().len();
                    if filled == 0 {
                        break;
                    }
                    total_read += filled;
                }
                Poll::Ready(Err(e)) => {
                    if e.kind() == std::io::ErrorKind::UnexpectedEof && total_read > 0 {
                        break;
                    } else if e.kind() == std::io::ErrorKind::UnexpectedEof {
                        return Poll::Ready(None);
                    } else {
                        return Poll::Ready(Some(Err(e)));
                    }
                }
                Poll::Pending => return Poll::Pending,
            }
        }

        if total_read == 0 {
            Poll::Ready(None)
        } else {
            buffer.truncate(total_read);
            let samples = crate::convert_bytes_to_complex(this.config.iq_format, &buffer);
            Poll::Ready(Some(Ok(samples)))
        }
    }
}

impl IqFormat {
    fn bytes_per_sample(self) -> usize {
        match self {
            IqFormat::Cu8 | IqFormat::Cs8 => 2,
            IqFormat::Cs16 => 4,
            IqFormat::Cf32 => 8,
        }
    }
}

fn expanduser(path: PathBuf) -> PathBuf {
    // Check if the path starts with "~"
    if let Some(stripped) = path.to_str().and_then(|p| p.strip_prefix("~"))
        && let Some(home_dir) = dirs::home_dir()
    {
        // Join the home directory with the rest of the path
        return home_dir.join(stripped.trim_start_matches('/'));
    }
    path
}
