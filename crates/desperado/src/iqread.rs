//! I/Q Data Reading Module
//!
//! This module provides functionality to read I/Q samples from various sources,
//! including files, standard input, and TCP streams. It supports different I/Q
//! data formats and provides both synchronous and asynchronous interfaces for
//! reading I/Q samples.
use std::io::Read;
use std::path::Path;
use std::pin::Pin;
use std::task::{Context, Poll};

use futures::Stream;
use num_complex::Complex;
use tokio::io::AsyncBufRead;

use crate::{IqFormat, error, expanduser};

pub type ZstdIqReader = zstd::stream::read::Decoder<'static, std::io::BufReader<std::fs::File>>;
pub type AsyncZstdIqReader = tokio::io::BufReader<
    async_compression::tokio::bufread::ZstdDecoder<tokio::io::BufReader<tokio::fs::File>>,
>;

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
    ended: bool,
    pending_truncation: Option<error::Error>,
}

impl<R: Read> IqRead<R> {
    fn new(config: IqConfig, reader: R) -> Self {
        Self {
            config,
            reader,
            ended: false,
            pending_truncation: None,
        }
    }

    fn read_samples(&mut self) -> error::Result<Option<Vec<Complex<f32>>>> {
        let bytes_per_sample = self.config.iq_format.bytes_per_sample();
        let mut buffer = vec![0u8; self.config.chunk_size * bytes_per_sample];
        let mut total_read = 0;

        while total_read < buffer.len() {
            match self.reader.read(&mut buffer[total_read..]) {
                Ok(0) => break,
                Ok(read) => total_read += read,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
                Err(e) => return Err(e.into()),
            }
        }

        if total_read == 0 {
            self.ended = true;
            return Ok(None);
        }

        let complete_bytes = total_read - total_read % bytes_per_sample;
        let remaining_bytes = total_read - complete_bytes;
        if remaining_bytes != 0 {
            let error = error::Error::truncated_iq(self.config.iq_format, remaining_bytes);
            if complete_bytes == 0 {
                self.ended = true;
                return Err(error);
            }
            self.pending_truncation = Some(error);
        }

        buffer.truncate(complete_bytes);
        let samples = crate::convert_bytes_to_complex(self.config.iq_format, &buffer);
        Ok(Some(samples))
    }
}

impl IqRead<std::io::BufReader<std::fs::File>> {
    pub fn from_file<P: AsRef<Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let path = expanduser(path.as_ref().to_path_buf());
        let file = std::fs::File::open(path)?;
        let reader = std::io::BufReader::new(file);
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Ok(Self::new(config, reader))
    }
}

impl IqRead<ZstdIqReader> {
    pub fn from_zstd_file<P: AsRef<Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let path = expanduser(path.as_ref().to_path_buf());
        let file = std::fs::File::open(path)?;
        let reader = zstd::stream::read::Decoder::new(file)?;
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Ok(Self::new(config, reader))
    }
}

impl IqRead<std::io::BufReader<std::io::Stdin>> {
    pub fn from_stdin(
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> Self {
        let reader = std::io::BufReader::new(std::io::stdin());
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Self::new(config, reader)
    }
}

impl IqRead<std::io::BufReader<std::net::TcpStream>> {
    pub fn from_tcp(
        addr: &str,
        port: u16,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let stream = std::net::TcpStream::connect((addr, port))?;
        let reader = std::io::BufReader::new(stream);
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Ok(Self::new(config, reader))
    }
}

impl<R: Read> Iterator for IqRead<R> {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ended {
            return None;
        }
        if let Some(error) = self.pending_truncation.take() {
            self.ended = true;
            return Some(Err(error));
        }

        match self.read_samples() {
            Ok(Some(samples)) => Some(Ok(samples)),
            Ok(None) => None,
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
    /// Bytes retained across `Poll::Pending` while filling one API chunk.
    pending: Vec<u8>,
    /// Number of initialized bytes in `pending`.
    pending_len: usize,
    ended: bool,
    pending_truncation: Option<error::Error>,
}

impl<R: tokio::io::AsyncBufRead + Unpin> IqAsyncRead<R> {
    fn new(config: IqConfig, reader: R) -> Self {
        Self {
            config,
            reader,
            pending: Vec::new(),
            pending_len: 0,
            ended: false,
            pending_truncation: None,
        }
    }
}

impl IqAsyncRead<tokio::io::BufReader<tokio::fs::File>> {
    pub fn from_file<P: AsRef<Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> impl std::future::Future<
        Output = error::Result<IqAsyncRead<tokio::io::BufReader<tokio::fs::File>>>,
    > {
        let path = expanduser(path.as_ref().to_path_buf());
        async move {
            let file = tokio::fs::File::open(path).await?;
            let reader = tokio::io::BufReader::new(file);
            let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
            Ok(IqAsyncRead::new(config, reader))
        }
    }
}

impl IqAsyncRead<AsyncZstdIqReader> {
    pub async fn from_zstd_file<P: AsRef<Path>>(
        path: P,
        center_freq: u32,
        sample_rate: u32,
        chunk_size: usize,
        iq_format: IqFormat,
    ) -> error::Result<Self> {
        let path = expanduser(path.as_ref().to_path_buf());
        let file = tokio::fs::File::open(path).await?;
        let decoder =
            async_compression::tokio::bufread::ZstdDecoder::new(tokio::io::BufReader::new(file));
        let reader = tokio::io::BufReader::new(decoder);
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Ok(Self::new(config, reader))
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
        Self::new(config, reader)
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
    ) -> error::Result<Self> {
        let stream = tokio::net::TcpStream::connect((address, port)).await?;
        let reader = tokio::io::BufReader::new(stream);
        let config = IqConfig::new(center_freq, sample_rate, chunk_size, iq_format);
        Ok(Self::new(config, reader))
    }
}

impl<R: AsyncBufRead + Unpin + Send + 'static> Stream for IqAsyncRead<R> {
    type Item = error::Result<Vec<Complex<f32>>>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();
        if this.ended {
            return Poll::Ready(None);
        }
        if let Some(error) = this.pending_truncation.take() {
            this.ended = true;
            return Poll::Ready(Some(Err(error)));
        }

        let bytes_per_sample = this.config.iq_format.bytes_per_sample();
        let chunk_bytes = this.config.chunk_size * bytes_per_sample;

        if this.pending.is_empty() {
            this.pending.resize(chunk_bytes, 0);
            this.pending_len = 0;
        }

        while this.pending_len < this.pending.len() {
            let mut read_buf = tokio::io::ReadBuf::new(&mut this.pending[this.pending_len..]);
            match Pin::new(&mut this.reader).poll_read(cx, &mut read_buf) {
                Poll::Ready(Ok(())) => {
                    let filled = read_buf.filled().len();
                    if filled == 0 {
                        break;
                    }
                    this.pending_len += filled;
                }
                Poll::Ready(Err(e)) => {
                    if e.kind() == std::io::ErrorKind::UnexpectedEof && this.pending_len > 0 {
                        break;
                    } else if e.kind() == std::io::ErrorKind::UnexpectedEof {
                        this.ended = true;
                        return Poll::Ready(None);
                    } else {
                        return Poll::Ready(Some(Err(e.into())));
                    }
                }
                Poll::Pending => return Poll::Pending,
            }
        }

        if this.pending_len == 0 {
            this.pending.clear();
            this.ended = true;
            return Poll::Ready(None);
        }

        let mut buffer = std::mem::take(&mut this.pending);
        let total_read = std::mem::take(&mut this.pending_len);
        let complete_bytes = total_read - total_read % bytes_per_sample;
        let remaining_bytes = total_read - complete_bytes;
        if remaining_bytes != 0 {
            let error = error::Error::truncated_iq(this.config.iq_format, remaining_bytes);
            if complete_bytes == 0 {
                this.ended = true;
                return Poll::Ready(Some(Err(error)));
            }
            this.pending_truncation = Some(error);
        }

        buffer.truncate(complete_bytes);
        let samples = crate::convert_bytes_to_complex(this.config.iq_format, &buffer);
        Poll::Ready(Some(Ok(samples)))
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

#[cfg(test)]
mod tests {
    use super::*;
    use futures::StreamExt;
    use tokio::io::{AsyncBufRead, AsyncRead, ReadBuf};

    /// Delivers one byte, yields `Pending`, then delivers the remaining bytes.
    /// The split occurs inside the first complex CU8 sample.
    struct PendingThenBytes {
        phase: u8,
    }

    impl AsyncRead for PendingThenBytes {
        fn poll_read(
            mut self: Pin<&mut Self>,
            cx: &mut Context<'_>,
            buf: &mut ReadBuf<'_>,
        ) -> Poll<std::io::Result<()>> {
            match self.phase {
                0 => {
                    buf.put_slice(&[0]);
                    self.phase = 1;
                    Poll::Ready(Ok(()))
                }
                1 => {
                    self.phase = 2;
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
                2 => {
                    buf.put_slice(&[255, 127, 128]);
                    self.phase = 3;
                    Poll::Ready(Ok(()))
                }
                _ => Poll::Ready(Ok(())),
            }
        }
    }

    impl AsyncBufRead for PendingThenBytes {
        fn poll_fill_buf(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
        ) -> Poll<std::io::Result<&[u8]>> {
            Poll::Ready(Ok(&[]))
        }

        fn consume(self: Pin<&mut Self>, _amt: usize) {}
    }

    #[tokio::test]
    async fn retains_partial_bytes_across_pending() {
        let config = IqConfig::new(162_000_000, 96_000, 2, IqFormat::Cu8);
        let mut reader = IqAsyncRead::new(config, PendingThenBytes { phase: 0 });

        let chunk = reader.next().await.unwrap().unwrap();
        assert_eq!(chunk.len(), 2);
        assert_eq!(chunk[0], Complex::new(-127.5 / 128.0, 127.5 / 128.0));
        assert_eq!(chunk[1], Complex::new(-0.5 / 128.0, 0.5 / 128.0));
        assert!(reader.next().await.is_none());
    }
}
