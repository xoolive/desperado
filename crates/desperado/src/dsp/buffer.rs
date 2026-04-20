//! Streaming buffer abstraction for DSP processing.
//!
//! This module provides a `StreamBuffer<T>` that efficiently manages
//! sample accumulation and consumption patterns common in DSP pipelines.
//!
//! # Pattern
//!
//! The streaming buffer follows a simple lifecycle:
//! 1. **Accumulate**: Push samples via `push_slice()`
//! 2. **Process**: Access accumulated samples via indexing
//! 3. **Consume**: Remove processed samples via `consume()`
//!
//! This pattern avoids frequent allocations by reusing buffer capacity.
//!
//! # Example
//!
//! ```rust
//! use desperado::dsp::buffer::StreamBuffer;
//!
//! let mut buffer = StreamBuffer::new();
//!
//! // Push samples
//! buffer.push_slice(&[1, 2, 3, 4, 5]);
//! assert_eq!(buffer.len(), 5);
//!
//! // Process (e.g., need groups of 3)
//! let available = buffer.len();
//! if available >= 3 {
//!     // Process first 3 samples
//!     let _processed = &buffer[0..3];
//!     // Remove processed samples
//!     buffer.consume(3);
//!     assert_eq!(buffer.len(), 2);
//! }
//! ```
//!
//! # Performance Considerations
//!
//! - **Memory**: Uses `Vec::drain()` which shifts remaining samples left
//!   (O(n) where n = remaining samples after consume)
//! - **Allocation**: No new allocations after first `push_slice()` that exceeds
//!   the internal capacity
//! - **Throughput**: Suitable for streaming DSP at 6-24 MSPS with typical
//!   buffer sizes (filter taps, decimation factors)

/// Streaming buffer for DSP sample accumulation and consumption.
///
/// Manages a buffer that accumulates samples and allows efficient consumption
/// of processed data. Used in decimators, resamplers, and similar filters
/// that process samples in chunks.
#[derive(Debug, Clone)]
pub struct StreamBuffer<T> {
    /// Internal sample buffer
    buffer: Vec<T>,
}

impl<T> StreamBuffer<T> {
    /// Creates a new empty streaming buffer.
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    /// use num_complex::Complex;
    ///
    /// let buffer: StreamBuffer<Complex<f32>> = StreamBuffer::new();
    /// assert_eq!(buffer.len(), 0);
    /// ```
    pub fn new() -> Self {
        Self { buffer: Vec::new() }
    }

    /// Creates a streaming buffer with pre-allocated capacity.
    ///
    /// # Arguments
    /// * `capacity` - Number of elements to pre-allocate
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    ///
    /// let buffer: StreamBuffer<f32> = StreamBuffer::with_capacity(1024);
    /// assert_eq!(buffer.capacity(), 1024);
    /// ```
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            buffer: Vec::with_capacity(capacity),
        }
    }

    /// Returns the current number of samples in the buffer.
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    ///
    /// let mut buffer = StreamBuffer::new();
    /// buffer.push_slice(&[1, 2, 3]);
    /// assert_eq!(buffer.len(), 3);
    /// ```
    #[inline]
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Returns true if the buffer is empty.
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    ///
    /// let buffer: StreamBuffer<i32> = StreamBuffer::new();
    /// assert!(buffer.is_empty());
    /// ```
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    /// Returns the allocated capacity of the internal buffer.
    pub fn capacity(&self) -> usize {
        self.buffer.capacity()
    }

    /// Pushes all samples from a slice into the buffer.
    ///
    /// # Arguments
    /// * `samples` - Slice of samples to push
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    ///
    /// let mut buffer = StreamBuffer::new();
    /// buffer.push_slice(&[1, 2, 3]);
    /// buffer.push_slice(&[4, 5]);
    /// assert_eq!(buffer.len(), 5);
    /// ```
    #[inline]
    pub fn push_slice(&mut self, samples: &[T])
    where
        T: Clone,
    {
        self.buffer.extend_from_slice(samples);
    }

    /// Removes the first `n` samples from the buffer.
    ///
    /// # Arguments
    /// * `n` - Number of samples to remove (clamped to buffer length)
    ///
    /// # Performance
    ///
    /// This operation is O(n) where n is the number of remaining samples,
    /// as it requires shifting data within the vector. For typical DSP usage
    /// (filter taps, decimation factors), this overhead is negligible.
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    ///
    /// let mut buffer = StreamBuffer::new();
    /// buffer.push_slice(&[1, 2, 3, 4, 5]);
    /// buffer.consume(2);
    /// assert_eq!(buffer.len(), 3);
    /// ```
    pub fn consume(&mut self, n: usize)
    where
        T: Clone,
    {
        let n = n.min(self.buffer.len());
        self.buffer.drain(0..n);
    }

    /// Clears all samples from the buffer, resetting state.
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    ///
    /// let mut buffer = StreamBuffer::new();
    /// buffer.push_slice(&[1, 2, 3]);
    /// buffer.clear();
    /// assert!(buffer.is_empty());
    /// ```
    #[inline]
    pub fn clear(&mut self) {
        self.buffer.clear();
    }

    /// Returns a reference to the internal buffer as a slice.
    ///
    /// # Example
    ///
    /// ```rust
    /// use desperado::dsp::buffer::StreamBuffer;
    ///
    /// let mut buffer = StreamBuffer::new();
    /// buffer.push_slice(&[1, 2, 3]);
    /// assert_eq!(buffer.as_slice(), &[1, 2, 3]);
    /// ```
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        &self.buffer
    }
}

impl<T> Default for StreamBuffer<T> {
    fn default() -> Self {
        Self::new()
    }
}

// Implement indexing for convenient access
impl<T> std::ops::Index<usize> for StreamBuffer<T> {
    type Output = T;

    #[inline]
    fn index(&self, idx: usize) -> &T {
        &self.buffer[idx]
    }
}

impl<T> std::ops::Index<std::ops::Range<usize>> for StreamBuffer<T> {
    type Output = [T];

    #[inline]
    fn index(&self, range: std::ops::Range<usize>) -> &[T] {
        &self.buffer[range]
    }
}

impl<T> std::ops::Index<std::ops::RangeTo<usize>> for StreamBuffer<T> {
    type Output = [T];

    #[inline]
    fn index(&self, range: std::ops::RangeTo<usize>) -> &[T] {
        &self.buffer[range]
    }
}

impl<T> std::ops::Index<std::ops::RangeFrom<usize>> for StreamBuffer<T> {
    type Output = [T];

    #[inline]
    fn index(&self, range: std::ops::RangeFrom<usize>) -> &[T] {
        &self.buffer[range]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let buffer: StreamBuffer<i32> = StreamBuffer::new();
        assert!(buffer.is_empty());
        assert_eq!(buffer.len(), 0);
    }

    #[test]
    fn test_with_capacity() {
        let buffer: StreamBuffer<i32> = StreamBuffer::with_capacity(100);
        assert_eq!(buffer.capacity(), 100);
        assert_eq!(buffer.len(), 0);
    }

    #[test]
    fn test_push_slice() {
        let mut buffer = StreamBuffer::new();
        buffer.push_slice(&[1, 2, 3]);
        assert_eq!(buffer.len(), 3);
        buffer.push_slice(&[4, 5]);
        assert_eq!(buffer.len(), 5);
    }

    #[test]
    fn test_consume() {
        let mut buffer = StreamBuffer::new();
        buffer.push_slice(&[1, 2, 3, 4, 5]);
        buffer.consume(2);
        assert_eq!(buffer.len(), 3);
        assert_eq!(buffer.as_slice(), &[3, 4, 5]);
    }

    #[test]
    fn test_consume_exceeds_length() {
        let mut buffer = StreamBuffer::new();
        buffer.push_slice(&[1, 2, 3]);
        buffer.consume(10); // More than available
        assert!(buffer.is_empty());
    }

    #[test]
    fn test_clear() {
        let mut buffer = StreamBuffer::new();
        buffer.push_slice(&[1, 2, 3]);
        buffer.clear();
        assert!(buffer.is_empty());
        assert_eq!(buffer.len(), 0);
    }

    #[test]
    fn test_indexing() {
        let mut buffer = StreamBuffer::new();
        buffer.push_slice(&[10, 20, 30, 40, 50]);
        assert_eq!(buffer[0], 10);
        assert_eq!(buffer[2], 30);
        assert_eq!(buffer[0..3], [10, 20, 30]);
    }

    #[test]
    fn test_as_slice() {
        let mut buffer = StreamBuffer::new();
        buffer.push_slice(&[1, 2, 3]);
        assert_eq!(buffer.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn test_streaming_pattern() {
        // Simulate a typical DSP streaming pattern:
        // - Accumulate samples
        // - Process when enough available
        // - Consume processed samples
        let mut buffer = StreamBuffer::new();

        // Chunk 1: 100 samples arrive
        buffer.push_slice(&vec![1; 100]);
        assert_eq!(buffer.len(), 100);

        // Process groups of 32
        let groups = buffer.len() / 32;
        assert_eq!(groups, 3);

        // Consume 96 samples (3 groups * 32)
        buffer.consume(96);
        assert_eq!(buffer.len(), 4);

        // Chunk 2: 96 more samples arrive
        buffer.push_slice(&vec![2; 96]);
        assert_eq!(buffer.len(), 100);

        // Process again
        let groups = buffer.len() / 32;
        assert_eq!(groups, 3);

        buffer.consume(96);
        assert_eq!(buffer.len(), 4);
    }

    #[test]
    fn test_default() {
        let buffer: StreamBuffer<i32> = StreamBuffer::default();
        assert!(buffer.is_empty());
    }
}
