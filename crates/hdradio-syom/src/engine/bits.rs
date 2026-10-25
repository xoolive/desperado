//! MSB-first bit I/O for AAC (ISO/IEC 14496-3 bitstreams).

use super::error::{Error, Result};

/// MSB-first reader. Upcoming bits sit in the high end of `cache`.
#[derive(Clone, Copy)]
pub struct BitReader<'a> {
    data: &'a [u8],
    /// Next unread byte (bytes before this are already in `cache` or consumed).
    byte_pos: usize,
    cache: u64,
    ncache: u32,
}

impl<'a> BitReader<'a> {
    /// Start at bit 0 of `data`.
    #[must_use]
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            byte_pos: 0,
            cache: 0,
            ncache: 0,
        }
    }

    /// Bits already consumed.
    #[must_use]
    pub fn bit_position(&self) -> u64 {
        (self.byte_pos as u64)
            .saturating_mul(8)
            .saturating_sub(u64::from(self.ncache))
    }

    /// Remaining bits in the slice.
    #[must_use]
    pub fn bits_remaining(&self) -> u64 {
        (self.data.len() * 8) as u64 - self.bit_position()
    }

    /// Copy of bits `[start, end)` of the underlying data, MSB-first
    /// (transmission order). Used to re-feed a parsed region to the
    /// LATM `StreamMuxConfig()` CRC.
    #[must_use]
    pub fn bits_range(&self, start: u64, end: u64) -> Vec<bool> {
        let mut out = Vec::with_capacity(end.saturating_sub(start) as usize);
        for pos in start..end {
            let byte = self.data[(pos / 8) as usize];
            out.push(byte & (0x80 >> (pos % 8)) != 0);
        }
        out
    }

    #[inline(always)]
    fn refill(&mut self) {
        if self.ncache > 56 {
            return;
        }
        let rem = self.data.len().saturating_sub(self.byte_pos);
        if rem == 0 {
            return;
        }
        if self.ncache == 0 && rem >= 8 {
            let s = &self.data[self.byte_pos..self.byte_pos + 8];
            self.cache = u64::from_be_bytes([s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]]);
            self.byte_pos += 8;
            self.ncache = 64;
            return;
        }
        while self.ncache <= 56 && self.byte_pos < self.data.len() {
            self.cache |= u64::from(self.data[self.byte_pos]) << (56 - self.ncache);
            self.byte_pos += 1;
            self.ncache += 8;
        }
    }

    /// Next bit, or [`Error::UnexpectedEnd`].
    #[inline(always)]
    pub fn read_bit(&mut self) -> Result<bool> {
        if self.ncache == 0 {
            self.refill();
            if self.ncache == 0 {
                return Err(Error::UnexpectedEnd);
            }
        }
        let b = self.cache & (1u64 << 63) != 0;
        self.cache <<= 1;
        self.ncache -= 1;
        Ok(b)
    }

    /// 12-bit lookahead after refill, or `None` if the stream is short.
    #[inline(always)]
    pub fn try_peek12(&mut self) -> Option<u32> {
        if self.ncache < 12 {
            self.refill();
            if self.ncache < 12 {
                return None;
            }
        }
        Some((self.cache >> 52) as u32)
    }

    /// Consume `n` bits already sitting in the cache (`n <= ncache`).
    #[inline(always)]
    pub fn eat(&mut self, n: u32) {
        self.cache <<= n;
        self.ncache -= n;
    }

    /// Construct a reader starting at `byte_pos`.
    #[cfg(test)]
    #[must_use]
    pub fn with_position(data: &'a [u8], byte_pos: usize) -> Self {
        Self {
            data,
            byte_pos: byte_pos.min(data.len()),
            cache: 0,
            ncache: 0,
        }
    }

    /// Byte index of the next bit.
    #[cfg(test)]
    #[must_use]
    pub fn byte_position(&self) -> usize {
        (self.bit_position() / 8) as usize
    }

    /// True when `bit_position` is a multiple of 8.
    #[cfg(test)]
    #[must_use]
    pub fn is_byte_aligned(&self) -> bool {
        self.bit_position().is_multiple_of(8)
    }

    /// Alias of [`Self::byte_align`].
    #[cfg(test)]
    pub fn align_to_byte(&mut self) -> Result<()> {
        self.byte_align()
    }

    /// Next `n` bits as the low bits of a `u32` (`n` in `0..=32`).
    #[inline(always)]
    pub fn read(&mut self, n: u32) -> Result<u32> {
        if n == 0 {
            return Ok(0);
        }
        if n > 32 {
            return Err(Error::Format("bit read wider than 32"));
        }
        if self.ncache < n {
            self.refill();
            if self.ncache < n {
                return Err(Error::UnexpectedEnd);
            }
        }
        let v = (self.cache >> (64 - n)) as u32;
        self.cache <<= n;
        self.ncache -= n;
        Ok(v)
    }

    /// Alias of [`Self::read`].
    pub fn read_u32(&mut self, n: u32) -> Result<u32> {
        self.read(n)
    }

    /// One-bit unsigned.
    #[cfg(test)]
    pub fn read_u1(&mut self) -> Result<u32> {
        self.read(1)
    }

    /// Signed `n`-bit two's complement.
    #[cfg(test)]
    pub fn read_i32(&mut self, n: u32) -> Result<i32> {
        let u = self.read(n)?;
        if n == 0 {
            return Ok(0);
        }
        let sign = 1u32 << (n - 1);
        if u & sign == 0 {
            Ok(u as i32)
        } else {
            Ok((u as i32) - (1i32 << n))
        }
    }

    /// Up to 64 bits (reads in 32-bit chunks).
    #[cfg(test)]
    pub fn read_u64(&mut self, n: u32) -> Result<u64> {
        if n <= 32 {
            return Ok(u64::from(self.read(n)?));
        }
        let hi = u64::from(self.read(n - 32)?);
        let lo = u64::from(self.read(32)?);
        Ok((hi << 32) | lo)
    }

    /// Look ahead without consuming. May refill the cache.
    #[inline(always)]
    pub fn peek_u32(&mut self, n: u32) -> Result<u32> {
        if n == 0 {
            return Ok(0);
        }
        if n > 32 {
            return Err(Error::Format("bit read wider than 32"));
        }
        if self.ncache < n {
            self.refill();
            if self.ncache < n {
                return Err(Error::UnexpectedEnd);
            }
        }
        Ok((self.cache >> (64 - n)) as u32)
    }

    /// Alias of [`Self::skip`].
    #[cfg(test)]
    pub fn consume(&mut self, n: u32) -> Result<()> {
        self.skip(n)
    }

    /// Skip `n` bits.
    pub fn skip(&mut self, n: u32) -> Result<()> {
        if self.bits_remaining() < u64::from(n) {
            return Err(Error::UnexpectedEnd);
        }
        let mut left = n;
        if left <= self.ncache {
            self.cache <<= left;
            self.ncache -= left;
            return Ok(());
        }
        left -= self.ncache;
        self.cache = 0;
        self.ncache = 0;
        self.byte_pos += (left / 8) as usize;
        left %= 8;
        if left > 0 {
            self.refill();
            self.cache <<= left;
            self.ncache -= left;
        }
        Ok(())
    }

    /// Pad to the next byte boundary (0–7 bits discarded).
    pub fn byte_align(&mut self) -> Result<()> {
        let rem = (self.bit_position() % 8) as u32;
        if rem != 0 {
            self.skip(8 - rem)?;
        }
        Ok(())
    }
}

/// MSB-first writer used to build LC / SBR test frames.
#[derive(Default)]
pub struct BitWriter {
    buf: Vec<u8>,
    acc: u8,
    bits: u8,
}

impl BitWriter {
    /// Empty writer.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Append the low `n` bits of `value`, MSB first.
    pub fn write(&mut self, value: u32, n: u32) {
        if n == 0 {
            return;
        }
        for i in (0..n).rev() {
            let bit = ((value >> i) & 1) as u8;
            self.acc = (self.acc << 1) | bit;
            self.bits += 1;
            if self.bits == 8 {
                self.buf.push(self.acc);
                self.acc = 0;
                self.bits = 0;
            }
        }
    }

    /// Alias of [`Self::write`].
    pub fn write_u32(&mut self, value: u32, n: u32) {
        self.write(value, n)
    }

    /// Append one bit.
    pub fn write_bit(&mut self, bit: bool) {
        self.write(u32::from(bit), 1);
    }

    /// Bits written so far (before the final byte-align pad).
    #[cfg(test)]
    #[must_use]
    pub fn bit_len(&self) -> u64 {
        (self.buf.len() as u64) * 8 + u64::from(self.bits)
    }

    /// Pad with zeros to a byte boundary and return the bytes.
    #[must_use]
    pub fn finish(mut self) -> Vec<u8> {
        if self.bits > 0 {
            self.acc <<= 8 - self.bits;
            self.buf.push(self.acc);
        }
        self.buf
    }
}

#[cfg(test)]
#[path = "bits_tests.rs"]
mod bits_tests;
