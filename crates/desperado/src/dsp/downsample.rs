/// Stateless-by-design integer downsampler that emits every N-th sample.
///
/// This is useful when anti-aliasing filtering is already applied upstream.
#[derive(Debug, Clone)]
pub struct EveryN {
    factor: u32,
    counter: u32,
}

impl EveryN {
    pub fn new(factor: u32) -> Self {
        assert!(factor > 0, "downsample factor must be > 0");
        Self { factor, counter: 0 }
    }

    /// Returns true when the current sample should be kept.
    #[inline]
    pub fn keep(&mut self) -> bool {
        self.counter += 1;
        if self.counter < self.factor {
            return false;
        }
        self.counter = 0;
        true
    }

    pub fn reset(&mut self) {
        self.counter = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emits_every_nth() {
        let mut d = EveryN::new(3);
        let keeps: Vec<bool> = (0..9).map(|_| d.keep()).collect();
        assert_eq!(
            keeps,
            vec![false, false, true, false, false, true, false, false, true]
        );
    }
}
