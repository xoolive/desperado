//! Generic Morse code parsing for radio navigation identifiers.
//!
//! This module provides reusable Morse code decoding functionality for extracting
//! 3-letter identifiers (VOR, NDB, ILS, DME, etc.) from modulated audio signals.
//!
//! The algorithm:
//! 1. Converts envelope signal to bit stream (threshold-based detection)
//! 2. Extracts run lengths to identify dots (short) vs dashes (long)
//! 3. Decodes Morse symbols to characters
//! 4. Extracts 3-letter sliding windows with minimum complexity
//! 5. Clusters near-matches (Hamming distance ≤ 1) and votes for consensus
//!
//! This approach handles noisy conditions by trying multiple thresholds and
//! clustering similar results rather than relying on a single decoding.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Decode Morse ident from an audio signal envelope.
///
/// # Arguments
///
/// * `env` - Signal envelope (magnitude over time)
/// * `sample_rate` - Sample rate in Hz
///
/// # Returns
///
/// Tuple of (best_ident, all_tokens, decode_attempts)
/// - `best_ident`: Most likely 3-letter identifier (or None if no valid result)
/// - `all_tokens`: All extracted 3-letter tokens from all thresholds
/// - `decode_attempts`: Metadata about each threshold attempt (for debugging)
pub fn decode_morse_ident(
    env: &[f64],
    sample_rate: f64,
) -> (Option<String>, Vec<String>, Vec<MorseDecodeAttempt>) {
    if env.is_empty() {
        return (None, Vec::new(), Vec::new());
    }

    let (all_tokens, attempts) = decode_tokens_from_env(env, sample_rate);
    let ident = pick_best_ident(&all_tokens);
    (ident, all_tokens, attempts)
}

fn decode_tokens_from_env(env: &[f64], sample_rate: f64) -> (Vec<String>, Vec<MorseDecodeAttempt>) {
    let mean = env.iter().sum::<f64>() / env.len() as f64;
    let var = env
        .iter()
        .map(|x| {
            let d = x - mean;
            d * d
        })
        .sum::<f64>()
        / env.len() as f64;
    let std = var.sqrt();

    let mut out = Vec::new();
    let mut attempts = Vec::new();

    for k in [0.10, 0.20, 0.30, 0.40] {
        let thr = mean + k * std;
        let bits: Vec<bool> = env.iter().map(|&x| x > thr).collect();

        let on_ratio = bits.iter().filter(|&&b| b).count() as f64 / bits.len() as f64;

        if let Some(tokens) = decode_tokens_from_bits(&bits) {
            let runs = run_lengths(&bits);
            let on_durations: Vec<usize> = runs
                .iter()
                .filter_map(|(on, n)| if *on { Some(*n) } else { None })
                .collect();

            let dot_samples = estimate_dot_len(&on_durations);
            let dot_ms = dot_samples.map(|d| d * 1000.0 / sample_rate);

            attempts.push(MorseDecodeAttempt {
                threshold_k: k,
                on_ratio,
                dot_ms,
                tokens_found: tokens.len(),
                sample_tokens: tokens.iter().take(5).cloned().collect(),
            });

            out.extend(tokens);
        } else {
            attempts.push(MorseDecodeAttempt {
                threshold_k: k,
                on_ratio,
                dot_ms: None,
                tokens_found: 0,
                sample_tokens: Vec::new(),
            });
        }
    }
    (out, attempts)
}

fn decode_tokens_from_bits(bits: &[bool]) -> Option<Vec<String>> {
    let on_ratio = bits.iter().filter(|&&b| b).count() as f64 / bits.len() as f64;
    if !(0.25..0.55).contains(&on_ratio) {
        return None;
    }

    let runs = run_lengths(bits);
    if runs.is_empty() {
        return None;
    }

    let on_durations: Vec<usize> = runs
        .iter()
        .filter_map(|(on, n)| if *on { Some(*n) } else { None })
        .collect();
    if on_durations.is_empty() {
        return None;
    }

    let dot = estimate_dot_len(&on_durations)?;
    let mut symbols = Vec::<String>::new();
    let mut cur = String::new();

    for (on, n) in runs {
        if on {
            if (n as f64) <= 2.2 * dot {
                cur.push('.');
            } else {
                cur.push('-');
            }
        } else {
            let nf = n as f64;
            if nf >= 6.0 * dot {
                if !cur.is_empty() {
                    symbols.push(cur.clone());
                    cur.clear();
                }
                symbols.push("/".to_string());
            } else if nf >= 2.5 * dot && !cur.is_empty() {
                symbols.push(cur.clone());
                cur.clear();
            }
        }
    }
    if !cur.is_empty() {
        symbols.push(cur);
    }

    let has_dash = symbols.iter().any(|s| s.contains('-'));
    if !has_dash {
        return None;
    }

    // Build contiguous letter spans split by word boundaries.
    let mut spans: Vec<Vec<(char, bool)>> = Vec::new();
    let mut current: Vec<(char, bool)> = Vec::new();
    for s in symbols {
        if s == "/" {
            if !current.is_empty() {
                spans.push(current);
                current = Vec::new();
            }
            continue;
        }

        if let Some(ch) = morse_to_char(&s) {
            current.push((ch, s.len() >= 3));
        }
    }
    if !current.is_empty() {
        spans.push(current);
    }

    // Extract 3-letter sliding windows with at least two "complex" letters.
    let mut tokens = Vec::<String>::new();
    for span in spans {
        if span.len() < 3 {
            continue;
        }
        for i in 0..=(span.len() - 3) {
            let win = &span[i..i + 3];
            let complex_letters = win.iter().filter(|(_, c)| *c).count();
            let last_is_complex = win[2].1;
            if complex_letters < 2 || !last_is_complex {
                continue;
            }
            let mut t = String::with_capacity(3);
            for (ch, _) in win {
                t.push(*ch);
            }
            tokens.push(t);
        }
    }

    if tokens.is_empty() {
        None
    } else {
        Some(tokens)
    }
}

pub fn run_lengths(bits: &[bool]) -> Vec<(bool, usize)> {
    if bits.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut cur = bits[0];
    let mut n = 1usize;
    for &b in &bits[1..] {
        if b == cur {
            n += 1;
        } else {
            out.push((cur, n));
            cur = b;
            n = 1;
        }
    }
    out.push((cur, n));
    out
}

fn estimate_dot_len(on_durations: &[usize]) -> Option<f64> {
    let mut v: Vec<usize> = on_durations.iter().copied().filter(|&x| x > 1).collect();
    if v.is_empty() {
        return None;
    }
    v.sort_unstable();
    let half = (v.len() / 2).max(1);
    let short = &v[..half];
    Some(short.iter().sum::<usize>() as f64 / short.len() as f64)
}

fn pick_best_ident(tokens: &[String]) -> Option<String> {
    let mut counts: HashMap<String, usize> = HashMap::new();
    for token in tokens {
        let t = token.to_uppercase();
        // Only accept 3-letter tokens with alphabetic characters (A-Z)
        if t.len() == 3 && t.chars().all(|c| c.is_ascii_alphabetic()) {
            *counts.entry(t).or_insert(0) += 1;
        }
    }

    if counts.is_empty() {
        return None;
    }

    // Cluster near-matches (Hamming distance <= 1), e.g. KLO vs KLT.
    let tokens: Vec<String> = counts.keys().cloned().collect();
    let mut best_center: Option<String> = None;
    let mut best_score = 0usize;
    for center in &tokens {
        let mut score = 0usize;
        for t in &tokens {
            if hamming3(center, t) <= 1 {
                score += counts.get(t).copied().unwrap_or(0);
            }
        }
        if score > best_score {
            best_score = score;
            best_center = Some(center.clone());
        }
    }

    let center = best_center?;
    let cluster: Vec<(String, usize)> = tokens
        .into_iter()
        .filter_map(|t| {
            if hamming3(&center, &t) <= 1 {
                Some((t.clone(), counts.get(&t).copied().unwrap_or(0)))
            } else {
                None
            }
        })
        .collect();

    consensus3(&cluster)
}

fn hamming3(a: &str, b: &str) -> usize {
    a.chars().zip(b.chars()).filter(|(x, y)| x != y).count()
}

fn consensus3(cluster: &[(String, usize)]) -> Option<String> {
    if cluster.is_empty() {
        return None;
    }

    let mut out = String::with_capacity(3);
    for pos in 0..3 {
        let mut hist: HashMap<char, usize> = HashMap::new();
        for (token, w) in cluster {
            if let Some(ch) = token.chars().nth(pos) {
                *hist.entry(ch).or_insert(0) += *w;
            }
        }
        let best = hist
            .into_iter()
            .max_by_key(|(ch, w)| (*w, std::cmp::Reverse(*ch)))
            .map(|(ch, _)| ch)?;
        out.push(best);
    }

    Some(out)
}

fn morse_to_char(code: &str) -> Option<char> {
    Some(match code {
        ".-" => 'A',
        "-..." => 'B',
        "-.-." => 'C',
        "-.." => 'D',
        "." => 'E',
        "..-." => 'F',
        "--." => 'G',
        "...." => 'H',
        ".." => 'I',
        ".---" => 'J',
        "-.-" => 'K',
        ".-.." => 'L',
        "--" => 'M',
        "-." => 'N',
        "---" => 'O',
        ".--." => 'P',
        "--.-" => 'Q',
        ".-." => 'R',
        "..." => 'S',
        "-" => 'T',
        "..-" => 'U',
        "...-" => 'V',
        ".--" => 'W',
        "-..-" => 'X',
        "-.--" => 'Y',
        "--.." => 'Z',
        "-----" => '0',
        ".----" => '1',
        "..---" => '2',
        "...--" => '3',
        "....-" => '4',
        "....." => '5',
        "-...." => '6',
        "--..." => '7',
        "---.." => '8',
        "----." => '9',
        _ => return None,
    })
}

/// Metadata about a single Morse decode attempt.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MorseDecodeAttempt {
    pub threshold_k: f64,
    pub on_ratio: f64,
    pub dot_ms: Option<f64>,
    pub tokens_found: usize,
    pub sample_tokens: Vec<String>,
}
