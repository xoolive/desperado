//! Optional RDS-TMC / GLR location-table lookup.
//!
//! Tables are licensed separately. This crate accepts a simple CSV:
//!
//! ```text
//! location_code,latitude,longitude,name
//! 12345,59.9139,10.7522,Oslo
//! ```
//!
//! or a directory containing `points.csv` / `POINTS.CSV` in that layout.
//! Without a table, GeoJSON `geometry` is left null and the location code
//! is still reported in properties.

use crate::geojson::Geometry;
use std::collections::HashMap;
use std::fs;
use std::io::{self, BufRead};
use std::path::Path;

/// One resolved TMC location.
#[derive(Debug, Clone)]
pub struct Location {
    pub code: u16,
    pub lat: f64,
    pub lon: f64,
    pub name: Option<String>,
}

/// In-memory location table keyed by TMC location code.
#[derive(Debug, Default, Clone)]
pub struct LocationTable {
    pub table_number: Option<u8>,
    points: HashMap<u16, Location>,
}

impl LocationTable {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&mut self, loc: Location) {
        self.points.insert(loc.code, loc);
    }

    pub fn get(&self, code: u16) -> Option<&Location> {
        self.points.get(&code)
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Resolve a location code to a GeoJSON Point, if present.
    pub fn geometry(&self, code: u16) -> Option<Geometry> {
        self.get(code).map(|loc| Geometry::Point {
            coordinates: [loc.lon, loc.lat],
        })
    }

    /// Load a CSV file (`location_code,latitude,longitude[,name]`).
    pub fn load_csv(path: &Path) -> io::Result<Self> {
        let file = fs::File::open(path)?;
        let reader = io::BufReader::new(file);
        let mut table = Self::new();
        for (i, line) in reader.lines().enumerate() {
            let line = line?;
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if i == 0 && line.to_ascii_lowercase().contains("location") {
                continue;
            }
            let mut parts = line.split(',');
            let code: u16 = match parts.next().and_then(|s| s.trim().parse().ok()) {
                Some(c) => c,
                None => continue,
            };
            let lat: f64 = match parts.next().and_then(|s| s.trim().parse().ok()) {
                Some(v) => v,
                None => continue,
            };
            let lon: f64 = match parts.next().and_then(|s| s.trim().parse().ok()) {
                Some(v) => v,
                None => continue,
            };
            let name = parts
                .next()
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty());
            table.insert(Location {
                code,
                lat,
                lon,
                name,
            });
        }
        Ok(table)
    }

    /// Load from a file or a directory containing `points.csv`.
    pub fn load(path: &Path) -> io::Result<Self> {
        if path.is_dir() {
            for name in ["points.csv", "POINTS.CSV", "points.CSV"] {
                let candidate = path.join(name);
                if candidate.is_file() {
                    return Self::load_csv(&candidate);
                }
            }
            Err(io::Error::new(
                io::ErrorKind::NotFound,
                "no points.csv in location-table directory",
            ))
        } else {
            Self::load_csv(path)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookup_returns_point_geometry() {
        let mut table = LocationTable::new();
        table.insert(Location {
            code: 42,
            lat: 59.9,
            lon: 10.7,
            name: Some("test".into()),
        });
        match table.geometry(42) {
            Some(Geometry::Point { coordinates }) => {
                assert!((coordinates[0] - 10.7).abs() < 1e-9);
                assert!((coordinates[1] - 59.9).abs() < 1e-9);
            }
            _ => panic!("expected point"),
        }
        assert!(table.geometry(1).is_none());
    }
}
