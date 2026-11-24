//! A GNSS receiver in the street canyon, simulated at the level of its
//! position solutions.
//!
//! Every satellite is traced against the city: in line of sight, received
//! only by a reflection off a facade (NLOS, with the extra path length as a
//! pseudorange bias), or not at all. The single-point error is the weighted
//! least-squares solution's response to those biases and the code noise.
//! RTK fixes with enough line-of-sight satellites; with NLOS satellites in
//! the solution some fixes are wrong, by a few decimetres to metres, for a
//! few epochs at a time, while still reporting centimetre accuracy: the
//! failure a localizer has to catch.

use crate::city::City;
use crate::geometry::Vec2;
use crate::linalg::cholesky_solve;
use crate::rng::{hash01, Rng};

/// Kind of position solution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FixKind {
    /// Code-only single point solution (metres).
    Single,
    /// RTK with float ambiguities (decimetres).
    Float,
    /// RTK with fixed ambiguities (centimetres, when right).
    Fixed,
}

impl FixKind {
    /// Short name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Single => "single",
            Self::Float => "RTK float",
            Self::Fixed => "RTK fixed",
        }
    }
}

/// One position solution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    /// Horizontal position, metres.
    pub pos: Vec2,
    /// The receiver's own horizontal standard deviation per axis, metres.
    pub std_m: f64,
    /// Solution kind.
    pub kind: FixKind,
    /// Satellites in the solution.
    pub satellites: usize,
}

/// A satellite's direction, fixed over the run (a few minutes).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Satellite {
    /// Azimuth, clockwise from north, radians.
    pub az_rad: f64,
    /// Elevation, radians.
    pub el_rad: f64,
}

/// How a satellite reached the antenna in one epoch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Path {
    /// Direct.
    LineOfSight,
    /// Off a facade `reflect_m` away: `bias_m` of extra path.
    Reflected {
        /// Distance to the reflecting facade.
        reflect_m: f64,
        /// Extra path length.
        bias_m: f64,
    },
    /// Not received.
    Blocked,
}

/// One epoch: the solution (if any) and every satellite's path.
#[derive(Debug, Clone, PartialEq)]
pub struct Epoch {
    /// The solution.
    pub fix: Option<Fix>,
    /// Per satellite, in the order of [`Receiver::satellites`].
    pub paths: Vec<Path>,
    /// Whether the fix is a wrong RTK fix (ground truth, for the plots).
    pub wrong_fix: bool,
}

impl Epoch {
    /// Satellites in line of sight.
    pub fn line_of_sight(&self) -> usize {
        self.paths.iter().filter(|p| matches!(p, Path::LineOfSight)).count()
    }
    /// Satellites received only by reflection.
    pub fn reflected(&self) -> usize {
        self.paths
            .iter()
            .filter(|p| matches!(p, Path::Reflected { .. }))
            .count()
    }
}

/// The receiver.
#[derive(Debug, Clone)]
pub struct Receiver {
    /// Satellites above the horizon (GPS, Galileo, QZSS and BeiDou in view
    /// over Tokyo: about 30).
    pub satellites: Vec<Satellite>,
    rng: Rng,
    /// A wrong fix that persists: offset and epochs left.
    wrong: Option<(Vec2, u32)>,
}

/// Reflections from facades farther than this are too weak to track, metres.
const MAX_REFLECTION_M: f64 = 50.0;
/// The receiver excludes a satellite whose normalized residual exceeds this.
const RAIM_THRESHOLD: f64 = 4.0;

/// Elevation mask, radians.
const MASK_RAD: f64 = 10.0 * std::f64::consts::PI / 180.0;

impl Receiver {
    /// A sky of `count` satellites (deterministic for a seed); the noise
    /// stream is `seed` too.
    pub fn new(count: usize, seed: u64) -> Self {
        let mut rng = Rng::new(seed ^ 0x6E55);
        let mut satellites: Vec<Satellite> = (0..count)
            .map(|_| Satellite {
                az_rad: rng.range(0.0, std::f64::consts::TAU),
                // Uniform over the sky's solid angle above the mask.
                el_rad: rng.range(MASK_RAD.sin(), 1.0).asin(),
            })
            .collect();
        // QZSS: one satellite near the zenith over Japan.
        satellites.push(Satellite {
            az_rad: 3.0,
            el_rad: 80f64.to_radians(),
        });
        Self {
            satellites,
            rng: rng.fork(1),
            wrong: None,
        }
    }

    /// Paths from every satellite to an antenna at `p`.
    pub fn paths(&self, city: &City, p: Vec2) -> Vec<Path> {
        self.satellites
            .iter()
            .enumerate()
            .map(|(k, s)| {
                if !city.sky_blocked(p, s.az_rad, s.el_rad) {
                    return Path::LineOfSight;
                }
                // A facade behind the antenna (opposite the satellite)
                // reflects the signal down to it when it is tall enough
                // where the ray meets it. Reflections lose power: only some
                // stay above the receiver's C/N0 mask, the same ones at the
                // same place (by 4 m cell).
                let away = Vec2::new(-s.az_rad.sin(), -s.az_rad.cos());
                let strong = hash01((p.x / 4.0).floor() as i64, (p.y / 4.0).floor() as i64, k as u64 * 7919) < 0.5;
                match city.first_facade(p, away, MAX_REFLECTION_M) {
                    Some((r, b))
                        if strong
                            && city.buildings[b].height_m > crate::city::ANTENNA_HEIGHT_M + r * s.el_rad.tan() =>
                    {
                        Path::Reflected {
                            reflect_m: r,
                            bias_m: 2.0 * r * s.el_rad.cos(),
                        }
                    }
                    _ => Path::Blocked,
                }
            })
            .collect()
    }

    /// One epoch at the true antenna position `truth`.
    pub fn epoch(&mut self, city: &City, truth: Vec2) -> Epoch {
        let paths = self.paths(city, truth);
        let los = paths.iter().filter(|p| matches!(p, Path::LineOfSight)).count();
        let nothing = |paths| Epoch {
            fix: None,
            paths,
            wrong_fix: false,
        };
        let Some(solution) = self.least_squares(&paths) else {
            self.wrong = None;
            return nothing(paths);
        };
        let nlos = solution.reflected_used;
        let mut wrong_fix = false;
        let (error, std_m, kind) = if los >= 7 && nlos <= 2 && self.rng.uniform() < 0.97 - 0.1 * nlos as f64 {
            // RTK fixed. With reflected satellites in the solution the
            // ambiguities are sometimes fixed to wrong integers, and stay
            // wrong for a few epochs.
            if self.wrong.is_none() && nlos > 0 && self.rng.uniform() < 0.04 * nlos as f64 {
                let offset = Vec2::from_angle(self.rng.range(0.0, std::f64::consts::TAU)) * self.rng.range(0.4, 2.5);
                self.wrong = Some((offset, 3 + (self.rng.uniform() * 12.0) as u32));
            }
            let noise = Vec2::new(self.rng.normal(), self.rng.normal()) * 0.015;
            match self.wrong.as_mut() {
                Some((offset, left)) => {
                    let offset = *offset;
                    *left -= 1;
                    if *left == 0 {
                        self.wrong = None;
                    }
                    wrong_fix = true;
                    (offset + noise, 0.02, FixKind::Fixed)
                }
                None => (noise, 0.02, FixKind::Fixed),
            }
        } else if los >= 5 {
            self.wrong = None;
            let noise = Vec2::new(self.rng.normal(), self.rng.normal()) * 0.12;
            (solution.error * 0.3 + noise, 0.3, FixKind::Float)
        } else {
            self.wrong = None;
            (solution.error, solution.sigma_m, FixKind::Single)
        };
        Epoch {
            fix: Some(Fix {
                pos: truth + error,
                std_m,
                kind,
                satellites: solution.used,
            }),
            paths,
            wrong_fix,
        }
    }

    /// A weighted least-squares single-point solution (east, north, up,
    /// clock) from code noise and the reflected satellites' biases, with
    /// the receiver's fault exclusion: the satellite with the largest
    /// normalized residual is dropped while it exceeds [`RAIM_THRESHOLD`]
    /// (up to four). `None` with fewer than five satellites.
    fn least_squares(&mut self, paths: &[Path]) -> Option<Solution> {
        // Rows: geometry, weight, pseudorange error, reflected.
        let mut rows: Vec<([f64; 4], f64, f64, bool)> = Vec::new();
        for (s, path) in self.satellites.iter().zip(paths) {
            let (bias, reflected) = match path {
                Path::LineOfSight => (0.0, false),
                Path::Reflected { bias_m, .. } => (*bias_m, true),
                Path::Blocked => continue,
            };
            let sigma = 0.6 / s.el_rad.sin().max(0.2);
            let row = [
                -s.el_rad.cos() * s.az_rad.sin(),
                -s.el_rad.cos() * s.az_rad.cos(),
                -s.el_rad.sin(),
                1.0,
            ];
            rows.push((row, 1.0 / (sigma * sigma), bias + sigma * self.rng.normal(), reflected));
        }
        for round in 0..=4 {
            if rows.len() < 5 {
                return None;
            }
            let mut normal = [0.0; 16];
            let mut rhs = [0.0; 4];
            for (row, w, e, _) in &rows {
                for i in 0..4 {
                    rhs[i] += w * row[i] * e;
                    for j in 0..4 {
                        normal[i * 4 + j] += w * row[i] * row[j];
                    }
                }
            }
            let mut x = rhs;
            if !cholesky_solve(&mut normal.clone(), &mut x, 4) {
                return None;
            }
            let worst = rows
                .iter()
                .enumerate()
                .map(|(k, (row, w, e, _))| {
                    let v = e - (0..4).map(|i| row[i] * x[i]).sum::<f64>();
                    (k, v.abs() * w.sqrt())
                })
                .max_by(|a, b| a.1.total_cmp(&b.1));
            if let Some((k, v)) = worst {
                if v > RAIM_THRESHOLD && rows.len() > 6 && round < 4 {
                    rows.swap_remove(k);
                    continue;
                }
            }
            // The receiver's horizontal sigma (blind to the biases): east
            // and north variances from the inverse normal matrix.
            let mut var = 0.0;
            for axis in 0..2 {
                let mut e = [0.0; 4];
                e[axis] = 1.0;
                if cholesky_solve(&mut normal.clone(), &mut e, 4) {
                    var += e[axis];
                }
            }
            return Some(Solution {
                error: Vec2::new(x[0], x[1]),
                sigma_m: var.sqrt().max(0.3),
                used: rows.len(),
                reflected_used: rows.iter().filter(|r| r.3).count(),
            });
        }
        None
    }
}

/// A single-point solution's outcome.
#[derive(Debug, Clone, Copy)]
struct Solution {
    error: Vec2,
    sigma_m: f64,
    used: usize,
    reflected_used: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_sky_fixes_and_the_canyon_degrades() {
        let city = City::district(1);
        let mut rx = Receiver::new(30, 2);
        let park = rx.epoch(&city, Vec2::new(320.0, 170.0));
        assert_eq!(park.fix.map(|f| f.kind), Some(FixKind::Fixed));
        let canyon = rx.epoch(&city, Vec2::new(204.0, 170.0));
        assert!(canyon.line_of_sight() < park.line_of_sight());
    }

    #[test]
    fn a_reflection_carries_twice_the_facade_distance() {
        let city = City::district(1);
        let rx = Receiver::new(30, 2);
        for (s, p) in rx.satellites.iter().zip(rx.paths(&city, Vec2::new(204.0, 124.0))) {
            if let Path::Reflected { reflect_m, bias_m } = p {
                assert!((bias_m - 2.0 * reflect_m * s.el_rad.cos()).abs() < 1e-9);
            }
        }
    }
}
