//! A small 2.5D city: box buildings with heights along a grid of streets, a
//! route through it, and the people on its sidewalks.
//!
//! The layout follows a Tokyo business district in miniature: low-rise
//! blocks in the west, a block of towers in the middle (with a narrow alley
//! between them), mid-rise blocks in the east and an open park. The route
//! starts in the park, runs down the tower canyon, up the alley, along the
//! low-rise street and back, so every method meets open sky, deep canyons and
//! the long featureless street walls that LiDAR cannot pin along their
//! length.

use crate::geometry::{Pose2, Vec2};
use crate::rng::Rng;

/// Height of the GNSS antenna above the street, metres.
pub const ANTENNA_HEIGHT_M: f64 = 1.5;

/// A box building: footprint and height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Building {
    /// South-west corner of the footprint.
    pub min: Vec2,
    /// North-east corner of the footprint.
    pub max: Vec2,
    /// Roof height above the street, metres.
    pub height_m: f64,
}

impl Building {
    /// The footprint's four edges, counterclockwise.
    pub fn edges(&self) -> [(Vec2, Vec2); 4] {
        let (a, b) = (self.min, self.max);
        let (c0, c1, c2, c3) = (a, Vec2::new(b.x, a.y), b, Vec2::new(a.x, b.y));
        [(c0, c1), (c1, c2), (c2, c3), (c3, c0)]
    }

    /// Where a ray from `from` along unit `dir` enters and leaves the
    /// footprint (`t` in metres), if it does at a positive distance.
    pub fn ray(&self, from: Vec2, dir: Vec2) -> Option<(f64, f64)> {
        let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
        for (o, d, lo, hi) in [
            (from.x, dir.x, self.min.x, self.max.x),
            (from.y, dir.y, self.min.y, self.max.y),
        ] {
            if d.abs() < 1e-12 {
                if o < lo || o > hi {
                    return None;
                }
            } else {
                let (a, b) = ((lo - o) / d, (hi - o) / d);
                t0 = t0.max(a.min(b));
                t1 = t1.min(a.max(b));
            }
        }
        (t1 >= t0.max(0.0)).then_some((t0.max(0.0), t1))
    }

    /// Whether a point lies inside the footprint.
    pub fn contains(&self, p: Vec2) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }
}

/// The city.
#[derive(Debug, Clone)]
pub struct City {
    /// Buildings.
    pub buildings: Vec<Building>,
    /// The route's waypoints (a closed loop: the last returns to the first).
    pub route: Vec<Vec2>,
    /// South-west corner of the area.
    pub min: Vec2,
    /// North-east corner of the area.
    pub max: Vec2,
}

impl City {
    /// The miniature district (deterministic for a seed).
    pub fn district(seed: u64) -> Self {
        let mut rng = Rng::new(seed ^ 0xC17E);
        let xs = [(32.0, 128.0), (152.0, 248.0), (272.0, 368.0)];
        let ys = [(32.0, 108.0), (132.0, 208.0), (232.0, 288.0)];
        let mut buildings = Vec::new();
        for (bi, &(x0, x1)) in xs.iter().enumerate() {
            for (bj, &(y0, y1)) in ys.iter().enumerate() {
                if bi == 2 && bj == 1 {
                    continue; // the park
                }
                let height = |rng: &mut Rng| match bi {
                    0 => rng.range(8.0, 26.0),
                    1 => rng.range(90.0, 220.0),
                    _ => rng.range(28.0, 60.0),
                };
                // The tower block north of the canyon has an alley through it.
                let (cx, gap) = if bi == 1 && bj == 1 {
                    (204.0, 6.0)
                } else {
                    ((x0 + x1) * 0.5, 2.0)
                };
                let cy = (y0 + y1) * 0.5;
                for (lx0, lx1) in [(x0, cx - gap), (cx + gap, x1)] {
                    for (ly0, ly1) in [(y0, cy - 2.0), (cy + 2.0, y1)] {
                        // Setbacks: low-rise lots sit on the building line,
                        // towers stand back behind plazas some of the time.
                        let back = |rng: &mut Rng| if rng.uniform() < 0.6 { 0.0 } else { rng.range(1.0, 5.0) };
                        let b = Building {
                            min: Vec2::new(lx0 + back(&mut rng), ly0 + back(&mut rng)),
                            max: Vec2::new(lx1 - back(&mut rng), ly1 - back(&mut rng)),
                            height_m: height(&mut rng),
                        };
                        buildings.push(b);
                    }
                }
            }
        }
        let route = [
            (320.0, 170.0),
            (320.0, 124.0),
            (204.0, 124.0),
            (204.0, 224.0),
            (144.0, 224.0),
            (144.0, 24.0),
            (264.0, 24.0),
            (264.0, 170.0),
        ]
        .iter()
        .map(|&(x, y)| Vec2::new(x, y))
        .collect();
        Self {
            buildings,
            route,
            min: Vec2::new(0.0, 0.0),
            max: Vec2::new(400.0, 300.0),
        }
    }

    /// The map as surveyed a while ago: every footprint off by a few
    /// centimetres to decimetres (the survey's error), two buildings built
    /// since missing from it, and a kiosk and a fence removed since still in
    /// it. Returns the map's buildings.
    pub fn outdated_map(&self, seed: u64) -> Vec<Building> {
        let mut rng = Rng::new(seed ^ 0x01D_3A9);
        let mut map: Vec<Building> = self
            .buildings
            .iter()
            .map(|b| {
                let shift = Vec2::new(0.12 * rng.normal(), 0.12 * rng.normal());
                Building {
                    min: b.min + shift,
                    max: b.max + shift,
                    height_m: b.height_m,
                }
            })
            .collect();
        // Built since the survey: drop two from the map.
        for _ in 0..2 {
            let k = (rng.uniform() * map.len() as f64) as usize;
            map.swap_remove(k);
        }
        // Removed since: a kiosk and a fence the street no longer has.
        for &(x, y, w, h) in &[(214.0, 116.0, 6.0, 3.0), (150.0, 160.0, 1.5, 12.0)] {
            map.push(Building {
                min: Vec2::new(x, y),
                max: Vec2::new(x + w, y + h),
                height_m: 3.0,
            });
        }
        map
    }

    /// A long straight road: two 600 m lanes either side of a median
    /// barrier, between noise barriers, looped at the ends. The walls are
    /// low (the sky is open) and straight: the LiDAR pins the robot across
    /// the road but not along it, except near the ends.
    pub fn boulevard() -> Self {
        let wall = |x0: f64, y0: f64, x1: f64, y1: f64, h: f64| Building {
            min: Vec2::new(x0, y0),
            max: Vec2::new(x1, y1),
            height_m: h,
        };
        let buildings = vec![
            wall(-10.0, -1.0, 650.0, 0.0, 6.0),
            wall(-10.0, 44.0, 650.0, 45.0, 6.0),
            wall(40.0, 21.7, 600.0, 22.3, 1.2),
            wall(-11.0, -1.0, -10.0, 45.0, 6.0),
            wall(650.0, -1.0, 651.0, 45.0, 6.0),
        ];
        let route = [(20.0, 11.0), (620.0, 11.0), (620.0, 33.0), (20.0, 33.0)]
            .iter()
            .map(|&(x, y)| Vec2::new(x, y))
            .collect();
        Self {
            buildings,
            route,
            min: Vec2::new(-20.0, -10.0),
            max: Vec2::new(660.0, 55.0),
        }
    }

    /// The map without the buildings in a rectangle (`lo`, `hi`): a part of
    /// the city nobody has mapped yet.
    pub fn map_without(&self, lo: Vec2, hi: Vec2) -> Vec<Building> {
        self.buildings
            .iter()
            .filter(|b| b.max.x < lo.x || b.min.x > hi.x || b.max.y < lo.y || b.min.y > hi.y)
            .copied()
            .collect()
    }

    /// The map from before a block was rebuilt: the buildings in a
    /// rectangle (`lo`, `hi`) stand `shift` from where the map has them.
    pub fn rebuilt_map(&self, lo: Vec2, hi: Vec2, shift: Vec2) -> Vec<Building> {
        self.buildings
            .iter()
            .map(|&b| {
                let inside = b.min.x >= lo.x && b.max.x <= hi.x && b.min.y >= lo.y && b.max.y <= hi.y;
                if inside {
                    Building {
                        min: b.min - shift,
                        max: b.max - shift,
                        ..b
                    }
                } else {
                    b
                }
            })
            .collect()
    }

    /// Footprint edges of every building: the LiDAR's prior map.
    pub fn map_segments(&self) -> Vec<(Vec2, Vec2)> {
        self.buildings.iter().flat_map(|b| b.edges()).collect()
    }

    /// Distance along unit `dir` from `from` to the first facade, up to `max_m`.
    pub fn first_facade(&self, from: Vec2, dir: Vec2, max_m: f64) -> Option<(f64, usize)> {
        let mut best: Option<(f64, usize)> = None;
        for (k, b) in self.buildings.iter().enumerate() {
            if let Some((t, _)) = b.ray(from, dir) {
                if t <= max_m && best.is_none_or(|(bt, _)| t < bt) {
                    best = Some((t, k));
                }
            }
        }
        best
    }

    /// Whether the sky at azimuth `az_rad` (clockwise from north) and
    /// elevation `el_rad` is hidden by a building, seen from `p` at antenna
    /// height. A rising ray is lowest where it enters a footprint, so a
    /// building blocks it when its roof is higher there.
    pub fn sky_blocked(&self, p: Vec2, az_rad: f64, el_rad: f64) -> bool {
        let dir = Vec2::new(az_rad.sin(), az_rad.cos());
        let slope = el_rad.tan();
        self.buildings.iter().any(|b| {
            b.ray(p, dir)
                .is_some_and(|(t, _)| b.height_m > ANTENNA_HEIGHT_M + t * slope)
        })
    }

    /// Total route length, metres.
    pub fn route_length_m(&self) -> f64 {
        let n = self.route.len();
        (0..n).map(|i| self.route[i].dist(self.route[(i + 1) % n])).sum()
    }

    /// The pose at arc length `s` along the closed route.
    pub fn route_pose(&self, s: f64) -> Pose2 {
        let n = self.route.len();
        let mut s = s.rem_euclid(self.route_length_m());
        for i in 0..n {
            let (a, b) = (self.route[i], self.route[(i + 1) % n]);
            let len = a.dist(b);
            if s <= len {
                let p = a.lerp(b, s / len);
                return Pose2::new(p.x, p.y, (b.y - a.y).atan2(b.x - a.x));
            }
            s -= len;
        }
        let p = self.route[0];
        Pose2::new(p.x, p.y, 0.0)
    }
}

/// People walking the sidewalks: LiDAR returns that are not in the map.
#[derive(Debug, Clone)]
pub struct Crowd {
    walkers: Vec<Walker>,
}

#[derive(Debug, Clone, Copy)]
struct Walker {
    /// Arc length along the route where the walker's stretch starts.
    start_m: f64,
    /// Length of the stretch walked back and forth.
    stretch_m: f64,
    /// Offset to the left of the route, metres.
    side_m: f64,
    speed_m_s: f64,
    phase: f64,
}

impl Crowd {
    /// `count` people spread along the city's route; a quarter of them walk
    /// close to the route, the rest along the building lines.
    pub fn new(city: &City, count: usize, seed: u64) -> Self {
        let mut rng = Rng::new(seed ^ 0xC80D);
        let length = city.route_length_m();
        let walkers = (0..count)
            .map(|k| Walker {
                start_m: rng.range(0.0, length),
                stretch_m: rng.range(20.0, 80.0),
                side_m: if k % 4 == 0 {
                    rng.range(-2.5, 2.5)
                } else {
                    let s = rng.range(4.0, 9.0);
                    if rng.uniform() < 0.5 {
                        s
                    } else {
                        -s
                    }
                },
                speed_m_s: rng.range(0.8, 1.6),
                phase: rng.range(0.0, 1.0),
            })
            .collect();
        Self { walkers }
    }

    /// Nobody.
    pub fn empty() -> Self {
        Self { walkers: Vec::new() }
    }

    /// Positions at time `t_s`.
    pub fn at(&self, city: &City, t_s: f64) -> Vec<Vec2> {
        self.walkers
            .iter()
            .map(|w| {
                let cycle = 2.0 * w.stretch_m;
                let u = (w.phase * cycle + w.speed_m_s * t_s).rem_euclid(cycle);
                let s = w.start_m + if u < w.stretch_m { u } else { cycle - u };
                let pose = city.route_pose(s);
                pose.transform(Vec2::new(0.0, w.side_m))
            })
            .filter(|p| !city.buildings.iter().any(|b| b.contains(*p)))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_route_runs_along_streets() {
        let city = City::district(1);
        let len = city.route_length_m();
        for k in 0..400 {
            let p = city.route_pose(len * k as f64 / 400.0).pos();
            assert!(!city.buildings.iter().any(|b| b.contains(p)), "{p:?}");
        }
    }

    #[test]
    fn towers_hide_low_satellites_and_the_park_sees_them() {
        let city = City::district(1);
        // In the tower canyon, looking north at 30 degrees.
        assert!(city.sky_blocked(Vec2::new(204.0, 124.0), 0.3, 30f64.to_radians()));
        // In the middle of the park the zenith is always clear.
        assert!(!city.sky_blocked(Vec2::new(320.0, 170.0), 0.0, 85f64.to_radians()));
    }

    #[test]
    fn a_ray_enters_a_box_where_it_should() {
        let b = Building {
            min: Vec2::new(10.0, -1.0),
            max: Vec2::new(12.0, 1.0),
            height_m: 5.0,
        };
        let (t0, t1) = b.ray(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap();
        assert!((t0 - 10.0).abs() < 1e-12 && (t1 - 12.0).abs() < 1e-12);
        assert!(b.ray(Vec2::new(0.0, 0.0), Vec2::new(-1.0, 0.0)).is_none());
    }
}
