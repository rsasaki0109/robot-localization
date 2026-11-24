//! A planar LiDAR in the city and a scan-to-map matcher.
//!
//! The scan is ray traced against the building footprints and the people
//! (who are not in the map). The matcher aligns the scan's points to the
//! map's facade segments by Huber-weighted point-to-line Gauss-Newton and
//! reports the covariance from the inverse information: in a street with
//! two straight walls the scan pins the robot across the street but hardly
//! along it, and the covariance says so.

use crate::city::City;
use crate::geometry::{normalize_angle, point_segment, Pose2, Vec2};
use crate::linalg::{inverse3, Mat3};
use crate::rng::Rng;

/// Scanner settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarConfig {
    /// Beams over the full circle.
    pub beams: usize,
    /// Maximum range, metres.
    pub max_range_m: f64,
    /// Range noise (1 sigma), metres.
    pub noise_m: f64,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            beams: 180,
            max_range_m: 40.0,
            noise_m: 0.03,
        }
    }
}

/// Radius of a person as the LiDAR sees them, metres.
pub const PERSON_RADIUS_M: f64 = 0.3;

/// Scan endpoints in the sensor frame (beams without a return are left out).
pub fn scan(city: &City, people: &[Vec2], pose: Pose2, config: &LidarConfig, rng: &mut Rng) -> Vec<Vec2> {
    let mut points = Vec::with_capacity(config.beams);
    for k in 0..config.beams {
        let angle = std::f64::consts::TAU * k as f64 / config.beams as f64;
        let dir = Vec2::from_angle(pose.theta_rad + angle);
        let mut range = city
            .first_facade(pose.pos(), dir, config.max_range_m)
            .map_or(f64::INFINITY, |(t, _)| t);
        for &q in people {
            // Ray against a circle.
            let d = q - pose.pos();
            let along = d.dot(dir);
            let off2 = d.dot(d) - along * along;
            let r2 = PERSON_RADIUS_M * PERSON_RADIUS_M;
            if along > 0.0 && off2 < r2 {
                range = range.min(along - (r2 - off2).sqrt());
            }
        }
        if range < config.max_range_m {
            let r = range + config.noise_m * rng.normal();
            points.push(Vec2::from_angle(angle) * r);
        }
    }
    points
}

/// A scan matched to the map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanMatch {
    /// Matched pose.
    pub pose: Pose2,
    /// Covariance of (x, y, theta).
    pub covariance: Mat3,
    /// Of the scan's points within reach of a mapped facade (1.5 m), the
    /// share within 0.3 m of one. People in the open street are not near any
    /// facade and do not count either way.
    pub inlier_fraction: f64,
}

/// The prior map: facade segments with a grid index.
#[derive(Debug, Clone)]
pub struct MapMatcher {
    segments: Vec<(Vec2, Vec2)>,
    origin: Vec2,
    cell_m: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

/// Correspondences farther than this are ignored, metres.
const MAX_CORRESPONDENCE_M: f64 = 1.5;
/// The search narrows to this as the pose converges, metres.
const MIN_CORRESPONDENCE_M: f64 = 0.3;
/// Huber threshold, metres.
const HUBER_M: f64 = 0.1;
/// Floor on the per-point standard deviation for the covariance, metres.
const POINT_SIGMA_M: f64 = 0.05;

impl MapMatcher {
    /// Indexes the city's facades.
    pub fn new(city: &City) -> Self {
        Self::from_buildings(city, &city.buildings)
    }

    /// Indexes `buildings` (a map that may differ from the city) over the
    /// city's area.
    pub fn from_buildings(city: &City, buildings: &[crate::city::Building]) -> Self {
        let segments: Vec<(Vec2, Vec2)> = buildings.iter().flat_map(|b| b.edges()).collect();
        let cell_m = 2.0;
        let origin = city.min - Vec2::new(50.0, 50.0);
        let span = city.max - origin + Vec2::new(50.0, 50.0);
        let (nx, ny) = ((span.x / cell_m) as usize + 1, (span.y / cell_m) as usize + 1);
        let mut cells = vec![Vec::new(); nx * ny];
        for (k, &(a, b)) in segments.iter().enumerate() {
            // Every cell within reach of the segment.
            let lo = Vec2::new(a.x.min(b.x), a.y.min(b.y)) - Vec2::new(MAX_CORRESPONDENCE_M, MAX_CORRESPONDENCE_M);
            let hi = Vec2::new(a.x.max(b.x), a.y.max(b.y)) + Vec2::new(MAX_CORRESPONDENCE_M, MAX_CORRESPONDENCE_M);
            let (i0, j0) = (
                ((lo.x - origin.x) / cell_m) as usize,
                ((lo.y - origin.y) / cell_m) as usize,
            );
            let (i1, j1) = (
                ((hi.x - origin.x) / cell_m) as usize,
                ((hi.y - origin.y) / cell_m) as usize,
            );
            for j in j0..=j1.min(ny - 1) {
                for i in i0..=i1.min(nx - 1) {
                    let c = origin + Vec2::new((i as f64 + 0.5) * cell_m, (j as f64 + 0.5) * cell_m);
                    if point_segment(c, a, b).0 <= MAX_CORRESPONDENCE_M + cell_m {
                        cells[j * nx + i].push(k as u32);
                    }
                }
            }
        }
        Self {
            segments,
            origin,
            cell_m,
            nx,
            ny,
            cells,
        }
    }

    /// The nearest facade point to `p` within reach, with its distance.
    fn nearest(&self, p: Vec2) -> Option<(f64, Vec2, Vec2)> {
        self.nearest_within(p, MAX_CORRESPONDENCE_M)
    }

    /// The nearest facade point to `p` within `reach_m` (at most
    /// [`MAX_CORRESPONDENCE_M`]), with its distance and the facade normal.
    fn nearest_within(&self, p: Vec2, reach_m: f64) -> Option<(f64, Vec2, Vec2)> {
        let i = ((p.x - self.origin.x) / self.cell_m).floor();
        let j = ((p.y - self.origin.y) / self.cell_m).floor();
        if i < 0.0 || j < 0.0 || i as usize >= self.nx || j as usize >= self.ny {
            return None;
        }
        let mut best: Option<(f64, Vec2, Vec2)> = None;
        for &k in &self.cells[j as usize * self.nx + i as usize] {
            let (a, b) = self.segments[k as usize];
            let (d, q) = point_segment(p, a, b);
            if d <= reach_m && best.is_none_or(|(bd, _, _)| d < bd) {
                // The segment's normal.
                let n = Vec2::new(-(b - a).y, (b - a).x).normalized();
                best = Some((d, q, n));
            }
        }
        best
    }

    /// Aligns `points` (sensor frame) to the map from `prior`. `None` when
    /// fewer than 30 points correspond or the problem is singular.
    pub fn align(&self, points: &[Vec2], prior: Pose2) -> Option<ScanMatch> {
        let mut pose = prior;
        let mut information = [[0.0; 3]; 3];
        for iteration in 0..12 {
            // Correspondences are searched ever closer to the facades as the
            // pose converges, so people standing a metre from a wall stop
            // pulling the scan once the walls have been found.
            let reach = (MAX_CORRESPONDENCE_M * 0.6f64.powi(iteration)).max(MIN_CORRESPONDENCE_M);
            let mut h = [[0.0; 3]; 3];
            let mut g = [0.0; 3];
            let mut used = 0;
            for &s in points {
                let p = pose.transform(s);
                let Some((_, q, n)) = self.nearest_within(p, reach) else {
                    continue;
                };
                let r = n.dot(p - q);
                let w = if r.abs() <= HUBER_M { 1.0 } else { HUBER_M / r.abs() };
                // d p / d theta = R'(theta) s.
                let dp = s.rotated(pose.theta_rad + std::f64::consts::FRAC_PI_2);
                let j = [n.x, n.y, n.dot(dp)];
                for a in 0..3 {
                    g[a] += w * j[a] * r;
                    for b in 0..3 {
                        h[a][b] += w * j[a] * j[b];
                    }
                }
                used += 1;
            }
            if used < 30 {
                return None;
            }
            // A little damping keeps a degenerate direction from running off.
            let mut damped = h;
            for (k, row) in damped.iter_mut().enumerate() {
                row[k] += 1e-3 * (1.0 + h[k][k]);
            }
            let inv = inverse3(&damped)?;
            let step: Vec<f64> = (0..3).map(|a| -(0..3).map(|b| inv[a][b] * g[b]).sum::<f64>()).collect();
            pose = Pose2::new(
                pose.x + step[0],
                pose.y + step[1],
                normalize_angle(pose.theta_rad + step[2]),
            );
            information = h;
            if reach <= MIN_CORRESPONDENCE_M && step[0].hypot(step[1]) < 1e-4 && step[2].abs() < 1e-5 {
                break;
            }
        }
        let (mut near, mut inliers) = (0, 0);
        for &s in points {
            if let Some((d, _, _)) = self.nearest(pose.transform(s)) {
                near += 1;
                inliers += usize::from(d < 0.3);
            }
        }
        // Covariance from the information, floored so one direction of a
        // flat wall does not report impossible certainty.
        let s2 = POINT_SIGMA_M * POINT_SIGMA_M;
        let mut info = information.map(|row| row.map(|v| v / s2));
        for (k, row) in info.iter_mut().enumerate() {
            row[k] += 1e-4;
        }
        // Map and model errors the point noise does not cover: 5 cm and
        // 0.6 degrees at least.
        let mut covariance = inverse3(&info)?;
        covariance[0][0] += 0.05 * 0.05;
        covariance[1][1] += 0.05 * 0.05;
        covariance[2][2] += 0.01 * 0.01;
        Some(ScanMatch {
            pose,
            covariance,
            inlier_fraction: inliers as f64 / near.max(1) as f64,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scan_snaps_back_to_where_it_was_taken() {
        let city = City::district(1);
        let matcher = MapMatcher::new(&city);
        let truth = Pose2::new(144.0, 180.0, 1.5);
        let mut rng = Rng::new(3);
        let points = scan(&city, &[], truth, &LidarConfig::default(), &mut rng);
        let m = matcher.align(&points, Pose2::new(144.6, 180.3, 1.52)).unwrap();
        assert!(m.pose.pos().dist(truth.pos()) < 0.1, "{:?}", m.pose);
        assert!(m.inlier_fraction > 0.9);
    }

    #[test]
    fn two_parallel_walls_leave_the_along_street_direction_loose() {
        let city = City::district(1);
        let matcher = MapMatcher::new(&city);
        // Halfway along the tower canyon (walls north and south only).
        let truth = Pose2::new(230.0, 124.0, std::f64::consts::PI);
        let mut rng = Rng::new(4);
        let cfg = LidarConfig {
            max_range_m: 20.0,
            ..LidarConfig::default()
        };
        let points = scan(&city, &[], truth, &cfg, &mut rng);
        let m = matcher.align(&points, truth).unwrap();
        // Along the street (x) far looser than across it (y).
        // (Above the floor that every match gets.)
        let floor = 0.05 * 0.05;
        assert!(
            m.covariance[0][0] - floor > 10.0 * (m.covariance[1][1] - floor),
            "{:?}",
            m.covariance
        );
    }
}
