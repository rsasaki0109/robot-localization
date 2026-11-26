//! A sliding-window factor graph over the last few seconds of poses.
//!
//! A node per scan, odometry between consecutive nodes, a LiDAR match on
//! each node it matched (Huber-weighted, with the match's own covariance)
//! and a GNSS fix on the node it arrived at (Cauchy-weighted). Gauss-Newton
//! with iteratively reweighted robust kernels: nothing is switched, and a
//! fix that disagrees with the scans and the wheels loses its weight instead
//! of being gated by one threshold.

use super::{gnss_variance, lidar_covariance, Localizer, Source, Start, LIDAR_START_SIGMA_M, MIN_INLIERS};
use crate::ekf::residual;
use crate::geometry::{normalize_angle, Pose2, Vec2};
use crate::gnss::{Fix, FixKind};
use crate::lidar::ScanMatch;
use crate::linalg::{cholesky_solve, inverse3, Mat3};
use std::collections::VecDeque;

/// Nodes in the window (8 s at 5 Hz).
const WINDOW: usize = 40;
/// Gauss-Newton iterations per update.
const ITERATIONS: usize = 4;
/// Cauchy scale for GNSS residuals, in sigmas.
const CAUCHY_C: f64 = 2.0;
/// Huber threshold for LiDAR residuals, in sigmas (Mahalanobis).
const HUBER_K: f64 = 3.0;
/// Prior on the oldest node: per-axis sigma, metres, and yaw sigma.
const PRIOR_SIGMA_M: f64 = 0.3;
const PRIOR_SIGMA_RAD: f64 = 0.05;
/// A source counts as in use while its factors carried weight this recently.
const RECENT_S: f64 = 2.0;
/// For this long after an RTK fixed fix carried weight, seconds, a LiDAR
/// match farther than `RTK_HOLD_M` from the graph is left out: the graph is
/// anchored to centimetres, and a match that disagrees is the one that slid
/// (people beside a wall, a street that pins it only one way).
const RTK_HOLD_S: f64 = 3.0;
const RTK_HOLD_M: f64 = 0.3;

#[derive(Debug, Clone)]
struct Node {
    pose: Pose2,
    /// Odometry from the previous node, and its per-axis sigmas.
    odom: Pose2,
    odom_sigma: [f64; 3],
    gnss: Option<(Vec2, f64)>,
    lidar: Option<(Pose2, Mat3)>,
}

/// The factor graph.
#[derive(Debug, Clone)]
pub struct FactorGraph {
    nodes: VecDeque<Node>,
    prior: Pose2,
    /// Odometry since the newest node.
    acc: Pose2,
    acc_dist_m: f64,
    acc_turn_rad: f64,
    t: f64,
    last_gnss_s: f64,
    last_lidar_s: f64,
    /// When an RTK fixed fix last carried weight.
    last_fixed_s: f64,
    /// Sigmas of the prior on the oldest node: the start's until GNSS has
    /// anchored the graph, then [`PRIOR_SIGMA_M`] and [`PRIOR_SIGMA_RAD`].
    prior_sigma: [f64; 3],
    /// Whether the graph knows where it is: from the start when the start
    /// is known, otherwise once an RTK fixed fix agrees with it. Until then
    /// GNSS factors are plain least squares (a robust kernel would discount
    /// the very fixes that place the robot) and no scan is matched.
    anchored: bool,
}

impl FactorGraph {
    /// Starts at `start`.
    pub fn new(start: impl Into<Start>) -> Self {
        let Start {
            pose: start,
            sigma_m,
            sigma_rad,
        } = start.into();
        let first = Node {
            pose: start,
            odom: Pose2::default(),
            odom_sigma: [1.0; 3],
            gnss: None,
            lidar: None,
        };
        Self {
            nodes: VecDeque::from([first]),
            prior: start,
            acc: Pose2::default(),
            acc_dist_m: 0.0,
            acc_turn_rad: 0.0,
            t: 0.0,
            last_gnss_s: 0.0,
            last_lidar_s: 0.0,
            last_fixed_s: f64::NEG_INFINITY,
            prior_sigma: [
                sigma_m.max(PRIOR_SIGMA_M),
                sigma_m.max(PRIOR_SIGMA_M),
                sigma_rad.max(PRIOR_SIGMA_RAD),
            ],
            anchored: sigma_m <= LIDAR_START_SIGMA_M,
        }
    }

    fn add_node(&mut self) {
        let last = self.nodes.back().expect("a node").pose;
        let sigma_xy = 0.01 + 0.03 * self.acc_dist_m;
        let sigma_th = 0.002 + 0.02 * self.acc_turn_rad + 0.004 * self.acc_dist_m;
        self.nodes.push_back(Node {
            pose: last.compose(self.acc),
            odom: self.acc,
            odom_sigma: [sigma_xy, sigma_xy, sigma_th],
            gnss: None,
            lidar: None,
        });
        self.acc = Pose2::default();
        self.acc_dist_m = 0.0;
        self.acc_turn_rad = 0.0;
        while self.nodes.len() > WINDOW {
            self.nodes.pop_front();
            self.prior = self.nodes[0].pose;
            if self.anchored {
                self.prior_sigma = [PRIOR_SIGMA_M, PRIOR_SIGMA_M, PRIOR_SIGMA_RAD];
            }
        }
    }

    /// Gauss-Newton with reweighting; returns the final weights of the
    /// newest node's GNSS and LiDAR factors.
    fn solve(&mut self) -> (f64, f64) {
        let n = self.nodes.len();
        let dim = 3 * n;
        let (mut w_gnss, mut w_lidar) = (0.0, 0.0);
        for _ in 0..ITERATIONS {
            let mut h = vec![0.0; dim * dim];
            let mut g = vec![0.0; dim];
            let mut add = |blocks: &[(usize, Vec<[f64; 3]>)], omega: &[Vec<f64>], r: &[f64]| {
                let m = r.len();
                // omega * r and omega * J_b, row by row.
                let or: Vec<f64> = (0..m).map(|i| (0..m).map(|k| omega[i][k] * r[k]).sum()).collect();
                for (a, ja) in blocks {
                    for p in 0..3 {
                        g[3 * a + p] += (0..m).map(|i| ja[i][p] * or[i]).sum::<f64>();
                    }
                    for (b, jb) in blocks {
                        for p in 0..3 {
                            for q in 0..3 {
                                let mut s = 0.0;
                                for i in 0..m {
                                    for k in 0..m {
                                        s += ja[i][p] * omega[i][k] * jb[k][q];
                                    }
                                }
                                h[(3 * a + p) * dim + 3 * b + q] += s;
                            }
                        }
                    }
                }
            };
            // Prior on the oldest node.
            let r0 = residual(self.nodes[0].pose, self.prior);
            let omega0 = diag(&self.prior_sigma);
            add(&[(0, identity_rows())], &omega0, &r0);
            for k in 0..n {
                let node = &self.nodes[k];
                if k > 0 {
                    let a = self.nodes[k - 1].pose;
                    let b = node.pose;
                    let (s, c) = a.theta_rad.sin_cos();
                    let d = b.pos() - a.pos();
                    let r = [
                        c * d.x + s * d.y - node.odom.x,
                        -s * d.x + c * d.y - node.odom.y,
                        normalize_angle(b.theta_rad - a.theta_rad - node.odom.theta_rad),
                    ];
                    let ja = vec![
                        [-c, -s, -s * d.x + c * d.y],
                        [s, -c, -c * d.x - s * d.y],
                        [0.0, 0.0, -1.0],
                    ];
                    let jb = vec![[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]];
                    add(&[(k - 1, ja), (k, jb)], &diag(&node.odom_sigma), &r);
                }
                if let Some((z, var)) = node.gnss {
                    let r = [node.pose.x - z.x, node.pose.y - z.y];
                    let e2 = (r[0] * r[0] + r[1] * r[1]) / var;
                    let w = if self.anchored {
                        1.0 / (1.0 + e2 / (CAUCHY_C * CAUCHY_C))
                    } else {
                        1.0
                    };
                    if k == n - 1 {
                        w_gnss = w;
                    }
                    let omega = vec![vec![w / var, 0.0], vec![0.0, w / var]];
                    add(&[(k, vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])], &omega, &r);
                }
                if let Some((z, info)) = node.lidar {
                    let r = residual(node.pose, z);
                    let d2: f64 = (0..3)
                        .map(|i| r[i] * (0..3).map(|j| info[i][j] * r[j]).sum::<f64>())
                        .sum();
                    let d = d2.max(0.0).sqrt();
                    let w = if d <= HUBER_K { 1.0 } else { HUBER_K / d };
                    if k == n - 1 {
                        w_lidar = w;
                    }
                    let omega: Vec<Vec<f64>> = info.iter().map(|row| row.iter().map(|v| w * v).collect()).collect();
                    add(&[(k, identity_rows())], &omega, &r);
                }
            }
            for i in 0..dim {
                h[i * dim + i] += 1e-9;
            }
            let mut step: Vec<f64> = g.iter().map(|v| -v).collect();
            if !cholesky_solve(&mut h, &mut step, dim) {
                break;
            }
            for (k, node) in self.nodes.iter_mut().enumerate() {
                node.pose = Pose2::new(
                    node.pose.x + step[3 * k],
                    node.pose.y + step[3 * k + 1],
                    normalize_angle(node.pose.theta_rad + step[3 * k + 2]),
                );
            }
            if step.iter().map(|v| v * v).sum::<f64>() < 1e-10 {
                break;
            }
        }
        (w_gnss, w_lidar)
    }
}

fn diag(sigmas: &[f64]) -> Vec<Vec<f64>> {
    (0..sigmas.len())
        .map(|i| {
            (0..sigmas.len())
                .map(|j| if i == j { 1.0 / (sigmas[i] * sigmas[i]) } else { 0.0 })
                .collect()
        })
        .collect()
}

fn identity_rows() -> Vec<[f64; 3]> {
    vec![[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

impl Localizer for FactorGraph {
    fn name(&self) -> &'static str {
        "factor graph"
    }

    fn predict(&mut self, dt: f64, v: f64, w: f64) {
        let d = v * dt;
        let half = 0.5 * w * dt;
        self.acc = self.acc.compose(Pose2::new(d * half.cos(), d * half.sin(), w * dt));
        self.acc_dist_m += d.abs();
        self.acc_turn_rad += (w * dt).abs();
        self.t += dt;
    }

    fn gnss(&mut self, fix: Option<&Fix>, t: f64) {
        let Some(fix) = fix else { return };
        if self.acc_dist_m > 0.05 || self.acc_turn_rad > 0.01 {
            self.add_node();
        }
        // Float and single fixes join only when they agree with the graph
        // (3 sigma): in a canyon they are biased the same way for many
        // epochs, and even a robust kernel lets a run of them drag the
        // window. RTK fixed fixes always join, under the Cauchy kernel.
        let var = gnss_variance(fix);
        let node = self.nodes.back_mut().expect("a node");
        if self.anchored && fix.kind != FixKind::Fixed && node.pose.pos().dist(fix.pos) > 3.0 * var.sqrt() {
            return;
        }
        node.gnss = Some((fix.pos, var));
        let weight = self.solve().0;
        if !self.anchored && fix.kind == FixKind::Fixed {
            let node = self.nodes.back().expect("a node");
            if node.pose.pos().dist(fix.pos) < 0.3 && self.nodes.len() > 5 {
                self.anchored = true;
            }
        }
        if weight > 0.3 {
            self.last_gnss_s = t;
            if fix.kind == FixKind::Fixed {
                self.last_fixed_s = t;
            }
        }
    }

    fn lidar(&mut self, m: Option<&ScanMatch>, t: f64) {
        self.add_node();
        let held = t - self.last_fixed_s <= RTK_HOLD_S;
        if let Some(m) = m.filter(|m| self.anchored && m.inlier_fraction >= MIN_INLIERS) {
            let node = self.nodes.back_mut().expect("a node");
            let slid = held && m.pose.pos().dist(node.pose.pos()) > RTK_HOLD_M;
            if let (false, Some(info)) = (slid, inverse3(&lidar_covariance(m))) {
                node.lidar = Some((m.pose, info));
            }
        }
        if self.solve().1 > 0.3 {
            self.last_lidar_s = t;
        }
    }

    fn pose(&self) -> Pose2 {
        self.nodes.back().expect("a node").pose.compose(self.acc)
    }

    fn source(&self) -> Source {
        match (
            self.t - self.last_gnss_s <= RECENT_S,
            self.t - self.last_lidar_s <= RECENT_S,
        ) {
            (true, true) => Source::Both,
            (true, false) => Source::Gnss,
            (false, true) => Source::Lidar,
            (false, false) => Source::DeadReckoning,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unknown_start_is_anchored_by_rtk_and_only_then_uses_scans() {
        // Starts 25 m off with a 30 m sigma; the truth drives along x.
        let start = Start {
            pose: Pose2::new(25.0, 0.0, 0.0),
            sigma_m: 30.0,
            sigma_rad: 0.2,
        };
        let mut fg = FactorGraph::new(start);
        let tight = [[1e-4, 0.0, 0.0], [0.0, 1e-4, 0.0], [0.0, 0.0, 1e-6]];
        for k in 1..=40 {
            for _ in 0..4 {
                fg.predict(0.05, 1.0, 0.0);
            }
            let x = k as f64 * 0.2;
            let t = k as f64 * 0.2;
            // A scan matched from the wrong estimate fits the wrong place.
            let wrong = ScanMatch {
                pose: Pose2::new(x + 25.0, 0.0, 0.0),
                covariance: tight,
                inlier_fraction: 0.9,
            };
            fg.lidar(Some(&wrong), t);
            if k % 5 == 0 {
                let fix = Fix {
                    pos: Vec2::new(x, 0.0),
                    std_m: 0.02,
                    kind: FixKind::Fixed,
                    satellites: 12,
                };
                fg.gnss(Some(&fix), t);
            }
        }
        assert!(fg.anchored);
        assert!(fg.pose().x < 9.0, "{:?}", fg.pose());
    }

    #[test]
    fn odometry_chain_reproduces_dead_reckoning_without_measurements() {
        let mut fg = FactorGraph::new(Pose2::new(0.0, 0.0, 0.0));
        for _ in 0..50 {
            for _ in 0..4 {
                fg.predict(0.05, 1.0, 0.1);
            }
            fg.lidar(None, 0.0);
        }
        let mut ekf = crate::ekf::Ekf::new(Pose2::new(0.0, 0.0, 0.0), 0.1, 0.01);
        for _ in 0..200 {
            ekf.predict(0.05, 1.0, 0.1);
        }
        assert!(
            fg.pose().pos().dist(ekf.pose.pos()) < 1e-6,
            "{:?} {:?}",
            fg.pose(),
            ekf.pose
        );
    }

    #[test]
    fn a_wrong_fix_against_agreeing_scans_loses_its_weight() {
        let mut fg = FactorGraph::new(Pose2::new(0.0, 0.0, 0.0));
        let tight = [[1e-4, 0.0, 0.0], [0.0, 1e-4, 0.0], [0.0, 0.0, 1e-6]];
        for k in 1..=30 {
            for _ in 0..4 {
                fg.predict(0.05, 1.0, 0.0);
            }
            let x = k as f64 * 0.2;
            let m = ScanMatch {
                pose: Pose2::new(x, 0.0, 0.0),
                covariance: tight,
                inlier_fraction: 0.9,
            };
            fg.lidar(Some(&m), k as f64 * 0.2);
            // An RTK "fixed" fix 1.5 m off, claiming centimetres.
            let fix = Fix {
                pos: Vec2::new(x, 1.5),
                std_m: 0.02,
                kind: FixKind::Fixed,
                satellites: 9,
            };
            fg.gnss(Some(&fix), k as f64 * 0.2);
        }
        assert!(fg.pose().y.abs() < 0.2, "{:?}", fg.pose());
    }
}
