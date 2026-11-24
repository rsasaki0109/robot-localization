//! Single-sensor EKFs and the loosely coupled EKF that fuses both.

use super::{gnss_variance, lidar_covariance, Localizer, Source, GATE_2DOF, GATE_3DOF, MIN_INLIERS};
use crate::ekf::Ekf;
use crate::geometry::{Pose2, Vec2};
use crate::gnss::{Fix, FixKind};
use crate::lidar::ScanMatch;

/// After this many consecutive gated-out RTK fixed fixes that agree with
/// each other relative to the odometry (all offsets from the estimate within
/// `RESET_SPREAD_M` of their mean), the estimate is taken to be the one that
/// is wrong and is moved onto them. Longer than a wrong fix lasts.
const RESET_AFTER_REJECTED_FIXED: usize = 20;
const RESET_SPREAD_M: f64 = 0.5;
/// A source counts as in use while it was fused this recently, seconds.
const RECENT_S: f64 = 2.0;

/// Offsets from the estimate of the latest consecutive gated-out RTK fixed
/// fixes.
#[derive(Debug, Clone, Default)]
pub(super) struct RejectedRun(Vec<Vec2>);

/// Without a fused fix for this long, seconds, the estimate may have drifted
/// beyond what its covariance admits, and an RTK fixed fix passes a wider
/// gate.
pub(super) const GNSS_LOST_S: f64 = 10.0;

/// Shared GNSS handling: gate (an RTK fixed fix passes the wider
/// [`RELOCALIZE_GATE`] when `lost`), fuse, and re-anchor after a long
/// consistent run of rejected RTK fixed fixes. Returns whether the fix was
/// used.
pub(super) fn fuse_gnss(ekf: &mut Ekf, fix: &Fix, rejected: &mut RejectedRun, lost: bool) -> bool {
    let var = gnss_variance(fix);
    let r = [[var, 0.0], [0.0, var]];
    let gate = if lost && fix.kind == FixKind::Fixed {
        RELOCALIZE_GATE
    } else {
        GATE_2DOF
    };
    if ekf.position_d2(fix.pos, r) <= gate {
        ekf.update_position(fix.pos, r);
        rejected.0.clear();
        return true;
    }
    if fix.kind != FixKind::Fixed {
        rejected.0.clear();
        return false;
    }
    rejected.0.push(fix.pos - ekf.pose.pos());
    let n = rejected.0.len();
    let mean = rejected.0.iter().fold(Vec2::default(), |a, &o| a + o) * (1.0 / n as f64);
    if rejected.0.iter().any(|o| o.dist(mean) > RESET_SPREAD_M) {
        // Not consistent: start the run again from this fix.
        let last = rejected.0[n - 1];
        rejected.0.clear();
        rejected.0.push(last);
        return false;
    }
    if n >= RESET_AFTER_REJECTED_FIXED {
        ekf.pose = Pose2::new(ekf.pose.x + mean.x, ekf.pose.y + mean.y, ekf.pose.theta_rad);
        ekf.cov[0][0] = ekf.cov[0][0].max(0.25);
        ekf.cov[1][1] = ekf.cov[1][1].max(0.25);
        ekf.update_position(fix.pos, r);
        rejected.0.clear();
        return true;
    }
    false
}

/// Gate for a match while LiDAR has not been fused for a while (the
/// estimate has drifted and its covariance may not say how far).
pub(super) const RELOCALIZE_GATE: f64 = 60.0;

/// Shared LiDAR handling: gate (wider when `lost`) and fuse. Returns whether
/// the match was fused.
pub(super) fn fuse_lidar(ekf: &mut Ekf, m: &ScanMatch, lost: bool) -> bool {
    if m.inlier_fraction < MIN_INLIERS {
        return false;
    }
    let r = lidar_covariance(m);
    if ekf.pose_d2(m.pose, &r) > if lost { RELOCALIZE_GATE } else { GATE_3DOF } {
        return false;
    }
    ekf.update_pose(m.pose, &r);
    true
}

/// GNSS alone: a gated EKF on wheel odometry.
#[derive(Debug, Clone)]
pub struct GnssEkf {
    ekf: Ekf,
    rejected_fixed: RejectedRun,
    last_fused_s: f64,
    t: f64,
}

impl GnssEkf {
    /// Starts at `start`.
    pub fn new(start: Pose2) -> Self {
        Self {
            ekf: Ekf::new(start, 0.1, 0.02),
            rejected_fixed: RejectedRun::default(),
            last_fused_s: 0.0,
            t: 0.0,
        }
    }
}

impl Localizer for GnssEkf {
    fn name(&self) -> &'static str {
        "GNSS EKF"
    }
    fn predict(&mut self, dt: f64, v: f64, w: f64) {
        self.ekf.predict(dt, v, w);
        self.t += dt;
    }
    fn gnss(&mut self, fix: Option<&Fix>, t: f64) {
        if let Some(fix) = fix {
            let lost = t - self.last_fused_s > GNSS_LOST_S;
            if fuse_gnss(&mut self.ekf, fix, &mut self.rejected_fixed, lost) {
                self.last_fused_s = t;
            }
        }
    }
    fn lidar(&mut self, _: Option<&ScanMatch>, _: f64) {}
    fn uses_lidar(&self) -> bool {
        false
    }
    fn pose(&self) -> Pose2 {
        self.ekf.pose
    }
    fn source(&self) -> Source {
        if self.t - self.last_fused_s <= RECENT_S {
            Source::Gnss
        } else {
            Source::DeadReckoning
        }
    }
}

/// The LiDAR map alone: a gated EKF on wheel odometry, started at the
/// true pose.
#[derive(Debug, Clone)]
pub struct LidarEkf {
    ekf: Ekf,
    last_fused_s: f64,
    t: f64,
}

impl LidarEkf {
    /// Starts at `start`.
    pub fn new(start: Pose2) -> Self {
        Self {
            ekf: Ekf::new(start, 0.1, 0.02),
            last_fused_s: 0.0,
            t: 0.0,
        }
    }
}

impl Localizer for LidarEkf {
    fn name(&self) -> &'static str {
        "LiDAR EKF"
    }
    fn predict(&mut self, dt: f64, v: f64, w: f64) {
        self.ekf.predict(dt, v, w);
        self.t += dt;
    }
    fn gnss(&mut self, _: Option<&Fix>, _: f64) {}
    fn lidar(&mut self, m: Option<&ScanMatch>, t: f64) {
        if let Some(m) = m {
            let lost = t - self.last_fused_s > RECENT_S;
            if fuse_lidar(&mut self.ekf, m, lost) {
                self.last_fused_s = t;
            }
        }
    }
    fn pose(&self) -> Pose2 {
        self.ekf.pose
    }
    fn source(&self) -> Source {
        if self.t - self.last_fused_s <= RECENT_S {
            Source::Lidar
        } else {
            Source::DeadReckoning
        }
    }
}

/// Loosely coupled: every gated GNSS fix and LiDAR match into one EKF, no
/// switching.
#[derive(Debug, Clone)]
pub struct LooseEkf {
    ekf: Ekf,
    rejected_fixed: RejectedRun,
    last_gnss_s: f64,
    last_lidar_s: f64,
    t: f64,
}

impl LooseEkf {
    /// Starts at `start`.
    pub fn new(start: Pose2) -> Self {
        Self {
            ekf: Ekf::new(start, 0.1, 0.02),
            rejected_fixed: RejectedRun::default(),
            last_gnss_s: 0.0,
            last_lidar_s: 0.0,
            t: 0.0,
        }
    }
}

impl Localizer for LooseEkf {
    fn name(&self) -> &'static str {
        "loose EKF"
    }
    fn predict(&mut self, dt: f64, v: f64, w: f64) {
        self.ekf.predict(dt, v, w);
        self.t += dt;
    }
    fn gnss(&mut self, fix: Option<&Fix>, t: f64) {
        if let Some(fix) = fix {
            let lost = t - self.last_gnss_s.max(self.last_lidar_s) > GNSS_LOST_S;
            if fuse_gnss(&mut self.ekf, fix, &mut self.rejected_fixed, lost) {
                self.last_gnss_s = t;
            }
        }
    }
    fn lidar(&mut self, m: Option<&ScanMatch>, t: f64) {
        if let Some(m) = m {
            let lost = t - self.last_lidar_s > RECENT_S;
            if fuse_lidar(&mut self.ekf, m, lost) {
                self.last_lidar_s = t;
            }
        }
    }
    fn pose(&self) -> Pose2 {
        self.ekf.pose
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
