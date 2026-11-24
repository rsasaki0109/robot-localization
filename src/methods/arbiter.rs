//! An arbiter that puts one source in charge at a time.
//!
//! With [`Policy::Switching`] GNSS and the LiDAR map take turns: GNSS hands
//! over after a few epochs it cannot use, LiDAR takes over when its scan
//! matches, and GNSS takes back over after a run of good RTK fixed fixes.
//! In a street canyon the fixes come and go, and so does the source.
//!
//! With [`Policy::LidarPrimary`] LiDAR takes charge wherever the scan
//! matches and keeps it; RTK fixed fixes that agree with it are fused as
//! corrections instead of taking over. GNSS is in charge only where the map
//! cannot place the robot.

use super::filters::{fuse_gnss as gnss_into, fuse_lidar as lidar_into, RejectedRun, RELOCALIZE_GATE};
use super::{lidar_covariance, Localizer, Source, GATE_2DOF, GATE_3DOF, MIN_INLIERS};
use crate::ekf::Ekf;
use crate::geometry::Pose2;
use crate::gnss::{Fix, FixKind};
use crate::lidar::ScanMatch;

/// How the arbiter hands over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Policy {
    /// One source at a time, handed over on streaks.
    Switching,
    /// LiDAR wherever it matches; RTK corrects it.
    LidarPrimary,
}

/// Epochs without a usable fix before GNSS hands over.
const LEAVE_GNSS_EPOCHS: u32 = 3;
/// Good RTK fixed fixes in a row before GNSS takes over.
const ENTER_GNSS_EPOCHS: u32 = 5;
/// Scans without a usable match before LiDAR hands over (5 s: corners and
/// open intersections lose a few).
const LEAVE_LIDAR_SCANS: u32 = 25;

/// The arbiter.
#[derive(Debug, Clone)]
pub struct Arbiter {
    policy: Policy,
    ekf: Ekf,
    mode: Source,
    bad_gnss: u32,
    good_fixed: u32,
    bad_lidar: u32,
    rejected_fixed: RejectedRun,
}

impl Arbiter {
    /// Starts at `start` with GNSS in charge.
    pub fn new(start: Pose2, policy: Policy) -> Self {
        Self {
            policy,
            ekf: Ekf::new(start, 0.1, 0.02),
            mode: Source::Gnss,
            bad_gnss: 0,
            good_fixed: 0,
            bad_lidar: 0,
            rejected_fixed: RejectedRun::default(),
        }
    }

    /// An RTK fixed fix that passes the gate (the wider one while dead
    /// reckoning, when the estimate may have drifted).
    fn good_fixed(&self, fix: &Fix) -> bool {
        let var = super::gnss_variance(fix);
        let gate = if self.mode == Source::DeadReckoning {
            RELOCALIZE_GATE
        } else {
            GATE_2DOF
        };
        fix.kind == FixKind::Fixed && self.ekf.position_d2(fix.pos, [[var, 0.0], [0.0, var]]) <= gate
    }
}

impl Localizer for Arbiter {
    fn name(&self) -> &'static str {
        match self.policy {
            Policy::Switching => "switching",
            Policy::LidarPrimary => "LiDAR primary",
        }
    }

    fn predict(&mut self, dt: f64, v: f64, w: f64) {
        self.ekf.predict(dt, v, w);
    }

    fn gnss(&mut self, fix: Option<&Fix>, _: f64) {
        match self.mode {
            Source::Gnss => {
                let fused = fix.is_some_and(|f| gnss_into(&mut self.ekf, f, &mut self.rejected_fixed, false));
                self.bad_gnss = if fused { 0 } else { self.bad_gnss + 1 };
                if self.bad_gnss >= LEAVE_GNSS_EPOCHS {
                    self.mode = Source::DeadReckoning;
                    self.bad_gnss = 0;
                }
            }
            Source::Lidar if self.policy == Policy::LidarPrimary => {
                // Corrections, not a takeover.
                if let Some(f) = fix.filter(|f| self.good_fixed(f)) {
                    let var = super::gnss_variance(f);
                    self.ekf.update_position(f.pos, [[var, 0.0], [0.0, var]]);
                }
            }
            _ => {
                let good = fix.is_some_and(|f| self.good_fixed(f));
                self.good_fixed = if good { self.good_fixed + 1 } else { 0 };
                if self.good_fixed >= ENTER_GNSS_EPOCHS {
                    if let Some(f) = fix {
                        gnss_into(&mut self.ekf, f, &mut self.rejected_fixed, true);
                    }
                    self.mode = Source::Gnss;
                    self.good_fixed = 0;
                    self.bad_gnss = 0;
                }
            }
        }
    }

    fn lidar(&mut self, m: Option<&ScanMatch>, _: f64) {
        let usable = m.filter(|m| m.inlier_fraction >= MIN_INLIERS);
        match self.mode {
            Source::Lidar => {
                let fused = usable.is_some_and(|m| lidar_into(&mut self.ekf, m, false));
                self.bad_lidar = if fused { 0 } else { self.bad_lidar + 1 };
                if self.bad_lidar >= LEAVE_LIDAR_SCANS {
                    self.mode = Source::DeadReckoning;
                    self.bad_lidar = 0;
                }
            }
            Source::DeadReckoning => {
                if let Some(m) = usable {
                    let r = lidar_covariance(m);
                    if self.ekf.pose_d2(m.pose, &r) <= RELOCALIZE_GATE {
                        self.ekf.update_pose(m.pose, &r);
                        self.mode = Source::Lidar;
                        self.bad_lidar = 0;
                        self.good_fixed = 0;
                    }
                }
            }
            Source::Gnss if self.policy == Policy::LidarPrimary => {
                if let Some(m) = usable {
                    let r = lidar_covariance(m);
                    if self.ekf.pose_d2(m.pose, &r) <= GATE_3DOF {
                        self.ekf.update_pose(m.pose, &r);
                        self.mode = Source::Lidar;
                        self.bad_lidar = 0;
                    }
                }
            }
            _ => {}
        }
    }

    fn arbitrates(&self) -> bool {
        true
    }

    fn pose(&self) -> Pose2 {
        self.ekf.pose
    }

    fn source(&self) -> Source {
        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Vec2;

    fn fixed(x: f64) -> Fix {
        Fix {
            pos: Vec2::new(x, 0.0),
            std_m: 0.02,
            kind: FixKind::Fixed,
            satellites: 12,
        }
    }

    fn matched(x: f64) -> ScanMatch {
        ScanMatch {
            pose: Pose2::new(x, 0.0, 0.0),
            covariance: [[0.003, 0.0, 0.0], [0.0, 0.003, 0.0], [0.0, 0.0, 1e-4]],
            inlier_fraction: 0.9,
        }
    }

    /// Drives along x with RTK fixed fixes every second and a scan match
    /// every 0.2 s, the fixes dropping out for `outage` seconds in the middle;
    /// returns the number of changes of source.
    fn hand_overs(policy: Policy, outage: std::ops::Range<f64>) -> usize {
        let mut a = Arbiter::new(Pose2::new(0.0, 0.0, 0.0), policy);
        let (mut changes, mut last) = (0, a.source());
        for k in 1..=600 {
            let t = k as f64 * 0.05;
            a.predict(0.05, 1.0, 0.0);
            if k % 4 == 0 {
                a.lidar(Some(&matched(t)), t);
            }
            if k % 20 == 0 {
                let fix = (!outage.contains(&t)).then(|| fixed(t));
                a.gnss(fix.as_ref(), t);
            }
            if a.source() != last {
                changes += 1;
                last = a.source();
            }
        }
        changes
    }

    #[test]
    fn switching_trades_places_and_lidar_primary_keeps_charge() {
        // Switching: GNSS -> dead reckoning -> LiDAR -> GNSS around the gap.
        assert_eq!(hand_overs(Policy::Switching, 10.0..15.0), 3);
        // LiDAR primary: LiDAR takes over at the first scan and keeps it.
        assert_eq!(hand_overs(Policy::LidarPrimary, 10.0..15.0), 1);
    }

    #[test]
    fn lidar_primary_fuses_rtk_as_corrections() {
        let mut a = Arbiter::new(Pose2::new(0.0, 0.0, 0.0), Policy::LidarPrimary);
        a.lidar(Some(&matched(0.0)), 0.0);
        assert_eq!(a.source(), Source::Lidar);
        for k in 0..10 {
            a.gnss(Some(&fixed(0.1)), f64::from(k));
        }
        assert_eq!(a.source(), Source::Lidar);
        assert!(a.pose().x > 0.08, "{:?}", a.pose());
    }
}
