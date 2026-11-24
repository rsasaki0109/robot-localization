//! The localization methods, behind one interface.
//!
//! | Method | GNSS | LiDAR map | How they combine |
//! |---|---|---|---|
//! | [`GnssEkf`] | ✓ | | gated EKF |
//! | [`LidarEkf`] | | ✓ | gated EKF |
//! | [`Arbiter`] with [`Policy::Switching`] | ✓ | ✓ | one source in charge at a time, handed over on streaks |
//! | [`Arbiter`] with [`Policy::LidarPrimary`] | ✓ | ✓ | LiDAR in charge wherever it matches; RTK fixes correct it |
//! | [`LooseEkf`] | ✓ | ✓ | every gated measurement into one EKF |
//! | [`FactorGraph`] | ✓ | ✓ | sliding-window least squares, robust kernels |

mod arbiter;
mod factor_graph;
mod filters;

pub use arbiter::{Arbiter, Policy};
pub use factor_graph::FactorGraph;
pub use filters::{GnssEkf, LidarEkf, LooseEkf};

use crate::geometry::Pose2;
use crate::gnss::{Fix, FixKind};
use crate::lidar::ScanMatch;

/// What the estimate rests on at the moment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Source {
    /// GNSS fixes.
    Gnss,
    /// LiDAR map matches.
    Lidar,
    /// Both, fused.
    Both,
    /// Odometry alone.
    DeadReckoning,
}

impl Source {
    /// Short name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Gnss => "GNSS",
            Self::Lidar => "LiDAR",
            Self::Both => "GNSS+LiDAR",
            Self::DeadReckoning => "dead reckoning",
        }
    }
}

/// A localizer fed by the simulation.
pub trait Localizer {
    /// Display name.
    fn name(&self) -> &'static str;
    /// Wheel odometry: forward speed and yaw rate over `dt`.
    fn predict(&mut self, dt: f64, v: f64, w: f64);
    /// A GNSS epoch at time `t` (`None`: no solution).
    fn gnss(&mut self, fix: Option<&Fix>, t: f64);
    /// A scan matched from [`Localizer::pose`] at time `t` (`None`: no match).
    fn lidar(&mut self, m: Option<&ScanMatch>, t: f64);
    /// Whether the localizer wants scan matches (matching is skipped otherwise).
    fn uses_lidar(&self) -> bool {
        true
    }
    /// Whether one source is in charge at a time (so a change of source is a
    /// hand-over rather than a change in which measurements carry weight).
    fn arbitrates(&self) -> bool {
        false
    }
    /// Current estimate.
    fn pose(&self) -> Pose2;
    /// What the estimate rests on.
    fn source(&self) -> Source;
}

/// A fix's per-axis variance: the receiver's sigma, floored by kind (the
/// receiver does not know about its reflected satellites).
pub fn gnss_variance(fix: &Fix) -> f64 {
    let floor = match fix.kind {
        FixKind::Fixed => 0.03,
        FixKind::Float => 0.5,
        FixKind::Single => 3.0,
    };
    let s = fix.std_m.max(floor);
    s * s
}

/// 2-DOF chi-square at 99 %.
pub const GATE_2DOF: f64 = 9.21;
/// 3-DOF chi-square at 99.7 %.
pub const GATE_3DOF: f64 = 14.0;
/// Least share of a scan on mapped facades for a match to count.
pub const MIN_INLIERS: f64 = 0.5;

/// A match's covariance, inflated: neighbouring scans' errors are
/// correlated and the point noise model is optimistic.
pub fn lidar_covariance(m: &ScanMatch) -> crate::linalg::Mat3 {
    m.covariance.map(|row| row.map(|v| 4.0 * v))
}
