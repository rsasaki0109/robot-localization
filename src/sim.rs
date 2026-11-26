//! The simulation: a robot drives the city's route while a localizer
//! estimates where it is from wheel odometry, GNSS and a LiDAR map.
//!
//! The robot follows the route by its true pose, so every method sees the
//! same drive and the same sensor noise; only the estimates differ.

use crate::city::{City, Crowd};
use crate::geometry::{normalize_angle, Pose2, Vec2};
use crate::gnss::{Epoch, Receiver};
use crate::lidar::{scan, LidarConfig, MapMatcher};
use crate::methods::{Localizer, Source, Start};
use crate::rng::Rng;

/// Simulation step, seconds.
pub const DT_S: f64 = 0.05;
/// GNSS epochs every this many steps (1 Hz).
const GNSS_EVERY: usize = 20;
/// LiDAR scans every this many steps (5 Hz).
const LIDAR_EVERY: usize = 4;
/// Driving speed, m/s.
pub const SPEED_M_S: f64 = 1.2;

/// The LiDAR map a condition uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapState {
    /// The world itself.
    Exact,
    /// A year old ([`City::outdated_map`]).
    Outdated,
    /// The south-west quarter of the district not mapped yet.
    Unmapped,
    /// A tower block rebuilt since the survey, 2.5 m from where the map has it.
    Rebuilt,
}

/// Where a condition happens.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scene {
    /// [`City::district`].
    District,
    /// [`City::boulevard`].
    Boulevard,
}

/// What the street is like.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Condition {
    /// Name for tables.
    pub name: &'static str,
    /// Where.
    pub scene: Scene,
    /// People on the sidewalks.
    pub people: usize,
    /// The LiDAR map.
    pub map: MapState,
    /// Whether the localizers start without knowing where they are (25 m
    /// off, the heading from a magnetometer within about 10 degrees).
    pub unknown_start: bool,
}

impl Condition {
    const BASE: Condition = Condition {
        name: "clear",
        scene: Scene::District,
        people: 0,
        map: MapState::Exact,
        unknown_start: false,
    };
    /// An empty street and an exact map.
    pub const CLEAR: Condition = Self::BASE;
    /// Rush hour: 600 people on the sidewalks, some walking beside the robot.
    pub const CROWD: Condition = Condition {
        name: "crowd",
        people: 600,
        ..Self::BASE
    };
    /// An empty street, but the map is a year old.
    pub const OUTDATED_MAP: Condition = Condition {
        name: "outdated map",
        map: MapState::Outdated,
        ..Self::BASE
    };
    /// Switched on somewhere: nobody tells the localizers where.
    pub const UNKNOWN_START: Condition = Condition {
        name: "unknown start",
        unknown_start: true,
        ..Self::BASE
    };
    /// A quarter of the district is not in the map.
    pub const OFF_MAP: Condition = Condition {
        name: "off the map",
        map: MapState::Unmapped,
        ..Self::BASE
    };
    /// 600 m of straight road between featureless walls.
    pub const LONG_ROAD: Condition = Condition {
        name: "long straight road",
        scene: Scene::Boulevard,
        ..Self::BASE
    };
    /// A tower block rebuilt since the map was made.
    pub const CONSTRUCTION: Condition = Condition {
        name: "construction",
        map: MapState::Rebuilt,
        ..Self::BASE
    };
    /// All conditions, in table order.
    pub const ALL: [Condition; 7] = [
        Self::CLEAR,
        Self::CROWD,
        Self::OUTDATED_MAP,
        Self::UNKNOWN_START,
        Self::OFF_MAP,
        Self::LONG_ROAD,
        Self::CONSTRUCTION,
    ];

    /// The city.
    pub fn city(&self) -> City {
        match self.scene {
            Scene::District => City::district(1),
            Scene::Boulevard => City::boulevard(),
        }
    }

    /// The LiDAR's map of `city`.
    pub fn map(&self, city: &City) -> Vec<crate::city::Building> {
        match self.map {
            MapState::Exact => city.buildings.clone(),
            MapState::Outdated => city.outdated_map(1),
            MapState::Unmapped => city.map_without(Vec2::new(20.0, 20.0), Vec2::new(250.0, 115.0)),
            MapState::Rebuilt => {
                city.rebuilt_map(Vec2::new(150.0, 30.0), Vec2::new(250.0, 110.0), Vec2::new(2.0, -1.5))
            }
        }
    }

    /// Where the localizers start, and how sure they are.
    pub fn start(&self, city: &City, seed: u64) -> Start {
        let truth = start_pose(city);
        if !self.unknown_start {
            return Start::known(truth);
        }
        let mut rng = Rng::new(seed ^ 0x57A7);
        let off = Vec2::from_angle(rng.range(0.0, std::f64::consts::TAU)) * 25.0;
        Start {
            pose: Pose2::new(truth.x + off.x, truth.y + off.y, truth.theta_rad + 0.17 * rng.normal()),
            sigma_m: 30.0,
            sigma_rad: 0.2,
        }
    }
}

/// One recorded frame, for the renders.
#[derive(Debug, Clone)]
pub struct Frame {
    /// Time, seconds.
    pub t: f64,
    /// True pose.
    pub truth: Pose2,
    /// Estimate.
    pub estimate: Pose2,
    /// What the estimate rested on.
    pub source: Source,
    /// The latest GNSS epoch.
    pub epoch: Option<Epoch>,
    /// People.
    pub people: Vec<Vec2>,
    /// The latest scan in the world frame, placed at the true pose.
    pub scan: Vec<Vec2>,
}

/// The result of a run.
#[derive(Debug, Clone)]
pub struct Report {
    /// Method name.
    pub method: &'static str,
    /// Horizontal error at every step, metres.
    pub errors: Vec<f64>,
    /// Seconds spent per source: GNSS, LiDAR, both, dead reckoning.
    pub source_s: [f64; 4],
    /// Changes of source.
    pub switches: usize,
    /// Whether the method hands over between sources
    /// ([`Localizer::arbitrates`]).
    pub arbitrates: bool,
    /// Recorded frames (every 0.5 s) when asked for.
    pub frames: Vec<Frame>,
    /// Estimate and source every 0.5 s (for trails).
    pub track: Vec<(Pose2, Pose2, Source)>,
}

impl Report {
    /// Root mean square error, metres.
    pub fn rmse(&self) -> f64 {
        (self.errors.iter().map(|e| e * e).sum::<f64>() / self.errors.len().max(1) as f64).sqrt()
    }
    /// Largest error, metres.
    pub fn max(&self) -> f64 {
        self.errors.iter().copied().fold(0.0, f64::max)
    }
    /// When the error last exceeded `threshold_m`, seconds (0 if never):
    /// for an unknown start, how long the method took to find itself.
    pub fn last_over_s(&self, threshold_m: f64) -> f64 {
        self.errors
            .iter()
            .rposition(|&e| e > threshold_m)
            .map_or(0.0, |k| (k + 1) as f64 * DT_S)
    }
    /// 95th percentile error, metres.
    pub fn p95(&self) -> f64 {
        let mut e = self.errors.clone();
        e.sort_by(f64::total_cmp);
        e.get((e.len() as f64 * 0.95) as usize).copied().unwrap_or(0.0)
    }
}

fn source_index(s: Source) -> usize {
    match s {
        Source::Gnss => 0,
        Source::Lidar => 1,
        Source::Both => 2,
        Source::DeadReckoning => 3,
    }
}

/// Pure-pursuit lookahead, metres.
const LOOKAHEAD_M: f64 = 3.0;

/// The start pose: the route's first waypoint, facing along it.
pub fn start_pose(city: &City) -> Pose2 {
    city.route_pose(0.0)
}

/// The true drive: a unicycle at [`SPEED_M_S`] following the route by pure
/// pursuit, one pose and yaw rate per step, for one lap. The wheels and the
/// gyro measure exactly this motion (plus their errors), corners included.
pub fn drive(city: &City) -> Vec<(Pose2, f64)> {
    let steps = (city.route_length_m() / SPEED_M_S / DT_S) as usize;
    let mut pose = start_pose(city);
    let mut s = 0.0;
    let mut out = Vec::with_capacity(steps);
    for _ in 0..steps {
        s += SPEED_M_S * DT_S;
        let target = city.route_pose(s + LOOKAHEAD_M).pos();
        let local = (target - pose.pos()).rotated(-pose.theta_rad);
        let alpha = local.y.atan2(local.x);
        let w = 2.0 * SPEED_M_S * alpha.sin() / LOOKAHEAD_M;
        let half = pose.theta_rad + 0.5 * w * DT_S;
        pose = Pose2::new(
            pose.x + SPEED_M_S * DT_S * half.cos(),
            pose.y + SPEED_M_S * DT_S * half.sin(),
            normalize_angle(pose.theta_rad + w * DT_S),
        );
        out.push((pose, w));
    }
    out
}

/// Drives one lap of the route with `localizer`. `seed` sets the sensor
/// noise and the crowd; `record` keeps frames for rendering.
pub fn run(localizer: &mut dyn Localizer, city: &City, condition: Condition, seed: u64, record: bool) -> Report {
    let matcher = MapMatcher::from_buildings(city, &condition.map(city));
    let mut rng = Rng::new(seed);
    let mut odom_rng = rng.fork(1);
    let mut lidar_rng = rng.fork(2);
    let mut receiver = Receiver::new(30, 7 ^ seed.wrapping_mul(31));
    let crowd = if condition.people > 0 {
        Crowd::new(city, condition.people, seed)
    } else {
        Crowd::empty()
    };
    let lidar_config = LidarConfig::default();
    let path = drive(city);
    // Wheel odometry: about 1 % scale error and a calibrated MEMS gyro's
    // residual bias (about 100 deg/h).
    let (scale, bias) = (1.0 + 0.01 * odom_rng.normal(), 0.0005 + 0.0003 * odom_rng.normal());
    let mut report = Report {
        method: localizer.name(),
        errors: Vec::with_capacity(path.len()),
        source_s: [0.0; 4],
        switches: 0,
        arbitrates: localizer.arbitrates(),
        frames: Vec::new(),
        track: Vec::new(),
    };
    let mut last_source = localizer.source();
    let (mut last_epoch, mut last_scan, mut last_people) = (None, Vec::new(), Vec::new());
    for (k, &(truth, w_true)) in path.iter().enumerate() {
        let k = k + 1;
        let t = k as f64 * DT_S;
        let v = SPEED_M_S * scale + 0.02 * odom_rng.normal();
        let w = w_true + bias + 0.01 * odom_rng.normal();
        localizer.predict(DT_S, v, w);

        if k % LIDAR_EVERY == 0 && localizer.uses_lidar() {
            let people = crowd.at(city, t);
            let points = scan(city, &people, truth, &lidar_config, &mut lidar_rng);
            let m = matcher.align(&points, localizer.pose());
            localizer.lidar(m.as_ref(), t);
            if record {
                last_scan = points.iter().map(|&p| truth.transform(p)).collect();
                last_people = people;
            }
        } else if record && k % LIDAR_EVERY == 0 {
            last_people = crowd.at(city, t);
        }
        if k % GNSS_EVERY == 0 {
            let epoch = receiver.epoch(city, truth.pos());
            localizer.gnss(epoch.fix.as_ref(), t);
            last_epoch = Some(epoch);
        }

        let estimate = localizer.pose();
        report.errors.push(estimate.pos().dist(truth.pos()));
        let source = localizer.source();
        report.source_s[source_index(source)] += DT_S;
        if source != last_source {
            report.switches += 1;
            last_source = source;
        }
        if k % 10 == 0 {
            report.track.push((truth, estimate, source));
            if record {
                report.frames.push(Frame {
                    t,
                    truth,
                    estimate,
                    source,
                    epoch: last_epoch.clone(),
                    people: last_people.clone(),
                    scan: last_scan.clone(),
                });
            }
        }
    }
    report
}

/// Every method, started at `start`, in table order.
pub fn all_methods(start: Start) -> Vec<Box<dyn Localizer>> {
    use crate::methods::{Arbiter, FactorGraph, GnssEkf, LidarEkf, LooseEkf, Policy};
    vec![
        Box::new(GnssEkf::new(start)),
        Box::new(LidarEkf::new(start)),
        Box::new(Arbiter::new(start, Policy::Switching)),
        Box::new(Arbiter::new(start, Policy::LidarPrimary)),
        Box::new(LooseEkf::new(start)),
        Box::new(FactorGraph::new(start)),
    ]
}
