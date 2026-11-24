//! GNSS / LiDAR / odometry localization methods, compared in a simulated
//! street canyon.
//!
//! * [`city`] — a miniature business district: box buildings with heights,
//!   a route through open sky, tower canyons and an alley, people on the
//!   sidewalks.
//! * [`gnss`] — a receiver simulated at the level of its solutions: every
//!   satellite traced against the buildings (line of sight, reflected, or
//!   blocked), least-squares errors from the reflections' biases, RTK fixes
//!   that are sometimes wrong while claiming centimetres.
//! * [`lidar`] — a planar scan of the facades and the people, and a
//!   point-to-line scan-to-map matcher that reports its own covariance.
//! * [`methods`] — the localizers: GNSS-only and LiDAR-only EKFs, an
//!   arbiter that switches between them, a LiDAR-primary arbiter that lets
//!   RTK correct the map match, a loosely coupled EKF and a sliding-window
//!   factor graph with robust kernels.
//! * [`sim`] — drives the route and scores a method.
//!
//! Everything is deterministic for a seed and has no dependencies; the
//! `render` feature adds GIF output through `robo-planning`'s renderer.

#![warn(missing_docs)]

pub mod city;
pub mod ekf;
pub mod geometry;
pub mod gnss;
pub mod lidar;
pub mod linalg;
pub mod methods;
pub mod rng;
pub mod sim;
