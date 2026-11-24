//! A planar EKF over (x, y, theta) with wheel-odometry prediction and
//! position or pose updates.

use crate::geometry::{normalize_angle, Pose2, Vec2};
use crate::linalg::{inverse2, inverse3, mul3, transpose3, Mat3};

/// The filter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ekf {
    /// Estimate.
    pub pose: Pose2,
    /// Covariance of (x, y, theta).
    pub cov: Mat3,
}

/// Odometry noise: speed sigma = `SPEED_SIGMA_M_S + SPEED_SIGMA_REL * |v|`.
const SPEED_SIGMA_M_S: f64 = 0.02;
const SPEED_SIGMA_REL: f64 = 0.25;
/// Yaw-rate sigma, rad/s.
const YAW_RATE_SIGMA: f64 = 0.01;

impl Ekf {
    /// Starts at `pose` with a diagonal covariance.
    pub fn new(pose: Pose2, sigma_xy_m: f64, sigma_theta_rad: f64) -> Self {
        let mut cov = [[0.0; 3]; 3];
        cov[0][0] = sigma_xy_m * sigma_xy_m;
        cov[1][1] = sigma_xy_m * sigma_xy_m;
        cov[2][2] = sigma_theta_rad * sigma_theta_rad;
        Self { pose, cov }
    }

    /// Integrates forward speed `v` and yaw rate `w` over `dt`.
    pub fn predict(&mut self, dt: f64, v: f64, w: f64) {
        let mid = self.pose.theta_rad + 0.5 * w * dt;
        let (s, c) = mid.sin_cos();
        self.pose = Pose2::new(
            self.pose.x + v * dt * c,
            self.pose.y + v * dt * s,
            normalize_angle(self.pose.theta_rad + w * dt),
        );
        let f = [[1.0, 0.0, -v * dt * s], [0.0, 1.0, v * dt * c], [0.0, 0.0, 1.0]];
        let g = [
            [dt * c, -0.5 * v * dt * dt * s],
            [dt * s, 0.5 * v * dt * dt * c],
            [0.0, dt],
        ];
        let sv = SPEED_SIGMA_M_S + SPEED_SIGMA_REL * v.abs();
        let q = [sv * sv, YAW_RATE_SIGMA * YAW_RATE_SIGMA];
        let mut p = mul3(&mul3(&f, &self.cov), &transpose3(&f));
        for (i, row) in p.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v += g[i][0] * g[j][0] * q[0] + g[i][1] * g[j][1] * q[1];
            }
        }
        self.cov = p;
    }

    /// Squared Mahalanobis distance of a position measurement with
    /// covariance `r`.
    pub fn position_d2(&self, z: Vec2, r: [[f64; 2]; 2]) -> f64 {
        let s = [
            [self.cov[0][0] + r[0][0], self.cov[0][1] + r[0][1]],
            [self.cov[1][0] + r[1][0], self.cov[1][1] + r[1][1]],
        ];
        let Some(si) = inverse2(s) else { return f64::INFINITY };
        let d = [z.x - self.pose.x, z.y - self.pose.y];
        d[0] * (si[0][0] * d[0] + si[0][1] * d[1]) + d[1] * (si[1][0] * d[0] + si[1][1] * d[1])
    }

    /// Fuses a position measurement.
    pub fn update_position(&mut self, z: Vec2, r: [[f64; 2]; 2]) {
        let p = self.cov;
        let s = [
            [p[0][0] + r[0][0], p[0][1] + r[0][1]],
            [p[1][0] + r[1][0], p[1][1] + r[1][1]],
        ];
        let Some(si) = inverse2(s) else { return };
        // K = P H^T S^-1 with H = [I2 0].
        let mut k = [[0.0; 2]; 3];
        for (i, row) in k.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = p[i][0] * si[0][j] + p[i][1] * si[1][j];
            }
        }
        let d = [z.x - self.pose.x, z.y - self.pose.y];
        self.pose = Pose2::new(
            self.pose.x + k[0][0] * d[0] + k[0][1] * d[1],
            self.pose.y + k[1][0] * d[0] + k[1][1] * d[1],
            normalize_angle(self.pose.theta_rad + k[2][0] * d[0] + k[2][1] * d[1]),
        );
        let mut new = p;
        for i in 0..3 {
            for j in 0..3 {
                new[i][j] = p[i][j] - (k[i][0] * p[0][j] + k[i][1] * p[1][j]);
            }
        }
        self.cov = symmetric(new);
    }

    /// Squared Mahalanobis distance of a full pose measurement.
    pub fn pose_d2(&self, z: Pose2, r: &Mat3) -> f64 {
        let mut s = self.cov;
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += r[i][j];
            }
        }
        let Some(si) = inverse3(&s) else { return f64::INFINITY };
        let d = residual(z, self.pose);
        (0..3)
            .map(|i| d[i] * (0..3).map(|j| si[i][j] * d[j]).sum::<f64>())
            .sum()
    }

    /// Fuses a full pose measurement.
    pub fn update_pose(&mut self, z: Pose2, r: &Mat3) {
        let p = self.cov;
        let mut s = p;
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += r[i][j];
            }
        }
        let Some(si) = inverse3(&s) else { return };
        let k = mul3(&p, &si);
        let d = residual(z, self.pose);
        let dx: Vec<f64> = (0..3).map(|i| (0..3).map(|j| k[i][j] * d[j]).sum()).collect();
        self.pose = Pose2::new(
            self.pose.x + dx[0],
            self.pose.y + dx[1],
            normalize_angle(self.pose.theta_rad + dx[2]),
        );
        let kp = mul3(&k, &p);
        let mut new = p;
        for i in 0..3 {
            for j in 0..3 {
                new[i][j] -= kp[i][j];
            }
        }
        self.cov = symmetric(new);
    }

    /// Horizontal standard deviation (root of the position variances' sum).
    pub fn horizontal_sigma_m(&self) -> f64 {
        (self.cov[0][0] + self.cov[1][1]).max(0.0).sqrt()
    }
}

/// `z - x` with the angle wrapped.
pub fn residual(z: Pose2, x: Pose2) -> [f64; 3] {
    [z.x - x.x, z.y - x.y, normalize_angle(z.theta_rad - x.theta_rad)]
}

fn symmetric(m: Mat3) -> Mat3 {
    let mut s = m;
    for i in 0..3 {
        for j in 0..3 {
            s[i][j] = 0.5 * (m[i][j] + m[j][i]);
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_position_update_pulls_toward_the_measurement_and_shrinks_the_covariance() {
        let mut ekf = Ekf::new(Pose2::new(0.0, 0.0, 0.0), 1.0, 0.1);
        ekf.update_position(Vec2::new(1.0, 0.0), [[1.0, 0.0], [0.0, 1.0]]);
        assert!((ekf.pose.x - 0.5).abs() < 1e-9);
        assert!((ekf.cov[0][0] - 0.5).abs() < 1e-9);
    }

    #[test]
    fn prediction_drives_forward_and_grows_uncertainty() {
        let mut ekf = Ekf::new(Pose2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2), 0.01, 0.001);
        for _ in 0..100 {
            ekf.predict(0.1, 1.0, 0.0);
        }
        assert!((ekf.pose.y - 10.0).abs() < 1e-9 && ekf.pose.x.abs() < 1e-9);
        assert!(ekf.cov[0][0] > 1e-4);
    }
}
