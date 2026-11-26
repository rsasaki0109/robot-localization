//! Planar geometry: vectors, poses and angles.

use std::ops::{Add, Mul, Neg, Sub};

/// 2D vector / point in metres (x east, y north).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    /// x, metres
    pub x: f64,
    /// y, metres
    pub y: f64,
}

impl Vec2 {
    /// Creates a vector.
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    /// Euclidean norm.
    pub fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }
    /// Distance to another point.
    pub fn dist(self, o: Vec2) -> f64 {
        (self - o).norm()
    }
    /// Dot product.
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    /// z component of the cross product.
    pub fn cross(self, o: Vec2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    /// Unit vector at `angle_rad` from the x axis.
    pub fn from_angle(angle_rad: f64) -> Self {
        Self::new(angle_rad.cos(), angle_rad.sin())
    }
    /// The vector turned by `angle_rad` counterclockwise.
    pub fn rotated(self, angle_rad: f64) -> Self {
        let (s, c) = angle_rad.sin_cos();
        Self::new(c * self.x - s * self.y, s * self.x + c * self.y)
    }
    /// Unit vector in the same direction (zero stays zero).
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            self * (1.0 / n)
        } else {
            self
        }
    }
    /// Linear interpolation.
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self) * t
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}
impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

/// Planar pose.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    /// x, metres
    pub x: f64,
    /// y, metres
    pub y: f64,
    /// heading, radians counterclockwise from x
    pub theta_rad: f64,
}

impl Pose2 {
    /// Creates a pose.
    pub const fn new(x: f64, y: f64, theta_rad: f64) -> Self {
        Self { x, y, theta_rad }
    }
    /// Position part.
    pub fn pos(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }
    /// A point given in this pose's frame, in the world frame.
    pub fn transform(self, p: Vec2) -> Vec2 {
        self.pos() + p.rotated(self.theta_rad)
    }
    /// This pose followed by `delta` (given in this pose's frame).
    pub fn compose(self, delta: Pose2) -> Pose2 {
        let p = self.transform(delta.pos());
        Pose2::new(p.x, p.y, normalize_angle(self.theta_rad + delta.theta_rad))
    }
    /// The pose `other` seen from this one: `self.compose(self.between(other)) == other`.
    pub fn between(self, other: Pose2) -> Pose2 {
        let d = (other.pos() - self.pos()).rotated(-self.theta_rad);
        Pose2::new(d.x, d.y, normalize_angle(other.theta_rad - self.theta_rad))
    }
}

/// Wraps an angle to `(-pi, pi]`.
pub fn normalize_angle(a: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let mut a = a % two_pi;
    if a <= -std::f64::consts::PI {
        a += two_pi;
    } else if a > std::f64::consts::PI {
        a -= two_pi;
    }
    a
}

/// Distance from `p` to the segment `a`-`b`, with the closest point.
pub fn point_segment(p: Vec2, a: Vec2, b: Vec2) -> (f64, Vec2) {
    let ab = b - a;
    let len2 = ab.dot(ab);
    let t = if len2 > 0.0 {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let q = a + ab * t;
    (p.dist(q), q)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compose_and_between_are_inverse() {
        let a = Pose2::new(1.0, -2.0, 0.7);
        let b = Pose2::new(-3.0, 4.0, -2.9);
        let c = a.compose(a.between(b));
        assert!((c.x - b.x).abs() < 1e-12 && (c.y - b.y).abs() < 1e-12);
        assert!(normalize_angle(c.theta_rad - b.theta_rad).abs() < 1e-12);
    }

    #[test]
    fn angles_wrap_into_half_open_range() {
        assert!((normalize_angle(3.0 * std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-12);
        assert!((normalize_angle(-0.5) + 0.5).abs() < 1e-12);
    }
}
