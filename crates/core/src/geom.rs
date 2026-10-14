use std::f32::consts::PI;
use std::ops::{Add, AddAssign, Mul, Neg, Sub};

/// Minimal 2D vector so the protocol crate stays dependency-free.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V2 {
    pub x: f32,
    pub y: f32,
}

impl V2 {
    pub const ZERO: V2 = V2 { x: 0.0, y: 0.0 };

    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn from_angle(a: f32) -> Self {
        Self::new(a.cos(), a.sin())
    }

    pub fn dot(self, o: V2) -> f32 {
        self.x * o.x + self.y * o.y
    }

    /// z-component of the 3D cross product.
    pub fn cross(self, o: V2) -> f32 {
        self.x * o.y - self.y * o.x
    }

    pub fn len(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn dist(self, o: V2) -> f32 {
        (self - o).len()
    }

    /// Unit vector, or zero for (near) zero input.
    pub fn norm(self) -> V2 {
        let l = self.len();
        if l < 1e-6 {
            V2::ZERO
        } else {
            self * (1.0 / l)
        }
    }

    /// Rotated +90 degrees.
    pub fn perp(self) -> V2 {
        V2::new(-self.y, self.x)
    }

    pub fn rotate(self, a: f32) -> V2 {
        let (s, c) = a.sin_cos();
        V2::new(c * self.x - s * self.y, s * self.x + c * self.y)
    }

    pub fn angle(self) -> f32 {
        self.y.atan2(self.x)
    }

    pub fn clamp_len(self, max: f32) -> V2 {
        let l = self.len();
        if l > max {
            self * (max / l)
        } else {
            self
        }
    }
}

impl Add for V2 {
    type Output = V2;
    fn add(self, o: V2) -> V2 {
        V2::new(self.x + o.x, self.y + o.y)
    }
}

impl AddAssign for V2 {
    fn add_assign(&mut self, o: V2) {
        self.x += o.x;
        self.y += o.y;
    }
}

impl Sub for V2 {
    type Output = V2;
    fn sub(self, o: V2) -> V2 {
        V2::new(self.x - o.x, self.y - o.y)
    }
}

impl Mul<f32> for V2 {
    type Output = V2;
    fn mul(self, s: f32) -> V2 {
        V2::new(self.x * s, self.y * s)
    }
}

impl Neg for V2 {
    type Output = V2;
    fn neg(self) -> V2 {
        V2::new(-self.x, -self.y)
    }
}

/// Axis-aligned box, used for walls and dropped obstacles.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: V2,
    pub max: V2,
}

impl Aabb {
    pub fn new(min: V2, max: V2) -> Self {
        Self { min, max }
    }

    pub fn from_center(c: V2, half: V2) -> Self {
        Self::new(c - half, c + half)
    }

    pub fn center(&self) -> V2 {
        (self.min + self.max) * 0.5
    }

    pub fn half(&self) -> V2 {
        (self.max - self.min) * 0.5
    }

    /// Euclidean distance from `p` to the box (0 inside).
    pub fn dist(&self, p: V2) -> f32 {
        let dx = (self.min.x - p.x).max(0.0).max(p.x - self.max.x);
        let dy = (self.min.y - p.y).max(0.0).max(p.y - self.max.y);
        V2::new(dx, dy).len()
    }
}

/// Wrap an angle to (-PI, PI].
pub fn wrap_angle(a: f32) -> f32 {
    let mut a = (a + PI) % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a - PI
}

/// Smallest signed difference between two headings of a symmetric object
/// (a sofa facing +x or -x is equally "aligned"), in (-PI/2, PI/2].
pub fn axis_error(target: f32, current: f32) -> f32 {
    let e = wrap_angle(target - current);
    if e > PI / 2.0 {
        e - PI
    } else if e <= -PI / 2.0 {
        e + PI
    } else {
        e
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_stays_in_range() {
        for a in [-10.0, -PI, 0.0, PI, 7.0] {
            let w = wrap_angle(a);
            assert!(w > -PI - 1e-5 && w <= PI + 1e-5, "{a} -> {w}");
        }
    }

    #[test]
    fn axis_error_treats_flipped_object_as_aligned() {
        assert!(axis_error(0.0, PI).abs() < 1e-5);
        assert!((axis_error(0.2, 0.0) - 0.2).abs() < 1e-5);
    }
}
