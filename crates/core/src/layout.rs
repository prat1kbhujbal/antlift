use crate::geom::{Aabb, V2};
use std::f32::consts::FRAC_PI_2;

/// Static level description, known to both the sim and the edge.
#[derive(Clone, Debug)]
pub struct Layout {
    pub arena: Aabb,
    pub walls: Vec<Aabb>,
    pub object_start: V2,
    pub object_start_angle: f32,
    /// Half extents of the payload (x = long axis).
    pub object_half: V2,
    pub goal: V2,
    pub n_robots: usize,
}

impl Layout {
    /// A long sofa that starts sideways and has to be turned to fit through a
    /// 3 m doorway, then dodge a pillar on the way to the goal.
    pub fn doorway() -> Self {
        let arena = Aabb::new(V2::ZERO, V2::new(40.0, 24.0));
        let t = 0.3; // half wall thickness
        let door_lo = 10.5;
        let door_hi = 13.5;
        let walls = vec![
            // Arena border.
            Aabb::new(V2::new(-t, -t), V2::new(40.0 + t, t)),
            Aabb::new(V2::new(-t, 24.0 - t), V2::new(40.0 + t, 24.0 + t)),
            Aabb::new(V2::new(-t, -t), V2::new(t, 24.0 + t)),
            Aabb::new(V2::new(40.0 - t, -t), V2::new(40.0 + t, 24.0 + t)),
            // Dividing wall with a doorway.
            Aabb::new(V2::new(20.0 - t, 0.0), V2::new(20.0 + t, door_lo)),
            Aabb::new(V2::new(20.0 - t, door_hi), V2::new(20.0 + t, 24.0)),
            // Pillar in the second room.
            Aabb::from_center(V2::new(27.0, 15.0), V2::new(1.0, 1.0)),
        ];
        Self {
            arena,
            walls,
            object_start: V2::new(8.0, 6.0),
            object_start_angle: FRAC_PI_2,
            object_half: V2::new(2.0, 0.6),
            goal: V2::new(33.0, 19.0),
            n_robots: 16,
        }
    }
}

/// Robots are discs of this radius.
pub const ROBOT_RADIUS: f32 = 0.25;
/// Grip points sit this far outside the payload boundary.
pub const SLOT_OFFSET: f32 = ROBOT_RADIUS + 0.02;
