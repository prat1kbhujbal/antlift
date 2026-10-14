//! Rapier wrapper: the payload, robots, walls and dropped crates.
//!
//! Top-down 2D with no gravity; floor friction is modelled with damping.

use antlift_core::{Aabb, Layout, ObjectState, V2};
use rapier2d::prelude::*;

pub use antlift_core::ROBOT_RADIUS;
pub const ROBOT_MAX_FORCE: f32 = 25.0;

fn v(p: V2) -> Vector {
    Vector::new(p.x, p.y)
}

fn p(v: Vector) -> V2 {
    V2::new(v.x, v.y)
}

pub struct RobotBody {
    pub handle: RigidBodyHandle,
    pub grip: Option<ImpulseJointHandle>,
    /// Where we gripped, in the object's local frame.
    pub grip_slot: V2,
    pub active: bool,
}

pub struct SimWorld {
    pub physics: PhysicsWorld,
    pub object: RigidBodyHandle,
    pub object_half: V2,
    pub robots: Vec<RobotBody>,
    pub crates: Vec<(RigidBodyHandle, V2)>,
}

impl SimWorld {
    pub fn new(layout: &Layout) -> Self {
        let mut physics = PhysicsWorld::new();
        physics.gravity = Vector::ZERO;

        for w in &layout.walls {
            let c = w.center();
            let h = w.half();
            physics.insert_collider(
                ColliderBuilder::cuboid(h.x, h.y)
                    .translation(v(c))
                    .friction(0.3),
                None,
            );
        }

        let (object, _) = physics.insert(
            RigidBodyBuilder::dynamic()
                .translation(v(layout.object_start))
                .rotation(layout.object_start_angle)
                .linear_damping(3.0)
                .angular_damping(3.0)
                .can_sleep(false),
            ColliderBuilder::cuboid(layout.object_half.x, layout.object_half.y)
                .density(8.0)
                .friction(0.3),
        );

        // Robots start parked in a loose grid in the corner of the first room.
        let robots = (0..layout.n_robots)
            .map(|i| {
                let start = V2::new(2.0 + (i % 4) as f32 * 1.0, 13.0 + (i / 4) as f32 * 1.0);
                let (handle, _) = physics.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(v(start))
                        .linear_damping(4.0)
                        .angular_damping(4.0)
                        .can_sleep(false),
                    ColliderBuilder::ball(ROBOT_RADIUS)
                        .density(5.0)
                        .friction(0.5),
                );
                RobotBody {
                    handle,
                    grip: None,
                    grip_slot: V2::ZERO,
                    active: true,
                }
            })
            .collect();

        Self {
            physics,
            object,
            object_half: layout.object_half,
            robots,
            crates: Vec::new(),
        }
    }

    pub fn step(&mut self, dt: f32) {
        self.physics.integration_parameters.dt = dt;
        self.physics.step();
    }

    pub fn object_state(&self) -> ObjectState {
        let b = &self.physics.bodies[self.object];
        ObjectState {
            pos: p(b.translation()),
            angle: b.rotation().angle(),
            vel: p(b.linvel()),
            angvel: b.angvel(),
            half: self.object_half,
        }
    }

    pub fn robot_pos(&self, i: usize) -> V2 {
        p(self.physics.bodies[self.robots[i].handle].translation())
    }

    pub fn robot_vel(&self, i: usize) -> V2 {
        p(self.physics.bodies[self.robots[i].handle].linvel())
    }

    /// Replaces last tick's drive force.
    pub fn drive(&mut self, i: usize, force: V2) {
        let f = force.clamp_len(ROBOT_MAX_FORCE);
        let b = &mut self.physics.bodies[self.robots[i].handle];
        b.reset_forces(false);
        b.add_force(v(f), true);
    }

    /// Pin the robot to the object where it stands now.
    pub fn grip(&mut self, i: usize) {
        if self.robots[i].grip.is_some() {
            return;
        }
        let o = self.object_state();
        let rel = (self.robot_pos(i) - o.pos).rotate(-o.angle);
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(v(rel))
            .local_anchor2(Vector::ZERO)
            .contacts_enabled(false);
        let h = self
            .physics
            .insert_impulse_joint(self.object, self.robots[i].handle, joint);
        self.robots[i].grip = Some(h);
        self.robots[i].grip_slot = rel;
    }

    pub fn release(&mut self, i: usize) {
        if let Some(h) = self.robots[i].grip.take() {
            self.physics.remove_impulse_joint(h);
        }
    }

    pub fn disable_robot(&mut self, i: usize) {
        self.release(i);
        self.drive(i, V2::ZERO);
        self.robots[i].active = false;
    }

    /// Remove the crate under `at`, if any. Returns whether one was removed.
    pub fn remove_crate_at(&mut self, at: V2) -> bool {
        let hit = self.crate_boxes().iter().position(|&(c, angle, half)| {
            let l = (at - c).rotate(-angle);
            l.x.abs() <= half.x && l.y.abs() <= half.y
        });
        let Some(i) = hit else { return false };
        let (h, _) = self.crates.remove(i);
        self.physics.remove_body(h);
        true
    }

    pub fn add_crate(&mut self, at: V2) {
        let half = V2::new(0.6, 0.6);
        let (h, _) = self.physics.insert(
            RigidBodyBuilder::dynamic()
                .translation(v(at))
                .linear_damping(6.0)
                .angular_damping(6.0),
            ColliderBuilder::cuboid(half.x, half.y).density(30.0),
        );
        self.crates.push((h, half));
    }

    pub fn crate_boxes(&self) -> Vec<(V2, f32, V2)> {
        self.crates
            .iter()
            .map(|(h, half)| {
                let b = &self.physics.bodies[*h];
                (p(b.translation()), b.rotation().angle(), *half)
            })
            .collect()
    }

    /// Conservative axis-aligned bounds of every crate, for the camera feed.
    pub fn crate_aabbs(&self) -> Vec<Aabb> {
        self.crate_boxes()
            .into_iter()
            .map(|(c, _, half)| Aabb::from_center(c, V2::new(half.len(), half.len())))
            .collect()
    }

    /// Hit the payload with an impulse at a world point (the "kick" tool).
    pub fn kick(&mut self, at: V2, impulse: V2) {
        let b = &mut self.physics.bodies[self.object];
        b.apply_impulse_at_point(v(impulse), v(at), true);
    }
}
