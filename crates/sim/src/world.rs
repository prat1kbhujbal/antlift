//! Rapier wrapper: the payloads, robots, walls and dropped crates.
//!
//! Top-down 2D with no gravity; floor friction is modelled with damping.

use antlift_core::{Aabb, Layout, ObjectState, Snapshot, V2};
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
    /// Which payload the grip is on (meaningful while `grip` is set).
    pub holding: usize,
    /// Where we gripped, in the object's local frame.
    pub grip_slot: V2,
    pub active: bool,
}

pub struct Payload {
    pub handle: RigidBodyHandle,
    pub half: V2,
}

pub struct SimWorld {
    pub physics: PhysicsWorld,
    pub objects: Vec<Payload>,
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

        let objects = layout
            .payloads
            .iter()
            .map(|spec| {
                let (handle, _) = physics.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(v(spec.start))
                        .rotation(spec.angle)
                        .linear_damping(3.0)
                        .angular_damping(3.0)
                        .can_sleep(false),
                    ColliderBuilder::cuboid(spec.half.x, spec.half.y)
                        .density(8.0)
                        .friction(0.3),
                );
                Payload {
                    handle,
                    half: spec.half,
                }
            })
            .collect();

        // Each team starts parked in a loose grid near its payload.
        let robots = (0..layout.n_robots())
            .map(|i| {
                let start = layout.spawn_pos(i);
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
                    holding: 0,
                    grip_slot: V2::ZERO,
                    active: true,
                }
            })
            .collect();

        Self {
            physics,
            objects,
            robots,
            crates: Vec::new(),
        }
    }

    /// Rebuild a world from what the edge's camera saw: the payload's pose
    /// and velocity, robot positions and grips, and dropped crates. This is
    /// the edge's model of the present, used to simulate possible futures.
    pub fn from_snapshot(layout: &Layout, snap: &Snapshot) -> Self {
        let mut l = layout.clone();
        for (spec, o) in l.payloads.iter_mut().zip(&snap.objects) {
            spec.start = o.pos;
            spec.angle = o.angle;
        }
        let mut w = Self::new(&l);
        for (k, o) in snap.objects.iter().enumerate() {
            let b = &mut w.physics.bodies[w.objects[k].handle];
            b.set_linvel(v(o.vel), true);
            b.set_angvel(o.angvel, true);
        }
        for (i, r) in snap.robots.iter().enumerate() {
            let h = w.robots[i].handle;
            w.physics.bodies[h].set_translation(v(r.pos), true);
            if !r.active {
                w.disable_robot(i);
            } else if r.attached {
                w.grip(i, r.holding as usize);
            }
        }
        for o in &snap.obstacles {
            w.add_crate(o.center());
        }
        w
    }

    pub fn step(&mut self, dt: f32) {
        self.physics.integration_parameters.dt = dt;
        self.physics.step();
    }

    pub fn object_state(&self, k: usize) -> ObjectState {
        let o = &self.objects[k];
        let b = &self.physics.bodies[o.handle];
        ObjectState {
            pos: p(b.translation()),
            angle: b.rotation().angle(),
            vel: p(b.linvel()),
            angvel: b.angvel(),
            half: o.half,
        }
    }

    pub fn object_states(&self) -> Vec<ObjectState> {
        (0..self.objects.len())
            .map(|k| self.object_state(k))
            .collect()
    }

    /// The payload whose centre is closest to `at`.
    pub fn nearest_object(&self, at: V2) -> usize {
        (0..self.objects.len())
            .min_by(|&a, &b| {
                let da = self.object_state(a).pos.dist(at);
                let db = self.object_state(b).pos.dist(at);
                da.total_cmp(&db)
            })
            .unwrap_or(0)
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

    /// Pin robot `i` to payload `k` where it stands now.
    pub fn grip(&mut self, i: usize, k: usize) {
        if self.robots[i].grip.is_some() {
            return;
        }
        let o = self.object_state(k);
        let rel = (self.robot_pos(i) - o.pos).rotate(-o.angle);
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(v(rel))
            .local_anchor2(Vector::ZERO)
            .contacts_enabled(false);
        let h =
            self.physics
                .insert_impulse_joint(self.objects[k].handle, self.robots[i].handle, joint);
        self.robots[i].grip = Some(h);
        self.robots[i].holding = k;
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

    /// Hit the payload nearest `at` with an impulse there (the "kick" tool).
    pub fn kick(&mut self, at: V2, impulse: V2) {
        let k = self.nearest_object(at);
        let b = &mut self.physics.bodies[self.objects[k].handle];
        b.apply_impulse_at_point(v(impulse), v(at), true);
    }
}
