//! The decision logic: tree execution, control laws and the reflex fallback.
//!
//! A brain only sees a [`Sense`], never the world. In speculative mode it
//! runs on the robot; in the centralized baseline the edge runs the very same
//! brain remotely and streams the resulting commands, so the two modes differ
//! only in *where* the decisions are made.

use crate::{
    axis_error, wrap_angle, Action, ContingencyTree, NodeId, ObjectState, Predicate, RobotId, Role,
    Uplink, DELIVERED_RADIUS, ROBOT_RADIUS, SLOT_OFFSET, V2,
};

/// Stalled = the payload has moved less than this, and turned less than
/// this, since the progress anchor was set. Measuring net progress rather
/// than instantaneous speed keeps a payload jiggling against a wall stalled.
const STALL_DIST: f32 = 0.15;
const STALL_TURN: f32 = 0.05;
/// How long a robot waits on an unplanned stall before shouting for help.
pub const SURPRISE_AFTER: f32 = 3.0;
/// Within this distance of the slot a robot may grip.
const GRIP_DIST: f32 = 0.3;
/// A planned jam recovery that keeps failing becomes a surprise after this many tries.
const JAM_RETRIES: u8 = 3;
/// Keep the current grip if the edge's slot is this close to it.
const SLOT_KEEP_DIST: f32 = 0.4;
const HEARTBEAT_PERIOD: f32 = 1.0;
/// A transit waypoint counts as reached within this distance (m)...
const VIA_REACHED: f32 = 0.6;
/// ...and the route is dropped for the direct approach this close to the slot (m).
const VIA_DROP: f32 = 2.5;
/// The final hold keeps the payload on its goal with this fraction of the
/// carrying effort.
const HOLD_GAIN: f32 = 0.5;
/// Robots park once their payload is this slow at its goal (m/s).
const PARK_SPEED: f32 = 0.1;
/// Permit value meaning "no traffic limit".
pub const NO_LIMIT: NodeId = NodeId::MAX;
/// Parked robots only check in this often.
const PARKED_HEARTBEAT: f32 = 5.0;
/// A parked robot whose payload is nudged further than this from its goal
/// goes back to work.
const UNPARK_RADIUS: f32 = 2.5;

/// Everything a robot can perceive locally this tick.
#[derive(Clone, Copy, Debug)]
pub struct Sense {
    pub t: f32,
    pub pos: V2,
    pub vel: V2,
    pub attached: bool,
    /// Current grip point in object frame, when attached.
    pub grip_slot: V2,
    /// Payload pose from on-board perception.
    pub object: ObjectState,
    /// Bearing beacon at the goal (the only global hint in reflex mode).
    pub goal: V2,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Command {
    pub force: V2,
    /// When set, the force is just `track(vel, own velocity)`: a remote
    /// controller can send this setpoint and let the robot close the loop.
    pub vel: Option<V2>,
    pub grip: bool,
    pub release: bool,
}

/// Robot-local velocity loop gain (N per m/s).
const VEL_GAIN: f32 = 20.0;

/// Force that drives a robot moving at `vel` toward `v_des`.
pub fn track(v_des: V2, vel: V2) -> V2 {
    (v_des - vel) * VEL_GAIN
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Following an edge-issued contingency tree.
    Tree,
    /// No valid tree: ant-style local rules toward the goal beacon.
    Reflex,
    /// Plan finished with the payload delivered: holding it, nearly silent.
    Parked,
}

/// Something worth drawing or counting this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Hit,
    Miss,
}

pub struct Brain {
    pub id: RobotId,
    /// The payload this robot works on. Starts from the layout; the edge
    /// can move it to another team through its tree.
    pub team: u8,
    pub tree: Option<ContingencyTree>,
    pub node: NodeId,
    node_time: f32,
    stall_time: f32,
    /// Payload pose when it last made real progress.
    anchor: Option<(V2, f32)>,
    next_heartbeat: f32,
    /// Slot chosen locally when running on reflexes.
    reflex_slot: Option<V2>,
    /// Planned jam recoveries tried since the last real progress.
    jam_tries: u8,
    /// Last tree edge taken: (from, to, speculative, sim time).
    pub last_edge: Option<(NodeId, NodeId, bool, f32)>,
    /// Delivered and holding: see [`Mode::Parked`].
    parked: bool,
    /// Next point of the tree's transit route to follow.
    via_next: usize,
    /// Traffic permit: furthest tree node this robot may advance to.
    permit: NodeId,
    /// This leg is done but the next one isn't permitted yet.
    traffic_hold: bool,
    /// Heading the payload had when the plan's final hold began.
    hold_heading: Option<f32>,
}

impl Brain {
    pub fn new(id: RobotId, team: u8) -> Self {
        Self {
            id,
            team,
            tree: None,
            node: 0,
            node_time: 0.0,
            stall_time: 0.0,
            anchor: None,
            // Stagger heartbeats so they don't all collide.
            next_heartbeat: id as f32 * 0.061 % HEARTBEAT_PERIOD,
            reflex_slot: None,
            jam_tries: 0,
            last_edge: None,
            parked: false,
            via_next: 0,
            permit: NO_LIMIT,
            traffic_hold: false,
            hold_heading: None,
        }
    }

    pub fn mode(&self) -> Mode {
        if self.parked {
            Mode::Parked
        } else if self.tree.is_some() {
            Mode::Tree
        } else {
            Mode::Reflex
        }
    }

    pub fn role(&self) -> Option<Role> {
        self.tree.as_ref().map(|t| t.role)
    }

    pub fn action(&self) -> Option<Action> {
        let t = self.tree.as_ref()?;
        Some(t.nodes.get(self.node as usize)?.action)
    }

    /// Install a tree. Returns `(version, valid_until)` if accepted, so the
    /// robot can acknowledge both the plan and the lease it now holds.
    pub fn receive(&mut self, bytes: &[u8]) -> Option<(u32, f32)> {
        let tree = ContingencyTree::decode(bytes)?;
        if self.tree.as_ref().is_some_and(|t| t.version > tree.version) {
            return None; // reordered stale packet
        }
        let ack = (tree.version, tree.valid_until);
        if tree.team != self.team {
            self.team = tree.team;
            self.reflex_slot = None;
        }
        let same_plan = self.tree.as_ref().is_some_and(|t| t.nodes == tree.nodes);
        // A transit route is planned from where the robot is now.
        self.via_next = 0;
        self.permit = tree.permit;
        self.tree = Some(tree);
        if !same_plan {
            self.enter(0);
        }
        Some(ack)
    }

    /// A traffic permit for plan `version` (ignored if it's for another plan).
    pub fn receive_permit(&mut self, version: u32, node: NodeId) {
        if self.tree.as_ref().is_some_and(|t| t.version == version) {
            self.permit = node;
        }
    }

    /// Transit waypoints still to visit on the way to the payload.
    pub fn transit(&self) -> &[V2] {
        self.tree
            .as_ref()
            .map_or(&[], |t| &t.via[self.via_next.min(t.via.len())..])
    }

    /// Furthest tree node this robot may currently advance to.
    pub fn permit(&self) -> NodeId {
        self.permit
    }

    /// Held back by traffic: this leg is done, the next isn't permitted yet.
    pub fn held_by_traffic(&self) -> bool {
        self.tree.is_some() && self.traffic_hold
    }

    pub fn tick(&mut self, s: &Sense, dt: f32, up: &mut Vec<Uplink>) -> (Command, Option<Event>) {
        // A lease expiring means the edge has gone quiet. A parked robot's
        // job is done, so it keeps holding rather than falling back to reflexes.
        if !self.parked && self.tree.as_ref().is_some_and(|t| s.t > t.valid_until) {
            self.tree = None;
        }

        let mut event = None;
        let mut cmd = Command::default();

        if self.tree.is_some() {
            event = self.follow_edges(s, up);
            let tree = self.tree.as_ref().unwrap();
            let role = tree.role;
            let slot = tree.slot;
            let action = tree.nodes[self.node as usize].action;

            // Swap grip if the edge moved our slot far away.
            if s.attached && s.grip_slot.dist(slot) > SLOT_KEEP_DIST {
                cmd.release = true;
            }

            let pushing = matches!(action, Action::Push { .. } | Action::Rotate { .. });
            // Waiting for traffic isn't a stall.
            if pushing && s.attached && !self.traffic_hold {
                let o = &s.object;
                let progressed = self.anchor.is_none_or(|(p, a)| {
                    o.pos.dist(p) > STALL_DIST || wrap_angle(o.angle - a).abs() > STALL_TURN
                });
                if progressed {
                    self.anchor = Some((o.pos, o.angle));
                    self.stall_time = 0.0;
                } else {
                    self.stall_time += dt;
                }
            } else {
                self.anchor = None;
                self.stall_time = 0.0;
            }

            if self.stall_time > SURPRISE_AFTER {
                let tree = self.tree.as_ref().unwrap();
                if !tree.has_stall_edge(self.node) {
                    // Keep shouting every SURPRISE_AFTER seconds while stuck.
                    self.stall_time = 0.0;
                    self.anchor = None;
                    event = Some(Event::Miss);
                    up.push(Uplink::Surprise {
                        robot: self.id,
                        version: tree.version,
                        node: self.node,
                        at: s.object.pos,
                    });
                }
            }

            let delivered =
                s.object.pos.dist(s.goal) < DELIVERED_RADIUS && s.object.vel.len() < PARK_SPEED;
            if !s.attached && delivered {
                // Our payload is already parked on its goal: pushing in to
                // grip it now would only shove it off. Stay put.
                cmd.vel = Some(V2::ZERO);
                cmd.force = track(V2::ZERO, s.vel);
            } else if !s.attached || cmd.release {
                let target = slot_world(&s.object, slot);
                let tree = self.tree.as_ref().unwrap();
                // Follow the transit route round walls first, skipping points
                // already reached, until the payload is close.
                while tree
                    .via
                    .get(self.via_next)
                    .is_some_and(|p| s.pos.dist(*p) < VIA_REACHED)
                {
                    self.via_next += 1;
                }
                let via = tree
                    .via
                    .get(self.via_next)
                    .copied()
                    .filter(|_| s.pos.dist(target) > VIA_DROP);
                let v = match via {
                    Some(p) => ((p - s.pos) * 2.0).clamp_len(1.5),
                    None => approach(s, slot),
                };
                cmd.vel = Some(v);
                cmd.force = track(v, s.vel);
                if s.pos.dist(slot_world(&s.object, slot)) < GRIP_DIST {
                    cmd.grip = true;
                }
            } else if is_final_hold(tree, self.node) && s.object.pos.dist(s.goal) < UNPARK_RADIUS {
                // A final hold at the goal keeps the payload on it (gently,
                // at the heading it arrived with), not just still: otherwise
                // it can coast or creep off it with nobody to bring it back.
                let heading = *self.hold_heading.get_or_insert(s.object.angle);
                let hold = Action::Push {
                    target: s.goal,
                    heading,
                };
                cmd.force = carry(s, hold, role) * HOLD_GAIN;
            } else if self.held_by_traffic() {
                // This leg isn't permitted yet: hold the payload where it is.
                cmd.force = carry(s, Action::Hold, role);
            } else {
                cmd.force = carry(s, action, role);
            }
        } else {
            cmd = self.reflex(s);
        }

        // Parked: the plan's final hold, gripping, with the payload at its goal.
        let at_goal = s.object.pos.dist(s.goal);
        let final_hold = self
            .tree
            .as_ref()
            .is_some_and(|t| is_final_hold(t, self.node));
        let settled = s.object.vel.len() < PARK_SPEED;
        if final_hold && s.attached && at_goal < DELIVERED_RADIUS && settled {
            self.parked = true;
        } else if self.parked && (!final_hold || at_goal > UNPARK_RADIUS) {
            self.parked = false;
        }

        if s.t >= self.next_heartbeat {
            let period = if self.parked {
                PARKED_HEARTBEAT
            } else {
                HEARTBEAT_PERIOD
            };
            self.next_heartbeat = s.t + period;
            up.push(Uplink::Heartbeat {
                robot: self.id,
                node: self.node,
                attached: s.attached,
            });
        }

        self.node_time += dt;
        (cmd, event)
    }

    fn enter(&mut self, node: NodeId) {
        self.hold_heading = None;
        self.node = node;
        self.node_time = 0.0;
        self.stall_time = 0.0;
        self.anchor = None;
        self.parked = false;
    }

    fn follow_edges(&mut self, s: &Sense, up: &mut Vec<Uplink>) -> Option<Event> {
        self.traffic_hold = false;
        let tree = self.tree.as_ref()?;
        let node = tree.nodes.get(self.node as usize)?;
        let edge = node
            .edges
            .iter()
            .find(|e| holds(e.when, s, self.node_time, self.stall_time))
            .copied()?;
        // Nominal progress stops at the traffic permit: hold here until the
        // edge grants the next leg. Gripping is always allowed.
        if !edge.speculative && edge.to > self.permit && edge.when != Predicate::InContact {
            self.traffic_hold = true;
            return None;
        }
        let version = tree.version;
        self.last_edge = Some((self.node, edge.to, edge.speculative, s.t));
        up.push(Uplink::BranchFired {
            robot: self.id,
            version,
            node: edge.to,
            speculative: edge.speculative,
        });
        self.enter(edge.to);
        if edge.when == Predicate::InContact {
            // Just (re)gripped: the swarm may be further along than we are.
            self.catch_up(s);
        }
        if !edge.speculative {
            self.jam_tries = 0;
        } else if matches!(edge.when, Predicate::Stalled { .. }) {
            self.jam_tries += 1;
            if self.jam_tries >= JAM_RETRIES {
                // The prediction was right but the planned fix isn't working.
                self.jam_tries = 0;
                up.push(Uplink::Surprise {
                    robot: self.id,
                    version,
                    node: edge.to,
                    at: s.object.pos,
                });
                return Some(Event::Miss);
            }
        }
        edge.speculative.then_some(Event::Hit)
    }

    /// Re-localize in the plan: follow the nominal chain from here and jump
    /// to the leg whose target the payload is clearly closest to, so a robot
    /// that gripped late doesn't drag the payload back to a passed waypoint.
    fn catch_up(&mut self, s: &Sense) {
        let Some(tree) = &self.tree else { return };
        let mut best: Option<(NodeId, f32)> = None;
        let mut n = self.node;
        loop {
            let Some(node) = tree.nodes.get(n as usize) else {
                break;
            };
            if n > self.permit && best.is_some() {
                break; // not permitted to skip ahead past the traffic limit
            }
            if let Action::Push { target, .. } = node.action {
                let d = s.object.pos.dist(target);
                match best {
                    // The first leg anchors the comparison, even if we're
                    // sitting on a turn before it.
                    None => best = Some((self.node, d)),
                    Some((_, bd)) if d + 0.5 < bd => best = Some((n, d)),
                    _ => {}
                }
            }
            // The nominal chain only ever moves forward.
            match node.edges.iter().find(|e| !e.speculative) {
                Some(e) if e.to > n => n = e.to,
                _ => break,
            }
        }
        if let Some((target, _)) = best {
            if target != self.node {
                self.enter(target);
            }
        }
    }

    /// Ant rules: grab the nearest free bit of the payload and shove it
    /// straight at the goal beacon. Works in open space, jams at walls.
    fn reflex(&mut self, s: &Sense) -> Command {
        if !s.attached {
            let slot = *self
                .reflex_slot
                .get_or_insert_with(|| nearest_slot(&s.object, s.pos));
            let v = approach(s, slot);
            let mut cmd = Command {
                force: track(v, s.vel),
                vel: Some(v),
                ..Default::default()
            };
            if s.pos.dist(slot_world(&s.object, slot)) < GRIP_DIST {
                cmd.grip = true;
            }
            return cmd;
        }
        self.reflex_slot = None;
        let heading = (s.goal - s.object.pos).angle();
        let role = if s.grip_slot.x.abs() > s.object.half.x * 0.6 {
            Role::Steerer
        } else {
            Role::Pusher
        };
        Command {
            force: carry(
                s,
                Action::Push {
                    target: s.goal,
                    heading,
                },
                role,
            ),
            ..Default::default()
        }
    }
}

/// The plan's last node: hold with nowhere further to go.
fn is_final_hold(tree: &ContingencyTree, node: NodeId) -> bool {
    let n = &tree.nodes[node as usize];
    n.action == Action::Hold && n.edges.is_empty()
}

fn holds(p: Predicate, s: &Sense, node_time: f32, stall_time: f32) -> bool {
    match p {
        Predicate::InContact => s.attached,
        Predicate::ContactLost => !s.attached,
        Predicate::Reached { target, radius } => s.object.pos.dist(target) < radius,
        Predicate::HeadingOff { heading, tol } => axis_error(heading, s.object.angle).abs() > tol,
        Predicate::HeadingOk { heading, tol } => axis_error(heading, s.object.angle).abs() < tol,
        Predicate::Stalled { secs } => stall_time >= secs,
        Predicate::After { secs } => node_time >= secs,
    }
}

pub fn slot_world(o: &ObjectState, slot: V2) -> V2 {
    o.pos + slot.rotate(o.angle)
}

/// Closest grip point on the payload's boundary to `pos`, in object frame.
pub fn nearest_slot(o: &ObjectState, pos: V2) -> V2 {
    let l = (pos - o.pos).rotate(-o.angle);
    let h = o.half;
    let c = V2::new(l.x.clamp(-h.x, h.x), l.y.clamp(-h.y, h.y));
    // Snap to the nearest face, then push out by the robot radius.
    let dx = h.x - c.x.abs();
    let dy = h.y - c.y.abs();
    if dx < dy {
        V2::new(c.x.signum() * (h.x + SLOT_OFFSET), c.y)
    } else {
        V2::new(c.x, c.y.signum() * (h.y + SLOT_OFFSET))
    }
}

/// Velocity to drive to a slot, swinging wide so we don't plough through the
/// payload, and matching the slot's own velocity to catch a moving payload.
fn approach(s: &Sense, slot: V2) -> V2 {
    let o = &s.object;
    let target = slot_world(o, slot);
    let to_target = target - s.pos;
    let outward = face_normal(o, slot).rotate(o.angle);
    let aim = if to_target.len() > 0.8 {
        target + outward * 0.8
    } else {
        target
    };
    let r = slot.rotate(o.angle);
    let slot_vel = o.vel + r.perp() * o.angvel;

    // If the payload is in the way, orbit around it instead.
    let me = (s.pos - o.pos).rotate(-o.angle);
    // Stand-off point in front of the slot; always outside the padded box.
    let standoff = slot + face_normal(o, slot) * 0.8;
    let pad = o.half + V2::new(ROBOT_RADIUS + 0.1, ROBOT_RADIUS + 0.1);
    let steer = if segment_hits_box(me, standoff, pad) {
        let around = if me.cross(standoff) >= 0.0 { 1.0 } else { -1.0 };
        let tangent = me.perp().norm() * around;
        let gap = box_dist(me, pad);
        let outward = (me - clamp_to_box(me, pad)).norm();
        (tangent * 1.2 + outward * (0.6 - gap).max(0.0) * 3.0).rotate(o.angle)
    } else {
        ((aim - s.pos) * 2.5).clamp_len(1.5)
    };
    slot_vel + steer
}

fn clamp_to_box(p: V2, half: V2) -> V2 {
    V2::new(p.x.clamp(-half.x, half.x), p.y.clamp(-half.y, half.y))
}

fn box_dist(p: V2, half: V2) -> f32 {
    p.dist(clamp_to_box(p, half))
}

/// Slab test: does the segment a-b pass through the centred box?
fn segment_hits_box(a: V2, b: V2, half: V2) -> bool {
    let d = b - a;
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    for (o, dir, h) in [(a.x, d.x, half.x), (a.y, d.y, half.y)] {
        if dir.abs() < 1e-6 {
            if o.abs() > h {
                return false;
            }
            continue;
        }
        let (mut lo, mut hi) = ((-h - o) / dir, (h - o) / dir);
        if lo > hi {
            std::mem::swap(&mut lo, &mut hi);
        }
        t0 = t0.max(lo);
        t1 = t1.min(hi);
        if t0 > t1 {
            return false;
        }
    }
    true
}

/// Outward normal of the payload face a slot sits on (object frame).
fn face_normal(o: &ObjectState, slot: V2) -> V2 {
    let dx = slot.x.abs() - o.half.x;
    let dy = slot.y.abs() - o.half.y;
    if dx > dy {
        V2::new(slot.x.signum(), 0.0)
    } else {
        V2::new(0.0, slot.y.signum())
    }
}

/// Gripped control law. Every robot runs the same law on the same estimate,
/// so the swarm agrees on what to do without talking.
fn carry(s: &Sense, action: Action, role: Role) -> V2 {
    let o = &s.object;
    let (v_des, heading) = match action {
        Action::Push { target, heading } => {
            let err = axis_error(heading, o.angle).abs();
            // Don't drive forward while badly misaligned: turn first.
            let align = (1.0 - err / 0.35).clamp(0.0, 1.0);
            let speed = (o.pos.dist(target) * 0.8).min(0.8);
            ((target - o.pos).norm() * speed * align, Some(heading))
        }
        Action::Rotate { heading } => (V2::ZERO, Some(heading)),
        Action::Hold | Action::Approach => (V2::ZERO, None),
    };

    let mut f = v_des * 12.0 + (v_des - o.vel) * 20.0;

    if role == Role::Steerer {
        let w_des = match heading {
            Some(h) => (axis_error(h, o.angle) * 1.5).clamp(-0.6, 0.6),
            None => 0.0,
        };
        let r = s.grip_slot.rotate(o.angle);
        let tangent = r.perp().norm();
        f += tangent * (w_des * 10.0 + (w_des - o.angvel) * 20.0);
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Edge, Node};

    fn push(target: V2, to: NodeId) -> Node {
        Node {
            action: Action::Push {
                target,
                heading: 0.0,
            },
            edges: vec![Edge {
                when: Predicate::Reached {
                    target,
                    radius: 0.5,
                },
                to,
                speculative: false,
            }],
        }
    }

    #[test]
    fn a_permit_holds_the_robot_at_the_end_of_its_grant() {
        let tree = ContingencyTree {
            version: 7,
            valid_until: 100.0,
            role: Role::Pusher,
            team: 0,
            slot: V2::new(-1.27, 0.0),
            nodes: vec![
                Node {
                    action: Action::Approach,
                    edges: vec![Edge {
                        when: Predicate::InContact,
                        to: 1,
                        speculative: false,
                    }],
                },
                push(V2::new(1.0, 0.0), 2),
                push(V2::new(3.0, 0.0), 3),
                Node {
                    action: Action::Hold,
                    edges: vec![],
                },
            ],
            permit: 1,
            via: vec![],
        };
        let mut b = Brain::new(0, 0);
        b.receive(&tree.encode()).unwrap();
        // Gripping, with the payload already at the first waypoint.
        let s = Sense {
            t: 0.0,
            pos: V2::new(-0.27, 0.0),
            vel: V2::ZERO,
            attached: true,
            grip_slot: V2::new(-1.27, 0.0),
            object: ObjectState {
                pos: V2::new(1.0, 0.0),
                half: V2::new(1.0, 0.5),
                ..Default::default()
            },
            goal: V2::new(3.0, 0.0),
        };
        let mut up = vec![];
        for _ in 0..3 {
            b.tick(&s, 0.1, &mut up);
        }
        assert!(b.held_by_traffic(), "next leg isn't permitted");
        assert!(b.action() == Some(tree.nodes[1].action));
        // Waiting for traffic is not a stall: nothing gets reported.
        for _ in 0..100 {
            b.tick(&s, 0.1, &mut up);
        }
        assert!(!up.iter().any(|u| matches!(u, Uplink::Surprise { .. })));
        // A permit for another plan is ignored; one for this plan releases it.
        b.receive_permit(6, NO_LIMIT);
        b.tick(&s, 0.1, &mut up);
        assert!(b.held_by_traffic());
        b.receive_permit(7, NO_LIMIT);
        b.tick(&s, 0.1, &mut up);
        assert!(!b.held_by_traffic());
        assert!(b.action() == Some(tree.nodes[2].action));
    }
}
