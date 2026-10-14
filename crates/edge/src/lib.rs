//! The edge server: plans the payload's route, speculates on what might go
//! wrong, and ships each robot a byte-budgeted contingency tree for its role.
//!
//! It is event-driven. It replans only when the world surprises it (a robot
//! reports a stall it had no branch for, obstacles change, the payload drifts
//! off the route, or robots reach the end of their planning horizon), and
//! otherwise just renews tree leases.
//!
//! In [`ControlMode::Centralized`] it is the baseline instead: it runs every
//! robot's brain itself on streamed robot state and sends force commands.

pub mod grid;

use antlift_core::{
    axis_error, Aabb, Action, Brain, ContingencyTree, ControlMode, DownBody, Downlink, Edge, Event,
    Layout, Node, NodeId, ObjectState, Predicate, Role, Sense, Snapshot, Uplink, ROBOT_RADIUS,
    SLOT_OFFSET, V2,
};
use grid::{Footprint, Grid, Pose};
use std::f32::consts::FRAC_PI_2;
use std::time::Instant;

#[derive(Clone, Copy, Debug)]
pub struct EdgeConfig {
    /// Max encoded tree size per robot, in bytes.
    pub tree_budget: usize,
    /// How long a tree stays valid without renewal (s).
    pub lease: f32,
    /// Renew leases this often (s). Must be below `lease`.
    pub refresh: f32,
    /// Max waypoints per tree.
    pub horizon: usize,
    /// Resend a tree if it isn't acknowledged within this long (s).
    pub resend_after: f32,
    pub control: ControlMode,
    /// Centralized mode: command rate (s).
    pub command_period: f32,
}

impl Default for EdgeConfig {
    fn default() -> Self {
        Self {
            tree_budget: 256,
            lease: 12.0,
            refresh: 8.0,
            horizon: 8,
            resend_after: 0.4,
            control: ControlMode::Speculative,
            command_period: 0.05,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Contingency {
    /// Payload wedged against something.
    Jam,
    /// Heading error large enough that pushers should pause.
    Misalign,
    /// Robot loses its grip.
    GripLoss,
}

impl Contingency {
    /// Rough cost of not having a branch for it.
    fn impact(self) -> f32 {
        match self {
            Contingency::Jam => 1.0,
            Contingency::Misalign => 0.5,
            Contingency::GripLoss => 0.6,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Candidate {
    pub wp: usize,
    pub kind: Contingency,
    /// Estimated probability it happens on this leg.
    pub p: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Waypoint {
    pub pos: V2,
    pub heading: f32,
    /// 0 = open space, 1 = barely fits.
    pub tightness: f32,
    /// Rotate here before moving on.
    pub turn_in_place: bool,
}

#[derive(Clone, Debug)]
pub struct Plan {
    pub version: u32,
    pub start: V2,
    pub path: Vec<V2>,
    pub waypoints: Vec<Waypoint>,
    pub candidates: Vec<Candidate>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EdgeStats {
    pub plans: u64,
    pub hits_reported: u64,
    pub surprises: u64,
    pub trees_sent: u64,
    pub grip_losses: u64,
    pub last_plan_us: u128,
    /// Centralized mode: branches/surprises inside the edge-hosted brains.
    pub local_hits: u64,
    pub local_misses: u64,
    pub commands_sent: u64,
}

struct RobotBook {
    active: bool,
    attached: bool,
    slot: Option<V2>,
    sent_version: u32,
    /// Lease end of the last tree sent, and of the last one acknowledged.
    sent_until: f32,
    acked_until: f32,
    last_sent: f32,
    /// When the current tree/lease was first sent (for RTT samples).
    first_sent: f32,
    retries: u32,
    last_bytes: Vec<u8>,
    /// Smoothed round-trip estimate from tree acks (s).
    rtt: f32,
    end_node: NodeId,
    /// Centralized mode: the brain we run on this robot's behalf.
    brain: Brain,
    /// Centralized mode: last streamed state.
    state: Option<RemoteState>,
}

#[derive(Clone, Copy)]
struct RemoteState {
    pos: V2,
    vel: V2,
    attached: bool,
    /// Where it gripped, in the payload frame (as reported by the robot).
    grip_slot: V2,
}

pub struct EdgeServer {
    layout: Layout,
    cfg: EdgeConfig,
    snapshot: Option<Snapshot>,
    plan: Option<Plan>,
    version: u32,
    need_replan: bool,
    robots: Vec<RobotBook>,
    /// Coarse (2 m) map of where unpredicted stalls happened. Learned prior.
    heat: Vec<f32>,
    heat_nx: usize,
    grid_cache: Option<(Vec<Aabb>, Grid)>,
    last_plan_t: f32,
    /// Sim time of the latest tick, for timing acks.
    now: f32,
    next_command: f32,
    pub stats: EdgeStats,
}

const HEAT_CELL: f32 = 2.0;
/// Seconds between attempts when no route could be found.
const PLAN_RETRY: f32 = 2.0;

impl EdgeServer {
    pub fn new(layout: Layout, cfg: EdgeConfig) -> Self {
        let size = layout.arena.max - layout.arena.min;
        let heat_nx = (size.x / HEAT_CELL).ceil() as usize;
        let heat_ny = (size.y / HEAT_CELL).ceil() as usize;
        let robots = (0..layout.n_robots)
            .map(|i| RobotBook {
                active: true,
                attached: false,
                slot: None,
                sent_version: 0,
                sent_until: 0.0,
                acked_until: 0.0,
                last_sent: f32::NEG_INFINITY,
                first_sent: 0.0,
                retries: 0,
                last_bytes: vec![],
                rtt: 0.2,
                end_node: 0,
                brain: Brain::new(i as u16),
                state: None,
            })
            .collect();
        Self {
            layout,
            cfg,
            snapshot: None,
            plan: None,
            version: 0,
            need_replan: true,
            robots,
            heat: vec![0.0; heat_nx * heat_ny],
            heat_nx,
            grid_cache: None,
            last_plan_t: f32::NEG_INFINITY,
            now: 0.0,
            next_command: 0.0,
            stats: EdgeStats::default(),
        }
    }

    /// Switch control mode mid-run. Forces a fresh plan version so every
    /// robot (or edge-hosted brain) gets a newly installed tree.
    pub fn set_control(&mut self, control: ControlMode) {
        if self.cfg.control == control {
            return;
        }
        self.cfg.control = control;
        self.need_replan = true;
        for r in &mut self.robots {
            r.retries = 0;
            r.acked_until = 0.0;
        }
    }

    pub fn plan(&self) -> Option<&Plan> {
        self.plan.as_ref()
    }

    /// Centralized mode: the brains the edge runs on the robots' behalf.
    pub fn remote_brains(&self) -> impl Iterator<Item = &Brain> {
        self.robots.iter().map(|r| &r.brain)
    }

    /// The learned surprise map: (values, columns, cell size in m).
    pub fn heat_grid(&self) -> (&[f32], usize, f32) {
        (&self.heat, self.heat_nx, HEAT_CELL)
    }

    pub fn heat_at(&self, p: V2) -> f32 {
        let l = p - self.layout.arena.min;
        let x = (l.x / HEAT_CELL).max(0.0) as usize;
        let y = (l.y / HEAT_CELL).max(0.0) as usize;
        self.heat
            .get(y * self.heat_nx + x.min(self.heat_nx - 1))
            .copied()
            .unwrap_or(0.0)
    }

    pub fn handle(&mut self, msg: Uplink) {
        match msg {
            Uplink::Camera(s) => self.on_snapshot(s),
            Uplink::Heartbeat {
                robot, attached, ..
            } => {
                if let Some(r) = self.robots.get_mut(robot as usize) {
                    if r.attached && !attached {
                        self.stats.grip_losses += 1;
                    }
                    r.attached = attached;
                }
            }
            Uplink::BranchFired {
                robot,
                version,
                node,
                speculative,
            } => {
                if speculative {
                    self.stats.hits_reported += 1;
                }
                let at_end = self
                    .robots
                    .get(robot as usize)
                    .is_some_and(|r| r.end_node == node);
                if version == self.version && at_end {
                    self.need_replan = true;
                }
            }
            Uplink::TreeAck {
                robot,
                version,
                valid_until,
            } => {
                let now = self.now;
                if let Some(r) = self.robots.get_mut(robot as usize) {
                    let fresh = r.acked_until + 0.01 < r.sent_until;
                    if version == r.sent_version {
                        r.acked_until = r.acked_until.max(valid_until);
                    }
                    // Karn's rule: only time acks for copies sent exactly once.
                    let matches = (valid_until - r.sent_until).abs() < 0.01;
                    if fresh && matches && r.retries == 0 {
                        r.rtt = 0.8 * r.rtt + 0.2 * (now - r.first_sent).max(0.0);
                    }
                }
            }
            Uplink::Surprise { at, .. } => {
                self.stats.surprises += 1;
                self.bump_heat(at);
                self.need_replan = true;
            }
            Uplink::State {
                robot,
                pos,
                vel,
                grip,
            } => {
                let Some(r) = self.robots.get_mut(robot as usize) else {
                    return;
                };
                let attached = grip.is_some();
                if r.state.is_some_and(|s| s.attached) && !attached {
                    self.stats.grip_losses += 1;
                }
                r.attached = attached;
                r.state = Some(RemoteState {
                    pos,
                    vel,
                    attached,
                    grip_slot: grip.unwrap_or(V2::ZERO),
                });
            }
        }
    }

    /// Call every sim tick; returns what to send to the robots.
    pub fn tick(&mut self, now: f32) -> Vec<Downlink> {
        self.now = now;
        if self.snapshot.is_none() {
            return vec![];
        }
        // A failed plan is retried on a timer, not on every camera frame.
        let failed = self.plan.as_ref().is_some_and(|p| p.path.is_empty());
        if failed && now - self.last_plan_t > PLAN_RETRY {
            self.need_replan = true;
        }
        if self.need_replan || self.plan.is_none() {
            self.last_plan_t = now;
            self.replan();
        }
        match self.cfg.control {
            ControlMode::Speculative => self.send_trees(now),
            ControlMode::Centralized => self.command_robots(now),
        }
    }

    fn send_trees(&mut self, now: f32) -> Vec<Downlink> {
        let mut out = vec![];
        for id in 0..self.robots.len() {
            let r = &mut self.robots[id];
            if !r.active {
                continue;
            }
            let since = now - r.last_sent;
            let current = r.sent_version == self.version;
            // Tolerate the wire format's millisecond quantization.
            let acked = r.acked_until + 0.01 >= r.sent_until;

            // Retransmit the identical bytes (so a late ack still matches),
            // with an RTT-based timeout and exponential backoff.
            let timeout = (2.5 * r.rtt).max(self.cfg.resend_after) * (1 << r.retries.min(3)) as f32;
            if current && !acked && since >= timeout {
                r.retries += 1;
                r.last_sent = now;
                self.stats.trees_sent += 1;
                out.push(Downlink {
                    robot: id as u16,
                    body: DownBody::Tree(r.last_bytes.clone()),
                });
                continue;
            }
            if current && since < self.cfg.refresh {
                continue;
            }

            // New plan, or time to renew the lease.
            let Some(tree) = self.build_tree(id, now) else {
                continue;
            };
            let bytes = tree.encode();
            let r = &mut self.robots[id];
            if r.sent_version != self.version {
                r.acked_until = 0.0;
            }
            r.sent_version = self.version;
            r.sent_until = tree.valid_until;
            r.last_sent = now;
            r.first_sent = now;
            r.retries = 0;
            r.last_bytes = bytes.clone();
            // The final Hold sits right after the push chain; contingency nodes follow it.
            r.end_node = self.plan.as_ref().map_or(0, |p| {
                (p.waypoints.len().min(self.cfg.horizon) + 1) as NodeId
            });
            self.stats.trees_sent += 1;
            out.push(Downlink {
                robot: id as u16,
                body: DownBody::Tree(bytes),
            });
        }
        out
    }

    /// Baseline: run each robot's brain here on the freshest state we have,
    /// using the very same trees, and stream the resulting commands.
    fn command_robots(&mut self, now: f32) -> Vec<Downlink> {
        let end_node = self.plan.as_ref().map_or(0, |p| {
            (p.waypoints.len().min(self.cfg.horizon) + 1) as NodeId
        });
        for id in 0..self.robots.len() {
            if !self.robots[id].active || self.robots[id].sent_version == self.version {
                continue;
            }
            if let Some(mut tree) = self.build_tree(id, now) {
                tree.valid_until = f32::MAX; // no lease: it never leaves the edge
                let r = &mut self.robots[id];
                r.brain.receive(&tree.encode());
                r.sent_version = self.version;
                r.end_node = end_node;
            }
        }

        if now < self.next_command {
            return vec![];
        }
        let dt = self.cfg.command_period;
        self.next_command = now + dt;
        let Some(snap) = &self.snapshot else {
            return vec![];
        };
        let object = extrapolate(snap.object, now - snap.t);
        let goal = self.layout.goal;

        let mut out = vec![];
        let mut internal = vec![];
        for (id, r) in self.robots.iter_mut().enumerate() {
            let Some(st) = r.state.filter(|_| r.active) else {
                continue;
            };
            let sense = Sense {
                t: now,
                pos: st.pos,
                vel: st.vel,
                attached: st.attached,
                grip_slot: st.grip_slot,
                object,
                goal,
            };
            let (cmd, event) = r.brain.tick(&sense, dt, &mut internal);
            match event {
                Some(Event::Hit) => self.stats.local_hits += 1,
                Some(Event::Miss) => self.stats.local_misses += 1,
                None => {}
            }
            self.stats.commands_sent += 1;
            out.push(Downlink {
                robot: id as u16,
                body: DownBody::Command {
                    force: cmd.force,
                    vel: cmd.vel,
                    grip: cmd.grip,
                    release: cmd.release,
                },
            });
        }
        for u in internal {
            if !matches!(u, Uplink::Heartbeat { .. }) {
                self.handle(u);
            }
        }
        out
    }

    fn on_snapshot(&mut self, s: Snapshot) {
        let prev = self.snapshot.take();
        let obstacles_changed = prev.as_ref().is_none_or(|p| {
            p.obstacles.len() != s.obstacles.len()
                || p.obstacles
                    .iter()
                    .zip(&s.obstacles)
                    .any(|(a, b)| a.center().dist(b.center()) > 0.5)
        });
        if obstacles_changed {
            self.need_replan = true;
        }
        if let Some(plan) = self.plan.as_ref().filter(|p| !p.path.is_empty()) {
            let off = plan
                .path
                .iter()
                .map(|p| p.dist(s.object.pos))
                .fold(f32::INFINITY, f32::min);
            if off > 2.5 {
                self.need_replan = true;
            }
            // Near the end of a truncated horizon: extend it, even if the
            // robots' "reached the end" reports got lost.
            if plan.waypoints.len() > self.cfg.horizon
                && plan.waypoints[self.cfg.horizon - 1].pos.dist(s.object.pos) < 1.2
            {
                self.need_replan = true;
            }
        }
        let mut roster_changed = false;
        for o in &s.robots {
            if let Some(r) = self.robots.get_mut(o.id as usize) {
                if r.active != o.active {
                    r.active = o.active;
                    roster_changed = true;
                }
            }
        }
        self.snapshot = Some(s);
        if roster_changed || self.robots.iter().all(|r| r.slot.is_none()) {
            self.assign_slots();
            self.need_replan = true;
        }
    }

    fn bump_heat(&mut self, at: V2) {
        let l = at - self.layout.arena.min;
        let x = (l.x / HEAT_CELL).max(0.0) as usize;
        let y = (l.y / HEAT_CELL).max(0.0) as usize;
        if let Some(h) = self
            .heat
            .get_mut(y * self.heat_nx + x.min(self.heat_nx - 1))
        {
            *h = (*h + 0.5).min(1.0);
        }
    }

    /// Spread active robots evenly around the payload. Robots already
    /// gripping keep their grip point so we don't make them let go.
    fn assign_slots(&mut self) {
        let Some(s) = &self.snapshot else { return };
        let o = s.object;
        let local = |p: V2| (p - o.pos).rotate(-o.angle);

        let active: Vec<usize> = (0..self.robots.len())
            .filter(|&i| self.robots[i].active)
            .collect();
        let mut taken: Vec<V2> = vec![];
        for &i in &active {
            let obs = &s.robots[i];
            self.robots[i].slot = if obs.attached {
                let l = local(obs.pos);
                taken.push(l);
                Some(l)
            } else {
                None
            };
        }

        let mut free: Vec<V2> = perimeter_slots(o.half, active.len())
            .into_iter()
            .filter(|p| taken.iter().all(|t| t.dist(*p) > 2.0 * ROBOT_RADIUS + 0.05))
            .collect();
        for &i in &active {
            if self.robots[i].slot.is_some() {
                continue;
            }
            let me = local(s.robots[i].pos);
            let best = free
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.dist(me).total_cmp(&b.1.dist(me)))
                .map(|(k, _)| k);
            if let Some(k) = best {
                self.robots[i].slot = Some(free.swap_remove(k));
            }
        }
    }

    fn grid(&mut self, extra: &[Aabb]) -> &Grid {
        let stale = self
            .grid_cache
            .as_ref()
            .is_none_or(|(obs, _)| obs.as_slice() != extra);
        if stale {
            let mut all = self.layout.walls.clone();
            all.extend_from_slice(extra);
            self.grid_cache = Some((extra.to_vec(), Grid::new(self.layout.arena, &all)));
        }
        &self.grid_cache.as_ref().unwrap().1
    }

    fn replan(&mut self) {
        let t0 = Instant::now();
        self.need_replan = false;
        self.version += 1;
        for h in &mut self.heat {
            *h *= 0.98;
        }
        let s = self.snapshot.clone().expect("snapshot before plan");
        let goal = self.layout.goal;
        // Payload plus the robots gripping it.
        let fp = Footprint::new(self.layout.object_half + V2::new(SLOT_OFFSET, SLOT_OFFSET) * 2.0);
        let start = Pose {
            pos: s.object.pos,
            heading: s.object.angle,
        };
        // Crates can be shoved aside: if they block every route, plan through
        // them and let the jam contingencies (and the heat map) deal with it.
        let mut poses = self.grid(&s.obstacles).plan(start, goal, &fp);
        if poses.is_none() && !s.obstacles.is_empty() {
            poses = self.grid(&[]).plan(start, goal, &fp);
        }
        let poses = poses.unwrap_or_default();
        let grid = self.grid(&s.obstacles);

        // Thin the pose chain into waypoints: every 2 m in the open, every
        // 1 m where it's tight, and wherever the heading changes notably.
        let mut waypoints: Vec<Waypoint> = vec![];
        let (mut last_pos, mut last_heading) = (start.pos, start.heading);
        for (k, p) in poses.iter().enumerate() {
            let is_last = k + 1 == poses.len();
            let slack = grid.slack(*p, &fp);
            let spacing = if slack < 0.5 { 1.0 } else { 2.0 };
            let moved = p.pos.dist(last_pos);
            let turned = axis_error(p.heading, last_heading).abs();
            if !(is_last || moved >= spacing || turned >= 0.35) {
                continue;
            }
            waypoints.push(Waypoint {
                pos: if is_last { goal } else { p.pos },
                heading: p.heading,
                tightness: (1.0 - slack / 0.6).clamp(0.0, 1.0),
                turn_in_place: moved < 0.5 && !is_last,
            });
            last_pos = p.pos;
            last_heading = p.heading;
        }
        let path: Vec<V2> = poses.iter().map(|p| p.pos).collect();

        let grip_p = (0.03 + 0.05 * self.stats.grip_losses as f32).min(0.8);
        let mut candidates = vec![];
        for (i, w) in waypoints.iter().enumerate().take(self.cfg.horizon) {
            let prev_heading = if i == 0 {
                s.object.angle
            } else {
                waypoints[i - 1].heading
            };
            let turn = axis_error(w.heading, prev_heading).abs();
            candidates.push(Candidate {
                wp: i,
                kind: Contingency::Jam,
                p: (0.05 + 0.8 * w.tightness + self.heat_at(w.pos)).min(1.0),
            });
            candidates.push(Candidate {
                wp: i,
                kind: Contingency::Misalign,
                p: (turn / FRAC_PI_2).min(1.0),
            });
            candidates.push(Candidate {
                wp: i,
                kind: Contingency::GripLoss,
                p: grip_p,
            });
        }
        candidates.sort_by(|a, b| (b.p * b.kind.impact()).total_cmp(&(a.p * a.kind.impact())));

        self.plan = Some(Plan {
            version: self.version,
            start: s.object.pos,
            path,
            waypoints,
            candidates,
        });
        self.stats.plans += 1;
        self.stats.last_plan_us = t0.elapsed().as_micros();
    }

    /// Nominal chain plus the most valuable contingencies that fit the budget,
    /// pruned to what this robot's role can act on.
    fn build_tree(&self, id: usize, now: f32) -> Option<ContingencyTree> {
        let plan = self.plan.as_ref()?;
        let slot = self.robots[id].slot?;
        let half = self.layout.object_half;
        let role = if slot.x.abs() > half.x * 0.6 {
            Role::Steerer
        } else {
            Role::Pusher
        };
        let wps = &plan.waypoints[..plan.waypoints.len().min(self.cfg.horizon)];
        let push_node = |i: usize| (i + 1) as NodeId;
        let final_node = (wps.len() + 1) as NodeId;

        let mut nodes = vec![Node {
            action: Action::Approach,
            edges: vec![nominal(Predicate::InContact, 1)],
        }];
        for (i, w) in wps.iter().enumerate() {
            let next = push_node(i + 1).min(final_node);
            nodes.push(if w.turn_in_place {
                Node {
                    action: Action::Rotate { heading: w.heading },
                    edges: vec![nominal(
                        Predicate::HeadingOk {
                            heading: w.heading,
                            tol: 0.1,
                        },
                        next,
                    )],
                }
            } else {
                // Hit squeeze waypoints precisely so the payload lines up with the gap.
                let radius = if i + 1 == wps.len() {
                    0.6
                } else {
                    1.0 - 0.7 * w.tightness
                };
                Node {
                    action: Action::Push {
                        target: w.pos,
                        heading: w.heading,
                    },
                    edges: vec![nominal(
                        Predicate::Reached {
                            target: w.pos,
                            radius,
                        },
                        next,
                    )],
                }
            });
        }
        nodes.push(Node {
            action: Action::Hold,
            edges: vec![],
        });

        let mut tree = ContingencyTree {
            version: plan.version,
            valid_until: now + self.cfg.lease,
            role,
            slot,
            nodes,
        };

        for c in &plan.candidates {
            if c.wp >= wps.len() {
                continue;
            }
            if c.kind == Contingency::Misalign && role == Role::Steerer {
                continue; // steerers are the ones fixing the heading
            }
            if c.p < 0.02 {
                continue;
            }
            let before = tree.clone();
            add_contingency(&mut tree, plan, c, push_node(c.wp));
            if tree.encode().len() > self.cfg.tree_budget {
                tree = before;
            }
        }
        Some(tree)
    }
}

fn nominal(when: Predicate, to: NodeId) -> Edge {
    Edge {
        when,
        to,
        speculative: false,
    }
}

fn spec(when: Predicate, to: NodeId) -> Edge {
    Edge {
        when,
        to,
        speculative: true,
    }
}

/// Append recovery nodes for `c` and hook them in front of push node `at`.
fn add_contingency(tree: &mut ContingencyTree, plan: &Plan, c: &Candidate, at: NodeId) {
    let w = plan.waypoints[c.wp];
    let next = tree.nodes.len() as NodeId;
    let trigger = match c.kind {
        Contingency::Jam => {
            // Back off towards where we came from, square up, try again.
            let back = if c.wp == 0 {
                plan.start
            } else {
                plan.waypoints[c.wp - 1].pos
            };
            tree.nodes.push(Node {
                action: Action::Push {
                    target: back,
                    heading: w.heading,
                },
                edges: vec![spec(Predicate::After { secs: 1.2 }, next + 1)],
            });
            tree.nodes.push(Node {
                action: Action::Rotate { heading: w.heading },
                edges: vec![
                    spec(
                        Predicate::HeadingOk {
                            heading: w.heading,
                            tol: 0.08,
                        },
                        at,
                    ),
                    spec(Predicate::After { secs: 3.0 }, at),
                ],
            });
            Predicate::Stalled { secs: 1.5 }
        }
        Contingency::Misalign => {
            tree.nodes.push(Node {
                action: Action::Hold,
                edges: vec![spec(
                    Predicate::HeadingOk {
                        heading: w.heading,
                        tol: 0.12,
                    },
                    at,
                )],
            });
            Predicate::HeadingOff {
                heading: w.heading,
                tol: 0.35,
            }
        }
        Contingency::GripLoss => {
            tree.nodes.push(Node {
                action: Action::Approach,
                edges: vec![spec(Predicate::InContact, at)],
            });
            Predicate::ContactLost
        }
    };
    tree.nodes[at as usize].edges.insert(0, spec(trigger, next));
}

/// Dead-reckon the payload forward to make up for camera latency.
fn extrapolate(o: ObjectState, age: f32) -> ObjectState {
    let age = age.clamp(0.0, 0.5);
    ObjectState {
        pos: o.pos + o.vel * age,
        angle: o.angle + o.angvel * age,
        ..o
    }
}

/// `n` grip points evenly spaced around the payload, in its local frame.
fn perimeter_slots(half: V2, n: usize) -> Vec<V2> {
    let w = half.x + SLOT_OFFSET;
    let h = half.y + SLOT_OFFSET;
    let per = 4.0 * (w + h);
    (0..n)
        .map(|k| {
            let mut s = (k as f32 + 0.5) * per / n as f32;
            // Walk the rectangle counter-clockwise from the bottom-left corner.
            if s < 2.0 * w {
                return V2::new(-w + s, -h);
            }
            s -= 2.0 * w;
            if s < 2.0 * h {
                return V2::new(w, -h + s);
            }
            s -= 2.0 * h;
            if s < 2.0 * w {
                return V2::new(w - s, h);
            }
            s -= 2.0 * w;
            V2::new(-w, h - s)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use antlift_core::{ObjectState, RobotObs};

    fn snapshot(layout: &Layout) -> Snapshot {
        Snapshot {
            t: 0.0,
            object: ObjectState {
                pos: layout.object_start,
                angle: layout.object_start_angle,
                half: layout.object_half,
                ..Default::default()
            },
            robots: (0..layout.n_robots)
                .map(|i| RobotObs {
                    id: i as u16,
                    pos: V2::new(3.0, 13.0),
                    active: true,
                    attached: false,
                })
                .collect(),
            obstacles: vec![],
        }
    }

    #[test]
    fn every_robot_gets_a_tree_within_budget() {
        let layout = Layout::doorway();
        let cfg = EdgeConfig::default();
        let mut edge = EdgeServer::new(layout.clone(), cfg);
        edge.handle(Uplink::Camera(snapshot(&layout)));
        let out = edge.tick(0.0);
        assert_eq!(out.len(), layout.n_robots);
        for d in &out {
            let DownBody::Tree(bytes) = &d.body else {
                panic!("speculative mode sends trees");
            };
            assert!(bytes.len() <= cfg.tree_budget, "{}", bytes.len());
            let t = ContingencyTree::decode(bytes).expect("valid tree");
            assert!(t.nodes.iter().flat_map(|n| &n.edges).any(|e| e.speculative));
        }
        // Unacknowledged trees are retransmitted...
        let resent = edge.tick(1.0);
        assert_eq!(resent.len(), layout.n_robots);
        assert_eq!(resent[0], out[0], "retransmissions are identical copies");
        // ...acknowledged ones (with their current lease) are not.
        for d in &resent {
            let DownBody::Tree(bytes) = &d.body else {
                unreachable!()
            };
            let t = ContingencyTree::decode(bytes).unwrap();
            edge.handle(Uplink::TreeAck {
                robot: d.robot,
                version: t.version,
                valid_until: t.valid_until,
            });
        }
        assert!(edge.tick(1.5).is_empty());
        // Leases get renewed.
        assert_eq!(edge.tick(cfg.refresh + 2.0).len(), layout.n_robots);
    }

    #[test]
    fn surprise_triggers_replan_and_raises_jam_prior() {
        let layout = Layout::doorway();
        let mut edge = EdgeServer::new(layout.clone(), EdgeConfig::default());
        edge.handle(Uplink::Camera(snapshot(&layout)));
        edge.tick(0.0);
        let at = V2::new(12.0, 9.0);
        let before = edge.heat_at(at);
        edge.handle(Uplink::Surprise {
            robot: 0,
            version: 1,
            node: 1,
            at,
        });
        assert!(!edge.tick(0.1).is_empty());
        assert!(edge.heat_at(at) > before);
        assert_eq!(edge.stats.plans, 2);
    }

    #[test]
    fn slots_are_spread_out() {
        let slots = perimeter_slots(V2::new(2.0, 0.6), 16);
        for (i, a) in slots.iter().enumerate() {
            for b in &slots[i + 1..] {
                assert!(a.dist(*b) > 2.0 * ROBOT_RADIUS, "{a:?} {b:?}");
            }
        }
    }
}
