//! The edge server: plans each payload's route, speculates on what might go
//! wrong, and ships each robot a byte-budgeted contingency tree for its role
//! in its team.
//!
//! It is event-driven. It replans a team only when the world surprises it (a
//! robot reports a stall it had no branch for, obstacles change, the payload
//! drifts off its route, robots reach the end of their planning horizon, a
//! choke point it is waiting for frees up), and otherwise renews tree leases.
//!
//! With several teams it also coordinates them: narrow passages
//! ([`Layout::chokepoints`]) are granted as reservations, a team only enters
//! one when it holds it (and every choke point chained right behind it, so
//! nobody stops inside a corridor), and robots of teams that have delivered
//! are moved to the teams that need help most.
//!
//! In [`ControlMode::Centralized`] it is the baseline instead: it runs every
//! robot's brain itself on streamed robot state and sends force commands.

pub mod grid;
pub mod rollout;

use antlift_core::{
    axis_error, Aabb, Action, Brain, ContingencyTree, ControlMode, DownBody, Downlink, Edge, Event,
    Layout, Node, NodeId, ObjectState, Predicate, Role, Sense, Snapshot, Uplink, ROBOT_RADIUS,
    SLOT_OFFSET, V2,
};
use grid::{Footprint, Grid, Pose};
use rollout::Ghost;
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
    /// Simulate "what if" futures to estimate contingency probabilities
    /// (ghost futures); otherwise use heuristic priors only.
    pub ghosts: bool,
    /// How far ahead each ghost future is simulated (s).
    pub ghost_horizon: f32,
    /// Request choke points this far ahead along the route (m).
    pub zone_lookahead: f32,
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
            ghosts: true,
            ghost_horizon: 10.0,
            zone_lookahead: 20.0,
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
    pub team: usize,
    pub version: u32,
    pub start: V2,
    /// The full route to the goal (even when the waypoints stop short of a
    /// choke point the team is waiting for).
    pub path: Vec<V2>,
    pub waypoints: Vec<Waypoint>,
    pub candidates: Vec<Candidate>,
    /// The simulated futures this plan's probabilities were learned from.
    pub ghosts: Vec<Ghost>,
    /// Choke point this team is queued for; the plan stops in front of it.
    pub waiting_for: Option<usize>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EdgeStats {
    pub plans: u64,
    pub hits_reported: u64,
    pub surprises: u64,
    pub trees_sent: u64,
    pub grip_losses: u64,
    pub last_plan_us: u128,
    /// Plans that had to stop short of a choke point another team holds.
    pub zone_waits: u64,
    /// Robots moved from a delivered team to one that needed help.
    pub reassigned: u64,
    /// Centralized mode: branches/surprises inside the edge-hosted brains.
    pub local_hits: u64,
    pub local_misses: u64,
    pub commands_sent: u64,
}

struct RobotBook {
    active: bool,
    attached: bool,
    team: usize,
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

#[derive(Default)]
struct Team {
    plan: Option<Plan>,
    need_replan: bool,
    last_plan_t: f32,
    delivered: bool,
    /// Where the other payloads were when this team last planned.
    others_at_plan: Vec<V2>,
    /// Spots where this team's own "as planned" future jammed: routes avoid them.
    jam_spots: Vec<V2>,
    /// A surprise arrived too soon after the last plan; replan when allowed.
    pending_surprise: bool,
}

pub struct EdgeServer {
    layout: Layout,
    cfg: EdgeConfig,
    snapshot: Option<Snapshot>,
    teams: Vec<Team>,
    version: u32,
    robots: Vec<RobotBook>,
    /// Owner of each choke point, and whether the owner's payload is in it.
    zones: Vec<Option<usize>>,
    zone_entered: Vec<bool>,
    /// Coarse (2 m) map of where unpredicted stalls happened. Learned prior.
    heat: Vec<f32>,
    heat_nx: usize,
    grid_cache: Option<(Vec<Aabb>, Grid)>,
    /// Sim time of the latest tick, for timing acks.
    now: f32,
    next_command: f32,
    pub stats: EdgeStats,
}

const HEAT_CELL: f32 = 2.0;
/// Seconds between attempts when no route could be found.
const PLAN_RETRY: f32 = 2.0;
/// Replan a team when another payload has moved this far since its last plan.
const OTHERS_MOVED: f32 = 3.0;
/// ...but no more often than this (s).
const OTHERS_REPLAN_GAP: f32 = 3.0;
/// A waiting team re-checks its queue this often anyway (s).
const WAIT_RECHECK: f32 = 5.0;
/// Clearance kept around other payloads (their robots) when planning (m).
const OTHER_MARGIN: f32 = 0.6;
/// Payload counts as delivered within this distance of its goal (m).
const DELIVERED: f32 = 1.0;
/// Surprises replan a team at most this often, so recoveries get to run (s).
const SURPRISE_REPLAN_GAP: f32 = 2.0;
/// Remember this many jam spots per team.
const MAX_JAM_SPOTS: usize = 6;

impl EdgeServer {
    pub fn new(layout: Layout, cfg: EdgeConfig) -> Self {
        let size = layout.arena.max - layout.arena.min;
        let heat_nx = (size.x / HEAT_CELL).ceil() as usize;
        let heat_ny = (size.y / HEAT_CELL).ceil() as usize;
        let robots = (0..layout.n_robots())
            .map(|i| {
                let team = layout.initial_team(i);
                RobotBook {
                    active: true,
                    attached: false,
                    team,
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
                    brain: Brain::new(i as u16, team as u8),
                    state: None,
                }
            })
            .collect();
        let teams = layout
            .payloads
            .iter()
            .map(|_| Team {
                need_replan: true,
                last_plan_t: f32::NEG_INFINITY,
                ..Default::default()
            })
            .collect();
        let n_zones = layout.chokepoints.len();
        Self {
            layout,
            cfg,
            snapshot: None,
            teams,
            version: 0,
            robots,
            zones: vec![None; n_zones],
            zone_entered: vec![false; n_zones],
            heat: vec![0.0; heat_nx * heat_ny],
            heat_nx,
            grid_cache: None,
            now: 0.0,
            next_command: 0.0,
            stats: EdgeStats::default(),
        }
    }

    /// Switch control mode mid-run. Forces fresh plans so every robot (or
    /// edge-hosted brain) gets a newly installed tree.
    pub fn set_control(&mut self, control: ControlMode) {
        if self.cfg.control == control {
            return;
        }
        self.cfg.control = control;
        for t in &mut self.teams {
            t.need_replan = true;
        }
        for r in &mut self.robots {
            r.retries = 0;
            r.acked_until = 0.0;
        }
    }

    /// The first team's plan (the only one on single-payload levels).
    pub fn plan(&self) -> Option<&Plan> {
        self.teams.first().and_then(|t| t.plan.as_ref())
    }

    /// Every team's current plan.
    pub fn plans(&self) -> Vec<Plan> {
        self.teams.iter().filter_map(|t| t.plan.clone()).collect()
    }

    /// Owner team of each choke point.
    pub fn zone_owners(&self) -> &[Option<usize>] {
        &self.zones
    }

    /// The team robot `id` currently works for.
    pub fn robot_team(&self, id: usize) -> usize {
        self.robots[id].team
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

    fn team_version(&self, team: usize) -> Option<u32> {
        self.teams[team].plan.as_ref().map(|p| p.version)
    }

    fn end_node_of(&self, team: usize) -> NodeId {
        self.teams[team].plan.as_ref().map_or(0, |p| {
            (p.waypoints.len().min(self.cfg.horizon) + 1) as NodeId
        })
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
                let Some(r) = self.robots.get(robot as usize) else {
                    return;
                };
                let team = r.team;
                if r.end_node == node && self.team_version(team) == Some(version) {
                    self.teams[team].need_replan = true;
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
            Uplink::Surprise { robot, at, .. } => {
                self.stats.surprises += 1;
                self.bump_heat(at);
                if let Some(r) = self.robots.get(robot as usize) {
                    let t = &mut self.teams[r.team];
                    if self.now - t.last_plan_t > SURPRISE_REPLAN_GAP {
                        t.need_replan = true;
                    } else {
                        t.pending_surprise = true;
                    }
                }
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
        for k in 0..self.teams.len() {
            let t = &self.teams[k];
            // A failed plan is retried on a timer, not on every camera frame.
            let failed = t.plan.as_ref().is_some_and(|p| p.path.is_empty());
            let waiting = t.plan.as_ref().is_some_and(|p| p.waiting_for.is_some());
            let since = now - t.last_plan_t;
            let surprised = t.pending_surprise && since > SURPRISE_REPLAN_GAP;
            if (failed && since > PLAN_RETRY) || (waiting && since > WAIT_RECHECK) || surprised {
                self.teams[k].need_replan = true;
                self.teams[k].pending_surprise = false;
            }
            if self.teams[k].need_replan || self.teams[k].plan.is_none() {
                self.teams[k].last_plan_t = now;
                self.replan(k);
            }
        }
        match self.cfg.control {
            ControlMode::Speculative => self.send_trees(now),
            ControlMode::Centralized => self.command_robots(now),
        }
    }

    fn send_trees(&mut self, now: f32) -> Vec<Downlink> {
        let mut out = vec![];
        for id in 0..self.robots.len() {
            let team = self.robots[id].team;
            let Some(version) = self.team_version(team) else {
                continue;
            };
            let r = &mut self.robots[id];
            if !r.active {
                continue;
            }
            let since = now - r.last_sent;
            let current = r.sent_version == version;
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
            let end_node = self.end_node_of(team);
            let r = &mut self.robots[id];
            if r.sent_version != version {
                r.acked_until = 0.0;
            }
            r.sent_version = version;
            r.sent_until = tree.valid_until;
            r.last_sent = now;
            r.first_sent = now;
            r.retries = 0;
            r.last_bytes = bytes.clone();
            // The final Hold sits right after the push chain; contingency nodes follow it.
            r.end_node = end_node;
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
        for id in 0..self.robots.len() {
            let team = self.robots[id].team;
            let Some(version) = self.team_version(team) else {
                continue;
            };
            if !self.robots[id].active || self.robots[id].sent_version == version {
                continue;
            }
            if let Some(mut tree) = self.build_tree(id, now) {
                tree.valid_until = f32::MAX; // no lease: it never leaves the edge
                let end_node = self.end_node_of(team);
                let r = &mut self.robots[id];
                r.brain.receive(&tree.encode());
                r.sent_version = version;
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
        let objects: Vec<ObjectState> = snap
            .objects
            .iter()
            .map(|o| extrapolate(*o, now - snap.t))
            .collect();

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
                object: objects[r.team],
                goal: self.layout.payloads[r.team].goal,
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
        let now = s.t;
        for k in 0..self.teams.len() {
            let o = s.objects[k];
            let t = &mut self.teams[k];
            if obstacles_changed {
                t.need_replan = true;
            }
            if let Some(plan) = t.plan.as_ref().filter(|p| !p.path.is_empty()) {
                let off = plan
                    .path
                    .iter()
                    .map(|p| p.dist(o.pos))
                    .fold(f32::INFINITY, f32::min);
                if off > 2.5 {
                    t.need_replan = true;
                }
                // Near the end of a truncated horizon: extend it, even if the
                // robots' "reached the end" reports got lost.
                if plan.waypoints.len() > self.cfg.horizon
                    && plan.waypoints[self.cfg.horizon - 1].pos.dist(o.pos) < 1.2
                {
                    t.need_replan = true;
                }
            }
            // Other payloads are obstacles in this team's plan: replan when
            // they have moved on enough to matter.
            let moved = t
                .others_at_plan
                .iter()
                .zip(s.objects.iter().enumerate().filter(|(j, _)| *j != k))
                .any(|(a, (_, b))| a.dist(b.pos) > OTHERS_MOVED);
            if moved && now - t.last_plan_t > OTHERS_REPLAN_GAP {
                t.need_replan = true;
            }
        }

        let mut roster_changed = vec![false; self.teams.len()];
        for o in &s.robots {
            if let Some(r) = self.robots.get_mut(o.id as usize) {
                if r.active != o.active {
                    r.active = o.active;
                    roster_changed[r.team] = true;
                }
            }
        }
        let first = prev.is_none();
        self.snapshot = Some(s);

        // Deliveries free robots and choke points.
        let mut newly_delivered = false;
        for k in 0..self.teams.len() {
            let snap = self.snapshot.as_ref().unwrap();
            let done = snap.objects[k].pos.dist(self.layout.payloads[k].goal) < DELIVERED;
            if done && !self.teams[k].delivered {
                self.teams[k].delivered = true;
                newly_delivered = true;
            }
        }
        self.update_zones();
        if newly_delivered || roster_changed.iter().any(|c| *c) {
            for k in self.rebalance() {
                roster_changed[k] = true;
            }
        }
        for (k, changed) in roster_changed.into_iter().enumerate() {
            if first || changed {
                self.assign_slots(k);
                self.teams[k].need_replan = true;
            }
        }
    }

    /// Track which reserved choke points their owners have entered and left;
    /// release finished ones and wake up teams queued for them.
    fn update_zones(&mut self) {
        let Some(s) = &self.snapshot else { return };
        let mut released = vec![];
        for z in 0..self.zones.len() {
            let Some(owner) = self.zones[z] else { continue };
            let o = s.objects[owner];
            let inside = self.layout.chokepoints[z].dist(o.pos) < o.half.len();
            if inside {
                self.zone_entered[z] = true;
            } else if self.zone_entered[z] || self.teams[owner].delivered {
                self.zones[z] = None;
                self.zone_entered[z] = false;
                released.push(z);
            }
        }
        for z in released {
            for t in &mut self.teams {
                if t.plan.as_ref().is_some_and(|p| p.waiting_for == Some(z)) {
                    t.need_replan = true;
                }
            }
        }
    }

    /// How many robots fit around payload `k` without crowding.
    fn capacity(&self, k: usize) -> usize {
        // Irregular existing grips waste some room, hence the 0.85.
        (0.85 * perimeter(self.layout.payloads[k].half) / (2.0 * ROBOT_RADIUS + 0.05)) as usize
    }

    /// Move robots of delivered teams to the undelivered teams with the fewest
    /// robots per square metre of payload, as long as they fit around it.
    /// Returns the teams that changed.
    fn rebalance(&mut self) -> Vec<usize> {
        let Some(s) = &self.snapshot else {
            return vec![];
        };
        let needy: Vec<usize> = (0..self.teams.len())
            .filter(|&k| !self.teams[k].delivered)
            .collect();
        if needy.is_empty() {
            return vec![];
        }
        let idle: Vec<usize> = (0..self.robots.len())
            .filter(|&i| self.robots[i].active && self.teams[self.robots[i].team].delivered)
            .collect();
        let mut changed = vec![];
        for i in idle {
            let area = |k: usize| {
                let h = self.layout.payloads[k].half;
                4.0 * h.x * h.y
            };
            let count = |k: usize| {
                self.robots
                    .iter()
                    .filter(|r| r.active && r.team == k)
                    .count() as f32
            };
            // Most in need first; ties go to the nearest payload. Full
            // payloads take no more: the robot stays put.
            let Some(target) = needy
                .iter()
                .copied()
                .filter(|&k| (count(k) as usize) < self.capacity(k))
                .min_by(|&a, &b| {
                    let (na, nb) = (count(a) / area(a), count(b) / area(b));
                    na.total_cmp(&nb).then_with(|| {
                        let p = s.robots[i].pos;
                        s.objects[a]
                            .pos
                            .dist(p)
                            .total_cmp(&s.objects[b].pos.dist(p))
                    })
                })
            else {
                continue;
            };
            let old = self.robots[i].team;
            self.robots[i].team = target;
            self.robots[i].slot = None;
            self.stats.reassigned += 1;
            changed.push(old);
            changed.push(target);
        }
        changed.sort_unstable();
        changed.dedup();
        changed
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

    /// Spread team `k`'s active robots evenly around its payload. Robots
    /// already gripping it keep their grip point so they don't let go.
    fn assign_slots(&mut self, k: usize) {
        let Some(s) = &self.snapshot else { return };
        let o = s.objects[k];
        let local = |p: V2| (p - o.pos).rotate(-o.angle);

        let members: Vec<usize> = (0..self.robots.len())
            .filter(|&i| self.robots[i].active && self.robots[i].team == k)
            .collect();
        let mut taken: Vec<V2> = vec![];
        for &i in &members {
            let obs = &s.robots[i];
            self.robots[i].slot = if obs.attached && obs.holding as usize == k {
                let l = local(obs.pos);
                taken.push(l);
                Some(l)
            } else {
                None
            };
        }

        // Fill the real gaps between existing grips: try points densely
        // around the perimeter, nearest robot first, keeping robots apart.
        let gap = 2.0 * ROBOT_RADIUS + 0.05;
        let dense = perimeter_slots(o.half, (perimeter(o.half) / 0.1) as usize);
        let mut waiting: Vec<usize> = members
            .iter()
            .copied()
            .filter(|&i| self.robots[i].slot.is_none())
            .collect();
        waiting.sort_by(|&a, &b| {
            let (pa, pb) = (local(s.robots[a].pos), local(s.robots[b].pos));
            let near = |p: V2| {
                dense
                    .iter()
                    .map(|d| d.dist(p))
                    .fold(f32::INFINITY, f32::min)
            };
            near(pa).total_cmp(&near(pb))
        });
        for i in waiting {
            let me = local(s.robots[i].pos);
            let best = dense
                .iter()
                .filter(|d| taken.iter().all(|t| t.dist(**d) > gap))
                .min_by(|a, b| a.dist(me).total_cmp(&b.dist(me)))
                .copied();
            if let Some(slot) = best {
                taken.push(slot);
                self.robots[i].slot = Some(slot);
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

    /// Release choke points team `k` reserved but hasn't entered yet, so a
    /// fresh plan can claim (or give up) exactly what it needs.
    fn release_unentered(&mut self, k: usize) {
        for z in 0..self.zones.len() {
            if self.zones[z] == Some(k) && !self.zone_entered[z] {
                self.zones[z] = None;
            }
        }
    }

    /// Choke points along `path` in order, as (zone, first path index, last path index).
    fn zones_on_path(&self, path: &[V2]) -> Vec<(usize, usize, usize)> {
        let mut out: Vec<(usize, usize, usize)> = vec![];
        for (i, p) in path.iter().enumerate() {
            for (z, a) in self.layout.chokepoints.iter().enumerate() {
                if a.dist(*p) > 0.0 {
                    continue;
                }
                match out.iter_mut().find(|(oz, _, _)| *oz == z) {
                    Some(e) => e.2 = i,
                    None => out.push((z, i, i)),
                }
            }
        }
        out.sort_by_key(|e| e.1);
        out
    }

    /// Claim the next chain of choke points on this route if they're free;
    /// otherwise return the path index to stop at and the zone to wait for.
    fn reserve_route(&mut self, k: usize, path: &[V2]) -> Option<(usize, usize)> {
        if path.is_empty() || self.zones.is_empty() {
            return None;
        }
        let mut arc = vec![0.0f32; path.len()];
        for i in 1..path.len() {
            arc[i] = arc[i - 1] + path[i].dist(path[i - 1]);
        }
        let body = self.layout.payloads[k].half.x * 2.0 + 2.0;
        let zones = self.zones_on_path(path);
        let mut i = 0;
        while i < zones.len() {
            // A chain: zones too close together to stop between.
            let mut j = i + 1;
            while j < zones.len() && arc[zones[j].1] - arc[zones[j - 1].2] < body {
                j += 1;
            }
            let chain = &zones[i..j];
            if arc[chain[0].1] > self.cfg.zone_lookahead {
                return None; // not close enough to ask yet
            }
            if let Some(&(z, entry, _)) = chain
                .iter()
                .find(|(z, _, _)| self.zones[*z].is_some_and(|o| o != k))
            {
                // Someone else holds part of the chain: stop a body length
                // before its first zone and queue for the blocker.
                let stop_at = arc[chain[0].1] - body;
                let cut = (0..=chain[0].1)
                    .rev()
                    .find(|&q| arc[q] <= stop_at)
                    .unwrap_or(0);
                let _ = entry;
                return Some((cut, z));
            }
            for &(z, _, _) in chain {
                self.zones[z] = Some(k);
            }
            i = j;
        }
        None
    }

    /// Plan team `k`. If its own "as planned" future jams, remember the spot
    /// and plan once more around it before anything is sent.
    fn replan(&mut self, k: usize) {
        let t0 = Instant::now();
        if let Some(spot) = self.plan_team(k) {
            let spots = &mut self.teams[k].jam_spots;
            spots.push(spot);
            if spots.len() > MAX_JAM_SPOTS {
                spots.remove(0);
            }
            self.plan_team(k);
        }
        self.stats.last_plan_us = t0.elapsed().as_micros();
    }

    /// One planning pass. Returns where the as-planned ghost future jammed.
    fn plan_team(&mut self, k: usize) -> Option<V2> {
        self.teams[k].need_replan = false;
        self.version += 1;
        for h in &mut self.heat {
            *h *= 0.98;
        }
        let s = self.snapshot.clone().expect("snapshot before plan");
        let me = s.objects[k];
        let spec = self.layout.payloads[k].clone();
        // Payload plus the robots gripping it.
        let fp = Footprint::new(spec.half + V2::new(SLOT_OFFSET, SLOT_OFFSET) * 2.0);
        let start = Pose {
            pos: me.pos,
            heading: me.angle,
        };

        // Other payloads (and their robots) are obstacles for now.
        let others: Vec<Aabb> = s
            .objects
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != k)
            .map(|(_, o)| payload_bounds(o, OTHER_MARGIN))
            .collect();
        let mut with_others = s.obstacles.clone();
        with_others.extend_from_slice(&others);
        // Fall back step by step: crates can be shoved aside and other
        // payloads will move, so if they block every route, plan through
        // them and let contingencies and replanning deal with it.
        // Steer away from what went wrong before: the learned surprise map
        // (everyone's) and this team's own jam spots from ghost futures.
        let (heat, heat_nx, origin) = (self.heat.clone(), self.heat_nx, self.layout.arena.min);
        let jam_spots = self.teams[k].jam_spots.clone();
        let penalty = move |p: V2| {
            let l = p - origin;
            let (x, y) = (
                (l.x / HEAT_CELL).max(0.0) as usize,
                (l.y / HEAT_CELL).max(0.0) as usize,
            );
            let h = heat
                .get(y * heat_nx + x.min(heat_nx - 1))
                .copied()
                .unwrap_or(0.0);
            let jams: f32 = jam_spots
                .iter()
                .map(|s| (1.0 - s.dist(p) / 2.0).max(0.0) * 6.0)
                .sum();
            3.0 * h + jams
        };
        let mut poses = self
            .grid(&with_others)
            .plan_with(start, spec.goal, &fp, &penalty);
        if poses.is_none() {
            poses = self
                .grid(&s.obstacles)
                .plan_with(start, spec.goal, &fp, &penalty);
        }
        if poses.is_none() && !s.obstacles.is_empty() {
            poses = self.grid(&[]).plan(start, spec.goal, &fp);
        }
        let poses = poses.unwrap_or_default();
        let path: Vec<V2> = poses.iter().map(|p| p.pos).collect();

        // Choke points: claim what's next, or stop short and queue.
        self.release_unentered(k);
        let wait = self.reserve_route(k, &path);
        if wait.is_some() {
            self.stats.zone_waits += 1;
        }
        let (route, target) = match wait {
            Some((cut, _)) => (&poses[..=cut.min(poses.len().saturating_sub(1))], None),
            None => (&poses[..], Some(spec.goal)),
        };
        let grid = self.grid(&s.obstacles);

        // Thin the pose chain into waypoints: every 2 m in the open, every
        // 1 m where it's tight, and wherever the heading changes notably.
        let mut waypoints: Vec<Waypoint> = vec![];
        let (mut last_pos, mut last_heading) = (start.pos, start.heading);
        for (i, p) in route.iter().enumerate() {
            let is_last = i + 1 == route.len();
            let slack = grid.slack(*p, &fp);
            let spacing = if slack < 0.5 { 1.0 } else { 2.0 };
            let moved = p.pos.dist(last_pos);
            let turned = axis_error(p.heading, last_heading).abs();
            if !(is_last || moved >= spacing || turned >= 0.35) {
                continue;
            }
            if is_last && moved < 0.3 && target.is_none() {
                continue; // already at the waiting spot
            }
            waypoints.push(Waypoint {
                pos: match (is_last, target) {
                    (true, Some(goal)) => goal,
                    _ => p.pos,
                },
                heading: p.heading,
                tightness: (1.0 - slack / 0.6).clamp(0.0, 1.0),
                turn_in_place: moved < 0.5 && !is_last,
            });
            last_pos = p.pos;
            last_heading = p.heading;
        }

        let grip_p = (0.03 + 0.05 * self.stats.grip_losses as f32).min(0.8);
        let mut candidates = vec![];
        for (i, w) in waypoints.iter().enumerate().take(self.cfg.horizon) {
            let prev_heading = if i == 0 {
                me.angle
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

        self.teams[k].others_at_plan = s
            .objects
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != k)
            .map(|(_, o)| o.pos)
            .collect();
        self.teams[k].plan = Some(Plan {
            team: k,
            version: self.version,
            start: me.pos,
            path,
            waypoints,
            candidates,
            ghosts: vec![],
            waiting_for: wait.map(|(_, z)| z),
        });
        let has_route = self.teams[k]
            .plan
            .as_ref()
            .is_some_and(|p| !p.waypoints.is_empty());
        self.stats.plans += 1;
        if self.cfg.ghosts && has_route && !self.teams[k].delivered {
            return self.learn_from_ghosts(k, &s);
        }
        None
    }

    /// Simulate the futures with the trees robots would get under the
    /// prior probabilities, then update team `k`'s contingency probabilities
    /// with what actually happened in them (a Beta-style blend: the prior
    /// counts as `PRIOR_WEIGHT` imaginary futures).
    fn learn_from_ghosts(&mut self, k: usize, snap: &Snapshot) -> Option<V2> {
        const PRIOR_WEIGHT: f32 = 2.0;
        let trees: Vec<Option<Vec<u8>>> = (0..self.robots.len())
            .map(|id| {
                let active = self.robots[id].active;
                self.build_tree(id, snap.t)
                    .filter(|_| active)
                    .map(|t| t.encode())
            })
            .collect();
        let ghosts = rollout::simulate_all(&self.layout, snap, &trees, self.cfg.ghost_horizon, k);

        let horizon = self.cfg.horizon;
        let plan = self.teams[k].plan.as_mut().expect("plan just built");
        let wps = &plan.waypoints[..plan.waypoints.len().min(horizon)];
        let nearest = |p: V2| {
            (0..wps.len()).min_by(|&a, &b| wps[a].pos.dist(p).total_cmp(&wps[b].pos.dist(p)))
        };
        let mut jams = vec![0.0f32; wps.len()];
        let mut drifts = vec![0.0f32; wps.len()];
        for g in &ghosts {
            if let Some(i) = g.stall.and_then(nearest) {
                jams[i] += 1.0;
            }
            let mut seen = vec![false; wps.len()];
            for &(p, a) in &g.poses {
                if let Some(i) = nearest(p) {
                    if !seen[i] && axis_error(wps[i].heading, a).abs() > 0.35 {
                        seen[i] = true;
                        drifts[i] += 1.0;
                    }
                }
            }
        }
        let n = ghosts.len() as f32;
        for c in &mut plan.candidates {
            if c.wp >= wps.len() {
                continue;
            }
            let seen = match c.kind {
                Contingency::Jam => jams[c.wp],
                Contingency::Misalign => drifts[c.wp],
                // Grip loss is injected in the futures, so they say nothing about it.
                Contingency::GripLoss => continue,
            };
            c.p = (c.p * PRIOR_WEIGHT + seen) / (PRIOR_WEIGHT + n);
        }
        plan.candidates
            .sort_by(|a, b| (b.p * b.kind.impact()).total_cmp(&(a.p * a.kind.impact())));
        let nominal_jam = ghosts
            .iter()
            .find(|g| g.what_if == rollout::WhatIf::Nominal)
            .and_then(|g| g.stall);
        plan.ghosts = ghosts;
        nominal_jam
    }

    /// Nominal chain plus the most valuable contingencies that fit the budget,
    /// pruned to what this robot's role can act on.
    fn build_tree(&self, id: usize, now: f32) -> Option<ContingencyTree> {
        let team = self.robots[id].team;
        let plan = self.teams[team].plan.as_ref()?;
        let slot = self.robots[id].slot?;
        let half = self.layout.payloads[team].half;
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
            team: team as u8,
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

/// Dead-reckon a payload forward to make up for camera latency.
fn extrapolate(o: ObjectState, age: f32) -> ObjectState {
    let age = age.clamp(0.0, 0.5);
    ObjectState {
        pos: o.pos + o.vel * age,
        angle: o.angle + o.angvel * age,
        ..o
    }
}

/// Axis-aligned bounds of a (rotated) payload plus a margin for its robots.
fn payload_bounds(o: &ObjectState, margin: f32) -> Aabb {
    let (s, c) = o.angle.sin_cos();
    let ex = (o.half.x * c).abs() + (o.half.y * s).abs() + margin;
    let ey = (o.half.x * s).abs() + (o.half.y * c).abs() + margin;
    Aabb::from_center(o.pos, V2::new(ex, ey))
}

/// Length of the loop grip points sit on around a payload (m).
fn perimeter(half: V2) -> f32 {
    4.0 * (half.x + half.y + 2.0 * SLOT_OFFSET)
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
            objects: layout
                .payloads
                .iter()
                .map(|p| ObjectState {
                    pos: p.start,
                    angle: p.angle,
                    half: p.half,
                    ..Default::default()
                })
                .collect(),
            robots: (0..layout.n_robots())
                .map(|i| RobotObs {
                    id: i as u16,
                    pos: layout.spawn_pos(i),
                    active: true,
                    attached: false,
                    holding: 0,
                })
                .collect(),
            obstacles: vec![],
        }
    }

    fn no_ghosts() -> EdgeConfig {
        EdgeConfig {
            ghosts: false,
            ..Default::default()
        }
    }

    #[test]
    fn every_robot_gets_a_tree_within_budget() {
        let layout = Layout::doorway();
        let cfg = EdgeConfig::default();
        let mut edge = EdgeServer::new(layout.clone(), cfg);
        edge.handle(Uplink::Camera(snapshot(&layout)));
        let out = edge.tick(0.0);
        assert_eq!(out.len(), layout.n_robots());
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
        assert_eq!(resent.len(), layout.n_robots());
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
        assert_eq!(edge.tick(cfg.refresh + 2.0).len(), layout.n_robots());
    }

    #[test]
    fn surprise_triggers_a_rate_limited_replan_and_raises_jam_prior() {
        let layout = Layout::doorway();
        let mut edge = EdgeServer::new(layout.clone(), no_ghosts());
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
        assert!(edge.heat_at(at) > before);
        // Too soon after the last plan: the replan is deferred, not dropped.
        edge.tick(0.1);
        assert_eq!(edge.stats.plans, 1);
        edge.tick(SURPRISE_REPLAN_GAP + 0.1);
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

    #[test]
    fn trees_carry_each_robots_team() {
        let layout = Layout::warehouse();
        let mut edge = EdgeServer::new(layout.clone(), no_ghosts());
        edge.handle(Uplink::Camera(snapshot(&layout)));
        for d in edge.tick(0.0) {
            let DownBody::Tree(bytes) = &d.body else {
                unreachable!()
            };
            let t = ContingencyTree::decode(bytes).unwrap();
            assert_eq!(t.team as usize, layout.initial_team(d.robot as usize));
        }
    }

    #[test]
    fn only_one_team_gets_a_contested_door() {
        // Sofa and beam both just in front of the single dispatch door.
        let layout = Layout::warehouse();
        let mut snap = snapshot(&layout);
        snap.objects[0].pos = V2::new(33.0, 17.0);
        snap.objects[0].angle = 0.0;
        snap.objects[2].pos = V2::new(32.0, 26.5);
        snap.objects[2].angle = 0.0;
        let mut edge = EdgeServer::new(layout.clone(), no_ghosts());
        edge.handle(Uplink::Camera(snap));
        edge.tick(0.0);
        let d3 = 2;
        let owner = edge.zone_owners()[d3].expect("someone holds the dispatch door");
        let other = if owner == 0 { 2 } else { 0 };
        let plans = edge.plans();
        let waiting = plans.iter().find(|p| p.team == other).unwrap();
        assert_eq!(waiting.waiting_for, Some(d3), "the other team queues");
        // Its waypoints stop short of the door.
        let door = layout.chokepoints[d3];
        assert!(waiting.waypoints.iter().all(|w| door.dist(w.pos) > 1.0));
        assert!(edge.stats.zone_waits >= 1);
    }

    #[test]
    fn delivered_team_lends_its_robots() {
        let layout = Layout::warehouse();
        let mut snap = snapshot(&layout);
        snap.objects[1].pos = layout.payloads[1].goal; // pallet delivered
        let mut edge = EdgeServer::new(layout.clone(), no_ghosts());
        edge.handle(Uplink::Camera(snap));
        let lent = layout
            .team_range(1)
            .filter(|&id| edge.robot_team(id) != 1)
            .count();
        assert!(lent > 0, "the pallet team should lend robots");
        assert_eq!(edge.stats.reassigned, lent as u64);
        // Nobody is crowded past what fits around their payload.
        for k in [0, 2] {
            let n = (0..layout.n_robots())
                .filter(|&id| edge.robot_team(id) == k)
                .count();
            assert!(
                n <= edge.capacity(k),
                "team {k}: {n} > {}",
                edge.capacity(k)
            );
        }
    }
}
