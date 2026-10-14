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
//! nobody stops inside a corridor), the open floor is shared through block
//! reservations and permits ([`traffic`]), and robots of teams that have
//! delivered are moved to the teams that need help most.
//!
//! In [`ControlMode::Centralized`] it is the baseline instead: it runs every
//! robot's brain itself on streamed robot state and sends force commands.

pub mod grid;
pub mod rollout;
pub mod traffic;

use antlift_core::{
    axis_error, Aabb, Action, Brain, ContingencyTree, ControlMode, DownBody, Downlink, Edge, Event,
    Layout, Mode, Node, NodeId, ObjectState, Predicate, Role, Sense, Snapshot, Uplink,
    DELIVERED_RADIUS, NO_LIMIT, ROBOT_RADIUS, SLOT_OFFSET, V2,
};
use grid::{Footprint, Grid, Pose};
use rollout::Ghost;
use std::f32::consts::FRAC_PI_2;
use std::time::Instant;
use traffic::Blocks;

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
    /// Claim choke points only this far ahead along the route (m); further
    /// ones cut the plan short until the team gets closer.
    pub zone_lookahead: f32,
    /// Smooth planned routes (shortcuts in open space, turning while moving).
    /// Off by default: it makes routes look far better but, in a 30-seed A/B
    /// test, tripled surprises for speculative control without making it
    /// faster (see docs/design.md, section 4.7).
    pub smooth: bool,
    /// Centralized mode: command rate (s).
    pub command_period: f32,
    /// Share the open floor between teams with block reservations and
    /// permits (off: teams only coordinate at choke points).
    pub traffic: bool,
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
            zone_lookahead: 10.0,
            smooth: false,
            command_period: 0.05,
            traffic: true,
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
    /// The choke point this plan stops short of because it's too far away
    /// to claim yet; the plan is extended as the team gets closer.
    pub short_of: Option<usize>,
    /// The team whose reserved floor this one is waiting at the end of its
    /// permit for.
    pub traffic_wait: Option<usize>,
    /// Waypoints the team is permitted to go up to (exclusive).
    pub granted: usize,
}

/// Where a plan must stop because of a choke point.
struct Stop {
    /// Last path index to keep.
    cut: usize,
    /// The choke point stopped in front of.
    zone: usize,
    /// The choke point held by someone else, if that's the reason.
    wait_for: Option<usize>,
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
    /// Times a team had to stop at the end of its permit for another team.
    pub traffic_waits: u64,
    /// Deadlocks (or long waits) broken by rerouting a team.
    pub reroutes: u64,
    /// Reroutes that found no way round, so the team made way instead.
    pub detours: u64,
    pub permits_sent: u64,
    /// Centralized mode: branches/surprises inside the edge-hosted brains.
    pub local_hits: u64,
    pub local_misses: u64,
    pub commands_sent: u64,
}

struct RobotBook {
    active: bool,
    attached: bool,
    /// Tree node from the robot's last heartbeat.
    reported_node: NodeId,
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
    /// Last traffic permit sent and when, and the one before it (where the
    /// robot would be stuck if the last one got lost).
    permit_sent: NodeId,
    permit_t: f32,
    permit_prev: NodeId,
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
    /// Spots where this team's own "as planned" future jammed: routes avoid them.
    jam_spots: Vec<V2>,
    /// A surprise arrived too soon after the last plan; replan when allowed.
    pending_surprise: bool,
    /// The next plan should ignore the current route (something went wrong
    /// with it). Otherwise replans prefer to stay near it, so the team
    /// doesn't flip between equally good routes as others move around.
    fresh_route: bool,
    /// Traffic: next waypoint the payload hasn't reached yet.
    progress: usize,
    /// Traffic: waypoints granted (the permit), and as a tree node.
    granted: usize,
    permit: NodeId,
    /// Traffic: who this team is stopped for, and since when.
    waits_for: Option<usize>,
    wait_since: f32,
    last_reroute: f32,
    /// Floor the next plan must keep off (a deadlock reroute).
    avoid: Vec<Aabb>,
    /// A make-way spot this team is heading for instead of its goal, and
    /// since when (a deadlock it couldn't route round).
    detour: Option<(V2, f32)>,
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
    /// Walls plus parked (delivered) payloads, for robots' transit routes.
    walls_grid: Grid,
    /// Traffic blocks and who owns each.
    blocks: Blocks,
    block_owner: Vec<Option<usize>>,
    /// Sim time of the latest tick, for timing acks.
    now: f32,
    next_command: f32,
    pub stats: EdgeStats,
}

const HEAT_CELL: f32 = 2.0;
/// Seconds between attempts when no route could be found.
const PLAN_RETRY: f32 = 2.0;
/// A waiting team re-checks its queue this often anyway (s).
const WAIT_RECHECK: f32 = 5.0;
/// Clearance kept around other payloads (their robots) when planning (m).
const OTHER_MARGIN: f32 = 0.6;
/// Robots further than this from their grip spot, without a straight line
/// to it, get a transit route (m).
const TRANSIT_MIN: f32 = 3.0;
/// Clearance a robot's transit route keeps from walls (m).
const TRANSIT_CLEAR: f32 = ROBOT_RADIUS + 0.1;
/// At most this many transit waypoints per tree.
const MAX_VIA: usize = 8;
/// Staying within this distance of the current route is preferred (m)...
const STICKY_RADIUS: f32 = 1.5;
/// ...by this extra cost multiplier on cells off it.
const STICKY_COST: f32 = 3.0;
/// Extend a plan that stops short once the payload is this close to its end (m)...
const EXTEND_WITHIN: f32 = 5.0;
/// ...but no more often than this (s), so a borderline distance can't make
/// it replan on every camera frame.
const EXTEND_GAP: f32 = 2.0;
/// Surprises replan a team at most this often, so recoveries get to run (s).
const SURPRISE_REPLAN_GAP: f32 = 2.0;
/// Remember this many jam spots per team.
const MAX_JAM_SPOTS: usize = 6;
/// Robots walking past a parked payload keep this much further off it (m).
const PARKED_BERTH: f32 = 0.3;
/// A payload slower than this at its goal has been delivered (m/s).
const SETTLED_SPEED: f32 = 0.1;
/// Traffic: grant floor this far along the route ahead of a payload (m).
const TRAFFIC_LOOKAHEAD: f32 = 5.0;
/// Traffic: a payload within this of a waypoint has reached it (m).
const TRAFFIC_REACHED: f32 = 1.2;
/// Traffic: reroute a team stopped this long for the same team (s)...
const STUCK_WAIT: f32 = 15.0;
/// ...or in a deadlock, but at most this often per team (s).
const REROUTE_GAP: f32 = 10.0;
/// A make-way detour ends after this long even if the spot isn't reached (s).
const DETOUR_MAX: f32 = 30.0;
/// Resend a permit if the robot hasn't moved on this long after it (s).
const PERMIT_RESEND: f32 = 1.5;

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
                    reported_node: 0,
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
                    permit_sent: NO_LIMIT,
                    permit_t: 0.0,
                    permit_prev: NO_LIMIT,
                }
            })
            .collect();
        let teams = layout
            .payloads
            .iter()
            .map(|_| Team {
                need_replan: true,
                last_plan_t: f32::NEG_INFINITY,
                permit: NO_LIMIT,
                last_reroute: f32::NEG_INFINITY,
                ..Default::default()
            })
            .collect();
        let n_zones = layout.chokepoints.len();
        let walls_grid = Grid::new(layout.arena, &layout.walls);
        let blocks = Blocks::new(layout.arena);
        Self {
            block_owner: vec![None; blocks.len()],
            blocks,
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
            walls_grid,
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

    /// Traffic blocks: owner of each, columns, block size (m).
    pub fn traffic_blocks(&self) -> (&[Option<usize>], usize, f32) {
        (&self.block_owner, self.blocks.nx, traffic::BLOCK)
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
                robot,
                attached,
                node,
            } => {
                if let Some(r) = self.robots.get_mut(robot as usize) {
                    if r.attached && !attached {
                        self.stats.grip_losses += 1;
                    }
                    r.attached = attached;
                    r.reported_node = node;
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
                    t.fresh_route = true;
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
            // Made way: back to the real goal.
            let pos = self.snapshot.as_ref().unwrap().objects[k].pos;
            if t.detour
                .is_some_and(|(spot, at)| pos.dist(spot) < 1.5 || now - at > DETOUR_MAX)
            {
                self.teams[k].detour = None;
                self.teams[k].need_replan = true;
                self.teams[k].fresh_route = true;
            }
            if (failed && since > PLAN_RETRY) || (waiting && since > WAIT_RECHECK) || surprised {
                self.teams[k].need_replan = true;
                self.teams[k].pending_surprise = false;
            }
            // A delivered team's job is done: no more plans for it.
            let done = self.teams[k].delivered && self.teams[k].plan.is_some();
            if !done && (self.teams[k].need_replan || self.teams[k].plan.is_none()) {
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
            // Robots of a delivered team that report they're at the end of
            // their plan are parked: they keep holding without a lease.
            let parked = self.teams[team].delivered && r.reported_node == r.end_node;
            if current && (since < self.cfg.refresh || parked) {
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
            r.permit_sent = self.teams[team].permit;
            r.permit_t = now;
            r.permit_prev = NO_LIMIT;
            // The final Hold sits right after the push chain; contingency nodes follow it.
            r.end_node = end_node;
            self.stats.trees_sent += 1;
            out.push(Downlink {
                robot: id as u16,
                body: DownBody::Tree(bytes),
            });
        }
        self.send_permits(now, &mut out);
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
        // The edge-hosted brains get permits directly (no radio).
        for r in &mut self.robots {
            let permit = self.teams[r.team].permit;
            if r.sent_version == self.teams[r.team].plan.as_ref().map_or(0, |p| p.version) {
                r.brain.receive_permit(r.sent_version, permit);
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
            if r.brain.mode() == Mode::Parked {
                continue; // done: the robot's watchdog holds it still
            }
            // Lent to another team but still gripping the old payload: it
            // must let go, not "carry" its new payload by dragging the old.
            let wrong_payload = snap
                .robots
                .get(id)
                .is_some_and(|o| o.attached && o.holding as usize != r.team);
            let sense = Sense {
                t: now,
                pos: st.pos,
                vel: st.vel,
                attached: st.attached && !wrong_payload,
                grip_slot: st.grip_slot,
                object: objects[r.team],
                goal: self.layout.payloads[r.team].goal,
            };
            let (mut cmd, event) = r.brain.tick(&sense, dt, &mut internal);
            if wrong_payload {
                cmd.release = true;
            }
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
        for k in 0..self.teams.len() {
            let o = s.objects[k];
            let t = &mut self.teams[k];
            if obstacles_changed {
                t.need_replan = true;
                t.fresh_route = true;
            }
            if let Some(plan) = t.plan.as_ref().filter(|p| !p.path.is_empty()) {
                let off = plan
                    .path
                    .iter()
                    .map(|p| p.dist(o.pos))
                    .fold(f32::INFINITY, f32::min);
                if off > 2.5 {
                    t.need_replan = true;
                    t.fresh_route = true;
                }
                // A plan that stops short of a choke point is extended once
                // the payload is close enough to claim it: earlier, the new
                // plan would just stop short again, and replanning that
                // often lets the route flip between equally good options.
                let near_end = plan
                    .waypoints
                    .last()
                    .is_none_or(|w| w.pos.dist(o.pos) < EXTEND_WITHIN);
                if let Some(z) = plan.short_of {
                    let claimable =
                        self.layout.chokepoints[z].dist(o.pos) < self.cfg.zone_lookahead;
                    if (near_end || claimable) && s.t - t.last_plan_t > EXTEND_GAP {
                        t.need_replan = true;
                    }
                }
                // Near the end of a truncated horizon: extend it, even if the
                // robots' "reached the end" reports got lost.
                if plan.waypoints.len() > self.cfg.horizon
                    && plan.waypoints[self.cfg.horizon - 1].pos.dist(o.pos) < 1.2
                {
                    t.need_replan = true;
                }
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
            // Settled at the goal, not just passing through it: a payload
            // still coasting would leave again once its robots are lent out.
            let o = &snap.objects[k];
            let done = o.pos.dist(self.layout.payloads[k].goal) < DELIVERED_RADIUS
                && o.vel.len() < SETTLED_SPEED;
            if done && !self.teams[k].delivered {
                self.teams[k].delivered = true;
                newly_delivered = true;
            }
        }
        if newly_delivered {
            // Everyone else planned as if it were still on the move: plan
            // round it now.
            for t in self.teams.iter_mut().filter(|t| !t.delivered) {
                t.need_replan = true;
                t.fresh_route = true;
            }
            // Parked payloads are as good as walls for robots walking past,
            // with a wider berth: brushing past one drags it off its goal.
            let snap = self.snapshot.as_ref().unwrap();
            let mut obstacles = self.layout.walls.clone();
            obstacles.extend(
                (0..self.teams.len())
                    .filter(|&k| self.teams[k].delivered)
                    .flat_map(|k| payload_pieces(&snap.objects[k], PARKED_BERTH)),
            );
            self.walls_grid = Grid::new(self.layout.arena, &obstacles);
        }
        self.update_zones();
        self.update_traffic();
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
                    t.fresh_route = true;
                }
            }
        }
    }

    /// How many robots fit around payload `k` without crowding.
    /// Free spots near payload `o` that are off the floor in `avoid`, nearest
    /// first, for a team that has to get out of the way.
    fn make_way_spots(
        &mut self,
        o: ObjectState,
        fp: &Footprint,
        obstacles: &[Aabb],
        avoid: &[Aabb],
    ) -> Vec<V2> {
        let centre =
            avoid.iter().fold(V2::ZERO, |a, b| a + b.center()) * (1.0 / avoid.len() as f32);
        let arena = self.layout.arena;
        let g = self.grid(obstacles);
        let mut spots = vec![];
        for r in [3.0, 4.5, 6.0, 8.0] {
            let mut ring: Vec<V2> = (0..16)
                .map(|i| o.pos + V2::from_angle(i as f32 * std::f32::consts::TAU / 16.0) * r)
                .filter(|p| {
                    p.x > arena.min.x && p.y > arena.min.y && p.x < arena.max.x && p.y < arena.max.y
                })
                .filter(|&p| {
                    [o.angle, o.angle + FRAC_PI_2]
                        .iter()
                        .any(|&heading| g.slack(Pose { pos: p, heading }, fp) > 0.2)
                })
                .collect();
            // Furthest from the others first.
            ring.sort_by(|a, b| b.dist(centre).total_cmp(&a.dist(centre)));
            spots.extend(ring);
            if spots.len() >= 4 {
                break;
            }
        }
        spots.truncate(4);
        spots
    }

    /// Block-reservation traffic control (see [`traffic`]): grant each team
    /// the floor along the next stretch of its route, in priority order, set
    /// its permit, and break deadlocks.
    fn update_traffic(&mut self) {
        if !self.cfg.traffic || self.teams.len() < 2 {
            return; // nobody to share the floor with: permits stay unlimited
        }
        let Some(s) = &self.snapshot else { return };
        let horizon = self.cfg.horizon;
        let ring = SLOT_OFFSET + ROBOT_RADIUS + 0.1;
        let mut reqs = vec![];
        let mut keys = vec![];
        for k in 0..self.teams.len() {
            let t = &self.teams[k];
            let o = s.objects[k];
            let plan = t.plan.as_ref().filter(|p| !p.waypoints.is_empty());
            if t.delivered || plan.is_none() {
                // Parked payloads keep their floor for good, and payloads
                // waiting with no route (queued at a door) keep theirs too.
                let mut body = vec![];
                self.blocks.cover(payload_bounds(&o, ring), &mut body);
                keys.push((false, f32::NEG_INFINITY));
                reqs.push(traffic::Request {
                    team: k,
                    body,
                    legs: vec![],
                    first: 0,
                    granted: 0,
                    reach: 0,
                });
                continue;
            }
            let plan = plan.unwrap();
            let wps = &plan.waypoints[..plan.waypoints.len().min(horizon)];
            let n = wps.len();
            // Waypoints the payload has reached (looking a few ahead, as
            // turn-in-place waypoints share a position).
            let from = t.progress.min(n);
            let progress = (from..(from + 4).min(n))
                .rev()
                .find(|&i| wps[i].pos.dist(o.pos) < TRAFFIC_REACHED)
                .map_or(from, |i| i + 1);
            // Grant just in time: legs within the lookahead, at least one.
            let mut reach = progress;
            let mut d = 0.0;
            let mut at = o.pos;
            while reach < n && d <= TRAFFIC_LOOKAHEAD {
                d += at.dist(wps[reach].pos);
                at = wps[reach].pos;
                reach += 1;
            }
            let reach = reach.max((progress + 1).min(n));
            let footprint = |pos: V2, heading: f32| {
                payload_bounds(
                    &ObjectState {
                        pos,
                        angle: heading,
                        ..o
                    },
                    ring,
                )
            };
            let mut body = vec![];
            self.blocks.cover(payload_bounds(&o, ring), &mut body);
            let upto = reach.max(t.granted).min(n);
            let legs = (progress..upto)
                .map(|i| {
                    let (from, heading) = if i == 0 {
                        (o.pos, o.angle)
                    } else {
                        (wps[i - 1].pos, wps[i - 1].heading)
                    };
                    let mut leg = vec![];
                    self.blocks.cover(footprint(from, heading), &mut leg);
                    self.blocks
                        .cover(footprint(wps[i].pos, wps[i].heading), &mut leg);
                    leg
                })
                .collect();
            let remaining: f32 = o.pos.dist(wps[progress.min(n - 1)].pos)
                + plan.waypoints[progress.min(n - 1)..]
                    .windows(2)
                    .map(|w| w[0].pos.dist(w[1].pos))
                    .sum::<f32>();
            // A team inside a doorway goes first, then the one nearer done.
            let in_door = self
                .zones
                .iter()
                .zip(&self.zone_entered)
                .any(|(z, e)| *z == Some(k) && *e);
            keys.push((!in_door, remaining));
            reqs.push(traffic::Request {
                team: k,
                body,
                legs,
                first: progress,
                granted: t.granted.max(progress),
                reach,
            });
        }
        let mut order: Vec<usize> = (0..reqs.len()).collect();
        order.sort_by(|&a, &b| {
            keys[a]
                .0
                .cmp(&keys[b].0)
                .then(keys[a].1.total_cmp(&keys[b].1))
        });
        let reqs: Vec<traffic::Request> = order.iter().map(|&i| reqs[i].clone()).collect();
        let out = traffic::allocate(self.blocks.len(), &reqs);

        let now = s.t;
        let mut waits = vec![None; self.teams.len()];
        for (r, (&granted, &waits_for)) in reqs.iter().zip(out.granted.iter().zip(&out.waits_for)) {
            let k = r.team;
            if self.teams[k].delivered || r.reach == 0 {
                continue; // just holding its floor
            }
            let n = r.legs.len() + r.first;
            let tree_len = self.end_node_of(k) as usize - 1;
            let t = &mut self.teams[k];
            t.progress = r.first;
            t.granted = granted;
            t.permit = if granted >= tree_len {
                NO_LIMIT
            } else {
                granted as NodeId
            };
            // Only stopped for real once the payload is at the end of its grant.
            let stopped = waits_for.filter(|_| granted <= r.first + 1 && granted < n);
            if stopped.is_some() && stopped != t.waits_for {
                t.wait_since = now;
                self.stats.traffic_waits += 1;
            }
            t.waits_for = stopped;
            waits[k] = stopped;
            if let Some(plan) = t.plan.as_mut() {
                plan.traffic_wait = stopped;
                plan.granted = granted;
            }
        }
        for k in 0..self.teams.len() {
            if self.teams[k].delivered {
                self.teams[k].permit = NO_LIMIT;
                self.teams[k].waits_for = None;
            }
        }
        self.block_owner = out.owner;

        // Deadlocks: the lowest-priority team in a wait cycle reroutes out of
        // the others' way (the next lowest if it just tried). A team stuck
        // behind one that isn't moving reroutes too.
        let rank = |k: usize| reqs.iter().position(|r| r.team == k).unwrap_or(usize::MAX);
        let cooling = |k: usize| now - self.teams[k].last_reroute < REROUTE_GAP;
        let mut reroute: Vec<(usize, Vec<usize>)> = vec![];
        for k in 0..self.teams.len() {
            if let Some(mut cycle) = traffic::cycle(&waits, k) {
                if cycle.iter().min() != Some(&k) {
                    continue; // handle each cycle once
                }
                cycle.sort_by_key(|&j| std::cmp::Reverse(rank(j)));
                if let Some(&low) = cycle.iter().find(|&&j| !cooling(j)) {
                    reroute.push((low, cycle.into_iter().filter(|&j| j != low).collect()));
                }
            } else if let Some(j) = waits[k] {
                if now - self.teams[k].wait_since > STUCK_WAIT && !cooling(k) {
                    reroute.push((k, vec![j]));
                }
            }
        }
        for (k, others) in reroute {
            // Keep off where the others are and are about to drive, but not
            // off the floor this team stands on, or it couldn't start.
            let mut theirs = vec![];
            for r in reqs.iter().filter(|r| others.contains(&r.team)) {
                theirs.extend(&r.body);
                r.legs.iter().for_each(|l| theirs.extend(l));
            }
            let mine: Vec<usize> = reqs
                .iter()
                .find(|r| r.team == k)
                .map_or(vec![], |r| r.body.clone());
            theirs.sort_unstable();
            theirs.dedup();
            let avoid: Vec<Aabb> = theirs
                .into_iter()
                .filter(|b| !mine.contains(b))
                .map(|b| self.blocks.rect(b))
                .collect();
            let t = &mut self.teams[k];
            t.last_reroute = now;
            t.wait_since = now;
            t.avoid = avoid;
            t.fresh_route = true;
            t.need_replan = true;
            self.stats.reroutes += 1;
        }
    }

    /// Permits for robots whose team's permit changed (or that seem to have
    /// missed it: still where they were when it was sent).
    fn send_permits(&mut self, now: f32, out: &mut Vec<Downlink>) {
        for id in 0..self.robots.len() {
            let team = self.robots[id].team;
            let Some(version) = self.team_version(team) else {
                continue;
            };
            let permit = self.teams[team].permit;
            let r = &mut self.robots[id];
            if !r.active || r.sent_version != version {
                continue; // a tree with the current permit is on its way
            }
            let changed = r.permit_sent != permit;
            let missed = r.reported_node == r.permit_prev
                && r.reported_node < permit
                && now - r.permit_t > PERMIT_RESEND;
            if !(changed || missed) {
                continue;
            }
            if changed {
                r.permit_prev = r.permit_sent;
            }
            r.permit_sent = permit;
            r.permit_t = now;
            self.stats.permits_sent += 1;
            out.push(Downlink {
                robot: id as u16,
                body: DownBody::Permit {
                    version,
                    node: permit,
                },
            });
        }
    }

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
            // Most in need first; ties go to the nearest payload. A robot is
            // only lent where a real grip spot is free (reserved right away);
            // otherwise it stays with its delivered team and keeps getting
            // that team's trees, rather than ending up with no tree at all.
            let p = s.robots[i].pos;
            let mut order: Vec<usize> = needy
                .iter()
                .copied()
                .filter(|&k| (count(k) as usize) < self.capacity(k))
                .collect();
            order.sort_by(|&a, &b| {
                let (na, nb) = (count(a) / area(a), count(b) / area(b));
                na.total_cmp(&nb).then_with(|| {
                    s.objects[a]
                        .pos
                        .dist(p)
                        .total_cmp(&s.objects[b].pos.dist(p))
                })
            });
            let Some((target, slot)) = order
                .into_iter()
                .find_map(|k| self.find_gap(k, p).map(|slot| (k, slot)))
            else {
                continue;
            };
            let old = self.robots[i].team;
            self.robots[i].team = target;
            self.robots[i].slot = Some(slot);
            self.stats.reassigned += 1;
            changed.push(old);
            changed.push(target);
        }
        changed.sort_unstable();
        changed.dedup();
        changed
    }

    /// A free grip spot around payload `k` for a robot at `pos` (payload
    /// frame), clear of every spot the team's robots grip or are assigned.
    fn find_gap(&self, k: usize, pos: V2) -> Option<V2> {
        let s = self.snapshot.as_ref()?;
        let o = s.objects[k];
        let local = |p: V2| (p - o.pos).rotate(-o.angle);
        let taken: Vec<V2> = (0..self.robots.len())
            .filter(|&i| self.robots[i].active && self.robots[i].team == k)
            .filter_map(|i| {
                let obs = &s.robots[i];
                if obs.attached && obs.holding as usize == k {
                    Some(local(obs.pos))
                } else {
                    self.robots[i].slot
                }
            })
            .collect();
        let gap = 2.0 * ROBOT_RADIUS + 0.05;
        let me = local(pos);
        perimeter_slots(o.half, (perimeter(o.half) / 0.1) as usize)
            .into_iter()
            .filter(|d| taken.iter().all(|t| t.dist(*d) > gap))
            .min_by(|a, b| a.dist(me).total_cmp(&b.dist(me)))
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
    /// already gripping it keep their grip point so they don't let go, and
    /// newcomers fill the gaps between them.
    fn assign_slots(&mut self, k: usize) {
        let Some(s) = &self.snapshot else { return };
        let o = s.objects[k];
        let local = |p: V2| (p - o.pos).rotate(-o.angle);

        let members: Vec<usize> = (0..self.robots.len())
            .filter(|&i| self.robots[i].active && self.robots[i].team == k)
            .collect();
        // Grips stay where they are, and spots already handed out (e.g. to
        // robots just lent to this team) are kept, so nobody loses theirs.
        let mut taken: Vec<V2> = vec![];
        for &i in &members {
            let obs = &s.robots[i];
            if obs.attached && obs.holding as usize == k {
                self.robots[i].slot = Some(local(obs.pos));
            }
            if let Some(slot) = self.robots[i].slot {
                taken.push(slot);
            }
        }

        if taken.is_empty() {
            // Nobody grips yet: spread the team evenly all round, so forces
            // and torques balance, and give each slot to the nearest robot.
            let mut free = perimeter_slots(o.half, members.len());
            for &i in &members {
                let me = local(s.robots[i].pos);
                let best = free
                    .iter()
                    .enumerate()
                    .min_by(|a, b| a.1.dist(me).total_cmp(&b.1.dist(me)))
                    .map(|(j, _)| j);
                if let Some(j) = best {
                    self.robots[i].slot = Some(free.swap_remove(j));
                }
            }
            return;
        }

        // Joining robots that already grip: fill the real gaps between their
        // grips, trying points densely round the perimeter, nearest robot first.
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

    /// Claim the next chain of choke points on this route if it's close and
    /// free. Returns where the plan must stop: in front of a chain someone
    /// else holds (and which zone to queue for), or in front of one that's
    /// still too far away to claim.
    fn reserve_route(&mut self, k: usize, path: &[V2]) -> Option<Stop> {
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
            // Stop a body length before the chain's first zone.
            let stop_at = arc[chain[0].1] - body;
            let cut = (0..=chain[0].1)
                .rev()
                .find(|&q| arc[q] <= stop_at)
                .unwrap_or(0);
            let mine = chain.iter().all(|(z, _, _)| self.zones[*z] == Some(k));
            if !mine && arc[chain[0].1] > self.cfg.zone_lookahead {
                // Too far to claim yet: don't hold a door others could use.
                return Some(Stop {
                    cut,
                    zone: chain[0].0,
                    wait_for: None,
                });
            }
            if let Some(&(z, _, _)) = chain
                .iter()
                .find(|(z, _, _)| self.zones[*z].is_some_and(|o| o != k))
            {
                // Someone else holds part of the chain: queue for the blocker.
                return Some(Stop {
                    cut,
                    zone: chain[0].0,
                    wait_for: Some(z),
                });
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
            self.teams[k].fresh_route = true;
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

        // Delivered payloads are parked for good: plan around them. Payloads
        // still on the move are not walls (even while briefly waiting or
        // turning): sharing doors with them is what reservations are for,
        // and treating them as walls makes routes flip as they come and go.
        let others: Vec<Aabb> = s
            .objects
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != k && self.teams[*j].delivered)
            .flat_map(|(_, o)| payload_pieces(o, OTHER_MARGIN))
            .collect();
        let mut with_others = s.obstacles.clone();
        with_others.extend_from_slice(&others);
        // Breaking a deadlock: keep off the floor the other teams hold.
        let avoid = std::mem::take(&mut self.teams[k].avoid);
        with_others.extend_from_slice(&avoid);
        // If there's no way round, somewhere nearby to get out of the way.
        let make_way = if avoid.is_empty() || self.teams[k].detour.is_some() {
            vec![]
        } else {
            self.make_way_spots(me, &fp, &with_others, &avoid)
        };
        let goal = self.teams[k].detour.map_or(spec.goal, |(spot, _)| spot);
        // Fall back step by step: crates can be shoved aside and other
        // payloads will move, so if they block every route, plan through
        // them and let contingencies and replanning deal with it.
        // Steer away from what went wrong before: the learned surprise map
        // (everyone's) and this team's own jam spots from ghost futures.
        let (heat, heat_nx, origin) = (self.heat.clone(), self.heat_nx, self.layout.arena.min);
        let jam_spots = self.teams[k].jam_spots.clone();
        // Unless the current route went wrong, prefer staying near it.
        let sticky = match (&self.teams[k].plan, self.teams[k].fresh_route) {
            (Some(prev), false) if !prev.path.is_empty() => {
                Some(near_mask(self.layout.arena, &prev.path, STICKY_RADIUS))
            }
            _ => None,
        };
        self.teams[k].fresh_route = false;
        let penalty = move |p: V2| {
            let off_route = sticky
                .as_ref()
                .is_some_and(|(mask, nx)| !mask[cell_index(origin, *nx, mask.len(), p)]);
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
            3.0 * h + jams + if off_route { STICKY_COST } else { 0.0 }
        };
        // Each attempt is smoothed against the same obstacles it was planned with.
        let smooth = self.cfg.smooth;
        let mut attempt = |obstacles: &[Aabb], penalty: &dyn Fn(V2) -> f32, goal: V2| {
            let g = self.grid(obstacles);
            g.plan_with(start, goal, &fp, penalty).map(|raw| {
                if smooth {
                    g.smooth(&raw, &fp)
                } else {
                    raw
                }
            })
        };
        let mut goal = goal;
        let mut poses = attempt(&with_others, &penalty, goal);
        let mut detour = None;
        if poses.is_none() {
            for &spot in &make_way {
                poses = attempt(&with_others, &penalty, spot);
                if poses.is_some() {
                    goal = spot;
                    detour = Some(spot);
                    break;
                }
            }
        }
        if poses.is_none() {
            poses = attempt(&s.obstacles, &penalty, goal);
        }
        if poses.is_none() && !s.obstacles.is_empty() {
            poses = attempt(&[], &|_| 0.0, goal);
        }
        if let Some(spot) = detour {
            self.teams[k].detour = Some((spot, s.t));
            self.stats.detours += 1;
        }
        let poses = poses.unwrap_or_default();
        let path: Vec<V2> = poses.iter().map(|p| p.pos).collect();

        // Choke points: claim what's next, or stop short and queue.
        self.release_unentered(k);
        let stop = self.reserve_route(k, &path);
        let waiting_for = stop.as_ref().and_then(|s| s.wait_for);
        if waiting_for.is_some() {
            self.stats.zone_waits += 1;
        }
        let (route, target) = match &stop {
            Some(st) => (&poses[..=st.cut.min(poses.len().saturating_sub(1))], None),
            None => (&poses[..], Some(goal)),
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

        self.teams[k].plan = Some(Plan {
            team: k,
            version: self.version,
            start: me.pos,
            path,
            waypoints,
            candidates,
            ghosts: vec![],
            waiting_for,
            short_of: stop
                .as_ref()
                .filter(|st| st.wait_for.is_none())
                .map(|st| st.zone),
            traffic_wait: None,
            granted: 0,
        });
        // A new plan starts with no floor granted beyond where it is.
        let t = &mut self.teams[k];
        t.progress = 0;
        t.granted = 0;
        self.update_traffic();
        let has_route = self.teams[k]
            .plan
            .as_ref()
            .is_some_and(|p| !p.waypoints.is_empty());
        self.stats.plans += 1;
        // Futures only mean something once the team has hold of its payload:
        // simulated from before that, nothing moves and every one "jams".
        let gripping = s
            .robots
            .iter()
            .filter(|r| r.attached && r.holding as usize == k)
            .count();
        let team_size = self
            .robots
            .iter()
            .filter(|r| r.active && r.team == k)
            .count();
        let holding = 2 * gripping >= team_size.max(1);
        if self.cfg.ghosts && has_route && holding && !self.teams[k].delivered {
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
                self.build_tree(id, snap.t).filter(|_| active).map(|mut t| {
                    t.permit = NO_LIMIT; // futures are about the route, not traffic
                    t.encode()
                })
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

    /// Waypoints round walls for a robot that isn't gripping and can't drive
    /// straight to its grip spot, e.g. one just lent to a team across the
    /// warehouse. Empty when a straight drive works.
    fn transit_route(&self, id: usize, team: usize, slot: V2) -> Vec<V2> {
        let Some(s) = &self.snapshot else {
            return vec![];
        };
        let obs = &s.robots[id];
        if obs.attached && obs.holding as usize == team {
            return vec![];
        }
        let o = s.objects[team];
        let target = o.pos + slot.rotate(o.angle);
        let g = &self.walls_grid;
        if obs.pos.dist(target) < TRANSIT_MIN || g.line_clear(obs.pos, target, TRANSIT_CLEAR) {
            return vec![];
        }
        g.route_2d(obs.pos, target, TRANSIT_CLEAR)
            .map(|r| {
                // Drop the start (where the robot is) and the end (the
                // approach takes over near the payload).
                let inner = &r[1..r.len().saturating_sub(1)];
                inner.iter().take(MAX_VIA).copied().collect()
            })
            .unwrap_or_default()
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
            permit: self.teams[team].permit,
            via: self.transit_route(id, team, slot),
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

/// Cells (on the planner's grid) within `radius` of `path`, and the column count.
fn near_mask(arena: Aabb, path: &[V2], radius: f32) -> (Vec<bool>, usize) {
    let size = arena.max - arena.min;
    let nx = (size.x / grid::CELL).ceil() as usize;
    let ny = (size.y / grid::CELL).ceil() as usize;
    let mut mask = vec![false; nx * ny];
    let r = (radius / grid::CELL).ceil() as isize;
    for p in path {
        let l = *p - arena.min;
        let (cx, cy) = ((l.x / grid::CELL) as isize, (l.y / grid::CELL) as isize);
        for y in (cy - r).max(0)..=(cy + r).min(ny as isize - 1) {
            for x in (cx - r).max(0)..=(cx + r).min(nx as isize - 1) {
                let c = arena.min
                    + V2::new((x as f32 + 0.5) * grid::CELL, (y as f32 + 0.5) * grid::CELL);
                if c.dist(*p) <= radius {
                    mask[y as usize * nx + x as usize] = true;
                }
            }
        }
    }
    (mask, nx)
}

/// Index of the grid cell containing `p` (clamped to the grid).
fn cell_index(origin: V2, nx: usize, len: usize, p: V2) -> usize {
    let l = p - origin;
    let ny = len / nx;
    let x = ((l.x / grid::CELL).max(0.0) as usize).min(nx - 1);
    let y = ((l.y / grid::CELL).max(0.0) as usize).min(ny - 1);
    y * nx + x
}

/// Axis-aligned bounds of a (rotated) payload plus a margin for its robots.
fn payload_bounds(o: &ObjectState, margin: f32) -> Aabb {
    let (s, c) = o.angle.sin_cos();
    let ex = (o.half.x * c).abs() + (o.half.y * s).abs() + margin;
    let ey = (o.half.x * s).abs() + (o.half.y * c).abs() + margin;
    Aabb::from_center(o.pos, V2::new(ex, ey))
}

/// A (rotated) payload plus a margin as a chain of small boxes along its
/// long axis: much tighter than one box round it when it lies at an angle.
fn payload_pieces(o: &ObjectState, margin: f32) -> Vec<Aabb> {
    let n = (o.half.x / o.half.y.max(0.1)).ceil().max(1.0) as usize;
    let piece = ObjectState {
        half: V2::new(o.half.x / n as f32, o.half.y),
        ..*o
    };
    let axis = V2::from_angle(o.angle);
    (0..n)
        .map(|i| {
            let along = (2.0 * i as f32 + 1.0 - n as f32) * piece.half.x;
            payload_bounds(
                &ObjectState {
                    pos: o.pos + axis * along,
                    ..piece
                },
                margin,
            )
        })
        .collect()
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
        let layout = Layout::hall();
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
        let layout = Layout::hall();
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
    fn a_held_door_makes_others_queue_in_front() {
        // The sofa (A) is in the dispatch door; the beam (C) is lined up 5 m
        // in front of it.
        let layout = Layout::warehouse();
        let mut snap = snapshot(&layout);
        snap.objects[2].pos = V2::new(33.0, 17.75);
        snap.objects[2].angle = 0.0;
        let mut edge = EdgeServer::new(layout.clone(), no_ghosts());
        edge.handle(Uplink::Camera(snap));
        let d3 = 2;
        edge.zones[d3] = Some(0);
        edge.zone_entered[d3] = true;
        edge.tick(0.0);
        assert_eq!(edge.zone_owners()[d3], Some(0), "A keeps the door it is in");
        let plans = edge.plans();
        let beam = plans.iter().find(|p| p.team == 2).unwrap();
        assert_eq!(beam.waiting_for, Some(d3), "the beam queues");
        // Its waypoints stop short of the door.
        let door = layout.chokepoints[d3];
        assert!(beam.waypoints.iter().all(|w| door.dist(w.pos) > 1.0));
        assert!(edge.stats.zone_waits >= 1);
    }

    #[test]
    fn traffic_stops_a_team_short_of_another() {
        // The pallet (B) sits right in front of the sofa (A), on its way to
        // the staging door.
        let layout = Layout::warehouse();
        let mut snap = snapshot(&layout);
        snap.objects[1].pos = V2::new(12.0, 6.5);
        snap.objects[1].angle = 0.0;
        let mut edge = EdgeServer::new(layout.clone(), no_ghosts());
        edge.handle(Uplink::Camera(snap.clone()));
        edge.tick(0.0);
        let a = &edge.teams[0];
        assert_eq!(a.waits_for, Some(1), "A waits for B");
        assert_ne!(a.permit, NO_LIMIT);
        // None of B's floor was granted to A.
        let mut b_floor = vec![];
        edge.blocks
            .cover(payload_bounds(&snap.objects[1], 0.0), &mut b_floor);
        let (owner, _, _) = edge.traffic_blocks();
        assert!(b_floor.iter().all(|&i| owner[i] == Some(1)));
        // A's robots get trees carrying the permit.
        let id = layout.team_range(0).start;
        assert_eq!(
            edge.build_tree(id, 0.0).unwrap().permit,
            edge.teams[0].permit
        );
        // Once B is out of the way, A gets a permit message going further.
        snap.t = 1.0;
        snap.objects[1].pos = V2::new(14.0, 20.0);
        let before = edge.teams[0].permit;
        edge.handle(Uplink::Camera(snap));
        let out = edge.tick(1.0);
        assert!(edge.teams[0].permit > before);
        assert!(out
            .iter()
            .any(|d| matches!(d.body, DownBody::Permit { .. }) && (d.robot as usize) < 16));
    }

    #[test]
    fn doors_are_claimed_just_in_time() {
        // At the start every team is more than 10 m from its first door:
        // nobody holds one yet, and every plan stops short instead of waiting.
        let layout = Layout::warehouse();
        let mut edge = EdgeServer::new(layout.clone(), no_ghosts());
        edge.handle(Uplink::Camera(snapshot(&layout)));
        edge.tick(0.0);
        assert!(edge.zone_owners().iter().all(|z| z.is_none()));
        for p in edge.plans() {
            assert!(
                p.short_of.is_some() && p.waiting_for.is_none(),
                "team {}",
                p.team
            );
        }
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
