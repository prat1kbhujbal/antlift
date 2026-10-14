//! Glue for running the whole system on one laptop: the swarm, the edge
//! server and the simulated radio between them.

pub mod scenario;

use antlift_core::{ControlMode, DownBody, Downlink, Layout, Uplink};
use antlift_edge::{EdgeConfig, EdgeServer, EdgeStats, Plan};
use antlift_net::{Link, LinkConfig};
use antlift_sim::Swarm;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;

/// The overhead camera's frame period. The centralized baseline needs a
/// fast feed to close its loop; the speculative edge only plans with it.
fn camera_period(control: ControlMode) -> f32 {
    match control {
        ControlMode::Speculative => 0.5,
        ControlMode::Centralized => 0.05,
    }
}

/// What the UI may read about the edge without touching its internals.
#[derive(Clone, Default)]
pub struct EdgeView {
    /// Every team's current plan.
    pub plans: Vec<Plan>,
    /// Owner team of each choke point.
    pub zones: Vec<Option<usize>>,
    pub stats: EdgeStats,
    /// Learned surprise map: values, columns, cell size (m).
    pub heat: (Vec<f32>, usize, f32),
    /// Traffic blocks: owner team of each, columns, block size (m).
    pub blocks: (Vec<Option<usize>>, usize, f32),
}

impl EdgeView {
    /// The first team's plan (the only one on single-payload levels).
    pub fn plan(&self) -> Option<&Plan> {
        self.plans.first()
    }
}

pub enum EdgeIn {
    Msg(Uplink),
    Tick(f32),
    SetControl(ControlMode),
}

/// The edge runs either inline (deterministic, for benchmarks and tests) or
/// on its own thread (like a separate box, for the interactive demo).
pub enum EdgeRuntime {
    Inline(Box<EdgeServer>),
    Thread {
        tx: Sender<EdgeIn>,
        rx: Receiver<Vec<Downlink>>,
        view: Arc<Mutex<EdgeView>>,
    },
}

impl EdgeRuntime {
    pub fn inline(layout: Layout, cfg: EdgeConfig) -> Self {
        Self::Inline(Box::new(EdgeServer::new(layout, cfg)))
    }

    pub fn spawn(layout: Layout, cfg: EdgeConfig) -> Self {
        let (tx, in_rx) = channel::<EdgeIn>();
        let (out_tx, rx) = channel();
        let view = Arc::new(Mutex::new(EdgeView::default()));
        let shared = view.clone();
        thread::Builder::new()
            .name("edge".into())
            .spawn(move || {
                let mut edge = EdgeServer::new(layout, cfg);
                for m in in_rx {
                    match m {
                        EdgeIn::Msg(u) => edge.handle(u),
                        EdgeIn::SetControl(c) => edge.set_control(c),
                        EdgeIn::Tick(now) => {
                            let out = edge.tick(now);
                            if let Ok(mut v) = shared.lock() {
                                v.stats = edge.stats;
                                let (h, nx, cell) = edge.heat_grid();
                                v.heat = (h.to_vec(), nx, cell);
                                v.zones = edge.zone_owners().to_vec();
                                let (b, bnx, bcell) = edge.traffic_blocks();
                                v.blocks = (b.to_vec(), bnx, bcell);
                                let plans = edge.plans();
                                let versions =
                                    |ps: &[Plan]| ps.iter().map(|p| p.version).collect::<Vec<_>>();
                                if versions(&v.plans) != versions(&plans) {
                                    v.plans = plans;
                                } else {
                                    // Same plans; traffic state moves on.
                                    for (old, new) in v.plans.iter_mut().zip(&plans) {
                                        old.traffic_wait = new.traffic_wait;
                                        old.granted = new.granted;
                                    }
                                }
                            }
                            if !out.is_empty() && out_tx.send(out).is_err() {
                                return;
                            }
                        }
                    }
                }
            })
            .expect("spawn edge thread");
        Self::Thread { tx, rx, view }
    }

    fn deliver(&mut self, u: Uplink) {
        match self {
            Self::Inline(e) => e.handle(u),
            Self::Thread { tx, .. } => {
                let _ = tx.send(EdgeIn::Msg(u));
            }
        }
    }

    /// Inline: plans now. Threaded: asks the edge to plan and collects
    /// whatever it finished since the last call (so compute time is real).
    fn tick(&mut self, now: f32) -> Vec<Downlink> {
        match self {
            Self::Inline(e) => e.tick(now),
            Self::Thread { tx, rx, .. } => {
                let _ = tx.send(EdgeIn::Tick(now));
                rx.try_iter().flatten().collect()
            }
        }
    }

    fn set_control(&mut self, control: ControlMode) {
        match self {
            Self::Inline(e) => e.set_control(control),
            Self::Thread { tx, .. } => {
                let _ = tx.send(EdgeIn::SetControl(control));
            }
        }
    }

    pub fn view(&self) -> EdgeView {
        match self {
            Self::Inline(e) => {
                let (h, nx, cell) = e.heat_grid();
                let (b, bnx, bcell) = e.traffic_blocks();
                EdgeView {
                    plans: e.plans(),
                    zones: e.zone_owners().to_vec(),
                    stats: e.stats,
                    heat: (h.to_vec(), nx, cell),
                    blocks: (b.to_vec(), bnx, bcell),
                }
            }
            Self::Thread { view, .. } => view.lock().map(|v| v.clone()).unwrap_or_default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RadioStats {
    pub up_bytes: u64,
    pub down_bytes: u64,
}

#[derive(Clone, Debug)]
pub struct SessionConfig {
    pub layout: Layout,
    pub control: ControlMode,
    pub link: LinkConfig,
    pub edge: EdgeConfig,
    /// Run the edge on its own thread (interactive) or inline (deterministic).
    pub threaded: bool,
    /// Seeds the radio's jitter and loss.
    pub seed: u64,
}

impl SessionConfig {
    pub fn new(control: ControlMode) -> Self {
        Self {
            layout: Layout::hall(),
            control,
            link: LinkConfig::default(),
            edge: EdgeConfig::default(),
            threaded: false,
            seed: 1,
        }
    }
}

/// What a robot radio packet carried, for visualisation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketKind {
    Tree,
    /// Traffic permits (go as far as node N).
    Permit,
    Command,
    State,
    Surprise,
    /// Heartbeats, acks, branch reports.
    Status,
}

/// One robot radio transmission, kept briefly for visualisation.
#[derive(Clone, Copy, Debug)]
pub struct Tx {
    pub t: f32,
    pub robot: u16,
    /// True if robot -> edge.
    pub uplink: bool,
    pub kind: PacketKind,
    pub bytes: usize,
    pub lost: bool,
}

fn up_kind(u: &Uplink) -> PacketKind {
    match u {
        Uplink::Surprise { .. } => PacketKind::Surprise,
        Uplink::State { .. } => PacketKind::State,
        _ => PacketKind::Status,
    }
}

/// One point of a run's history, sampled every [`HISTORY_PERIOD`].
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t: f32,
    /// Cumulative robot radio traffic per robot (KB).
    pub kb_per_robot: f32,
    /// Total distance left over every payload (m).
    pub dist: f32,
}

pub const HISTORY_PERIOD: f32 = 0.5;
/// How long transmissions are remembered for drawing.
pub const TX_MEMORY: f32 = 1.5;

pub struct Session {
    pub control: ControlMode,
    pub swarm: Swarm,
    pub up: Link<Uplink>,
    pub down: Link<Downlink>,
    pub edge: EdgeRuntime,
    pub radio: RadioStats,
    /// Recent robot radio transmissions (last [`TX_MEMORY`] seconds).
    pub tx: Vec<Tx>,
    pub history: Vec<Sample>,
    next_camera: f32,
}

impl Session {
    pub fn new(cfg: SessionConfig) -> Self {
        let edge_cfg = EdgeConfig {
            control: cfg.control,
            ..cfg.edge
        };
        let edge = if cfg.threaded {
            EdgeRuntime::spawn(cfg.layout.clone(), edge_cfg)
        } else {
            EdgeRuntime::inline(cfg.layout.clone(), edge_cfg)
        };
        Self {
            control: cfg.control,
            swarm: Swarm::new(cfg.layout, cfg.control),
            up: Link::new(cfg.link, cfg.seed.wrapping_mul(2)),
            down: Link::new(cfg.link, cfg.seed.wrapping_mul(2) + 1),
            edge,
            radio: RadioStats::default(),
            tx: Vec::new(),
            history: Vec::new(),
            next_camera: 0.0,
        }
    }

    /// Predicted branches that fired, wherever the brains run.
    pub fn hits(&self) -> u64 {
        self.swarm.stats.hits + self.edge.view().stats.local_hits
    }

    /// Unplanned stalls, wherever the brains run.
    pub fn misses(&self) -> u64 {
        self.swarm.stats.misses + self.edge.view().stats.local_misses
    }

    pub fn t(&self) -> f32 {
        self.swarm.t
    }

    pub fn edge_up(&self) -> bool {
        self.up.up
    }

    /// Simulate the edge box going offline (or coming back).
    pub fn set_edge_up(&mut self, up: bool) {
        self.up.up = up;
        self.down.up = up;
    }

    /// Hand control between robots and edge without restarting the run.
    pub fn set_control(&mut self, control: ControlMode) {
        self.control = control;
        self.swarm.set_control(control);
        self.edge.set_control(control);
        self.next_camera = self.swarm.t; // switch camera rate right away
    }

    pub fn set_link(&mut self, cfg: LinkConfig) {
        self.up.config = cfg;
        self.down.config = cfg;
    }

    pub fn step(&mut self, dt: f32) {
        let now = self.swarm.t;

        let arrived = self.down.recv(now);
        let produced = self.swarm.step(dt, arrived);
        self.tx.retain(|x| now - x.t < TX_MEMORY);
        for u in produced {
            let n = u.wire_len();
            self.radio.up_bytes += n as u64;
            let (robot, kind) = (u.robot(), up_kind(&u));
            let lost = !self.up.send(now, u, n);
            if let Some(robot) = robot {
                self.tx.push(Tx {
                    t: now,
                    robot,
                    uplink: true,
                    kind,
                    bytes: n,
                    lost,
                });
            }
        }
        if now >= self.next_camera {
            self.next_camera = now + camera_period(self.control);
            let snap = Uplink::Camera(self.swarm.snapshot());
            let n = snap.wire_len();
            self.up.send(now, snap, n);
        }

        for u in self.up.recv(now) {
            self.edge.deliver(u);
        }
        if self.up.up {
            for d in self.edge.tick(now) {
                let n = d.wire_len();
                self.radio.down_bytes += n as u64;
                let robot = d.robot;
                let kind = match d.body {
                    DownBody::Tree(_) => PacketKind::Tree,
                    DownBody::Permit { .. } => PacketKind::Permit,
                    DownBody::Command { .. } => PacketKind::Command,
                };
                let lost = !self.down.send(now, d, n);
                self.tx.push(Tx {
                    t: now,
                    robot,
                    uplink: false,
                    kind,
                    bytes: n,
                    lost,
                });
            }
        }

        if self
            .history
            .last()
            .is_none_or(|h| now - h.t >= HISTORY_PERIOD)
        {
            let n = self.swarm.brains.len().max(1) as f32;
            self.history.push(Sample {
                t: now,
                kb_per_robot: (self.radio.up_bytes + self.radio.down_bytes) as f32 / n / 1024.0,
                dist: self.distance_left(),
            });
        }
    }

    /// Total distance between payloads and their goals (m).
    pub fn distance_left(&self) -> f32 {
        let s = &self.swarm;
        (0..s.layout.payloads.len())
            .map(|k| s.world.object_state(k).pos.dist(s.layout.payloads[k].goal))
            .sum()
    }

    /// Robot radio bytes per robot per second, both directions.
    pub fn bytes_per_robot_s(&self) -> f32 {
        let n = self.swarm.brains.len().max(1) as f32;
        (self.radio.up_bytes + self.radio.down_bytes) as f32 / n / self.t().max(1e-3)
    }
}
