//! The swarm side: physics world plus one robot controller per robot.

pub mod world;

use antlift_core::{
    track, Brain, Command, ControlMode, DownBody, Downlink, Layout, RobotObs, Sense, Snapshot,
    Uplink, DELIVERED_RADIUS,
};
pub use antlift_core::{Event, Mode};
pub use world::SimWorld;

/// Centralized mode: robots stream their state this often.
const STATE_PERIOD: f32 = 0.05;
/// Centralized mode: stop if no command arrived for this long.
const COMMAND_TIMEOUT: f32 = 0.3;
/// Centralized mode: a robot the edge has stopped commanding (e.g. parked
/// at the goal) only reports its state this often.
const IDLE_STATE_PERIOD: f32 = 1.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct SwarmStats {
    /// Speculative branches that fired on robots: handled locally, no round trip.
    pub hits: u64,
    /// Stalls the robots' trees had no branch for.
    pub misses: u64,
    pub trees_received: u64,
    pub commands_received: u64,
}

pub struct Swarm {
    pub layout: Layout,
    pub control: ControlMode,
    pub world: SimWorld,
    pub brains: Vec<Brain>,
    pub stats: SwarmStats,
    pub t: f32,
    /// Per-robot event from the last tick, for visualisation.
    pub last_events: Vec<Option<Event>>,
    /// Centralized mode: last command and when it arrived.
    remote: Vec<Option<(Command, f32)>>,
    next_state: Vec<f32>,
}

impl Swarm {
    pub fn new(layout: Layout, control: ControlMode) -> Self {
        let world = SimWorld::new(&layout);
        let n = layout.n_robots();
        Self {
            brains: (0..n)
                .map(|i| Brain::new(i as u16, layout.initial_team(i) as u8))
                .collect(),
            last_events: vec![None; n],
            remote: vec![None; n],
            // Stagger state reports across the period.
            next_state: (0..n).map(|i| i as f32 * STATE_PERIOD / n as f32).collect(),
            world,
            layout,
            control,
            stats: SwarmStats::default(),
            t: 0.0,
        }
    }

    /// A swarm starting from a camera snapshot (see [`SimWorld::from_snapshot`]).
    pub fn from_snapshot(layout: Layout, snap: &Snapshot, control: ControlMode) -> Self {
        let mut s = Self::new(layout, control);
        s.world = SimWorld::from_snapshot(&s.layout, snap);
        s.t = snap.t;
        s
    }

    /// Advance one tick. `downlinks` are messages that arrived this tick; the
    /// return value is robot radio traffic produced this tick.
    pub fn step(&mut self, dt: f32, downlinks: Vec<Downlink>) -> Vec<Uplink> {
        let mut up = Vec::new();
        for d in downlinks {
            let i = d.robot as usize;
            if i >= self.brains.len() || !self.world.robots[i].active {
                continue;
            }
            match d.body {
                DownBody::Tree(bytes) => {
                    self.stats.trees_received += 1;
                    if let Some((version, valid_until)) = self.brains[i].receive(&bytes) {
                        up.push(Uplink::TreeAck {
                            robot: i as u16,
                            version,
                            valid_until,
                        });
                    }
                }
                DownBody::Command {
                    force,
                    vel,
                    grip,
                    release,
                } => {
                    self.remote[i] = Some((
                        Command {
                            force,
                            vel,
                            grip,
                            release,
                        },
                        self.t,
                    ));
                    self.stats.commands_received += 1;
                }
            }
        }

        let objects = self.world.object_states();
        for i in 0..self.brains.len() {
            self.last_events[i] = None;
            if !self.world.robots[i].active {
                continue;
            }
            let team = self.brains[i].team as usize;
            let rb = &self.world.robots[i];
            if self.control == ControlMode::Speculative && rb.grip.is_some() && rb.holding != team {
                // Moved to another team: let go of the old payload first.
                self.world.release(i);
            }
            let rb = &self.world.robots[i];
            let sense = Sense {
                t: self.t,
                pos: self.world.robot_pos(i),
                vel: self.world.robot_vel(i),
                attached: rb.grip.is_some(),
                grip_slot: rb.grip_slot,
                object: objects[team],
                goal: self.layout.payloads[team].goal,
            };

            let cmd = match self.control {
                ControlMode::Speculative => {
                    let (cmd, event) = self.brains[i].tick(&sense, dt, &mut up);
                    match event {
                        Some(Event::Hit) => self.stats.hits += 1,
                        Some(Event::Miss) => self.stats.misses += 1,
                        None => {}
                    }
                    self.last_events[i] = event;
                    cmd
                }
                ControlMode::Centralized => {
                    if self.t >= self.next_state[i] {
                        let idle = self.remote[i].is_some_and(|(_, at)| self.t - at > 1.0);
                        self.next_state[i] = self.t
                            + if idle {
                                IDLE_STATE_PERIOD
                            } else {
                                STATE_PERIOD
                            };
                        up.push(Uplink::State {
                            robot: i as u16,
                            pos: sense.pos,
                            vel: sense.vel,
                            grip: sense.attached.then_some(sense.grip_slot),
                        });
                    }
                    match &mut self.remote[i] {
                        Some((cmd, at)) if self.t - *at < COMMAND_TIMEOUT => {
                            let mut out = *cmd;
                            if let Some(v) = out.vel {
                                // Close the velocity loop locally at full rate.
                                out.force = track(v, sense.vel);
                            }
                            // Force is held until the next command; grip and
                            // release are one-shot.
                            cmd.grip = false;
                            cmd.release = false;
                            out
                        }
                        _ => Command::default(), // watchdog: stop
                    }
                }
            };

            if cmd.release {
                self.world.release(i);
            }
            if cmd.grip {
                // A remote-controlled robot grips whatever it is next to.
                let k = match self.control {
                    ControlMode::Speculative => team,
                    ControlMode::Centralized => self.world.nearest_object(sense.pos),
                };
                self.world.grip(i, k);
            }
            self.world.drive(i, cmd.force);
        }

        self.world.step(dt);
        self.t += dt;
        up
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            t: self.t,
            objects: self.world.object_states(),
            robots: self
                .world
                .robots
                .iter()
                .enumerate()
                .map(|(i, r)| RobotObs {
                    id: i as u16,
                    pos: self.world.robot_pos(i),
                    active: r.active,
                    attached: r.grip.is_some(),
                    holding: r.holding as u8,
                })
                .collect(),
            obstacles: self.world.crate_aabbs(),
        }
    }

    /// Switch control mode mid-run; robots keep their grips.
    pub fn set_control(&mut self, control: ControlMode) {
        self.control = control;
        self.remote.iter_mut().for_each(|r| *r = None);
    }

    /// Payload `k` is at its goal.
    pub fn delivered(&self, k: usize) -> bool {
        self.world
            .object_state(k)
            .pos
            .dist(self.layout.payloads[k].goal)
            < DELIVERED_RADIUS
    }

    /// Every payload is at its goal.
    pub fn at_goal(&self) -> bool {
        (0..self.layout.payloads.len()).all(|k| self.delivered(k))
    }
}
