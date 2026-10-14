//! Reproducible experiments: link conditions, scripted disruptions and
//! seeded start poses, run headless.

use crate::{Session, SessionConfig};
use antlift_core::{ControlMode, Layout, V2};
use antlift_edge::EdgeConfig;
use antlift_net::LinkConfig;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

pub const DT: f32 = 1.0 / 60.0;

#[derive(Clone, Copy, Debug)]
pub enum Disruption {
    /// Drop a crate at `pos`.
    Crate { at: f32, pos: V2 },
    /// Shove the payload sideways (impulse in its local frame, N s).
    Kick { at: f32, local: V2 },
    /// Every robot within 2.5 m of the payload's front end loses its grip.
    Shake { at: f32 },
    /// The edge is unreachable for `secs`.
    Outage { at: f32, secs: f32 },
    /// Every `every`-th robot dies.
    KillRobots { at: f32, every: usize },
}

#[derive(Clone, Debug)]
pub struct Condition {
    pub name: String,
    pub level: Level,
    pub link: LinkConfig,
    pub disruptions: Vec<Disruption>,
}

pub fn conditions() -> Vec<Condition> {
    let clean = LinkConfig::default();
    let slow = LinkConfig {
        latency: 0.15,
        jitter: 0.05,
        loss: 0.0,
    };
    let lossy = LinkConfig { loss: 0.2, ..clean };
    let harsh = LinkConfig { loss: 0.2, ..slow };
    vec![
        Condition {
            name: "clean".into(),
            level: Level::Doorway,
            link: clean,
            disruptions: vec![],
        },
        Condition {
            name: "slow".into(),
            level: Level::Doorway,
            link: slow,
            disruptions: vec![],
        },
        Condition {
            name: "lossy".into(),
            level: Level::Doorway,
            link: lossy,
            disruptions: vec![],
        },
        Condition {
            name: "slow+lossy".into(),
            level: Level::Doorway,
            link: harsh,
            disruptions: vec![],
        },
        Condition {
            name: "disrupted".into(),
            level: Level::Doorway,
            link: clean,
            disruptions: vec![
                Disruption::Kick {
                    at: 15.0,
                    local: V2::new(0.0, 120.0),
                },
                Disruption::Crate {
                    at: 20.0,
                    pos: V2::new(23.0, 12.0),
                },
                Disruption::Shake { at: 30.0 },
                Disruption::Kick {
                    at: 45.0,
                    local: V2::new(0.0, -120.0),
                },
            ],
        },
        Condition {
            name: "outage 20-30s".into(),
            level: Level::Doorway,
            link: clean,
            disruptions: vec![Disruption::Outage {
                at: 20.0,
                secs: 10.0,
            }],
        },
    ]
}

/// One-way latency from 0 to 1 s (jitter 20% of it) with 10% loss.
pub fn latency_sweep() -> Vec<(f32, Condition)> {
    [0.0, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0]
        .into_iter()
        .map(|lat| {
            let c = Condition {
                name: format!("latency {:.0} ms", lat * 1000.0),
                level: Level::Doorway,
                link: LinkConfig {
                    latency: lat,
                    jitter: lat * 0.2,
                    loss: 0.1,
                },
                disruptions: vec![],
            };
            (lat * 1000.0, c)
        })
        .collect()
}

/// The edge drops out at t = 20 s for 0 to 40 s, on a clean link.
pub fn outage_sweep() -> Vec<(f32, Condition)> {
    [0.0, 5.0, 10.0, 20.0, 30.0, 40.0]
        .into_iter()
        .map(|secs| {
            let c = Condition {
                name: format!("outage {secs:.0} s"),
                level: Level::Doorway,
                link: LinkConfig::default(),
                disruptions: vec![Disruption::Outage { at: 20.0, secs }],
            };
            (secs, c)
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Doorway,
    Warehouse,
}

impl Level {
    /// The level exactly as authored.
    pub fn base(self) -> Layout {
        match self {
            Level::Doorway => Layout::doorway(),
            Level::Warehouse => Layout::warehouse(),
        }
    }

    /// The level with its payloads' start poses jittered by `seed`.
    pub fn layout(self, seed: u64) -> Layout {
        let (mut l, jitter, turn) = match self {
            Level::Doorway => (Layout::doorway(), 1.0, 0.4),
            Level::Warehouse => (Layout::warehouse(), 0.5, 0.2),
        };
        let mut rng = ChaCha8Rng::seed_from_u64(seed);
        for p in &mut l.payloads {
            p.start += V2::new(
                rng.gen_range(-jitter..jitter),
                rng.gen_range(-jitter..jitter),
            );
            p.angle += rng.gen_range(-turn..turn);
        }
        l
    }

    /// Time limit for a run (s).
    pub fn limit(self) -> f32 {
        match self {
            Level::Doorway => 240.0,
            Level::Warehouse => 420.0,
        }
    }
}

/// The doorway level with the payload's start pose jittered by `seed`.
pub fn layout_for(seed: u64) -> Layout {
    Level::Doorway.layout(seed)
}

/// The three-team warehouse, under clean, lossy, slow, attrition and outage.
pub fn warehouse_conditions() -> Vec<Condition> {
    let clean = LinkConfig::default();
    let w = |name: &str, link: LinkConfig, disruptions: Vec<Disruption>| Condition {
        name: name.into(),
        level: Level::Warehouse,
        link,
        disruptions,
    };
    vec![
        w("warehouse", clean, vec![]),
        w(
            "warehouse slow",
            LinkConfig {
                latency: 0.15,
                jitter: 0.05,
                loss: 0.0,
            },
            vec![],
        ),
        w("warehouse lossy", LinkConfig { loss: 0.2, ..clean }, vec![]),
        w(
            "warehouse attrition",
            clean,
            vec![Disruption::KillRobots { at: 40.0, every: 4 }],
        ),
        w(
            "warehouse outage",
            clean,
            vec![Disruption::Outage {
                at: 40.0,
                secs: 15.0,
            }],
        ),
    ]
}

/// One way of running the swarm, as compared in the benchmark.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Variant {
    pub name: &'static str,
    pub control: ControlMode,
    /// Ghost futures on (probabilities from simulation) or off (heuristics).
    pub ghosts: bool,
}

pub const VARIANTS: [Variant; 3] = [
    Variant {
        name: "speculative",
        control: ControlMode::Speculative,
        ghosts: true,
    },
    Variant {
        name: "speculative-heuristic",
        control: ControlMode::Speculative,
        ghosts: false,
    },
    Variant {
        name: "centralized",
        control: ControlMode::Centralized,
        ghosts: true,
    },
];

#[derive(Clone, Debug)]
pub struct RunResult {
    pub condition: String,
    pub variant: Variant,
    pub seed: u64,
    /// Sim time the last payload was delivered, if every one was.
    pub reached: Option<f32>,
    /// When each payload first reached its goal.
    pub deliveries: Vec<Option<f32>>,
    pub final_dist: f32,
    pub hits: u64,
    pub misses: u64,
    pub plans: u64,
    /// Plans that had to stop short of a choke point another team held.
    pub zone_waits: u64,
    /// Robots lent from a delivered team to another.
    pub reassigned: u64,
    pub up_bytes: u64,
    pub down_bytes: u64,
    pub bytes_per_robot_s: f32,
}

pub fn run(cond: &Condition, variant: Variant, seed: u64, limit: f32) -> RunResult {
    let base = SessionConfig::new(variant.control);
    let mut sess = Session::new(SessionConfig {
        edge: EdgeConfig {
            ghosts: variant.ghosts,
            ..base.edge
        },
        layout: cond.level.layout(seed),
        link: cond.link,
        seed,
        ..base
    });
    let mut done = vec![false; cond.disruptions.len()];
    let mut reached = None;
    let mut deliveries = vec![None; sess.swarm.layout.payloads.len()];
    while sess.t() < limit {
        let t = sess.t();
        for (k, d) in cond.disruptions.iter().enumerate() {
            apply(&mut sess, d, t, &mut done[k]);
        }
        sess.step(DT);
        for (k, d) in deliveries.iter_mut().enumerate() {
            if d.is_none() && sess.swarm.delivered(k) {
                *d = Some(sess.t());
            }
        }
        // Done once every payload has been delivered (a delivered item is
        // done even if something nudges it afterwards).
        if deliveries.iter().all(|d| d.is_some()) {
            reached = Some(sess.t());
            break;
        }
    }
    let view = sess.edge.view();
    RunResult {
        condition: cond.name.clone(),
        variant,
        seed,
        reached,
        final_dist: sess.distance_left(),
        deliveries,
        hits: sess.hits(),
        misses: sess.misses(),
        plans: view.stats.plans,
        zone_waits: view.stats.zone_waits,
        reassigned: view.stats.reassigned,
        up_bytes: sess.radio.up_bytes,
        down_bytes: sess.radio.down_bytes,
        bytes_per_robot_s: sess.bytes_per_robot_s(),
    }
}

pub fn apply(sess: &mut Session, d: &Disruption, t: f32, done: &mut bool) {
    match *d {
        Disruption::Crate { at, pos } if !*done && t >= at => {
            sess.swarm.world.add_crate(pos);
            *done = true;
        }
        Disruption::Kick { at, local } if !*done && t >= at => {
            let o = sess.swarm.world.object_state(0);
            sess.swarm.world.kick(o.pos, local.rotate(o.angle));
            *done = true;
        }
        Disruption::Shake { at } if !*done && t >= at => {
            let o = sess.swarm.world.object_state(0);
            let front = o.pos + V2::from_angle(o.angle) * o.half.x;
            for i in 0..sess.swarm.brains.len() {
                if sess.swarm.world.robot_pos(i).dist(front) < 2.5 {
                    sess.swarm.world.release(i);
                }
            }
            *done = true;
        }
        Disruption::KillRobots { at, every } if !*done && t >= at => {
            for i in (0..sess.swarm.brains.len()).step_by(every) {
                sess.swarm.world.disable_robot(i);
            }
            *done = true;
        }
        Disruption::Outage { at, secs } => {
            let down = t >= at && t < at + secs;
            if down == sess.edge_up() {
                sess.set_edge_up(!down);
            }
        }
        _ => {}
    }
}

/// Mean and 95% confidence half-width (normal approximation).
pub fn mean_ci(xs: &[f32]) -> Option<(f32, f32)> {
    if xs.is_empty() {
        return None;
    }
    let n = xs.len() as f32;
    let mean = xs.iter().sum::<f32>() / n;
    if xs.len() < 2 {
        return Some((mean, 0.0));
    }
    let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / (n - 1.0);
    Some((mean, 1.96 * (var / n).sqrt()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layouts_are_reproducible_and_varied() {
        assert_eq!(
            layout_for(3).payloads[0].start,
            layout_for(3).payloads[0].start
        );
        assert_ne!(
            layout_for(3).payloads[0].start,
            layout_for(4).payloads[0].start
        );
    }

    #[test]
    fn ci_of_constant_is_zero() {
        assert_eq!(mean_ci(&[2.0, 2.0, 2.0]), Some((2.0, 0.0)));
        assert_eq!(mean_ci(&[]), None);
    }
}
