//! Reproducible experiments: link conditions, scripted disruptions and
//! seeded start poses, run headless.

use crate::{Session, SessionConfig};
use antlift_core::{ControlMode, Layout, V2};
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
}

#[derive(Clone, Debug)]
pub struct Condition {
    pub name: String,
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
            link: clean,
            disruptions: vec![],
        },
        Condition {
            name: "slow".into(),
            link: slow,
            disruptions: vec![],
        },
        Condition {
            name: "lossy".into(),
            link: lossy,
            disruptions: vec![],
        },
        Condition {
            name: "slow+lossy".into(),
            link: harsh,
            disruptions: vec![],
        },
        Condition {
            name: "disrupted".into(),
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
                link: LinkConfig::default(),
                disruptions: vec![Disruption::Outage { at: 20.0, secs }],
            };
            (secs, c)
        })
        .collect()
}

/// The doorway level with the payload's start pose jittered by `seed`.
pub fn layout_for(seed: u64) -> Layout {
    let mut l = Layout::doorway();
    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    l.object_start += V2::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0));
    l.object_start_angle += rng.gen_range(-0.4..0.4);
    l
}

#[derive(Clone, Debug)]
pub struct RunResult {
    pub condition: String,
    pub control: ControlMode,
    pub seed: u64,
    /// Sim time the payload reached the goal, if it did.
    pub reached: Option<f32>,
    pub final_dist: f32,
    pub hits: u64,
    pub misses: u64,
    pub plans: u64,
    pub up_bytes: u64,
    pub down_bytes: u64,
    pub bytes_per_robot_s: f32,
}

pub fn run(cond: &Condition, control: ControlMode, seed: u64, limit: f32) -> RunResult {
    let mut sess = Session::new(SessionConfig {
        layout: layout_for(seed),
        link: cond.link,
        seed,
        ..SessionConfig::new(control)
    });
    let mut done = vec![false; cond.disruptions.len()];
    let mut reached = None;
    while sess.t() < limit {
        let t = sess.t();
        for (k, d) in cond.disruptions.iter().enumerate() {
            apply(&mut sess, d, t, &mut done[k]);
        }
        sess.step(DT);
        if sess.swarm.at_goal() {
            reached = Some(sess.t());
            break;
        }
    }
    let view = sess.edge.view();
    RunResult {
        condition: cond.name.clone(),
        control,
        seed,
        reached,
        final_dist: sess
            .swarm
            .world
            .object_state()
            .pos
            .dist(sess.swarm.layout.goal),
        hits: sess.hits(),
        misses: sess.misses(),
        plans: view.stats.plans,
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
            let o = sess.swarm.world.object_state();
            sess.swarm.world.kick(o.pos, local.rotate(o.angle));
            *done = true;
        }
        Disruption::Shake { at } if !*done && t >= at => {
            let o = sess.swarm.world.object_state();
            let front = o.pos + V2::from_angle(o.angle) * o.half.x;
            for i in 0..sess.swarm.brains.len() {
                if sess.swarm.world.robot_pos(i).dist(front) < 2.5 {
                    sess.swarm.world.release(i);
                }
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
        assert_eq!(layout_for(3).object_start, layout_for(3).object_start);
        assert_ne!(layout_for(3).object_start, layout_for(4).object_start);
    }

    #[test]
    fn ci_of_constant_is_zero() {
        assert_eq!(mean_ci(&[2.0, 2.0, 2.0]), Some((2.0, 0.0)));
        assert_eq!(mean_ci(&[]), None);
    }
}
