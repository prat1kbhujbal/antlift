//! Ghost futures: simulate "what if" scenarios from the edge's snapshot of
//! the present, with the robots running the very trees they would receive,
//! and see where the payload jams or drifts off-axis.

use antlift_core::{ControlMode, Layout, Snapshot, V2};
use antlift_sim::Swarm;
use std::thread;

const DT: f32 = 1.0 / 60.0;
/// When the scripted disruption hits, after the rollout starts (s).
const DISRUPT_AT: f32 = 1.0;
/// Record a ghost pose this often (s).
const SAMPLE: f32 = 0.25;
/// No net progress over this window means the future jammed.
const STALL_WINDOW: f32 = 2.0;
const STALL_DIST: f32 = 0.25;
const STALL_TURN: f32 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WhatIf {
    Nominal,
    /// Sideways shove on the payload's front half; `side` is +1 or -1.
    Kick {
        side: f32,
        impulse: f32,
    },
    /// Robots near the front end lose their grip.
    GripLoss,
    /// A few robots die.
    RobotsDown,
}

impl WhatIf {
    pub fn label(self) -> &'static str {
        match self {
            WhatIf::Nominal => "as planned",
            WhatIf::Kick { .. } => "kicked",
            WhatIf::GripLoss => "grip lost",
            WhatIf::RobotsDown => "robots down",
        }
    }
}

/// The futures the edge imagines every time it plans.
pub const WHAT_IFS: [WhatIf; 7] = [
    WhatIf::Nominal,
    WhatIf::Kick {
        side: 1.0,
        impulse: 80.0,
    },
    WhatIf::Kick {
        side: -1.0,
        impulse: 80.0,
    },
    WhatIf::Kick {
        side: 1.0,
        impulse: 150.0,
    },
    WhatIf::Kick {
        side: -1.0,
        impulse: 150.0,
    },
    WhatIf::GripLoss,
    WhatIf::RobotsDown,
];

#[derive(Clone, Debug)]
pub struct Ghost {
    pub what_if: WhatIf,
    /// Payload (position, heading) every [`SAMPLE`] seconds.
    pub poses: Vec<(V2, f32)>,
    /// Where this future jammed, if it did.
    pub stall: Option<V2>,
}

/// Run one future for `horizon` seconds, watching payload `team`.
/// `trees[i]` is robot i's encoded tree; every team runs its own.
pub fn simulate(
    layout: &Layout,
    snap: &Snapshot,
    trees: &[Option<Vec<u8>>],
    what_if: WhatIf,
    horizon: f32,
    team: usize,
) -> Ghost {
    let mut swarm = Swarm::from_snapshot(layout.clone(), snap, ControlMode::Speculative);
    for (b, t) in swarm.brains.iter_mut().zip(trees) {
        if let Some(bytes) = t {
            b.receive(bytes);
        }
    }

    let t0 = swarm.t;
    let mut poses = vec![];
    let start = snap.objects[team];
    let mut anchor = (start.pos, start.angle, t0);
    let mut disrupted = false;
    let mut stall = None;
    while swarm.t - t0 < horizon {
        if !disrupted && swarm.t - t0 >= DISRUPT_AT {
            disrupted = true;
            disrupt(&mut swarm, what_if, team);
        }
        swarm.step(DT, vec![]);
        let o = swarm.world.object_state(team);
        if poses.len() as f32 * SAMPLE <= swarm.t - t0 {
            poses.push((o.pos, o.angle));
        }
        if swarm.delivered(team) {
            break;
        }
        let moved = o.pos.dist(anchor.0) > STALL_DIST
            || antlift_core::wrap_angle(o.angle - anchor.1).abs() > STALL_TURN;
        if moved {
            anchor = (o.pos, o.angle, swarm.t);
        } else if swarm.t - anchor.2 > STALL_WINDOW && swarm.t - t0 > DISRUPT_AT + 0.5 {
            stall = Some(o.pos);
            break;
        }
    }
    Ghost {
        what_if,
        poses,
        stall,
    }
}

fn disrupt(swarm: &mut Swarm, what_if: WhatIf, team: usize) {
    let team_robots: Vec<usize> = (0..swarm.brains.len())
        .filter(|&i| swarm.brains[i].team as usize == team)
        .collect();
    let w = &mut swarm.world;
    let o = w.object_state(team);
    let axis = V2::from_angle(o.angle);
    match what_if {
        WhatIf::Nominal => {}
        WhatIf::Kick { side, impulse } => {
            w.kick(o.pos + axis * 1.0, axis.perp() * side * impulse);
        }
        WhatIf::GripLoss => {
            let front = o.pos + axis * o.half.x;
            for &i in &team_robots {
                if w.robot_pos(i).dist(front) < 2.5 {
                    w.release(i);
                }
            }
        }
        WhatIf::RobotsDown => {
            for &i in team_robots.iter().step_by(5) {
                w.disable_robot(i);
            }
        }
    }
}

/// Run every future in [`WHAT_IFS`] in parallel. Results are in input order,
/// so they're deterministic.
pub fn simulate_all(
    layout: &Layout,
    snap: &Snapshot,
    trees: &[Option<Vec<u8>>],
    horizon: f32,
    team: usize,
) -> Vec<Ghost> {
    thread::scope(|s| {
        let handles: Vec<_> = WHAT_IFS
            .iter()
            .map(|&w| s.spawn(move || simulate(layout, snap, trees, w, horizon, team)))
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("rollout thread"))
            .collect()
    })
}
