//! How long lent robots take to grip their new payload (warehouse).
//!     cargo run --release --example lend_check -- [seed]   (0 = demo layout)
use antlift_app::scenario::Level;
use antlift_app::{EdgeRuntime, Session, SessionConfig};
use antlift_core::ControlMode;

fn main() {
    let seed: u64 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(0);
    let layout = if seed == 0 {
        Level::Warehouse.base()
    } else {
        Level::Warehouse.layout(seed)
    };
    let mut s = Session::new(SessionConfig {
        layout: layout.clone(),
        seed: seed.max(1),
        ..SessionConfig::new(ControlMode::Speculative)
    });
    let n = s.swarm.brains.len();
    let mut lent_at: Vec<Option<f32>> = vec![None; n];
    let mut gripped_at: Vec<Option<f32>> = vec![None; n];
    let mut delivered_at: Vec<Option<f32>> = vec![None; layout.payloads.len()];
    while s.t() < 250.0 {
        s.step(1.0 / 60.0);
        let EdgeRuntime::Inline(e) = &s.edge else {
            unreachable!()
        };
        for (k, d) in delivered_at.iter_mut().enumerate() {
            if d.is_none() && s.swarm.delivered(k) {
                *d = Some(s.t());
            }
        }
        for i in 0..n {
            let team = e.robot_team(i);
            if team != layout.initial_team(i) && lent_at[i].is_none() {
                lent_at[i] = Some(s.t());
            }
            let r = &s.swarm.world.robots[i];
            if lent_at[i].is_some()
                && gripped_at[i].is_none()
                && r.grip.is_some()
                && r.holding == team
            {
                gripped_at[i] = Some(s.t());
            }
        }
    }
    let EdgeRuntime::Inline(e) = &s.edge else {
        unreachable!()
    };
    for i in 0..n {
        if let Some(l) = lent_at[i] {
            let p = s.swarm.world.robot_pos(i);
            let target = e.robot_team(i);
            let note = match (gripped_at[i], delivered_at[target]) {
                (None, Some(d)) => format!("(target delivered at {d:.1}s, nothing left to grip)"),
                (None, None) => "STUCK".to_string(),
                _ => String::new(),
            };
            println!(
                "robot {i:2}: {} -> {}  lent at {l:5.1}s  {}  now at ({:.1},{:.1}) {note}",
                layout.payloads[layout.initial_team(i)].name,
                layout.payloads[e.robot_team(i)].name,
                gripped_at[i].map_or("NEVER GRIPPED".to_string(), |g| format!(
                    "gripped after {:4.1}s",
                    g - l
                )),
                p.x,
                p.y
            );
        }
    }
}
