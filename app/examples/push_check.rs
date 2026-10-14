//! Who is pushing a delivered payload: robots touching payload `k` between
//! two times, with their team, grip and mode.
//!     cargo run --release --example push_check -- <seed> <payload> <from s> <to s> [centralized]
use antlift_app::scenario::{Level, DT};
use antlift_app::{EdgeRuntime, Session, SessionConfig};
use antlift_core::ControlMode;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let arg = |i: usize, d: f32| a.get(i).and_then(|s| s.parse().ok()).unwrap_or(d);
    let seed = arg(1, 9.0) as u64;
    let k = arg(2, 2.0) as usize;
    let (from, to) = (arg(3, 98.0), arg(4, 112.0));
    let mut s = Session::new(SessionConfig {
        layout: Level::Warehouse.layout(seed),
        seed,
        ..SessionConfig::new(if a.get(5).map(String::as_str) == Some("centralized") {
            ControlMode::Centralized
        } else {
            ControlMode::Speculative
        })
    });
    let mut next = from;
    while s.t() < to {
        s.step(DT);
        if s.t() < next {
            continue;
        }
        next += 1.0;
        let EdgeRuntime::Inline(e) = &s.edge else {
            unreachable!()
        };
        let o = s.swarm.world.object_state(k);
        let mut line = vec![];
        for i in 0..s.swarm.brains.len() {
            let p = s.swarm.world.robot_pos(i) - o.pos;
            let l = p.rotate(-o.angle);
            let gap = ((l.x.abs() - o.half.x).max(0.0).powi(2)
                + (l.y.abs() - o.half.y).max(0.0).powi(2))
            .sqrt();
            let rb0 = &s.swarm.world.robots[i];
            let grips_it = rb0.grip.is_some() && rb0.holding == k;
            if gap < 0.35 || grips_it {
                let rb = &s.swarm.world.robots[i];
                let b = if s.control == ControlMode::Centralized {
                    e.remote_brains().nth(i).unwrap()
                } else {
                    &s.swarm.brains[i]
                };
                line.push(format!(
                    "#{i}(t{} g{} {:?} {:?} at ({:.1},{:.1}) gap {gap:.2} via {:?})",
                    e.robot_team(i),
                    if rb.grip.is_some() {
                        rb.holding.to_string()
                    } else {
                        "-".into()
                    },
                    b.mode(),
                    b.action().map(|a| format!("{a:?}")
                        .split_whitespace()
                        .next()
                        .unwrap_or("")
                        .to_string()),
                    s.swarm.world.robot_pos(i).x,
                    s.swarm.world.robot_pos(i).y,
                    b.transit()
                        .iter()
                        .map(|p| (p.x.round(), p.y.round()))
                        .collect::<Vec<_>>()
                ));
            }
        }
        println!(
            "t={:.0} pos ({:.2},{:.2}) v ({:+.2},{:+.2}) {}",
            s.t(),
            o.pos.x,
            o.pos.y,
            o.vel.x,
            o.vel.y,
            line.join(" ")
        );
    }
}
