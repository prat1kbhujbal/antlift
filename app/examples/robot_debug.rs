//! Per-robot view of one team in the warehouse demo layout: tree, slot,
//! grip and origin, every few seconds, plus grip/release events.
//!     cargo run --release --example robot_debug -- [team] [seconds]
use antlift_app::scenario::Level;
use antlift_app::{EdgeRuntime, Session, SessionConfig};
use antlift_core::ControlMode;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let team: usize = a.get(1).and_then(|v| v.parse().ok()).unwrap_or(1);
    let secs: f32 = a.get(2).and_then(|v| v.parse().ok()).unwrap_or(150.0);
    let layout = Level::Warehouse.base();
    let mut s = Session::new(SessionConfig {
        layout: layout.clone(),
        ..SessionConfig::new(ControlMode::Speculative)
    });
    let n = s.swarm.brains.len();
    let mut was_gripping = vec![false; n];
    let mut next = 5.0;
    while s.t() < secs {
        s.step(1.0 / 60.0);
        let EdgeRuntime::Inline(e) = &s.edge else {
            unreachable!()
        };
        for (i, was) in was_gripping.iter_mut().enumerate() {
            let g = s.swarm.world.robots[i].grip.is_some();
            if e.robot_team(i) == team && g != *was {
                let b = &s.swarm.brains[i];
                println!(
                    "  t={:.1} robot {i} (starts in {}) {} | brain team {} node {} action {:?}",
                    s.t(),
                    layout.payloads[layout.initial_team(i)].name,
                    if g { "GRIPS" } else { "RELEASES" },
                    b.team,
                    b.node,
                    b.action()
                        .map(|a| format!("{a:?}").chars().take(20).collect::<String>())
                );
            }
            *was = g;
        }
        if s.t() >= next {
            next += 10.0;
            let members: Vec<usize> = (0..n).filter(|&i| e.robot_team(i) == team).collect();
            let reflex: Vec<usize> = members
                .iter()
                .copied()
                .filter(|&i| s.swarm.brains[i].tree.is_none())
                .collect();
            let gripping = members
                .iter()
                .filter(|&&i| s.swarm.world.robots[i].grip.is_some())
                .count();
            let wrong_team: Vec<usize> = members
                .iter()
                .copied()
                .filter(|&i| s.swarm.brains[i].team as usize != team)
                .collect();
            println!(
                "t={:.0} team {}: {} members, {} gripping, reflex {:?}, brain on other team {:?}",
                s.t(),
                layout.payloads[team].name,
                members.len(),
                gripping,
                reflex,
                wrong_team
            );
        }
    }
}
