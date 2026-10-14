//! Replay one benchmark run and print what the swarm is doing every few seconds.
//!
//!     cargo run --release --example trace -- [--cond clean] [--mode speculative|central]
//!                                            [--seed 1] [--limit 240] [--every 5]

use antlift_app::scenario::{conditions, latency_sweep, outage_sweep, warehouse_conditions, DT};
use antlift_app::{Session, SessionConfig};
use antlift_core::{Action, ControlMode};
use std::collections::BTreeMap;

fn arg(name: &str, default: &str) -> String {
    let args: Vec<String> = std::env::args().collect();
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1).cloned())
        .unwrap_or_else(|| default.to_string())
}

fn main() {
    let cond_name = arg("--cond", "clean");
    let control = match arg("--mode", "speculative").as_str() {
        "central" | "centralized" => ControlMode::Centralized,
        _ => ControlMode::Speculative,
    };
    let seed: u64 = arg("--seed", "1").parse().expect("--seed");
    let limit: f32 = arg("--limit", "240").parse().expect("--limit");
    let every: f32 = arg("--every", "5").parse().expect("--every");
    let cond = conditions()
        .into_iter()
        .chain(latency_sweep().into_iter().map(|(_, c)| c))
        .chain(outage_sweep().into_iter().map(|(_, c)| c))
        .chain(warehouse_conditions())
        .find(|c| c.name == cond_name)
        .unwrap_or_else(|| panic!("unknown condition {cond_name}"));

    let mut s = Session::new(SessionConfig {
        layout: cond.level.layout(seed),
        link: cond.link,
        seed,
        ..SessionConfig::new(control)
    });
    let mut done = vec![false; cond.disruptions.len()];
    let mut next = 0.0;
    while s.t() < limit && !s.swarm.at_goal() {
        let t = s.t();
        for (k, d) in cond.disruptions.iter().enumerate() {
            antlift_app::scenario::apply(&mut s, d, t, &mut done[k]);
        }
        s.step(DT);
        if s.t() < next {
            continue;
        }
        next += every;
        let mut actions: BTreeMap<&str, usize> = BTreeMap::new();
        for b in &s.swarm.brains {
            let name = match (control, b.action()) {
                (ControlMode::Centralized, _) => "remote",
                (_, None) => "reflex",
                (_, Some(Action::Approach)) => "approach",
                (_, Some(Action::Push { .. })) => "push",
                (_, Some(Action::Rotate { .. })) => "rotate",
                (_, Some(Action::Hold)) => "hold",
            };
            *actions.entry(name).or_default() += 1;
        }
        let v = s.edge.view();
        let layout = &s.swarm.layout;
        let payloads: Vec<String> = (0..layout.payloads.len())
            .map(|k| {
                let o = s.swarm.world.object_state(k);
                let mark = if s.swarm.delivered(k) { "*" } else { "" };
                format!(
                    "{}{mark}({:.1},{:.1})",
                    layout.payloads[k].name, o.pos.x, o.pos.y
                )
            })
            .collect();
        let zones: Vec<String> = v
            .zones
            .iter()
            .map(|z| z.map_or("-".into(), |k| layout.payloads[k].name.to_string()))
            .collect();
        println!(
            "t={:5.1}  {}  doors[{}]  hits={} surprises={} plans={} waits={} lent={}  {:?}",
            s.t(),
            payloads.join(" "),
            zones.join(""),
            s.hits(),
            s.misses(),
            v.stats.plans,
            v.stats.zone_waits,
            v.stats.reassigned,
            actions
        );
    }
    println!("stopped at t={:.1}, at goal: {}", s.t(), s.swarm.at_goal());
}
