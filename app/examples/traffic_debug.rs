//! Print traffic waits, reroutes and payload progress in a warehouse run.
//!     cargo run --release --example traffic_debug -- <seed> [speculative|heuristic|centralized] [print every s] [traffic|notraffic] [condition]
use antlift_app::scenario::{warehouse_conditions, Level, DT};
use antlift_app::{Session, SessionConfig};
use antlift_core::ControlMode;
use antlift_edge::EdgeConfig;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let seed: u64 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let mode = a.get(2).map(String::as_str);
    let control = if mode == Some("centralized") {
        ControlMode::Centralized
    } else {
        ControlMode::Speculative
    };
    let layout = Level::Warehouse.layout(seed);
    let base = SessionConfig::new(control);
    // Optional link condition by name, e.g. "warehouse slow" (disruptions
    // aren't replayed).
    let link = a
        .get(5)
        .and_then(|n| warehouse_conditions().into_iter().find(|c| &c.name == n))
        .map_or(base.link, |c| c.link);
    let mut s = Session::new(SessionConfig {
        layout: layout.clone(),
        seed,
        link,
        edge: EdgeConfig {
            ghosts: mode != Some("heuristic"),
            traffic: a.get(4).map(String::as_str) != Some("notraffic"),
            ..base.edge
        },
        ..base
    });
    let every: f32 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let n = layout.payloads.len();
    let name = |k: usize| layout.payloads[k].name;
    let mut prev: Vec<Option<usize>> = vec![None; n];
    let mut next = 0.0;
    while s.t() < 420.0 && !(0..n).all(|k| s.swarm.delivered(k)) {
        s.step(DT);
        let view = s.edge.view();
        for (k, last) in prev.iter_mut().enumerate() {
            let w = view
                .plans
                .iter()
                .find(|p| p.team == k)
                .and_then(|p| p.traffic_wait);
            if w != *last {
                let o = s.swarm.world.object_state(k);
                let what = w.map_or("moves on".to_string(), |j| format!("WAITS for {}", name(j)));
                println!(
                    "t={:6.1} {} {what} at ({:.1},{:.1})",
                    s.t(),
                    name(k),
                    o.pos.x,
                    o.pos.y
                );
                *last = w;
            }
        }
        if s.t() >= next {
            next += every;
            let p: Vec<String> = (0..n)
                .map(|k| {
                    let o = s.swarm.world.object_state(k);
                    let done = if s.swarm.delivered(k) { "*" } else { "" };
                    format!("{}{done}({:.0},{:.0})", name(k), o.pos.x, o.pos.y)
                })
                .collect();
            let st = view.stats;
            println!(
                "          t={:.0} {}  plans {} waits {} reroutes {} permits {}",
                s.t(),
                p.join(" "),
                st.plans,
                st.traffic_waits,
                st.reroutes,
                st.permits_sent
            );
        }
    }
    println!("end t={:.1}", s.t());
}
