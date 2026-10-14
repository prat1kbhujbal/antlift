//! Follow one team in the warehouse: position, plan state and door owners.
//!
//!     cargo run --release --example team_debug -- [team] [seed] [seconds]
use antlift_app::scenario::Level;
use antlift_app::{EdgeRuntime, Session, SessionConfig};
use antlift_core::ControlMode;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let team: usize = a.get(1).and_then(|v| v.parse().ok()).unwrap_or(1);
    let seed: u64 = a.get(2).and_then(|v| v.parse().ok()).unwrap_or(1);
    let secs: f32 = a.get(3).and_then(|v| v.parse().ok()).unwrap_or(120.0);
    // Seed 0 is the demo: the level exactly as authored.
    let layout = if seed == 0 {
        Level::Warehouse.base()
    } else {
        Level::Warehouse.layout(seed)
    };
    let mut s = Session::new(SessionConfig {
        layout: layout.clone(),
        seed,
        ..SessionConfig::new(ControlMode::Speculative)
    });
    let mut next = 0.0;
    let mut last_version = 0;
    while s.t() < secs && !s.swarm.delivered(team) {
        s.step(1.0 / 60.0);
        if s.t() < next {
            continue;
        }
        next += 2.0;
        let EdgeRuntime::Inline(e) = &s.edge else {
            unreachable!()
        };
        let o = s.swarm.world.object_state(team);
        let plans = e.plans();
        let Some(p) = plans.iter().find(|p| p.team == team) else {
            continue;
        };
        let door = |x: f32, y: f32| {
            layout
                .chokepoints
                .iter()
                .position(|z| z.dist(antlift_core::V2::new(x, y)) == 0.0)
        };
        let route_doors: Vec<usize> = {
            let mut v = vec![];
            for q in &p.path {
                if let Some(z) = door(q.x, q.y) {
                    if v.last() != Some(&z) {
                        v.push(z);
                    }
                }
            }
            v
        };
        let target = p.waypoints.last().map(|w| (w.pos.x, w.pos.y));
        let owners: String = e
            .zone_owners()
            .iter()
            .map(|z| z.map_or('-', |k| (b'A' + k as u8) as char))
            .collect();
        let new_plan = if p.version != last_version {
            last_version = p.version;
            "NEW PLAN"
        } else {
            ""
        };
        let gripping = (0..s.swarm.brains.len())
            .filter(|&i| {
                s.swarm.world.robots[i].grip.is_some() && s.swarm.world.robots[i].holding == team
            })
            .count();
        let actions: Vec<String> = (0..s.swarm.brains.len())
            .filter(|&i| e.robot_team(i) == team)
            .filter_map(|i| {
                s.swarm.brains[i].action().map(|a| {
                    format!("{a:?}")
                        .split_whitespace()
                        .next()
                        .unwrap_or("")
                        .to_string()
                })
            })
            .collect();
        println!(
            "    angle {:.2} first wp heading {:?} gripping {} actions {:?}",
            o.angle,
            p.waypoints
                .first()
                .map(|w| (w.heading * 100.0).round() / 100.0),
            gripping,
            actions
        );
        println!("t={:5.1} {} at ({:5.1},{:5.1}) v=({:+.2},{:+.2})  route via doors {:?}  waiting_for={:?}  last wp {:?}  owners[{owners}] {new_plan}",
            s.t(), layout.payloads[team].name, o.pos.x, o.pos.y, o.vel.x, o.vel.y, route_doors, p.waiting_for,
            target.map(|(x, y)| format!("({x:.1},{y:.1})")), );
    }
}
