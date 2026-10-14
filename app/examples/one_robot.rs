//! Follow one robot in a warehouse run.
//!     cargo run --release --example one_robot -- <seed> <robot> <from> <to>
use antlift_app::scenario::Level;
use antlift_app::{EdgeRuntime, Session, SessionConfig};
use antlift_core::ControlMode;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let seed: u64 = a[1].parse().unwrap();
    let id: usize = a[2].parse().unwrap();
    let (from, to): (f32, f32) = (a[3].parse().unwrap(), a[4].parse().unwrap());
    let layout = if seed == 0 {
        Level::Warehouse.base()
    } else {
        Level::Warehouse.layout(seed)
    };
    let mut s = Session::new(SessionConfig {
        layout,
        seed: seed.max(1),
        ..SessionConfig::new(ControlMode::Speculative)
    });
    let mut next = from;
    while s.t() < to {
        s.step(1.0 / 60.0);
        if s.t() < next {
            continue;
        }
        next += 3.0;
        let EdgeRuntime::Inline(e) = &s.edge else {
            unreachable!()
        };
        let b = &s.swarm.brains[id];
        let p = s.swarm.world.robot_pos(id);
        let v = s.swarm.world.robot_vel(id);
        let tree = b.tree.as_ref();
        println!("t={:5.1} pos ({:5.1},{:5.1}) v {:.2} edge team {} brain team {} mode {:?} node {} via {:?} grip {}",
            s.t(), p.x, p.y, v.len(), e.robot_team(id), b.team, b.mode(), b.node,
            tree.map(|t| t.via.iter().map(|q| format!("({:.0},{:.0})", q.x, q.y)).collect::<Vec<_>>()),
            s.swarm.world.robots[id].grip.is_some());
    }
}
