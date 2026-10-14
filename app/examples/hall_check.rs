//! The hall demo as authored (no jitter), headless: progress and surprises.
//!     cargo run --release --example hall_check
use antlift_app::scenario::{Level, DT};
use antlift_app::{Session, SessionConfig};
use antlift_core::ControlMode;

fn main() {
    let mut s = Session::new(SessionConfig {
        layout: Level::Hall.base(),
        edge: antlift_edge::EdgeConfig {
            tree_budget: std::env::args()
                .nth(1)
                .and_then(|b| b.parse().ok())
                .unwrap_or(256),
            ..SessionConfig::new(ControlMode::Speculative).edge
        },
        ..SessionConfig::new(ControlMode::Speculative)
    });
    let mut next = 10.0;
    while s.t() < 240.0 && !s.swarm.delivered(0) {
        s.step(DT);
        if s.t() >= next {
            next += 10.0;
            let o = s.swarm.world.object_state(0);
            println!(
                "t={:.0} pos ({:.1},{:.1}) hits {} surprises {}",
                s.t(),
                o.pos.x,
                o.pos.y,
                s.hits(),
                s.misses()
            );
        }
    }
    println!("end t={:.1} delivered {}", s.t(), s.swarm.delivered(0));
}
