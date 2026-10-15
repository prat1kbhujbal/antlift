//! Radio once everything is delivered: run until every payload is delivered
//! and every robot has parked (or 10 s have passed), then report bytes per
//! robot per second over the next 20 s.
//!     cargo run --release --example quiet_check -- [hall|warehouse] [seed]
use antlift_app::scenario::{Level, DT};
use antlift_app::{EdgeRuntime, Session, SessionConfig};
use antlift_core::{ControlMode, Mode};

fn parked(s: &Session, control: ControlMode) -> usize {
    let EdgeRuntime::Inline(e) = &s.edge else {
        unreachable!()
    };
    match control {
        ControlMode::Speculative => s
            .swarm
            .brains
            .iter()
            .filter(|b| b.mode() == Mode::Parked)
            .count(),
        ControlMode::Centralized => e
            .remote_brains()
            .filter(|b| b.mode() == Mode::Parked)
            .count(),
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let level = if a.get(1).map(String::as_str) == Some("warehouse") {
        Level::Warehouse
    } else {
        Level::Hall
    };
    let seed: u64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
    for control in [ControlMode::Speculative, ControlMode::Centralized] {
        let mut s = Session::new(SessionConfig {
            layout: level.layout(seed),
            seed,
            ..SessionConfig::new(control)
        });
        let n = s.swarm.layout.payloads.len();
        let robots = s.swarm.brains.len();
        while s.t() < level.limit() && !(0..n).all(|k| s.swarm.delivered(k)) {
            s.step(DT);
        }
        let delivered = s.t();
        while s.t() < delivered + 10.0 && parked(&s, control) < robots {
            s.step(DT);
        }
        let settled = s.t();
        let before = s.radio.up_bytes + s.radio.down_bytes;
        while s.t() < settled + 20.0 {
            s.step(DT);
        }
        let bytes = s.radio.up_bytes + s.radio.down_bytes - before;
        println!(
            "{control:?}: delivered at {delivered:.0} s, parked {:.1} s later ({}/{robots}); then {:.1} B/robot/s",
            settled - delivered,
            parked(&s, control),
            bytes as f32 / 20.0 / robots as f32,
        );
    }
}
