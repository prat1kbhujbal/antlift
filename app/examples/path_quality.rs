//! Measure planned path quality: length vs a straight-ish lower bound, heading
//! changes, and turn-in-place stops.
use antlift_app::scenario::Level;
use antlift_app::{EdgeRuntime, Session, SessionConfig};
use antlift_core::{axis_error, ControlMode};

fn main() {
    for level in [Level::Hall, Level::Warehouse] {
        let mut s = Session::new(SessionConfig {
            layout: level.base(),
            ..SessionConfig::new(ControlMode::Speculative)
        });
        while s.t() < 1.0 {
            s.step(1.0 / 60.0);
        }
        let EdgeRuntime::Inline(e) = &s.edge else {
            unreachable!()
        };
        for p in e.plans() {
            let len: f32 = p.path.windows(2).map(|w| w[0].dist(w[1])).sum();
            let direct = p.path.first().unwrap().dist(*p.path.last().unwrap());
            let mut dir_changes = 0;
            for w in p.path.windows(3) {
                let a = (w[1] - w[0]).angle();
                let b = (w[2] - w[1]).angle();
                if antlift_core::wrap_angle(a - b).abs() > 0.1 {
                    dir_changes += 1;
                }
            }
            let turns = p.waypoints.iter().filter(|w| w.turn_in_place).count();
            let mut heading_flips = 0;
            for w in p.waypoints.windows(2) {
                if axis_error(w[0].heading, w[1].heading).abs() > 0.3 {
                    heading_flips += 1;
                }
            }
            println!("{level:?} team {}: path {:.1} m (straight line {:.1} m), {} cells, {} direction changes, {} waypoints, {} turn-in-place stops, {} heading changes >17°",
                p.team, len, direct, p.path.len(), dir_changes, p.waypoints.len(), turns, heading_flips);
        }
    }
}
