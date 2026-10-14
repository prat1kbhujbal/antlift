//! Seconds that two moving teams spend in contact (payloads with their robot
//! rings overlapping) in warehouse runs.
//!     cargo run --release --example contact_check -- [seeds]
use antlift_app::scenario::{Level, DT};
use antlift_app::{Session, SessionConfig};
use antlift_core::{Aabb, ControlMode, ObjectState, V2};

fn ring(o: &ObjectState) -> Aabb {
    let (s, c) = o.angle.sin_cos();
    let m = 0.55; // robots around the payload
    let ex = (o.half.x * c).abs() + (o.half.y * s).abs() + m;
    let ey = (o.half.x * s).abs() + (o.half.y * c).abs() + m;
    Aabb::from_center(o.pos, V2::new(ex, ey))
}

fn overlap(a: &Aabb, b: &Aabb) -> bool {
    a.min.x < b.max.x && b.min.x < a.max.x && a.min.y < b.max.y && b.min.y < a.max.y
}

fn main() {
    let seeds: u64 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(10);
    // `notraffic`: teams only coordinate at doors (for an A/B comparison).
    let traffic = std::env::args().nth(2).as_deref() != Some("notraffic");
    for control in [ControlMode::Speculative, ControlMode::Centralized] {
        let mut total = 0.0;
        let mut runs_with_contact = 0;
        let (mut time, mut plans, mut done) = (0.0, 0, 0);
        for seed in 1..=seeds {
            let mut s = Session::new(SessionConfig {
                layout: Level::Warehouse.layout(seed),
                seed,
                edge: antlift_edge::EdgeConfig {
                    traffic,
                    ..SessionConfig::new(control).edge
                },
                ..SessionConfig::new(control)
            });
            let n = s.swarm.layout.payloads.len();
            let mut contact = 0.0;
            while s.t() < 420.0 && !(0..n).all(|k| s.swarm.delivered(k)) {
                s.step(DT);
                let live: Vec<usize> = (0..n).filter(|&k| !s.swarm.delivered(k)).collect();
                let boxes: Vec<Aabb> = live
                    .iter()
                    .map(|&k| ring(&s.swarm.world.object_state(k)))
                    .collect();
                if (0..boxes.len())
                    .any(|i| (i + 1..boxes.len()).any(|j| overlap(&boxes[i], &boxes[j])))
                {
                    contact += DT;
                }
            }
            if (0..n).all(|k| s.swarm.delivered(k)) {
                done += 1;
            }
            time += s.t();
            plans += s.edge.view().stats.plans;
            if contact > 0.0 {
                runs_with_contact += 1;
            }
            total += contact;
        }
        println!(
            "{control:?}: teams in contact {:.1} s per run on average, in {runs_with_contact}/{seeds} runs; \
             delivered {done}/{seeds}, mean time {:.0} s, {:.0} plans per run",
            total / seeds as f32,
            time / seeds as f32,
            plans as f32 / seeds as f32
        );
    }
}
