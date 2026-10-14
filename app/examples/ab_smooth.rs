//! A/B test: route smoothing on vs off, warehouse conditions, many seeds.
//!
//!     cargo run --release --example ab_smooth -- [seeds]

use antlift_app::scenario::{run_with, warehouse_conditions, VARIANTS};
use antlift_edge::EdgeConfig;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::thread;

fn main() {
    let seeds: u64 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(30);
    let conds = warehouse_conditions();
    let mut jobs = vec![];
    for smooth in [false, true] {
        for (c, _) in conds.iter().enumerate() {
            for v in VARIANTS {
                for s in 1..=seeds {
                    jobs.push((smooth, c, v, s));
                }
            }
        }
    }
    let next = AtomicUsize::new(0);
    let out = Mutex::new(vec![]);
    thread::scope(|sc| {
        for _ in 0..thread::available_parallelism().map_or(4, |n| n.get()) {
            sc.spawn(|| loop {
                let k = next.fetch_add(1, Ordering::Relaxed);
                let Some(&(smooth, c, v, s)) = jobs.get(k) else {
                    break;
                };
                let cfg = EdgeConfig {
                    smooth,
                    ..Default::default()
                };
                let r = run_with(&conds[c], v, s, conds[c].level.limit(), cfg);
                out.lock()
                    .unwrap()
                    .push((smooth, v.name, r.reached, r.misses));
            });
        }
    });
    let out = out.into_inner().unwrap();
    println!(
        "{:<24} {:>8} {:>14} {:>10} {:>10}",
        "variant", "smooth", "success", "mean time", "surprises"
    );
    for v in VARIANTS {
        for smooth in [false, true] {
            let rows: Vec<_> = out
                .iter()
                .filter(|r| r.0 == smooth && r.1 == v.name)
                .collect();
            let ok: Vec<f32> = rows.iter().filter_map(|r| r.2).collect();
            let n = rows.len() as f32;
            let p = ok.len() as f32 / n;
            // 95% Wilson interval for the success rate.
            let z = 1.96f32;
            let c = (p + z * z / (2.0 * n)) / (1.0 + z * z / n);
            let h = z * ((p * (1.0 - p) / n + z * z / (4.0 * n * n)).sqrt()) / (1.0 + z * z / n);
            let mean = ok.iter().sum::<f32>() / ok.len().max(1) as f32;
            let surprises = rows.iter().map(|r| r.3 as f32).sum::<f32>() / n;
            println!(
                "{:<24} {:>8} {:>5}/{:<3} [{:.0}-{:.0}%] {:>8.1} s {:>10.1}",
                v.name,
                smooth,
                ok.len(),
                rows.len(),
                (c - h) * 100.0,
                (c + h) * 100.0,
                mean,
                surprises
            );
        }
    }
}
