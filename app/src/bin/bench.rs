//! Headless benchmark: every condition x control mode x seed, in parallel,
//! plus latency and outage sweeps.
//!
//!     cargo run --release --bin bench -- [--seeds 10] [--limit 240] [--out results]
//!
//! Writes `<out>/bench.csv` and `<out>/summary.md` (condition matrix) and
//! `<out>/sweeps.csv` (latency and outage sweeps).

use antlift_app::scenario::{
    conditions, latency_sweep, mean_ci, outage_sweep, run, Condition, RunResult,
};
use antlift_core::ControlMode;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::{fs, thread};

const MODES: [ControlMode; 2] = [ControlMode::Speculative, ControlMode::Centralized];

fn arg<T: std::str::FromStr>(name: &str, default: T) -> T {
    let args: Vec<String> = std::env::args().collect();
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn mode_name(m: ControlMode) -> &'static str {
    match m {
        ControlMode::Speculative => "speculative",
        ControlMode::Centralized => "centralized",
    }
}

/// Run every condition x mode x seed on all cores. Results come back in job order.
fn run_all(conds: &[Condition], seeds: u64, limit: f32) -> Vec<RunResult> {
    let jobs: Vec<(usize, ControlMode, u64)> = (0..conds.len())
        .flat_map(|c| {
            MODES
                .iter()
                .flat_map(move |&m| (1..=seeds).map(move |s| (c, m, s)))
        })
        .collect();
    let next = AtomicUsize::new(0);
    let results = Mutex::new(Vec::with_capacity(jobs.len()));
    let workers = thread::available_parallelism().map_or(4, |n| n.get());
    eprintln!("{} runs on {workers} threads...", jobs.len());
    thread::scope(|s| {
        for _ in 0..workers {
            s.spawn(|| loop {
                let k = next.fetch_add(1, Ordering::Relaxed);
                let Some(&(c, m, seed)) = jobs.get(k) else {
                    break;
                };
                let r = run(&conds[c], m, seed, limit);
                results.lock().unwrap().push((k, r));
            });
        }
    });
    let mut results = results.into_inner().unwrap();
    results.sort_by_key(|(k, _)| *k);
    results.into_iter().map(|(_, r)| r).collect()
}

fn fmt_ci(v: Option<(f32, f32)>) -> String {
    v.map_or("-".into(), |(m, h)| format!("{m:.1} ± {h:.1}"))
}

fn main() {
    let seeds: u64 = arg("--seeds", 10);
    let limit: f32 = arg("--limit", 240.0);
    let out_dir: String = arg("--out", "results".to_string());
    fs::create_dir_all(&out_dir).expect("create output dir");

    // Condition matrix.
    let conds = conditions();
    let results = run_all(&conds, seeds, limit);

    let mut csv = String::from(
        "condition,control,seed,reached,time_s,final_dist_m,hits,misses,plans,up_bytes,down_bytes,bytes_per_robot_s\n",
    );
    for r in &results {
        let _ = writeln!(
            csv,
            "{},{},{},{},{},{:.2},{},{},{},{},{},{:.1}",
            r.condition,
            mode_name(r.control),
            r.seed,
            r.reached.is_some(),
            r.reached.map_or(String::new(), |t| format!("{t:.2}")),
            r.final_dist,
            r.hits,
            r.misses,
            r.plans,
            r.up_bytes,
            r.down_bytes,
            r.bytes_per_robot_s
        );
    }
    fs::write(format!("{out_dir}/bench.csv"), csv).expect("write csv");

    let mut md = String::from(
        "| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |\n|---|---|---|---|---|---|---|\n",
    );
    for c in &conds {
        for m in MODES {
            let rows: Vec<&RunResult> = results
                .iter()
                .filter(|r| r.condition == c.name && r.control == m)
                .collect();
            let times: Vec<f32> = rows.iter().filter_map(|r| r.reached).collect();
            let bytes: Vec<f32> = rows.iter().map(|r| r.bytes_per_robot_s).collect();
            let avg = |f: fn(&RunResult) -> u64| {
                rows.iter().map(|r| f(r) as f32).sum::<f32>() / rows.len().max(1) as f32
            };
            let _ = writeln!(
                md,
                "| {} | {} | {}/{} | {} | {} | {:.1} | {:.1} |",
                c.name,
                mode_name(m),
                times.len(),
                rows.len(),
                fmt_ci(mean_ci(&times)),
                fmt_ci(mean_ci(&bytes)),
                avg(|r| r.hits),
                avg(|r| r.misses),
            );
        }
    }
    let _ = writeln!(
        md,
        "\n{seeds} seeds per cell (randomized start pose and radio RNG), {limit:.0} s limit. Time is over successful runs; ± is a 95% CI."
    );

    // Sweeps.
    let mut sweep_csv =
        String::from("sweep,value,control,seed,reached,time_s,final_dist_m,bytes_per_robot_s\n");
    for (sweep, label, points) in [
        (
            "latency_ms",
            "One-way latency (ms), 10% loss",
            latency_sweep(),
        ),
        ("outage_s", "Edge outage at t=20 s (s)", outage_sweep()),
    ] {
        let (values, conds): (Vec<f32>, Vec<Condition>) = points.into_iter().unzip();
        let results = run_all(&conds, seeds, limit);
        let _ = writeln!(
            md,
            "\n| {label} | Speculative success | Speculative time (s) | Centralized success | Centralized time (s) |\n|---|---|---|---|---|"
        );
        for (v, c) in values.iter().zip(&conds) {
            let _ = write!(md, "| {v:.0} ");
            for m in MODES {
                let rows: Vec<&RunResult> = results
                    .iter()
                    .filter(|r| r.condition == c.name && r.control == m)
                    .collect();
                for r in &rows {
                    let _ = writeln!(
                        sweep_csv,
                        "{sweep},{v},{},{},{},{},{:.2},{:.1}",
                        mode_name(m),
                        r.seed,
                        r.reached.is_some(),
                        r.reached.map_or(String::new(), |t| format!("{t:.2}")),
                        r.final_dist,
                        r.bytes_per_robot_s
                    );
                }
                let times: Vec<f32> = rows.iter().filter_map(|r| r.reached).collect();
                let _ = write!(
                    md,
                    "| {}/{} | {} ",
                    times.len(),
                    rows.len(),
                    fmt_ci(mean_ci(&times))
                );
            }
            let _ = writeln!(md, "|");
        }
    }

    fs::write(format!("{out_dir}/summary.md"), &md).expect("write summary");
    fs::write(format!("{out_dir}/sweeps.csv"), sweep_csv).expect("write sweeps");
    println!("{md}");
    eprintln!("wrote {out_dir}/bench.csv, summary.md, sweeps.csv");
}
