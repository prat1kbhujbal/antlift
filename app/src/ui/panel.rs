//! Side panels: live stats, the selected robot's tree, the side-by-side
//! comparison table and charts, legend and controls.

use super::theme::*;
use super::{text_width, tree, Pane};
use antlift_app::EdgeView;
use antlift_core::ControlMode;
use antlift_net::LinkConfig;
use antlift_sim::Mode;
use macroquad::prelude::*;

pub struct Ui<'a> {
    pub link: LinkConfig,
    pub paused: bool,
    pub speed: usize,
    pub selected: Option<usize>,
    pub help: &'a [&'a str],
}

/// A tiny cursor for stacking lines of text.
struct Col {
    x: f32,
    y: f32,
}

impl Col {
    fn line(&mut self, text: &str, size: f32, color: Color) {
        draw_text(text, self.x, self.y, size, color);
        self.y += size * 1.1;
    }
    fn gap(&mut self, h: f32) {
        self.y += h;
    }
}

fn mode_name(c: ControlMode) -> &'static str {
    match c {
        ControlMode::Speculative => "speculative",
        ControlMode::Centralized => "centralized",
    }
}

fn mode_color(c: ControlMode) -> Color {
    match c {
        ControlMode::Speculative => SPEC,
        ControlMode::Centralized => CENT,
    }
}

/// Panel for the single-world view.
pub fn single(rect: Rect, pane: &Pane, edge: &EdgeView, ui: &Ui) {
    draw_rectangle(rect.x, rect.y, rect.w, rect.h, PANEL);
    let s = &pane.session;
    let mut c = Col {
        x: rect.x + 16.0,
        y: rect.y + 32.0,
    };
    c.line("AntLift", 30.0, TEXT);
    let blurb = match s.control {
        ControlMode::Speculative => "speculative: robots run their own trees",
        ControlMode::Centralized => "centralized: edge streams every command",
    };
    c.line(blurb, 16.0, mode_color(s.control));
    c.gap(6.0);

    let edge_state = if s.edge_up() {
        ("online", HIT)
    } else {
        ("OFFLINE", MISS)
    };
    let goal = pane
        .reached_at
        .map_or(String::new(), |t| format!("   goal at {t:.1} s"));
    c.line(&format!("t = {:.1} s{goal}", s.t()), 18.0, TEXT);
    c.line(&format!("edge {}", edge_state.0), 18.0, edge_state.1);
    c.line(
        &format!(
            "link {:.0} ms, {:.0}% loss",
            ui.link.latency * 1000.0,
            ui.link.loss * 100.0
        ),
        18.0,
        TEXT,
    );
    c.line(
        &format!(
            "radio {:.0} B/robot/s   plans {} ({:.0} ms)",
            s.bytes_per_robot_s(),
            edge.stats.plans,
            edge.stats.last_plan_us as f32 / 1000.0
        ),
        18.0,
        TEXT,
    );
    c.line(
        &format!("predicted hits {}   surprises {}", s.hits(), s.misses()),
        18.0,
        TEXT,
    );
    if let Some(plan) = &edge.plan {
        let jammed = plan.ghosts.iter().filter(|g| g.stall.is_some()).count();
        if !plan.ghosts.is_empty() {
            c.line(
                &format!(
                    "ghost futures: {} simulated, {jammed} jammed",
                    plan.ghosts.len()
                ),
                18.0,
                if jammed > 0 { MISS } else { GHOST },
            );
        }
    }
    if s.control == ControlMode::Speculative {
        let (mut on_tree, mut reflex, mut dead) = (0, 0, 0);
        for (i, b) in s.swarm.brains.iter().enumerate() {
            if !s.swarm.world.robots[i].active {
                dead += 1;
            } else if b.mode() == Mode::Tree {
                on_tree += 1;
            } else {
                reflex += 1;
            }
        }
        c.line(
            &format!("robots: {on_tree} on tree, {reflex} reflex, {dead} dead"),
            18.0,
            TEXT,
        );
    }

    c.gap(10.0);
    c.line("CONTINGENCY TREE", 15.0, FAINT);
    let tree_rect = Rect::new(c.x, c.y - 10.0, rect.w - 32.0, 300.0);
    match (s.control, ui.selected) {
        (ControlMode::Centralized, _) => {
            c.line("Centralized: the edge runs every robot's", 16.0, DIM);
            c.line("brain remotely; robots just obey commands.", 16.0, DIM);
            c.gap(240.0);
        }
        (_, None) => {
            c.line("Click a robot to see its live tree:", 16.0, DIM);
            c.line("the plan it carries and the problems", 16.0, DIM);
            c.line("the edge predicted for it.", 16.0, DIM);
            c.gap(220.0);
        }
        (_, Some(i)) => {
            tree::draw(&s.swarm.brains[i], s.t(), tree_rect);
            c.gap(300.0);
        }
    }

    legend(&mut c);
    help(&mut c, ui);
}

fn legend(c: &mut Col) {
    c.gap(4.0);
    c.line("LEGEND", 15.0, FAINT);
    let items: [(Color, &str); 10] = [
        (PUSHER, "pusher"),
        (STEERER, "steerer"),
        (REFLEX, "reflex, no tree"),
        (REMOTE, "remote-controlled"),
        (SPEC, "tree packet"),
        (CENT, "command packet"),
        (DIM, "status packet"),
        (MISS, "surprise / lost (x)"),
        (GHOST, "ghost future"),
        (MISS, "jammed future (x)"),
    ];
    for pair in items.chunks(2) {
        let mut x = c.x;
        for (color, label) in pair {
            draw_circle(x + 6.0, c.y - 5.0, 5.0, *color);
            draw_text(label, x + 16.0, c.y, 15.0, DIM);
            x += 175.0;
        }
        c.y += 18.0;
    }
    c.line(
        "Packets are drawn slowed down so you can see them.",
        14.0,
        FAINT,
    );
}

fn help(c: &mut Col, ui: &Ui) {
    c.gap(6.0);
    c.line("CONTROLS", 15.0, FAINT);
    for h in ui.help {
        c.line(h, 15.0, DIM);
    }
    if ui.paused {
        c.line("PAUSED", 22.0, STEERER);
    } else if ui.speed > 1 {
        c.line(&format!("FAST x{}", ui.speed), 22.0, STEERER);
    }
}

/// Header strip above a world in compare mode.
pub fn pane_header(rect: Rect, pane: &Pane) {
    let s = &pane.session;
    let color = mode_color(s.control);
    draw_rectangle(rect.x, rect.y, 6.0, rect.h, color);
    let goal = pane
        .reached_at
        .map_or("on the way".to_string(), |t| format!("goal at {t:.1} s"));
    let text = format!(
        "{}   ·   {goal}   ·   {:.0} B/robot/s   ·   {} hits, {} surprises",
        mode_name(s.control).to_uppercase(),
        s.bytes_per_robot_s(),
        s.hits(),
        s.misses()
    );
    draw_text(&text, rect.x + 14.0, rect.y + 18.0, 17.0, TEXT);
}

/// Panel for the side-by-side view: comparison table and live charts.
pub fn compare(rect: Rect, panes: &[Pane], edges: &[EdgeView], ui: &Ui) {
    draw_rectangle(rect.x, rect.y, rect.w, rect.h, PANEL);
    let mut c = Col {
        x: rect.x + 16.0,
        y: rect.y + 32.0,
    };
    c.line("AntLift · side by side", 26.0, TEXT);
    c.line(
        "Same world, same clicks. Only where decisions run differs.",
        15.0,
        DIM,
    );
    c.line(
        &format!(
            "t = {:.1} s   link {:.0} ms, {:.0}% loss   edge {}",
            panes[0].session.t(),
            ui.link.latency * 1000.0,
            ui.link.loss * 100.0,
            if panes[0].session.edge_up() {
                "online"
            } else {
                "OFFLINE"
            }
        ),
        16.0,
        TEXT,
    );
    c.gap(8.0);

    // Table.
    let col_x = [c.x, c.x + 190.0, c.x + 320.0];
    for (k, p) in panes.iter().enumerate() {
        let name = mode_name(p.session.control);
        draw_text(name, col_x[k + 1], c.y, 16.0, mode_color(p.session.control));
    }
    c.y += 20.0;
    let rows: Vec<(&str, Cell)> = vec![
        (
            "time to goal",
            Box::new(|p, _| p.reached_at.map_or("-".into(), |t| format!("{t:.1} s"))),
        ),
        (
            "distance left",
            Box::new(|p, _| {
                let o = p.session.swarm.world.object_state();
                format!("{:.1} m", o.pos.dist(p.session.swarm.layout.goal))
            }),
        ),
        (
            "radio / robot",
            Box::new(|p, _| format!("{:.0} B/s", p.session.bytes_per_robot_s())),
        ),
        (
            "packets lost",
            Box::new(|p, _| {
                let l = p.session.up.stats.dropped_msgs + p.session.down.stats.dropped_msgs;
                format!("{l}")
            }),
        ),
        (
            "predicted hits",
            Box::new(|p, _| format!("{}", p.session.hits())),
        ),
        (
            "surprises",
            Box::new(|p, _| format!("{}", p.session.misses())),
        ),
        ("plans", Box::new(|_, e| format!("{}", e.stats.plans))),
    ];
    for (label, f) in &rows {
        draw_text(label, col_x[0], c.y, 16.0, DIM);
        for (k, (p, e)) in panes.iter().zip(edges).enumerate() {
            draw_text(f(p, e), col_x[k + 1], c.y, 16.0, TEXT);
        }
        c.y += 19.0;
    }
    c.gap(12.0);

    let chart_h = 150.0;
    let series = |f: fn(&antlift_app::Sample) -> f32| -> Vec<Series> {
        panes
            .iter()
            .map(|p| Series {
                name: mode_name(p.session.control),
                color: mode_color(p.session.control),
                points: p.session.history.iter().map(|h| (h.t, f(h))).collect(),
            })
            .collect()
    };
    chart(
        Rect::new(c.x, c.y, rect.w - 40.0, chart_h),
        "Radio traffic per robot (KB, cumulative)",
        &series(|h| h.kb_per_robot),
        "KB",
    );
    c.y += chart_h + 34.0;
    chart(
        Rect::new(c.x, c.y, rect.w - 40.0, chart_h),
        "Payload distance to goal (m)",
        &series(|h| h.dist),
        "m",
    );
    c.y += chart_h + 26.0;
    help(&mut c, ui);
}

/// Renders one table cell for a pane.
type Cell = Box<dyn Fn(&Pane, &EdgeView) -> String>;

struct Series {
    name: &'static str,
    color: Color,
    points: Vec<(f32, f32)>,
}

/// Minimal line chart: recessive grid, 2px lines, direct end labels, and a
/// hover crosshair that reads out every series at the cursor's time.
fn chart(rect: Rect, title: &str, series: &[Series], unit: &str) {
    draw_text(title, rect.x, rect.y, 16.0, TEXT);
    let plot = Rect::new(rect.x + 34.0, rect.y + 12.0, rect.w - 90.0, rect.h - 28.0);
    let t_max = series
        .iter()
        .flat_map(|s| s.points.iter().map(|p| p.0))
        .fold(10.0f32, f32::max);
    let v_max = series
        .iter()
        .flat_map(|s| s.points.iter().map(|p| p.1))
        .fold(1.0f32, f32::max)
        * 1.1;
    let x = |t: f32| plot.x + t / t_max * plot.w;
    let y = |v: f32| plot.y + plot.h - v / v_max * plot.h;

    for k in 0..=3 {
        let v = v_max * k as f32 / 3.0;
        let yy = y(v);
        draw_line(plot.x, yy, plot.x + plot.w, yy, 1.0, GRID);
        draw_text(format!("{v:.0}"), rect.x, yy + 4.0, 13.0, FAINT);
    }
    for k in 0..=4 {
        let t = t_max * k as f32 / 4.0;
        draw_text(
            format!("{t:.0}s"),
            x(t) - 8.0,
            plot.y + plot.h + 14.0,
            13.0,
            FAINT,
        );
    }

    for s in series {
        for w in s.points.windows(2) {
            draw_line(x(w[0].0), y(w[0].1), x(w[1].0), y(w[1].1), 2.0, s.color);
        }
    }
    // Direct labels at the line ends, nudged apart if they collide.
    let mut ends: Vec<(f32, &Series)> = series
        .iter()
        .filter_map(|s| s.points.last().map(|p| (y(p.1), s)))
        .collect();
    ends.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut last_y = f32::NEG_INFINITY;
    for (yy, s) in ends {
        let ly = yy.max(last_y + 14.0);
        last_y = ly;
        let end = s.points.last().unwrap();
        draw_circle(x(end.0), y(end.1), 4.0, s.color);
        draw_text(s.name, plot.x + plot.w + 8.0, ly + 4.0, 13.0, TEXT);
    }

    // Hover: crosshair + readout.
    let (mx, my) = mouse_position();
    if plot.contains(vec2(mx, my)) {
        let t = (mx - plot.x) / plot.w * t_max;
        draw_line(mx, plot.y, mx, plot.y + plot.h, 1.0, FAINT);
        let mut lines = vec![format!("t = {t:.0} s")];
        for s in series {
            if let Some(p) = s
                .points
                .iter()
                .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
            {
                draw_circle(x(p.0), y(p.1), 4.0, s.color);
                lines.push(format!("{}: {:.1} {unit}", s.name, p.1));
            }
        }
        let w = lines
            .iter()
            .map(|l| text_width(l, 14.0))
            .fold(0.0, f32::max)
            + 12.0;
        let bx = if mx + w + 12.0 > plot.x + plot.w {
            mx - w - 8.0
        } else {
            mx + 8.0
        };
        let by = plot.y + 4.0;
        draw_rectangle(bx, by, w, 6.0 + 16.0 * lines.len() as f32, BG);
        draw_rectangle_lines(bx, by, w, 6.0 + 16.0 * lines.len() as f32, 1.0, GRID);
        for (k, l) in lines.iter().enumerate() {
            draw_text(l, bx + 6.0, by + 16.0 + 16.0 * k as f32, 14.0, TEXT);
        }
    }
}
