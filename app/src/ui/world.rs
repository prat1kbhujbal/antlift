//! The arena: walls, plan, payload, robots, and the overlays that make the
//! invisible parts visible (radio packets, learned surprise map, risks).

use super::theme::*;
use super::{draw_box, Overlays, Pane, View, FLASH_SECS};
use antlift_app::{EdgeView, PacketKind};
use antlift_core::{ControlMode, Role, ROBOT_RADIUS, V2};
use antlift_edge::Contingency;
use antlift_sim::{Event, Mode};
use macroquad::prelude::*;

/// Where the edge server's access point is drawn (ceiling-mounted over the wall).
pub const EDGE_POS: V2 = V2::new(20.0, 22.6);
/// Packets are animated over at least this long so they can be seen.
const MIN_FLIGHT: f32 = 0.4;

pub fn draw(pane: &Pane, v: &View, edge: &EdgeView, ov: Overlays, selected: Option<usize>) {
    let s = &pane.session;
    let layout = &s.swarm.layout;
    let now = s.t();

    if ov.heat {
        draw_heat(v, edge);
    }

    for w in &layout.walls {
        draw_box(v, w.center(), 0.0, w.half(), WALL);
    }

    let g = v.px(layout.goal);
    draw_circle_lines(g.x, g.y, v.len(1.0), 2.0, HIT);
    draw_text("GOAL", g.x - 16.0, g.y + 5.0, 16.0, HIT);

    if let Some(plan) = &edge.plan {
        for w in plan.path.windows(2) {
            let (a, b) = (v.px(w[0]), v.px(w[1]));
            draw_line(a.x, a.y, b.x, b.y, 3.0, PATH);
        }
        for w in &plan.waypoints {
            let p = v.px(w.pos);
            let tick = V2::from_angle(w.heading) * 0.6;
            let (a, b) = (v.px(w.pos - tick), v.px(w.pos + tick));
            let c = Color::new(0.4 + 0.6 * w.tightness, 0.9 - 0.5 * w.tightness, 0.9, 0.9);
            draw_line(a.x, a.y, b.x, b.y, 2.0, c);
            draw_circle(p.x, p.y, 3.0, c);
        }
        if ov.risks {
            draw_risks(v, edge);
        }
    }

    for (c, angle, half) in s.swarm.world.crate_boxes() {
        draw_box(v, c, angle, half, CRATE);
    }

    let o = s.swarm.world.object_state();
    draw_box(v, o.pos, o.angle, o.half, PAYLOAD);
    let nose = v.px(o.pos + V2::from_angle(o.angle) * o.half.x);
    let mid = v.px(o.pos);
    draw_line(mid.x, mid.y, nose.x, nose.y, 2.0, BG);

    let r = v.len(ROBOT_RADIUS);
    for (i, b) in s.swarm.brains.iter().enumerate() {
        let rb = &s.swarm.world.robots[i];
        let p = v.px(s.swarm.world.robot_pos(i));
        let color = if !rb.active {
            DEAD
        } else if s.control == ControlMode::Centralized {
            REMOTE
        } else if b.mode() == Mode::Reflex {
            REFLEX
        } else if b.role() == Some(Role::Steerer) {
            STEERER
        } else {
            PUSHER
        };
        draw_circle(p.x, p.y, r, color);
        if rb.grip.is_some() {
            draw_circle_lines(p.x, p.y, r, 1.5, WHITE);
        }
        if let Some((e, t)) = pane.flashes[i] {
            let c = alpha(if e == Event::Hit { HIT } else { MISS }, t / FLASH_SECS);
            draw_circle_lines(p.x, p.y, r + 6.0 * (1.0 - t / FLASH_SECS) + 3.0, 3.0, c);
        }
        if selected == Some(i) {
            draw_circle_lines(p.x, p.y, r + 5.0, 2.0, TEXT);
            draw_text(format!("#{i}"), p.x + r + 6.0, p.y - r - 2.0, 16.0, TEXT);
        }
    }

    draw_edge_box(v, s.edge_up());
    if ov.packets {
        draw_packets(pane, v, now);
    }
}

fn draw_edge_box(v: &View, up: bool) {
    let p = v.px(EDGE_POS);
    let c = if up { HIT } else { MISS };
    draw_rectangle(p.x - 16.0, p.y - 9.0, 32.0, 18.0, PANEL);
    draw_rectangle_lines(p.x - 16.0, p.y - 9.0, 32.0, 18.0, 2.0, c);
    draw_text("EDGE", p.x - 13.0, p.y + 5.0, 14.0, c);
    if up {
        for k in 1..=2 {
            let rr = 10.0 + 6.0 * k as f32;
            draw_circle_lines(p.x, p.y - 9.0, rr, 1.0, alpha(c, 0.25));
        }
    }
}

/// Robot radio traffic, flying between each robot and the edge. Packets are
/// drawn slowed down (at least [`MIN_FLIGHT`]) so they're visible at all;
/// a lost one dies halfway with a red cross.
fn draw_packets(pane: &Pane, v: &View, now: f32) {
    let s = &pane.session;
    let flight = (s.up.config.latency + s.up.config.jitter * 0.5).max(MIN_FLIGHT);
    for x in &s.tx {
        let age = now - x.t;
        let robot = s.swarm.world.robot_pos(x.robot as usize);
        let (from, to) = if x.uplink {
            (robot, EDGE_POS)
        } else {
            (EDGE_POS, robot)
        };
        if x.lost {
            let k = (age / flight).min(0.5);
            let sp = v.px(from + (to - from) * k);
            if age > flight * 0.5 {
                // Died here; fade the cross out.
                let a = (1.0 - (age - flight * 0.5) / 0.6).max(0.0);
                let c = alpha(MISS, a);
                let d = 4.0;
                draw_line(sp.x - d, sp.y - d, sp.x + d, sp.y + d, 2.0, c);
                draw_line(sp.x - d, sp.y + d, sp.x + d, sp.y - d, 2.0, c);
                continue;
            }
            draw_circle(sp.x, sp.y, packet_radius(x.bytes as f32), alpha(MISS, 0.7));
            continue;
        }
        if age > flight {
            continue;
        }
        let color = match x.kind {
            PacketKind::Tree => SPEC,
            PacketKind::Command => alpha(CENT, 0.9),
            PacketKind::State => alpha(REMOTE, 0.75),
            PacketKind::Surprise => MISS,
            PacketKind::Status => alpha(DIM, 0.8),
        };
        let sp = v.px(from + (to - from) * (age / flight));
        draw_circle(sp.x, sp.y, packet_radius(x.bytes as f32), color);
    }
}

fn packet_radius(bytes: f32) -> f32 {
    1.5 + bytes.sqrt() / 4.0
}

/// The edge's learned map of where its predictions failed.
fn draw_heat(v: &View, edge: &EdgeView) {
    let (heat, nx, cell) = &edge.heat;
    if *nx == 0 {
        return;
    }
    for (i, h) in heat.iter().enumerate() {
        if *h < 0.02 {
            continue;
        }
        let x = (i % nx) as f32 * cell;
        let y = (i / nx) as f32 * cell;
        let a = v.px(V2::new(x, y + cell));
        draw_rectangle(
            a.x,
            a.y,
            v.len(*cell),
            v.len(*cell),
            alpha(MISS, 0.12 + 0.35 * h),
        );
    }
}

/// The edge's top predicted risks along the route, as small badges.
fn draw_risks(v: &View, edge: &EdgeView) {
    let Some(plan) = &edge.plan else { return };
    let mut best: Vec<Option<(Contingency, f32)>> = vec![None; plan.waypoints.len()];
    for c in &plan.candidates {
        let score = c.p;
        if c.wp < best.len() && best[c.wp].is_none_or(|(_, p)| score > p) {
            best[c.wp] = Some((c.kind, score));
        }
    }
    let mut last_label: Option<V2> = None;
    for (w, b) in plan.waypoints.iter().zip(best) {
        let Some((kind, p)) = b else { continue };
        // One badge per cluster of risky waypoints.
        if p < 0.3 || last_label.is_some_and(|l| l.dist(w.pos) < 3.0) {
            continue;
        }
        last_label = Some(w.pos);
        let label = match kind {
            Contingency::Jam => "jam",
            Contingency::Misalign => "turn",
            Contingency::GripLoss => "grip",
        };
        let text = format!("{label} {:.0}%", p * 100.0);
        let s = v.px(w.pos) + vec2(6.0, -8.0);
        let tw = super::text_width(&text, 14.0);
        draw_rectangle(s.x - 2.0, s.y - 11.0, tw + 4.0, 14.0, alpha(BG, 0.8));
        draw_text(&text, s.x, s.y, 14.0, alpha(RISK, 0.4 + 0.6 * p));
    }
}

/// Index of the robot under screen point `s`, if any.
pub fn robot_at(pane: &Pane, v: &View, s: Vec2) -> Option<usize> {
    let r = v.len(ROBOT_RADIUS) + 4.0;
    (0..pane.session.swarm.brains.len())
        .map(|i| (i, v.px(pane.session.swarm.world.robot_pos(i)).distance(s)))
        .filter(|(_, d)| *d <= r)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}
