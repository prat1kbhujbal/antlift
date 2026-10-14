//! The arena: walls, choke points, plans, payloads, robots, and the overlays
//! that make the invisible parts visible (radio packets, learned surprise
//! map, risks, ghost futures).

use super::theme::*;
use super::{draw_box, draw_box_lines, Overlays, Pane, View, FLASH_SECS};
use antlift_app::{EdgeView, PacketKind};
use antlift_core::{ControlMode, Layout, Role, ROBOT_RADIUS, V2};
use antlift_edge::Contingency;
use antlift_sim::{Event, Mode};
use macroquad::prelude::*;

/// Where the edge server's access point is drawn: ceiling-mounted, top centre.
pub fn edge_pos(layout: &Layout) -> V2 {
    let a = &layout.arena;
    V2::new((a.min.x + a.max.x) / 2.0, a.max.y - 1.4)
}
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

    if layout.payloads.len() > 1 {
        draw_blocks(v, edge);
    }
    draw_zones(v, layout, edge);

    for (k, spec) in layout.payloads.iter().enumerate() {
        let g = v.px(spec.goal);
        let done = s.swarm.delivered(k);
        draw_circle_lines(g.x, g.y, v.len(1.0), 2.0, HIT);
        if done {
            draw_circle(g.x, g.y, v.len(1.0), alpha(HIT, 0.15));
        }
        let label = if layout.payloads.len() > 1 {
            format!("{} goal", spec.name)
        } else {
            "GOAL".to_string()
        };
        draw_text(&label, g.x - 16.0, g.y + v.len(1.0) + 14.0, 15.0, HIT);
    }

    for plan in &edge.plans {
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
        if let (Some(_), Some(w)) = (plan.waiting_for, plan.waypoints.last()) {
            let p = v.px(w.pos);
            draw_text("WAIT", p.x - 14.0, p.y - 10.0, 15.0, RISK);
        }
    }
    if ov.risks {
        draw_risks(v, edge);
    }
    if ov.ghosts {
        draw_ghosts(v, edge, layout);
    }

    for (c, angle, half) in s.swarm.world.crate_boxes() {
        draw_box(v, c, angle, half, CRATE);
    }

    for (k, spec) in layout.payloads.iter().enumerate() {
        let o = s.swarm.world.object_state(k);
        draw_box(v, o.pos, o.angle, o.half, PAYLOAD);
        let nose = v.px(o.pos + V2::from_angle(o.angle) * o.half.x);
        let mid = v.px(o.pos);
        draw_line(mid.x, mid.y, nose.x, nose.y, 2.0, BG);
        if layout.payloads.len() > 1 {
            draw_text(spec.name, mid.x - 5.0, mid.y + 6.0, 18.0, BG);
        }
        let waiting = edge
            .plans
            .iter()
            .find(|p| p.team == k)
            .and_then(|p| p.traffic_wait)
            .filter(|_| !s.swarm.delivered(k));
        if let Some(other) = waiting {
            let text = format!("WAITING for {}", layout.payloads[other].name);
            let w = super::text_width(&text, 15.0);
            draw_rectangle(
                mid.x - w / 2.0 - 4.0,
                mid.y - 34.0,
                w + 8.0,
                18.0,
                alpha(BG, 0.85),
            );
            draw_text(&text, mid.x - w / 2.0, mid.y - 20.0, 15.0, RISK);
        }
    }

    let r = v.len(ROBOT_RADIUS);
    for (i, b) in s.swarm.brains.iter().enumerate() {
        let rb = &s.swarm.world.robots[i];
        let p = v.px(s.swarm.world.robot_pos(i));
        let color = if !rb.active {
            DEAD
        } else if s.control == ControlMode::Centralized {
            REMOTE
        } else if b.mode() == Mode::Parked {
            PARKED
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

    draw_edge_box(v, edge_pos(layout), s.edge_up());
    if ov.packets {
        draw_packets(pane, v, now);
    }
    if let Some(t) = pane.reached_at {
        draw_delivered_banner(v, t, layout.payloads.len(), pane.frozen);
    }
}

/// "Delivered" banner across the top of the world view.
fn draw_delivered_banner(v: &View, t: f32, payloads: usize, frozen: bool) {
    let title = if payloads > 1 {
        format!("ALL {payloads} PAYLOADS DELIVERED")
    } else {
        "DELIVERED".to_string()
    };
    let sub = if frozen {
        format!("in {t:.1} s  ·  run finished, robots parked  ·  R to restart")
    } else {
        format!("in {t:.1} s  ·  robots parking...")
    };
    let w = super::text_width(&title, 34.0).max(super::text_width(&sub, 17.0)) + 40.0;
    let x = v.rect.x + (v.rect.w - w) / 2.0;
    let y = v.rect.y + 18.0;
    draw_rectangle(x, y, w, 70.0, alpha(BG, 0.88));
    draw_rectangle_lines(x, y, w, 70.0, 2.0, HIT);
    draw_text(&title, x + 20.0, y + 34.0, 34.0, HIT);
    draw_text(&sub, x + 20.0, y + 58.0, 17.0, TEXT);
}

/// Choke points: free ones faint, reserved ones amber with their owner.
fn draw_zones(v: &View, layout: &Layout, edge: &EdgeView) {
    for (z, a) in layout.chokepoints.iter().enumerate() {
        let owner = edge.zones.get(z).copied().flatten();
        let tl = v.px(V2::new(a.min.x, a.max.y));
        let (w, h) = (v.len(a.max.x - a.min.x), v.len(a.max.y - a.min.y));
        let c = if owner.is_some() { RISK } else { FAINT };
        draw_rectangle(tl.x, tl.y, w, h, alpha(c, 0.08));
        super::dashed_line(tl, tl + vec2(w, 0.0), 1.5, alpha(c, 0.7));
        super::dashed_line(tl + vec2(0.0, h), tl + vec2(w, h), 1.5, alpha(c, 0.7));
        let waiting: Vec<&str> = edge
            .plans
            .iter()
            .filter(|p| p.waiting_for == Some(z))
            .map(|p| layout.payloads[p.team].name)
            .collect();
        let mut label = match owner {
            Some(k) => format!("held by {}", layout.payloads[k].name),
            None => "free".to_string(),
        };
        if !waiting.is_empty() {
            label.push_str(&format!(" · {} waits", waiting.join(",")));
        }
        draw_text(&label, tl.x + 2.0, tl.y - 4.0, 14.0, alpha(c, 0.9));
    }
}

fn draw_edge_box(v: &View, at: V2, up: bool) {
    let p = v.px(at);
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
    let edge = edge_pos(&s.swarm.layout);
    let flight = (s.up.config.latency + s.up.config.jitter * 0.5).max(MIN_FLIGHT);
    for x in &s.tx {
        let age = now - x.t;
        let robot = s.swarm.world.robot_pos(x.robot as usize);
        let (from, to) = if x.uplink {
            (robot, edge)
        } else {
            (edge, robot)
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
            PacketKind::Permit => alpha(RISK, 0.9),
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
/// Floor reserved by the traffic manager, tinted by owner team.
fn draw_blocks(v: &View, edge: &EdgeView) {
    let (owner, nx, cell) = &edge.blocks;
    if *nx == 0 {
        return;
    }
    for (i, o) in owner.iter().enumerate() {
        let Some(k) = o else { continue };
        let c = TEAMS[k % TEAMS.len()];
        let x = (i % nx) as f32 * cell;
        let y = (i / nx) as f32 * cell;
        let a = v.px(V2::new(x, y + cell));
        let s = v.len(*cell);
        draw_rectangle(a.x + 1.0, a.y + 1.0, s - 2.0, s - 2.0, alpha(c, 0.10));
    }
}

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
    for plan in &edge.plans {
        draw_plan_risks(v, plan);
    }
}

fn draw_plan_risks(v: &View, plan: &antlift_edge::Plan) {
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

/// The futures the edge simulated at its last plan: faint payload outlines
/// along each one, red with a cross where that future jammed.
fn draw_ghosts(v: &View, edge: &EdgeView, layout: &Layout) {
    for plan in &edge.plans {
        draw_plan_ghosts(v, plan, layout.payloads[plan.team].half);
    }
}

fn draw_plan_ghosts(v: &View, plan: &antlift_edge::Plan, half: V2) {
    let mut labelled: Vec<V2> = vec![];
    for g in &plan.ghosts {
        let color = if g.stall.is_some() { MISS } else { GHOST };
        let n = g.poses.len();
        // An outline every 1 s, fading in toward the end of the future.
        for (k, &(p, a)) in g.poses.iter().enumerate().step_by(4) {
            let f = (k + 1) as f32 / n.max(1) as f32;
            draw_box_lines(v, p, a, half, 1.0, alpha(color, 0.08 + 0.22 * f));
        }
        if let Some(&(p, a)) = g.poses.last() {
            draw_box_lines(v, p, a, half, 1.5, alpha(color, 0.55));
            // Label jams, and futures that end somewhere distinct.
            if g.stall.is_some() || labelled.iter().all(|l: &V2| l.dist(p) > 2.0) {
                labelled.push(p);
                let s = v.px(p);
                draw_text(
                    g.what_if.label(),
                    s.x + 4.0,
                    s.y - 4.0,
                    13.0,
                    alpha(color, 0.8),
                );
            }
        }
        if let Some(st) = g.stall {
            let s = v.px(st);
            let k = 6.0;
            draw_line(s.x - k, s.y - k, s.x + k, s.y + k, 2.5, MISS);
            draw_line(s.x - k, s.y + k, s.x + k, s.y - k, 2.5, MISS);
        }
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
