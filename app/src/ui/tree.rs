//! Live diagram of one robot's contingency tree.
//!
//! The nominal plan runs down the left column; each contingency the edge
//! predicted branches off to the right of the leg it guards. The robot's
//! current node is filled, and an edge that just fired flashes.

use super::theme::*;
use super::{dashed_line, text_width};
use antlift_core::{Action, Brain, NodeId, Predicate, Role};
use macroquad::prelude::*;
use std::f32::consts::PI;

const BOX_W: f32 = 86.0;
const BOX_H: f32 = 20.0;
const ROW: f32 = 27.0;
const COL: f32 = 132.0;
const FLASH: f32 = 1.0;

struct Placed {
    node: NodeId,
    row: usize,
    col: usize,
    /// Trigger on the edge that leads into this node from its left neighbour.
    via: Option<Predicate>,
}

/// Draw robot `brain`'s tree inside `rect`. Returns the height used.
pub fn draw(brain: &Brain, now: f32, rect: Rect) -> f32 {
    let Some(tree) = &brain.tree else {
        draw_text(
            "No valid tree: running on reflexes.",
            rect.x,
            rect.y + 14.0,
            16.0,
            REFLEX,
        );
        return 24.0;
    };

    let role = match tree.role {
        Role::Pusher => "pusher",
        Role::Steerer => "steerer",
    };
    let lease = tree.valid_until - now;
    let header = if lease > 1e5 {
        format!("robot #{} · {role} · plan v{}", brain.id, tree.version)
    } else {
        format!(
            "robot #{} · {role} · v{} · {} B · lease {:.0} s",
            brain.id,
            tree.version,
            tree.encode().len(),
            lease.max(0.0)
        )
    };
    draw_text(&header, rect.x, rect.y + 14.0, 16.0, TEXT);

    // Lay out: nominal chain down column 0, contingencies to the right.
    let mut placed: Vec<Placed> = vec![];
    let mut chain = vec![];
    let mut n: NodeId = 0;
    loop {
        chain.push(n);
        match tree.nodes[n as usize]
            .edges
            .iter()
            .find(|e| !e.speculative && e.to > n)
        {
            Some(e) => n = e.to,
            None => break,
        }
    }
    let on_chain = |x: NodeId| chain.contains(&x);
    for (row, &c) in chain.iter().enumerate() {
        placed.push(Placed {
            node: c,
            row,
            col: 0,
            via: None,
        });
    }
    for (row, &c) in chain.iter().enumerate() {
        // Only the first contingency per leg fits on its row; the rest are
        // still executed by the robot, just not drawn.
        let Some(first) = tree.nodes[c as usize]
            .edges
            .iter()
            .find(|e| e.speculative && !on_chain(e.to))
        else {
            continue;
        };
        let (mut at, mut via, mut col) = (first.to, Some(first.when), 1);
        while col < 3 && !placed.iter().any(|p| p.node == at) {
            placed.push(Placed {
                node: at,
                row,
                col,
                via,
            });
            let Some(next) = tree.nodes[at as usize]
                .edges
                .iter()
                .find(|e| !on_chain(e.to))
            else {
                break;
            };
            via = Some(next.when);
            at = next.to;
            col += 1;
        }
    }

    // Scroll so the current node stays visible.
    let top = rect.y + 26.0;
    let rows_fit = ((rect.h - 30.0) / ROW).floor().max(1.0) as usize;
    let cur_row = placed
        .iter()
        .find(|p| p.node == brain.node)
        .map_or(0, |p| p.row);
    let first_row = cur_row.saturating_sub(rows_fit.saturating_sub(3));
    let visible = |row: usize| row >= first_row && row < first_row + rows_fit;
    let pos = |p: &Placed| {
        vec2(
            rect.x + p.col as f32 * COL,
            top + (p.row - first_row) as f32 * ROW,
        )
    };

    let fired = brain
        .last_edge
        .filter(|(_, _, _, t)| now - t < FLASH)
        .map(|(a, b, _, t)| (a, b, 1.0 - (now - t) / FLASH));

    // Connectors first, boxes on top.
    for w in chain.windows(2) {
        let (Some(a), Some(b)) = (
            placed.iter().find(|p| p.node == w[0]),
            placed.iter().find(|p| p.node == w[1]),
        ) else {
            continue;
        };
        if !visible(a.row) || !visible(b.row) {
            continue;
        }
        let (pa, pb) = (pos(a), pos(b));
        let x = pa.x + 12.0;
        let flash = fired.filter(|(f, t, _)| *f == a.node && *t == b.node);
        let c = flash.map_or(FAINT, |(_, _, k)| alpha(HIT, 0.4 + 0.6 * k));
        draw_line(x, pa.y + BOX_H, x, pb.y, 2.0, c);
    }
    for p in placed.iter().filter(|p| p.col > 0 && visible(p.row)) {
        let left = placed
            .iter()
            .find(|q| q.row == p.row && q.col == p.col - 1)
            .map(pos)
            .unwrap_or_else(|| pos(p));
        let me = pos(p);
        let (a, b) = (
            vec2(left.x + BOX_W, left.y + BOX_H / 2.0),
            vec2(me.x, me.y + BOX_H / 2.0),
        );
        let from = placed
            .iter()
            .find(|q| q.row == p.row && q.col == p.col - 1)
            .map(|q| q.node);
        let flash = fired.filter(|(f, t, _)| Some(*f) == from && *t == p.node);
        let c = flash.map_or(FAINT, |(_, _, k)| alpha(HIT, 0.4 + 0.6 * k));
        dashed_line(a, b, if flash.is_some() { 3.0 } else { 1.5 }, c);
        if let Some(when) = p.via {
            let label = trigger(when);
            let w = text_width(&label, 13.0);
            draw_text(&label, (a.x + b.x - w) / 2.0, a.y - 3.0, 13.0, DIM);
        }
    }

    for p in placed.iter().filter(|p| visible(p.row)) {
        let at = pos(p);
        let node = &tree.nodes[p.node as usize];
        let label = action_label(node.action, p.col > 0, p.row);
        let current = p.node == brain.node;
        let flash = fired.filter(|(_, t, _)| *t == p.node);
        let border = match flash {
            Some((_, _, k)) => alpha(HIT, 0.5 + 0.5 * k),
            None if p.col > 0 => alpha(STEERER, 0.7),
            None => FAINT,
        };
        if current {
            draw_rectangle(at.x, at.y, BOX_W, BOX_H, SPEC);
        } else {
            draw_rectangle(at.x, at.y, BOX_W, BOX_H, PANEL);
        }
        draw_rectangle_lines(at.x, at.y, BOX_W, BOX_H, 1.5, border);
        let tc = if current { WHITE } else { TEXT };
        draw_text(&label, at.x + 6.0, at.y + 14.0, 14.0, tc);
    }

    let rows_drawn = chain.len().min(rows_fit);
    26.0 + rows_drawn as f32 * ROW
}

fn action_label(a: Action, contingency: bool, row: usize) -> String {
    match (a, contingency) {
        (Action::Approach, false) => "grip on".into(),
        (Action::Approach, true) => "re-grip".into(),
        (Action::Push { .. }, false) => format!("push > wp{row}"),
        (Action::Push { .. }, true) => "back off".into(),
        (Action::Rotate { heading }, false) => format!("turn {:.0}°", deg(heading)),
        (Action::Rotate { .. }, true) => "square up".into(),
        (Action::Hold, false) => "hold, done".into(),
        (Action::Hold, true) => "pause".into(),
    }
}

/// Short trigger labels: they sit in the gap between two boxes.
fn trigger(p: Predicate) -> String {
    match p {
        Predicate::InContact => "grip".into(),
        Predicate::ContactLost => "lost".into(),
        Predicate::Reached { .. } => "there".into(),
        Predicate::HeadingOff { tol, .. } => format!(">{:.0}°", deg(tol)),
        Predicate::HeadingOk { tol, .. } => format!("<{:.0}°", deg(tol)),
        Predicate::Stalled { .. } => "stuck".into(),
        Predicate::After { secs } => format!("{secs:.1}s"),
    }
}

fn deg(rad: f32) -> f32 {
    rad * 180.0 / PI
}
