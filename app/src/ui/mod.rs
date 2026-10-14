//! Drawing for the interactive demo: the world with its radio traffic and
//! overlays, the selected robot's contingency tree, and the side panels.

pub mod panel;
pub mod tree;
pub mod world;

use antlift_app::Session;
use antlift_core::{Layout, V2};
use antlift_sim::Event;
use macroquad::prelude::*;

pub const FLASH_SECS: f32 = 0.6;

/// One running simulation plus the bits of UI state that follow it.
pub struct Pane {
    pub session: Session,
    /// Per robot: (event, seconds left to show it).
    pub flashes: Vec<Option<(Event, f32)>>,
    /// When every payload had been delivered (the last first arrival).
    pub reached_at: Option<f32>,
    /// First arrival of each payload at its goal.
    deliveries: Vec<Option<f32>>,
    /// The run is over: the simulation no longer advances.
    pub frozen: bool,
}

/// After the last delivery, keep simulating this long so robots can settle
/// and park, then freeze the run.
const SETTLE: f32 = 3.0;

impl Pane {
    pub fn new(session: Session) -> Self {
        let n = session.swarm.brains.len();
        let payloads = session.swarm.layout.payloads.len();
        Self {
            session,
            flashes: vec![None; n],
            reached_at: None,
            deliveries: vec![None; payloads],
            frozen: false,
        }
    }

    pub fn step(&mut self, dt: f32) {
        if self.frozen {
            return;
        }
        self.session.step(dt);
        for (i, e) in self.session.swarm.last_events.iter().enumerate() {
            if let Some(e) = e {
                self.flashes[i] = Some((*e, FLASH_SECS));
            } else if let Some((_, t)) = &mut self.flashes[i] {
                *t -= dt;
                if *t <= 0.0 {
                    self.flashes[i] = None;
                }
            }
        }
        let t = self.session.t();
        for (k, d) in self.deliveries.iter_mut().enumerate() {
            if d.is_none() && self.session.swarm.delivered(k) {
                *d = Some(t);
            }
        }
        if self.reached_at.is_none() && self.deliveries.iter().all(|d| d.is_some()) {
            self.reached_at = Some(t);
        }
        if self.reached_at.is_some_and(|r| t - r > SETTLE) {
            self.frozen = true;
            self.session.tx.clear(); // nothing is in the air any more
        }
    }
}

/// Which overlays to draw on the world.
#[derive(Clone, Copy)]
pub struct Overlays {
    pub packets: bool,
    pub ghosts: bool,
    pub heat: bool,
    pub risks: bool,
}

/// World metres <-> screen pixels inside a screen rectangle, y up in the world.
pub struct View {
    pub rect: Rect,
    scale: f32,
    origin: Vec2,
    world_h: f32,
}

impl View {
    pub fn fit(layout: &Layout, rect: Rect) -> Self {
        let size = layout.arena.max - layout.arena.min;
        let pad = 12.0;
        let scale = ((rect.w - 2.0 * pad) / size.x).min((rect.h - 2.0 * pad) / size.y);
        // Centre the arena inside the rectangle.
        let used = vec2(size.x * scale, size.y * scale);
        let origin = vec2(rect.x, rect.y) + (vec2(rect.w, rect.h) - used) * 0.5;
        Self {
            rect,
            scale,
            origin,
            world_h: size.y,
        }
    }

    pub fn px(&self, p: V2) -> Vec2 {
        self.origin + vec2(p.x * self.scale, (self.world_h - p.y) * self.scale)
    }

    pub fn world(&self, s: Vec2) -> V2 {
        let l = (s - self.origin) / self.scale;
        V2::new(l.x, self.world_h - l.y)
    }

    pub fn len(&self, metres: f32) -> f32 {
        metres * self.scale
    }
}

/// Colours. Mode identities use the validated dark-surface categorical pair.
pub mod theme {
    use macroquad::prelude::Color;

    const fn rgb(r: u8, g: u8, b: u8) -> Color {
        Color::new(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0, 1.0)
    }

    pub const BG: Color = rgb(0x12, 0x14, 0x19);
    pub const PANEL: Color = rgb(0x1a, 0x1a, 0x19);
    pub const GRID: Color = rgb(0x33, 0x34, 0x38);
    pub const TEXT: Color = rgb(0xf0, 0xef, 0xec);
    pub const DIM: Color = rgb(0xc3, 0xc2, 0xb7);
    pub const FAINT: Color = rgb(0x80, 0x80, 0x7a);

    /// Mode identities (charts, headers).
    pub const SPEC: Color = rgb(0x39, 0x87, 0xe5);
    pub const CENT: Color = rgb(0xd9, 0x59, 0x26);

    pub const WALL: Color = rgb(0x5a, 0x5e, 0x69);
    pub const PAYLOAD: Color = rgb(0xd9, 0xcc, 0xa6);
    pub const CRATE: Color = rgb(0x8c, 0x61, 0x38);
    pub const PUSHER: Color = rgb(0x8c, 0xbf, 0xff);
    pub const STEERER: Color = rgb(0xff, 0xc2, 0x5c);
    pub const REFLEX: Color = rgb(0xb3, 0x73, 0xf2);
    pub const REMOTE: Color = rgb(0x5c, 0xd9, 0xbf);
    pub const DEAD: Color = rgb(0x40, 0x40, 0x47);
    pub const HIT: Color = rgb(0x40, 0xff, 0x73);
    pub const MISS: Color = rgb(0xff, 0x40, 0x40);
    pub const PATH: Color = Color::new(0.40, 0.90, 0.90, 0.35);
    pub const RISK: Color = rgb(0xff, 0xb3, 0x40);
    pub const PARKED: Color = rgb(0x8f, 0xd6, 0x9a);
    pub const GHOST: Color = rgb(0xa8, 0xd8, 0xff);
    /// Per-team tints for reserved floor.
    pub const TEAMS: [Color; 3] = [
        rgb(0x5c, 0xa8, 0xff),
        rgb(0xff, 0x9e, 0x5c),
        rgb(0xc0, 0x8c, 0xff),
    ];

    pub fn alpha(c: Color, a: f32) -> Color {
        Color::new(c.r, c.g, c.b, c.a * a)
    }
}

pub fn draw_box(v: &View, center: V2, angle: f32, half: V2, color: Color) {
    let c = [
        V2::new(-half.x, -half.y),
        V2::new(half.x, -half.y),
        V2::new(half.x, half.y),
        V2::new(-half.x, half.y),
    ]
    .map(|k| v.px(center + k.rotate(angle)));
    draw_triangle(c[0], c[1], c[2], color);
    draw_triangle(c[0], c[2], c[3], color);
}

pub fn draw_box_lines(v: &View, center: V2, angle: f32, half: V2, thickness: f32, color: Color) {
    let c = [
        V2::new(-half.x, -half.y),
        V2::new(half.x, -half.y),
        V2::new(half.x, half.y),
        V2::new(-half.x, half.y),
    ]
    .map(|k| v.px(center + k.rotate(angle)));
    for i in 0..4 {
        let (a, b) = (c[i], c[(i + 1) % 4]);
        draw_line(a.x, a.y, b.x, b.y, thickness, color);
    }
}

pub fn dashed_line(a: Vec2, b: Vec2, thickness: f32, color: Color) {
    let d = b - a;
    let len = d.length();
    if len < 1e-3 {
        return;
    }
    let dir = d / len;
    let (dash, gap) = (5.0, 4.0);
    let mut t = 0.0;
    while t < len {
        let e = (t + dash).min(len);
        let (p, q) = (a + dir * t, a + dir * e);
        draw_line(p.x, p.y, q.x, q.y, thickness, color);
        t += dash + gap;
    }
}

pub fn text_width(text: &str, size: f32) -> f32 {
    measure_text(text, None, size as u16, 1.0).width
}
