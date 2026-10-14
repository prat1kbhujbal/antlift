//! Interactive AntLift demo. Each edge server runs on its own thread and only
//! talks to its swarm through the simulated radio.
//!
//!     cargo run --release [-- --warehouse] [-- --compare] [-- --select <robot>]

mod ui;

use antlift_app::scenario::Level;
use antlift_app::{Session, SessionConfig};
use antlift_core::ControlMode;
use antlift_net::LinkConfig;
use macroquad::prelude::*;
use ui::{panel, theme, world, Overlays, Pane, View};

const DT: f32 = 1.0 / 60.0;
const PANEL_SINGLE: f32 = 400.0;
const PANEL_COMPARE: f32 = 480.0;
const HEADER: f32 = 26.0;
const LOSS_STEPS: [f32; 3] = [0.0, 0.1, 0.3];

const HELP: &[&str] = &[
    "L-click robot: inspect · elsewhere: drop/remove crate",
    "R-click kick · G shake loose · X kill robot",
    "E edge on/off · [ ] latency · L packet loss",
    "C switch mode live · V side-by-side compare",
    "T ghost futures · P packets · H heat map · K risks",
    "W doorway/warehouse level · F fast-forward",
    "Space pause · R reset · Esc deselect",
];

fn window_conf() -> Conf {
    Conf {
        window_title: "AntLift: speculative swarm control".into(),
        window_width: 1500,
        window_height: 900,
        high_dpi: true,
        ..Default::default()
    }
}

struct App {
    level: Level,
    panes: Vec<Pane>,
    link: LinkConfig,
    loss_idx: usize,
    paused: bool,
    speed: usize,
    selected: Option<usize>,
    overlays: Overlays,
}

fn session(control: ControlMode, link: LinkConfig, level: Level) -> Session {
    Session::new(SessionConfig {
        layout: level.base(),
        threaded: true,
        link,
        ..SessionConfig::new(control)
    })
}

impl App {
    fn new(level: Level) -> Self {
        let link = LinkConfig::default();
        Self {
            level,
            panes: vec![Pane::new(session(ControlMode::Speculative, link, level))],
            link,
            loss_idx: 0,
            paused: false,
            speed: 1,
            selected: None,
            overlays: Overlays {
                packets: true,
                ghosts: true,
                heat: true,
                risks: true,
            },
        }
    }

    fn comparing(&self) -> bool {
        self.panes.len() == 2
    }

    /// Fresh run(s) with the current link settings, in the current view.
    fn reset(&mut self) {
        self.panes = if self.comparing() {
            vec![
                Pane::new(session(ControlMode::Speculative, self.link, self.level)),
                Pane::new(session(ControlMode::Centralized, self.link, self.level)),
            ]
        } else {
            let control = self.panes[0].session.control;
            vec![Pane::new(session(control, self.link, self.level))]
        };
    }

    /// Screen rectangles for each pane's world (and header, when comparing).
    fn layout(&self) -> (Vec<(Rect, Rect)>, Rect) {
        let (w, h) = (screen_width(), screen_height());
        if self.comparing() {
            let ww = w - PANEL_COMPARE;
            let half = h / 2.0;
            let panes = (0..2)
                .map(|k| {
                    let y = k as f32 * half;
                    (
                        Rect::new(0.0, y, ww, HEADER),
                        Rect::new(0.0, y + HEADER, ww, half - HEADER),
                    )
                })
                .collect();
            (panes, Rect::new(ww, 0.0, PANEL_COMPARE, h))
        } else {
            let ww = w - PANEL_SINGLE;
            (
                vec![(Rect::new(0.0, 0.0, 0.0, 0.0), Rect::new(0.0, 0.0, ww, h))],
                Rect::new(ww, 0.0, PANEL_SINGLE, h),
            )
        }
    }
}

fn handle_input(app: &mut App, views: &[View]) {
    let (mx, my) = mouse_position();
    let mouse = vec2(mx, my);
    // The world point under the cursor, taken from whichever pane it is over;
    // world actions are then mirrored into every pane.
    let hovered = views.iter().position(|v| v.rect.contains(mouse));
    let m = hovered.map(|k| views[k].world(mouse));

    if let (Some(k), Some(m)) = (hovered, m) {
        if is_mouse_button_pressed(MouseButton::Left) {
            if let Some(i) = world::robot_at(&app.panes[k], &views[k], mouse) {
                app.selected = Some(i);
            } else {
                for p in &mut app.panes {
                    if !p.session.swarm.world.remove_crate_at(m) {
                        p.session.swarm.world.add_crate(m);
                    }
                }
            }
        }
        if is_mouse_button_pressed(MouseButton::Right) {
            for p in &mut app.panes {
                let w = &p.session.swarm.world;
                let o = w.object_state(w.nearest_object(m));
                p.session.swarm.world.kick(m, (o.pos - m).norm() * 80.0);
            }
        }
        if is_key_pressed(KeyCode::G) {
            for p in &mut app.panes {
                let w = &mut p.session.swarm.world;
                for i in 0..w.robots.len() {
                    if w.robot_pos(i).dist(m) < 2.5 {
                        w.release(i);
                    }
                }
            }
        }
        if is_key_pressed(KeyCode::X) {
            // Kill the same robot id in every pane, picked in the hovered one.
            let w = &app.panes[k].session.swarm.world;
            let nearest = (0..w.robots.len())
                .filter(|&i| w.robots[i].active)
                .min_by(|&a, &b| w.robot_pos(a).dist(m).total_cmp(&w.robot_pos(b).dist(m)));
            if let Some(i) = nearest {
                for p in &mut app.panes {
                    p.session.swarm.world.disable_robot(i);
                }
            }
        }
    }

    if is_key_pressed(KeyCode::E) {
        for p in &mut app.panes {
            let up = !p.session.edge_up();
            p.session.set_edge_up(up);
        }
    }
    let mut link_changed = false;
    if is_key_pressed(KeyCode::LeftBracket) {
        app.link.latency = (app.link.latency - 0.05).max(0.0);
        link_changed = true;
    }
    if is_key_pressed(KeyCode::RightBracket) {
        app.link.latency += 0.05;
        link_changed = true;
    }
    if is_key_pressed(KeyCode::L) {
        app.loss_idx = (app.loss_idx + 1) % LOSS_STEPS.len();
        app.link.loss = LOSS_STEPS[app.loss_idx];
        link_changed = true;
    }
    if link_changed {
        for p in &mut app.panes {
            p.session.set_link(app.link);
        }
    }

    if is_key_pressed(KeyCode::C) && !app.comparing() {
        let s = &mut app.panes[0].session;
        let next = match s.control {
            ControlMode::Speculative => ControlMode::Centralized,
            ControlMode::Centralized => ControlMode::Speculative,
        };
        s.set_control(next);
    }
    if is_key_pressed(KeyCode::V) {
        // Comparing always starts both runs fresh so they're on equal terms.
        app.panes = if app.comparing() {
            vec![Pane::new(session(
                ControlMode::Speculative,
                app.link,
                app.level,
            ))]
        } else {
            vec![
                Pane::new(session(ControlMode::Speculative, app.link, app.level)),
                Pane::new(session(ControlMode::Centralized, app.link, app.level)),
            ]
        };
    }
    if is_key_pressed(KeyCode::R) {
        app.reset();
    }
    if is_key_pressed(KeyCode::W) {
        app.level = match app.level {
            Level::Doorway => Level::Warehouse,
            Level::Warehouse => Level::Doorway,
        };
        app.selected = None;
        app.reset();
    }
    if is_key_pressed(KeyCode::T) {
        app.overlays.ghosts = !app.overlays.ghosts;
    }
    if is_key_pressed(KeyCode::P) {
        app.overlays.packets = !app.overlays.packets;
    }
    if is_key_pressed(KeyCode::H) {
        app.overlays.heat = !app.overlays.heat;
    }
    if is_key_pressed(KeyCode::K) {
        app.overlays.risks = !app.overlays.risks;
    }
    if is_key_pressed(KeyCode::Escape) {
        app.selected = None;
    }
    if is_key_pressed(KeyCode::F) {
        app.speed = if app.speed == 1 { 4 } else { 1 };
    }
    if is_key_pressed(KeyCode::Space) {
        app.paused = !app.paused;
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // Optional flags, handy for demos and screenshots.
    let args: Vec<String> = std::env::args().collect();
    let level = if args.iter().any(|a| a == "--warehouse") {
        Level::Warehouse
    } else {
        Level::Doorway
    };
    let mut app = App::new(level);
    if args.iter().any(|a| a == "--compare") {
        app.panes.push(Pane::new(session(
            ControlMode::Centralized,
            app.link,
            app.level,
        )));
    }
    app.selected = args
        .iter()
        .position(|a| a == "--select")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok());
    let mut acc = 0.0;
    loop {
        let (rects, panel_rect) = app.layout();
        let views: Vec<View> = rects
            .iter()
            .zip(&app.panes)
            .map(|((_, r), p)| View::fit(&p.session.swarm.layout, *r))
            .collect();
        handle_input(&mut app, &views);

        if !app.paused {
            acc += get_frame_time().min(0.1) * app.speed as f32;
            while acc >= DT {
                for p in &mut app.panes {
                    p.step(DT);
                }
                acc -= DT;
            }
        }

        clear_background(theme::BG);
        let edges: Vec<_> = app.panes.iter().map(|p| p.session.edge.view()).collect();
        // Rects may be stale for one frame after toggling compare; recompute.
        let (rects, panel_rect) = if rects.len() == app.panes.len() {
            (rects, panel_rect)
        } else {
            app.layout()
        };
        for (((header, r), p), e) in rects.iter().zip(&app.panes).zip(&edges) {
            let v = View::fit(&p.session.swarm.layout, *r);
            world::draw(p, &v, e, app.overlays, app.selected);
            if app.comparing() {
                panel::pane_header(*header, p);
            }
        }
        let ui = panel::Ui {
            link: app.link,
            paused: app.paused,
            speed: app.speed,
            selected: app.selected,
            help: HELP,
        };
        if app.comparing() {
            panel::compare(panel_rect, &app.panes, &edges, &ui);
        } else {
            panel::single(panel_rect, &app.panes[0], &edges[0], &ui);
        }
        next_frame().await;
    }
}
