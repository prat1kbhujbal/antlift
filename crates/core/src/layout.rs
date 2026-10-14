use crate::geom::{Aabb, V2};
use std::f32::consts::FRAC_PI_2;
use std::ops::Range;

/// One payload, its destination and the team that starts on it.
#[derive(Clone, Debug)]
pub struct PayloadSpec {
    pub name: &'static str,
    pub start: V2,
    pub angle: f32,
    /// Half extents (x = long axis).
    pub half: V2,
    pub goal: V2,
    /// Robots in the team that starts on this payload.
    pub robots: usize,
    /// Bottom-left corner of the grid the team's robots start parked in.
    pub spawn: V2,
    /// Columns in that parking grid.
    pub spawn_cols: usize,
}

/// Static level description, known to both the sim and the edge.
#[derive(Clone, Debug)]
pub struct Layout {
    pub arena: Aabb,
    pub walls: Vec<Aabb>,
    pub payloads: Vec<PayloadSpec>,
    /// Narrow passages only one payload may use at a time. The edge grants
    /// them as reservations.
    pub chokepoints: Vec<Aabb>,
}

impl Layout {
    pub fn n_robots(&self) -> usize {
        self.payloads.iter().map(|p| p.robots).sum()
    }

    /// Robot ids of the team that starts on payload `k` (contiguous).
    pub fn team_range(&self, k: usize) -> Range<usize> {
        let start: usize = self.payloads[..k].iter().map(|p| p.robots).sum();
        start..start + self.payloads[k].robots
    }

    /// The team robot `id` starts in.
    pub fn initial_team(&self, id: usize) -> usize {
        (0..self.payloads.len())
            .find(|&k| self.team_range(k).contains(&id))
            .unwrap_or(0)
    }

    /// Where robot `id` starts parked.
    pub fn spawn_pos(&self, id: usize) -> V2 {
        let k = self.initial_team(id);
        let p = &self.payloads[k];
        let i = id - self.team_range(k).start;
        p.spawn + V2::new((i % p.spawn_cols) as f32, (i / p.spawn_cols) as f32)
    }

    /// An assembly hall: a sofa weaves an S-shaped route between large
    /// machines to a loading dock. East under one machine, north up a 4 m
    /// channel between two machines (upright), east above the second one,
    /// south down the dock corridor, then flat into the dock bay.
    pub fn hall() -> Self {
        let mut walls = border(40.0, 24.0);
        walls.extend([
            // Press line: a big machine block over the west of the hall,
            // leaving an 8 m corridor beneath it.
            Aabb::new(V2::new(8.0, 8.0), V2::new(14.0, 24.0)),
            // CNC cell: a second block from the floor up, leaving a 4 m
            // channel between the two and an aisle above it.
            Aabb::new(V2::new(18.0, 0.0), V2::new(24.0, 13.0)),
            // Parts racks in the upper aisle.
            Aabb::from_center(V2::new(28.0, 19.0), V2::new(0.6, 0.6)),
            Aabb::from_center(V2::new(33.0, 20.5), V2::new(0.6, 0.6)),
            // Loading dock bay in the south-east corner: a roof wall and a
            // stub that narrow its mouth to 5.5 m.
            Aabb::new(V2::new(31.0, 9.0), V2::new(40.0, 9.6)),
            Aabb::new(V2::new(31.0, 0.0), V2::new(31.6, 3.5)),
        ]);
        Self {
            arena: Aabb::new(V2::ZERO, V2::new(40.0, 24.0)),
            walls,
            payloads: vec![PayloadSpec {
                name: "A",
                start: V2::new(5.0, 4.0),
                angle: 0.0,
                half: V2::new(2.0, 0.6),
                goal: V2::new(36.0, 5.5),
                robots: 16,
                // Parked in the corridor beside the sofa: robots drive
                // straight to their grip spot and can't path round machines.
                spawn: V2::new(9.0, 1.0),
                spawn_cols: 4,
            }],
            chokepoints: vec![],
        }
    }

    /// A 60 x 36 m warehouse: a staging room, a racked hall and a dispatch
    /// room, joined by three doorways. Three teams carry a sofa, a pallet and
    /// a 6 m beam; the sofa and the beam both need the single dispatch door.
    pub fn warehouse() -> Self {
        let (w, h) = (60.0, 36.0);
        let mut walls = border(w, h);
        let t = 0.3;
        walls.extend([
            // Staging | hall wall, 4 m doors at y 8..12 (D1) and 24..28 (D2):
            // wide enough for the pallet with robots on every side.
            Aabb::new(V2::new(20.0 - t, 0.0), V2::new(20.0 + t, 8.0)),
            Aabb::new(V2::new(20.0 - t, 12.0), V2::new(20.0 + t, 24.0)),
            Aabb::new(V2::new(20.0 - t, 28.0), V2::new(20.0 + t, h)),
            // Hall | dispatch wall, single door at y 16..19.5 (D3).
            Aabb::new(V2::new(40.0 - t, 0.0), V2::new(40.0 + t, 16.0)),
            Aabb::new(V2::new(40.0 - t, 19.5), V2::new(40.0 + t, h)),
            // Rack rows in the hall, with aisles between them.
            Aabb::new(V2::new(24.0, 3.0), V2::new(36.0, 4.5)),
            Aabb::new(V2::new(24.0, 13.0), V2::new(29.0, 14.2)),
            Aabb::new(V2::new(31.0, 13.0), V2::new(36.0, 14.2)),
            Aabb::new(V2::new(24.0, 21.5), V2::new(36.0, 22.7)),
            Aabb::new(V2::new(24.0, 31.0), V2::new(36.0, 32.5)),
            // Columns in the dispatch room.
            Aabb::from_center(V2::new(48.0, 12.0), V2::new(0.6, 0.6)),
            Aabb::from_center(V2::new(48.0, 24.0), V2::new(0.6, 0.6)),
        ]);
        // Just the doorway itself: routes that merely pass nearby don't need it.
        let choke =
            |x: f32, y0: f32, y1: f32| Aabb::new(V2::new(x - 1.5, y0), V2::new(x + 1.5, y1));
        Self {
            arena: Aabb::new(V2::ZERO, V2::new(w, h)),
            walls,
            payloads: vec![
                PayloadSpec {
                    name: "A",
                    start: V2::new(8.0, 6.0),
                    angle: FRAC_PI_2,
                    half: V2::new(2.0, 0.6),
                    goal: V2::new(53.0, 8.0),
                    robots: 16,
                    spawn: V2::new(2.0, 9.0),
                    spawn_cols: 4,
                },
                PayloadSpec {
                    name: "B",
                    start: V2::new(8.0, 18.0),
                    angle: 0.0,
                    half: V2::new(1.0, 1.0),
                    goal: V2::new(30.0, 17.5),
                    robots: 10,
                    spawn: V2::new(13.0, 14.0),
                    spawn_cols: 4,
                },
                PayloadSpec {
                    name: "C",
                    start: V2::new(10.0, 30.0),
                    angle: 0.0,
                    half: V2::new(3.0, 0.4),
                    goal: V2::new(53.0, 28.0),
                    robots: 18,
                    spawn: V2::new(3.0, 33.0),
                    spawn_cols: 6,
                },
            ],
            chokepoints: vec![
                choke(20.0, 8.0, 12.0),
                choke(20.0, 24.0, 28.0),
                choke(40.0, 16.0, 19.5),
            ],
        }
    }
}

fn border(w: f32, h: f32) -> Vec<Aabb> {
    let t = 0.3;
    vec![
        Aabb::new(V2::new(-t, -t), V2::new(w + t, t)),
        Aabb::new(V2::new(-t, h - t), V2::new(w + t, h + t)),
        Aabb::new(V2::new(-t, -t), V2::new(t, h + t)),
        Aabb::new(V2::new(w - t, -t), V2::new(w + t, h + t)),
    ]
}

/// Robots are discs of this radius.
pub const ROBOT_RADIUS: f32 = 0.25;
/// Grip points sit this far outside the payload boundary.
pub const SLOT_OFFSET: f32 = ROBOT_RADIUS + 0.02;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn teams_are_contiguous_and_cover_every_robot() {
        let l = Layout::warehouse();
        assert_eq!(l.n_robots(), 44);
        assert_eq!(l.team_range(1), 16..26);
        assert_eq!(l.initial_team(25), 1);
        assert_eq!(l.initial_team(26), 2);
    }

    #[test]
    fn spawns_are_clear_of_walls() {
        for l in [Layout::hall(), Layout::warehouse()] {
            for id in 0..l.n_robots() {
                let p = l.spawn_pos(id);
                assert!(
                    l.walls.iter().all(|w| w.dist(p) > 0.4),
                    "robot {id} at {p:?}"
                );
            }
        }
    }
}
