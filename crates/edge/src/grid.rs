//! Clearance map and an SE(2) planner for the payload.
//!
//! The swarm can push the payload in any direction and turn it in place, so
//! this is holonomic: states are (cell, heading) and moves are one-cell
//! translations or one-step turns. The footprint (payload plus the robots
//! gripping its sides) is checked as a chain of circles against an exact
//! clearance map. A 2D distance-to-goal field is the A* heuristic.

use antlift_core::{wrap_angle, Aabb, V2};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f32::consts::{PI, SQRT_2};

pub const CELL: f32 = 0.25;
/// Heading resolution over PI (the payload is symmetric end to end).
pub const HEADINGS: usize = 16;
const TURN_COST: f32 = 0.15;
/// Below this much slack a cell costs extra, keeping plans centred in gaps.
const COMFY_SLACK: f32 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub pos: V2,
    pub heading: f32,
}

/// Rectangle approximated by circles spaced along its long axis.
#[derive(Clone, Debug)]
pub struct Footprint {
    offsets: Vec<f32>,
    radius: f32,
    /// Half the short side: the most a centre-only plan has to respect.
    pub inner: f32,
}

impl Footprint {
    pub fn new(half: V2) -> Self {
        // End circles sit `e` in from each end so they reach the corners;
        // neighbours are at most 2e apart so the long sides are covered too.
        let e = half.y * 0.5;
        let span = (half.x - e).max(0.0);
        let n = ((span / e).ceil() as usize + 1).max(2);
        let step = 2.0 * span / (n - 1) as f32;
        let offsets = (0..n).map(|i| -span + i as f32 * step).collect();
        Self {
            offsets,
            radius: (e * e + half.y * half.y).sqrt(),
            inner: half.y,
        }
    }
}

pub struct Grid {
    pub nx: usize,
    pub ny: usize,
    origin: V2,
    /// Exact metres from each cell centre to the nearest obstacle.
    pub clearance: Vec<f32>,
}

impl Grid {
    pub fn new(arena: Aabb, obstacles: &[Aabb]) -> Self {
        let size = arena.max - arena.min;
        let nx = (size.x / CELL).ceil() as usize;
        let ny = (size.y / CELL).ceil() as usize;
        let origin = arena.min;
        let mut clearance = Vec::with_capacity(nx * ny);
        for y in 0..ny {
            for x in 0..nx {
                let c = origin + V2::new((x as f32 + 0.5) * CELL, (y as f32 + 0.5) * CELL);
                clearance.push(
                    obstacles
                        .iter()
                        .map(|o| o.dist(c))
                        .fold(f32::INFINITY, f32::min),
                );
            }
        }
        Self {
            nx,
            ny,
            origin,
            clearance,
        }
    }

    pub fn center(&self, x: usize, y: usize) -> V2 {
        self.origin + V2::new((x as f32 + 0.5) * CELL, (y as f32 + 0.5) * CELL)
    }

    pub fn cell_of(&self, p: V2) -> (usize, usize) {
        let l = p - self.origin;
        (
            ((l.x / CELL) as isize).clamp(0, self.nx as isize - 1) as usize,
            ((l.y / CELL) as isize).clamp(0, self.ny as isize - 1) as usize,
        )
    }

    pub fn clearance_at(&self, p: V2) -> f32 {
        let l = p - self.origin;
        if l.x < 0.0 || l.y < 0.0 {
            return 0.0;
        }
        let (x, y) = ((l.x / CELL) as usize, (l.y / CELL) as usize);
        if x >= self.nx || y >= self.ny {
            return 0.0;
        }
        self.clearance[y * self.nx + x]
    }

    /// How much room the footprint has at this pose (negative = collision).
    pub fn slack(&self, pose: Pose, fp: &Footprint) -> f32 {
        let axis = V2::from_angle(pose.heading);
        fp.offsets
            .iter()
            .map(|&o| self.clearance_at(pose.pos + axis * o) - fp.radius)
            .fold(f32::INFINITY, f32::min)
    }

    pub fn heading_of(h: usize) -> f32 {
        h as f32 * PI / HEADINGS as f32
    }

    fn heading_index(a: f32) -> usize {
        let a = wrap_angle(a).rem_euclid(PI);
        ((a / (PI / HEADINGS as f32)).round() as usize) % HEADINGS
    }

    /// Dijkstra from `goal` over cells with at least `min_clear` clearance.
    /// Admissible heuristic for [`Self::plan`].
    fn distance_field(&self, goal: (usize, usize), min_clear: f32) -> Vec<f32> {
        let n = self.nx * self.ny;
        let mut d = vec![f32::INFINITY; n];
        let gi = goal.1 * self.nx + goal.0;
        d[gi] = 0.0;
        let mut open = BinaryHeap::new();
        open.push(Open(0.0, gi));
        while let Some(Open(dist, i)) = open.pop() {
            if dist > d[i] {
                continue;
            }
            let (x, y) = ((i % self.nx) as isize, (i / self.nx) as isize);
            for (dx, dy, step) in MOVES {
                let (nx, ny) = (x + dx, y + dy);
                if nx < 0 || ny < 0 || nx >= self.nx as isize || ny >= self.ny as isize {
                    continue;
                }
                let j = ny as usize * self.nx + nx as usize;
                if self.clearance[j] < min_clear {
                    continue;
                }
                let nd = dist + step * CELL;
                if nd < d[j] {
                    d[j] = nd;
                    open.push(Open(nd, j));
                }
            }
        }
        d
    }

    pub fn plan(&self, from: Pose, to: V2, fp: &Footprint) -> Option<Vec<Pose>> {
        let start = self.cell_of(from.pos);
        let goal = self.cell_of(to);
        // Geometric distance only (any free cell), so a payload wedged in a
        // tight spot still gets a finite, admissible heuristic.
        let h2 = self.distance_field(goal, 0.01);
        let cells = self.nx * self.ny;
        let idx = |x: usize, y: usize, h: usize| (y * self.nx + x) * HEADINGS + h;

        let mut g = vec![f32::INFINITY; cells * HEADINGS];
        let mut parent = vec![u32::MAX; cells * HEADINGS];
        let mut open = BinaryHeap::new();
        if !h2[start.1 * self.nx + start.0].is_finite() {
            return None;
        }
        let s = idx(start.0, start.1, Self::heading_index(from.heading));
        g[s] = 0.0;
        open.push(Open(h2[start.1 * self.nx + start.0], s));

        while let Some(Open(f, i)) = open.pop() {
            let cell = i / HEADINGS;
            let (x, y, h) = (cell % self.nx, cell / self.nx, i % HEADINGS);
            if f > g[i] + h2[cell] + 1e-4 {
                continue; // stale entry
            }
            if (x, y) == goal {
                let mut out = vec![];
                let mut c = i;
                while c != u32::MAX as usize {
                    let cell = c / HEADINGS;
                    out.push(Pose {
                        pos: self.center(cell % self.nx, cell / self.nx),
                        heading: Self::heading_of(c % HEADINGS),
                    });
                    c = parent[c] as usize;
                }
                out.reverse();
                return Some(out);
            }

            let here = self.slack(
                Pose {
                    pos: self.center(x, y),
                    heading: Self::heading_of(h),
                },
                fp,
            );
            let mut relax = |nx: usize, ny: usize, nh: usize, base: f32| {
                if !h2[ny * self.nx + nx].is_finite() {
                    return; // can't reach the goal from there anyway
                }
                let pose = Pose {
                    pos: self.center(nx, ny),
                    heading: Self::heading_of(nh),
                };
                let slack = self.slack(pose, fp);
                // Collision-free, or at least getting out of a collision the
                // payload is already in (e.g. shoved against a wall).
                if slack < 0.0 && slack <= here + 1e-3 {
                    return;
                }
                let j = idx(nx, ny, nh);
                let ng = g[i] + base * (1.0 + 4.0 * (COMFY_SLACK - slack).max(0.0));
                if ng < g[j] {
                    g[j] = ng;
                    parent[j] = i as u32;
                    open.push(Open(ng + h2[ny * self.nx + nx], j));
                }
            };

            for (dx, dy, step) in MOVES {
                let (nx, ny) = (x as isize + dx, y as isize + dy);
                if nx < 0 || ny < 0 || nx >= self.nx as isize || ny >= self.ny as isize {
                    continue;
                }
                relax(nx as usize, ny as usize, h, step * CELL);
            }
            relax(x, y, (h + 1) % HEADINGS, TURN_COST);
            relax(x, y, (h + HEADINGS - 1) % HEADINGS, TURN_COST);
        }
        None
    }
}

const MOVES: [(isize, isize, f32); 8] = [
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, SQRT_2),
    (1, -1, SQRT_2),
    (-1, 1, SQRT_2),
    (-1, -1, SQRT_2),
];

/// Min-heap entry.
struct Open(f32, usize);

impl PartialEq for Open {
    fn eq(&self, o: &Self) -> bool {
        self.0 == o.0
    }
}
impl Eq for Open {}
impl PartialOrd for Open {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Open {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use antlift_core::{axis_error, Layout, SLOT_OFFSET};

    fn setup() -> (Layout, Grid, Footprint) {
        let l = Layout::doorway();
        let g = Grid::new(l.arena, &l.walls);
        let fp = Footprint::new(l.object_half + V2::new(SLOT_OFFSET, SLOT_OFFSET));
        (l, g, fp)
    }

    #[test]
    fn footprint_covers_rectangle() {
        let half = V2::new(2.5, 1.1);
        let fp = Footprint::new(half);
        // Every corner of the rectangle lies inside some circle.
        for c in [V2::new(2.5, 1.1), V2::new(-2.5, -1.1), V2::new(0.0, 1.1)] {
            assert!(
                fp.offsets
                    .iter()
                    .any(|&o| V2::new(o, 0.0).dist(c) <= fp.radius + 1e-4),
                "{c:?} uncovered"
            );
        }
    }

    #[test]
    fn plans_through_doorway_aligned() {
        let (l, g, fp) = setup();
        let start = Pose {
            pos: l.object_start,
            heading: l.object_start_angle,
        };
        let path = g.plan(start, l.goal, &fp).expect("path exists");
        let crossing = path
            .windows(2)
            .find(|w| w[0].pos.x < 20.0 && w[1].pos.x >= 20.0)
            .expect("crosses wall")[1];
        assert!((10.5..13.5).contains(&crossing.pos.y), "{crossing:?}");
        assert!(
            axis_error(0.0, crossing.heading).abs() < 0.3,
            "should be roughly aligned with the door: {crossing:?}"
        );
        for p in &path[1..] {
            assert!(g.slack(*p, &fp) >= 0.0);
        }
    }

    #[test]
    fn no_plan_when_door_is_sealed() {
        let (mut l, _, fp) = setup();
        l.walls
            .push(Aabb::new(V2::new(19.0, 10.0), V2::new(21.0, 14.0)));
        let g = Grid::new(l.arena, &l.walls);
        let start = Pose {
            pos: l.object_start,
            heading: l.object_start_angle,
        };
        assert!(g.plan(start, l.goal, &fp).is_none());
    }
}

#[cfg(test)]
mod escape_tests {
    use super::*;
    use antlift_core::{Layout, SLOT_OFFSET};

    #[test]
    fn plans_out_of_a_wedged_start() {
        let l = Layout::doorway();
        let g = Grid::new(l.arena, &l.walls);
        let fp = Footprint::new(l.object_half + V2::new(SLOT_OFFSET, SLOT_OFFSET) * 2.0);
        // Diagonal against the doorway's lower wall, like a reflex shove leaves it.
        let start = Pose {
            pos: V2::new(17.45, 12.33),
            heading: 0.54,
        };
        assert!(g.slack(start, &fp) < 0.0, "test needs a colliding start");
        assert!(g.plan(start, l.goal, &fp).is_some());
    }
}
