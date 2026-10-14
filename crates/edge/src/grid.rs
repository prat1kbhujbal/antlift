//! Clearance map and an SE(2) planner for the payload.
//!
//! The swarm can push the payload in any direction and turn it in place, so
//! this is holonomic: states are (cell, heading) and moves are one-cell
//! translations or one-step turns. The footprint (payload plus the robots
//! gripping its sides) is checked as a chain of circles against an exact
//! clearance map. A 2D distance-to-goal field is the A* heuristic.

use antlift_core::{axis_error, wrap_angle, Aabb, V2};
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

    /// Smooth a grid path: replace runs of cells with straight segments,
    /// turning gradually along each one instead of stopping to rotate. A
    /// shortcut keeps as much slack as the stretch it replaces had (capped at
    /// [`SMOOTH_MARGIN`]), and stretches tighter than [`SMOOTH_TIGHT`]
    /// (doorways, channels, a wedged start) are never shortcut: there the search's own
    /// centred, aligned cell-by-cell path is kept. The result is resampled
    /// every [`SMOOTH_STEP`] metres.
    pub fn smooth(&self, poses: &[Pose], fp: &Footprint) -> Vec<Pose> {
        if poses.len() < 3 {
            return poses.to_vec();
        }
        let slack: Vec<f32> = poses.iter().map(|p| self.slack(*p, fp)).collect();
        let last = poses.len() - 1;
        let mut keep = vec![0];
        let mut i = 0;
        while i < last {
            let ok = |j: usize| {
                let floor = slack[i..=j].iter().fold(SMOOTH_MARGIN, |m, &s| m.min(s));
                floor >= SMOOTH_TIGHT && self.segment_clear(poses[i], poses[j], fp, floor)
            };
            // Gallop forward from the next cell (always fine), then binary
            // search the furthest reachable pose.
            let (mut good, mut step) = (i + 1, 1);
            let mut bad = None;
            while good < last {
                let next = (good + step).min(last);
                if ok(next) {
                    good = next;
                    step *= 2;
                } else {
                    bad = Some(next);
                    break;
                }
            }
            if let Some(mut bad) = bad {
                while bad - good > 1 {
                    let mid = (good + bad) / 2;
                    if ok(mid) {
                        good = mid;
                    } else {
                        bad = mid;
                    }
                }
            }
            keep.push(good);
            i = good;
        }

        let mut out = vec![];
        for w in keep.windows(2) {
            let (a, b) = (poses[w[0]], poses[w[1]]);
            let n = (a.pos.dist(b.pos) / SMOOTH_STEP).ceil().max(1.0) as usize;
            for k in 0..n {
                out.push(lerp_pose(a, b, k as f32 / n as f32));
            }
        }
        out.push(poses[last]);
        out
    }

    /// The footprint keeps at least `floor` slack all along a to b.
    fn segment_clear(&self, a: Pose, b: Pose, fp: &Footprint, floor: f32) -> bool {
        let turn = axis_error(b.heading, a.heading).abs();
        let n = (a.pos.dist(b.pos) / 0.1).max(turn / 0.05).ceil().max(1.0) as usize;
        (1..n).all(|k| self.slack(lerp_pose(a, b, k as f32 / n as f32), fp) >= floor - 1e-3)
    }

    pub fn heading_of(h: usize) -> f32 {
        h as f32 * PI / HEADINGS as f32
    }

    fn heading_index(a: f32) -> usize {
        let a = wrap_angle(a).rem_euclid(PI);
        ((a / (PI / HEADINGS as f32)).round() as usize) % HEADINGS
    }

    /// Every point along a to b has at least `min_clear` clearance.
    pub fn line_clear(&self, a: V2, b: V2, min_clear: f32) -> bool {
        let n = (a.dist(b) / 0.1).ceil().max(1.0) as usize;
        (0..=n).all(|k| self.clearance_at(a + (b - a) * (k as f32 / n as f32)) >= min_clear)
    }

    /// A 2D route for a point needing `min_clear` clearance (a robot): a
    /// distance field from the goal, steepest descent from the start, then
    /// shortcuts wherever the straight line stays clear. Starts at `from`,
    /// ends at `to`. None if unreachable.
    pub fn route_2d(&self, from: V2, to: V2, min_clear: f32) -> Option<Vec<V2>> {
        let goal = self.cell_of(to);
        let field = self.distance_field(goal, min_clear);
        let idx = |(x, y): (usize, usize)| y * self.nx + x;
        // A robot squeezed against a wall may start in a cell that's "too
        // tight": step out to the nearest free cell first (the nearest, not
        // the one nearest the goal, which may be across a thin obstacle).
        let (sx, sy) = self.cell_of(from);
        let r = 6;
        let mut c = (sy.saturating_sub(r)..=(sy + r).min(self.ny - 1))
            .flat_map(|y| (sx.saturating_sub(r)..=(sx + r).min(self.nx - 1)).map(move |x| (x, y)))
            .filter(|&c| field[idx(c)].is_finite())
            .min_by(|&a, &b| {
                let d = |c: (usize, usize)| self.center(c.0, c.1).dist(from);
                d(a).total_cmp(&d(b))
                    .then(field[idx(a)].total_cmp(&field[idx(b)]))
            })?;
        let mut pts = vec![from, self.center(c.0, c.1)];
        while c != goal {
            let (x, y) = (c.0 as isize, c.1 as isize);
            let next = MOVES
                .iter()
                .map(|&(dx, dy, _)| (x + dx, y + dy))
                .filter(|&(nx, ny)| {
                    nx >= 0 && ny >= 0 && nx < self.nx as isize && ny < self.ny as isize
                })
                .map(|(nx, ny)| (nx as usize, ny as usize))
                .min_by(|&a, &b| field[idx(a)].total_cmp(&field[idx(b)]))?;
            if field[idx(next)] >= field[idx(c)] {
                break;
            }
            c = next;
            pts.push(self.center(c.0, c.1));
        }
        pts.push(to);

        // Shortcut: from each kept point jump to the furthest visible one.
        let mut out = vec![pts[0]];
        let mut i = 0;
        while i + 1 < pts.len() {
            let j = (i + 1..pts.len())
                .rev()
                .find(|&j| self.line_clear(pts[i], pts[j], min_clear))
                .unwrap_or(i + 1);
            out.push(pts[j]);
            i = j;
        }
        Some(out)
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
        self.plan_with(from, to, fp, &|_| 0.0)
    }

    /// Like [`Self::plan`], with an extra cost multiplier per location
    /// (e.g. learned trouble spots), added on top of the tightness cost.
    pub fn plan_with(
        &self,
        from: Pose,
        to: V2,
        fp: &Footprint,
        penalty: &dyn Fn(V2) -> f32,
    ) -> Option<Vec<Pose>> {
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
                let tight = 4.0 * (COMFY_SLACK - slack).max(0.0);
                let ng = g[i] + base * (1.0 + tight + penalty(pose.pos));
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
            // Turning in place sweeps the space between headings: check the
            // halfway angle too, so a long payload can't "teleport" round.
            let step = std::f32::consts::PI / HEADINGS as f32;
            for (nh, sign) in [
                ((h + 1) % HEADINGS, 1.0),
                ((h + HEADINGS - 1) % HEADINGS, -1.0),
            ] {
                let mid = Pose {
                    pos: self.center(x, y),
                    heading: Self::heading_of(h) + sign * step / 2.0,
                };
                let mid_slack = self.slack(mid, fp);
                if mid_slack < 0.0 && mid_slack <= here + 1e-3 {
                    continue;
                }
                relax(x, y, nh, TURN_COST);
            }
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

/// Shortcuts keep the slack of the stretch they replace, up to this much:
/// the same comfort margin the search aims for, so they never pull routes
/// tight against walls and door frames.
pub const SMOOTH_MARGIN: f32 = COMFY_SLACK;
/// Stretches with less slack than this are never shortcut.
pub const SMOOTH_TIGHT: f32 = 0.3;
/// Smoothed paths are resampled this often (m).
pub const SMOOTH_STEP: f32 = 0.25;

/// Straight-line blend between two poses; the heading takes the shorter
/// way round the payload's symmetric axis.
fn lerp_pose(a: Pose, b: Pose, t: f32) -> Pose {
    Pose {
        pos: a.pos + (b.pos - a.pos) * t,
        heading: a.heading + axis_error(b.heading, a.heading) * t,
    }
}

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
    use std::f32::consts::FRAC_PI_2;

    fn setup() -> (Layout, Grid, Footprint) {
        let l = Layout::hall();
        let g = Grid::new(l.arena, &l.walls);
        let fp = Footprint::new(l.payloads[0].half + V2::new(SLOT_OFFSET, SLOT_OFFSET));
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
    fn plans_the_s_route_upright_through_the_channel() {
        let (l, g, fp) = setup();
        let start = Pose {
            pos: l.payloads[0].start,
            heading: l.payloads[0].angle,
        };
        let path = g.plan(start, l.payloads[0].goal, &fp).expect("path exists");
        // The only way north is the 4 m channel between the two machines
        // (x 14..18), and the sofa has to be upright in it.
        let channel: Vec<&Pose> = path
            .iter()
            .filter(|p| (9.0..12.0).contains(&p.pos.y) && (14.0..18.0).contains(&p.pos.x))
            .collect();
        assert!(!channel.is_empty(), "route goes up the channel");
        for p in channel {
            assert!(
                axis_error(FRAC_PI_2, p.heading).abs() < 0.3,
                "upright in the channel: {p:?}"
            );
        }
        // It enters the dock through its 5.5 m mouth.
        let entry = path
            .windows(2)
            .find(|w| w[0].pos.x < 31.6 && w[1].pos.x >= 31.6)
            .expect("enters the dock")[1];
        assert!((3.5..9.0).contains(&entry.pos.y), "{entry:?}");
        for p in &path[1..] {
            assert!(g.slack(*p, &fp) >= 0.0);
        }
    }

    #[test]
    fn smoothing_straightens_and_stays_clear() {
        let (l, g, fp) = setup();
        let start = Pose {
            pos: l.payloads[0].start,
            heading: l.payloads[0].angle,
        };
        let raw = g.plan(start, l.payloads[0].goal, &fp).unwrap();
        let smooth = g.smooth(&raw, &fp);
        let length = |p: &[Pose]| p.windows(2).map(|w| w[0].pos.dist(w[1].pos)).sum::<f32>();
        let bends = |p: &[Pose]| {
            p.windows(3)
                .filter(|w| {
                    let (a, b) = ((w[1].pos - w[0].pos).angle(), (w[2].pos - w[1].pos).angle());
                    wrap_angle(a - b).abs() > 0.1
                })
                .count()
        };
        assert!(length(&smooth) <= length(&raw) + 1e-3);
        // Fewer bends (tight stretches keep their grid steps on purpose).
        assert!(
            bends(&smooth) < bends(&raw),
            "{} vs {}",
            bends(&smooth),
            bends(&raw)
        );
        // Every smoothed pose either lies on the original grid path (tight
        // spots are left alone) or keeps at least the tight threshold.
        for p in &smooth[1..] {
            let on_grid = raw.iter().any(|r| r.pos.dist(p.pos) < 0.01);
            assert!(on_grid || g.slack(*p, &fp) >= SMOOTH_TIGHT - 0.01, "{p:?}");
        }
        assert_eq!(smooth.last().unwrap().pos, raw.last().unwrap().pos);
    }

    #[test]
    fn robot_routes_go_round_machines() {
        let (l, g, _) = setup();
        // From the west strip, round the press line, to the far side of it.
        let (from, to) = (V2::new(3.0, 18.0), V2::new(16.0, 18.0));
        assert!(!g.line_clear(from, to, 0.3), "test needs a wall in the way");
        let route = g.route_2d(from, to, 0.3).expect("route exists");
        assert_eq!(route[0], from);
        assert_eq!(*route.last().unwrap(), to);
        // Clearance is looked up per cell, so allow half a cell of slack
        // where a leg rounds a corner; a robot (0.25 m) still clears it.
        for w in route.windows(2) {
            assert!(g.line_clear(w[0], w[1], 0.3 - CELL / 2.0), "{w:?}");
        }
        let _ = l;
    }

    #[test]
    fn no_plan_when_the_channel_is_sealed() {
        let (mut l, _, fp) = setup();
        l.walls
            .push(Aabb::new(V2::new(13.0, 9.0), V2::new(19.0, 11.0)));
        let g = Grid::new(l.arena, &l.walls);
        let start = Pose {
            pos: l.payloads[0].start,
            heading: l.payloads[0].angle,
        };
        assert!(g.plan(start, l.payloads[0].goal, &fp).is_none());
    }
}

#[cfg(test)]
mod escape_tests {
    use super::*;
    use antlift_core::{Layout, SLOT_OFFSET};

    #[test]
    fn plans_out_of_a_wedged_start() {
        let l = Layout::hall();
        let g = Grid::new(l.arena, &l.walls);
        let fp = Footprint::new(l.payloads[0].half + V2::new(SLOT_OFFSET, SLOT_OFFSET) * 2.0);
        // Tilted in the channel and jammed against both machines, like a
        // reflex shove can leave it. (Escape needs some move to increase
        // clearance, so a payload buried deep in both walls can't escape.)
        let start = Pose {
            pos: V2::new(16.0, 10.5),
            heading: 1.15,
        };
        assert!(g.slack(start, &fp) < 0.0, "test needs a colliding start");
        assert!(g.plan(start, l.payloads[0].goal, &fp).is_some());
    }
}
