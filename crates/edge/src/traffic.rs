//! Block-reservation traffic control between teams, as industrial AGV
//! traffic managers do it.
//!
//! The floor is divided into square blocks. Every team owns the blocks under
//! its payload and robots, and asks for the blocks along the next few metres
//! of its route, one leg (waypoint) at a time. Grants go out in priority
//! order and are never taken back, so a team only moves into floor that is
//! reserved for it. A team that can't get its next leg books the free blocks
//! of that leg, so lower-priority teams can't slip in front of it, and waits
//! at the end of its grant.
//!
//! Teams are told how far they may go with a tiny permit message ("up to
//! node N"); the plan itself doesn't change, so waiting costs no replans.
//! Who waits for whom forms a wait-for graph; a cycle in it is a deadlock,
//! which the edge breaks by rerouting the lowest-priority team in it.

use antlift_core::{Aabb, V2};

/// Side of a block (m).
pub const BLOCK: f32 = 1.0;

#[derive(Clone, Debug)]
pub struct Blocks {
    pub origin: V2,
    pub nx: usize,
    pub ny: usize,
}

impl Blocks {
    pub fn new(arena: Aabb) -> Self {
        let size = arena.max - arena.min;
        Self {
            origin: arena.min,
            nx: (size.x / BLOCK).ceil().max(1.0) as usize,
            ny: (size.y / BLOCK).ceil().max(1.0) as usize,
        }
    }

    pub fn len(&self) -> usize {
        self.nx * self.ny
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Every block `b` overlaps, appended to `out` (no duplicates added if
    /// `out` already has them).
    pub fn cover(&self, b: Aabb, out: &mut Vec<usize>) {
        let lo = b.min - self.origin;
        let hi = b.max - self.origin;
        let clamp = |v: f32, n: usize| ((v / BLOCK).floor().max(0.0) as usize).min(n - 1);
        for y in clamp(lo.y, self.ny)..=clamp(hi.y, self.ny) {
            for x in clamp(lo.x, self.nx)..=clamp(hi.x, self.nx) {
                let i = y * self.nx + x;
                if !out.contains(&i) {
                    out.push(i);
                }
            }
        }
    }

    pub fn rect(&self, i: usize) -> Aabb {
        let min = self.origin + V2::new((i % self.nx) as f32, (i / self.nx) as f32) * BLOCK;
        Aabb::new(min, min + V2::new(BLOCK, BLOCK))
    }
}

/// One team's claim this round.
#[derive(Clone, Debug)]
pub struct Request {
    pub team: usize,
    /// Blocks under the payload and its robots now.
    pub body: Vec<usize>,
    /// Blocks swept by each leg of the route, from leg `first`.
    pub legs: Vec<Vec<usize>>,
    /// Index of `legs[0]` in the plan (legs before it are behind the team).
    pub first: usize,
    /// Legs already granted (grants are never taken back).
    pub granted: usize,
    /// Grant at most up to this leg (exclusive): just in time, not the
    /// whole route at once.
    pub reach: usize,
}

#[derive(Clone, Debug, Default)]
pub struct Outcome {
    /// Owner team of every block.
    pub owner: Vec<Option<usize>>,
    /// New grant per request, in request order.
    pub granted: Vec<usize>,
    /// The team whose blocks stopped each request's grant.
    pub waits_for: Vec<Option<usize>>,
}

/// Hand out blocks. `reqs` are in priority order (most important first).
pub fn allocate(n_blocks: usize, reqs: &[Request]) -> Outcome {
    let mut owner: Vec<Option<usize>> = vec![None; n_blocks];
    // What teams physically occupy, then what they were already granted.
    for r in reqs {
        take(&mut owner, r.team, &r.body);
    }
    for r in reqs {
        for leg in held_legs(r) {
            take(&mut owner, r.team, leg);
        }
    }
    let mut granted = vec![];
    let mut waits_for = vec![];
    for r in reqs {
        let mut g = r.granted.max(r.first);
        let mut blocked_by = None;
        while g < r.reach {
            let Some(leg) = r.legs.get(g - r.first) else {
                break;
            };
            // Blocks under our own payload are ours to use even if a
            // neighbour's footprint reaches into them too.
            let other = leg
                .iter()
                .filter(|b| !r.body.contains(b))
                .find_map(|&b| owner[b].filter(|&o| o != r.team));
            if let Some(other) = other {
                blocked_by = Some(other);
                // Book what's free of this leg so nobody behind us takes it.
                take(&mut owner, r.team, leg);
                break;
            }
            take(&mut owner, r.team, leg);
            g += 1;
        }
        granted.push(g);
        waits_for.push(blocked_by);
    }
    Outcome {
        owner,
        granted,
        waits_for,
    }
}

fn take(owner: &mut [Option<usize>], team: usize, blocks: &[usize]) {
    for &b in blocks {
        owner[b].get_or_insert(team);
    }
}

/// Legs granted earlier and not yet passed.
fn held_legs(r: &Request) -> impl Iterator<Item = &Vec<usize>> {
    let held = r.granted.saturating_sub(r.first).min(r.legs.len());
    r.legs[..held].iter()
}

/// The deadlock cycle through `start` in the wait-for graph, if any.
pub fn cycle(waits_for: &[Option<usize>], start: usize) -> Option<Vec<usize>> {
    let mut seen = vec![start];
    let mut at = start;
    while let Some(next) = waits_for.get(at).copied().flatten() {
        if next == start {
            return Some(seen);
        }
        if seen.contains(&next) {
            return None; // a cycle, but not through `start`
        }
        seen.push(next);
        at = next;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn req(team: usize, body: &[usize], legs: &[&[usize]]) -> Request {
        Request {
            team,
            body: body.to_vec(),
            legs: legs.iter().map(|l| l.to_vec()).collect(),
            first: 0,
            granted: 0,
            reach: legs.len(),
        }
    }

    #[test]
    fn free_legs_are_granted_up_to_reach() {
        let mut r = req(0, &[0], &[&[1], &[2], &[3]]);
        r.reach = 2;
        let out = allocate(10, &[r]);
        assert_eq!(out.granted, vec![2]);
        assert_eq!(out.owner[3], None);
    }

    #[test]
    fn higher_priority_goes_first_and_the_other_waits() {
        // Both want block 5 next.
        let a = req(0, &[0], &[&[5], &[6]]);
        let b = req(1, &[9], &[&[5], &[4]]);
        let out = allocate(10, &[a, b]);
        assert_eq!(out.granted, vec![2, 0]);
        assert_eq!(out.waits_for, vec![None, Some(0)]);
    }

    #[test]
    fn grants_are_kept_and_blocked_legs_are_booked() {
        // B was granted block 5 earlier: A, though first in line, waits.
        let a = req(0, &[0], &[&[5, 7]]);
        let mut b = req(1, &[9], &[&[5]]);
        b.granted = 1;
        let c = req(2, &[8], &[&[7]]);
        let out = allocate(10, &[a, b, c]);
        assert_eq!(out.granted[0], 0);
        assert_eq!(out.waits_for[0], Some(1));
        // A booked block 7, so C can't take it.
        assert_eq!(out.waits_for[2], Some(0));
    }

    #[test]
    fn head_on_teams_form_a_cycle() {
        let a = req(0, &[0], &[&[9]]);
        let b = req(1, &[9], &[&[0]]);
        let out = allocate(10, &[a, b]);
        assert_eq!(cycle(&out.waits_for, 0), Some(vec![0, 1]));
        assert_eq!(cycle(&[Some(1), None], 0), None);
        // 0 waits on a cycle it isn't part of.
        assert_eq!(cycle(&[Some(1), Some(2), Some(1)], 0), None);
    }

    #[test]
    fn blocks_cover_boxes() {
        let g = Blocks::new(Aabb::new(V2::ZERO, V2::new(10.0, 5.0)));
        assert_eq!((g.nx, g.ny), (10, 5));
        let mut out = vec![];
        g.cover(Aabb::new(V2::new(1.5, 0.5), V2::new(2.5, 1.5)), &mut out);
        assert_eq!(out, vec![1, 2, 11, 12]);
        assert_eq!(g.rect(12).min, V2::new(2.0, 1.0));
    }
}
