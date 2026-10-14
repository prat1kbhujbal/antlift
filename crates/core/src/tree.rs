//! Contingency trees: the edge server's speculative plan, sliced per robot.
//!
//! A tree is a small guarded state machine. The robot sits in one node and
//! executes its [`Action`]. Every tick it checks the node's outgoing edges in
//! order and follows the first one whose [`Predicate`] holds, using only its
//! own local sensing. Edges marked `speculative` are contingencies the edge
//! predicted might happen; when one fires the robot handled a surprise with
//! zero network round trips (a "prediction hit").
//!
//! Trees go over the (simulated) radio, so they have a compact binary
//! encoding: positions are quantized to centimetres and angles to 0.1 mrad.

use crate::geom::V2;

pub type NodeId = u8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// Contributes translation force only.
    Pusher,
    /// Sits far from the centre of mass and also corrects heading.
    Steerer,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Action {
    /// Drive to the assigned slot on the object and grip it.
    Approach,
    /// Move the object towards `target` while aligning its long axis to `heading`.
    Push { target: V2, heading: f32 },
    /// Rotate the object in place to `heading`.
    Rotate { heading: f32 },
    /// Brake: hold the object still.
    Hold,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Predicate {
    InContact,
    ContactLost,
    /// Object centre within `radius` of `target`.
    Reached {
        target: V2,
        radius: f32,
    },
    /// Object axis further than `tol` from `heading`.
    HeadingOff {
        heading: f32,
        tol: f32,
    },
    /// Object axis within `tol` of `heading`.
    HeadingOk {
        heading: f32,
        tol: f32,
    },
    /// Object barely moving for `secs` while we push.
    Stalled {
        secs: f32,
    },
    /// Spent `secs` in the current node.
    After {
        secs: f32,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Edge {
    pub when: Predicate,
    pub to: NodeId,
    /// True for contingencies the edge speculated, false for the nominal plan.
    pub speculative: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    pub action: Action,
    pub edges: Vec<Edge>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContingencyTree {
    pub version: u32,
    /// Sim time (s) after which the robot must stop trusting this tree.
    pub valid_until: f32,
    pub role: Role,
    /// Which payload's team this robot is on (the edge can reassign it).
    pub team: u8,
    /// Grip point in the object's local frame.
    pub slot: V2,
    /// Node 0 is the entry node.
    pub nodes: Vec<Node>,
}

impl ContingencyTree {
    pub fn has_stall_edge(&self, node: NodeId) -> bool {
        self.nodes.get(node as usize).is_some_and(|n| {
            n.edges
                .iter()
                .any(|e| matches!(e.when, Predicate::Stalled { .. }))
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut w = Vec::with_capacity(64);
        w.extend_from_slice(&self.version.to_le_bytes());
        w.extend_from_slice(&((self.valid_until * 1000.0) as u32).to_le_bytes());
        w.push(match self.role {
            Role::Pusher => 0,
            Role::Steerer => 1,
        });
        w.push(self.team);
        put_pos(&mut w, self.slot);
        w.push(self.nodes.len() as u8);
        for n in &self.nodes {
            match n.action {
                Action::Approach => w.push(0),
                Action::Push { target, heading } => {
                    w.push(1);
                    put_pos(&mut w, target);
                    put_ang(&mut w, heading);
                }
                Action::Rotate { heading } => {
                    w.push(2);
                    put_ang(&mut w, heading);
                }
                Action::Hold => w.push(3),
            }
            w.push(n.edges.len() as u8);
            for e in &n.edges {
                let spec = if e.speculative { 0x80 } else { 0 };
                match e.when {
                    Predicate::InContact => w.push(spec),
                    Predicate::ContactLost => w.push(spec | 1),
                    Predicate::Reached { target, radius } => {
                        w.push(spec | 2);
                        put_pos(&mut w, target);
                        put_len(&mut w, radius);
                    }
                    Predicate::HeadingOff { heading, tol } => {
                        w.push(spec | 3);
                        put_ang(&mut w, heading);
                        put_ang(&mut w, tol);
                    }
                    Predicate::HeadingOk { heading, tol } => {
                        w.push(spec | 4);
                        put_ang(&mut w, heading);
                        put_ang(&mut w, tol);
                    }
                    Predicate::Stalled { secs } => {
                        w.push(spec | 5);
                        put_len(&mut w, secs);
                    }
                    Predicate::After { secs } => {
                        w.push(spec | 6);
                        put_len(&mut w, secs);
                    }
                }
                w.push(e.to);
            }
        }
        w
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = Reader { b: bytes, i: 0 };
        let version = u32::from_le_bytes(r.take(4)?.try_into().ok()?);
        let valid_until = u32::from_le_bytes(r.take(4)?.try_into().ok()?) as f32 / 1000.0;
        let role = match r.u8()? {
            0 => Role::Pusher,
            1 => Role::Steerer,
            _ => return None,
        };
        let team = r.u8()?;
        let slot = r.pos()?;
        let n_nodes = r.u8()?;
        let mut nodes = Vec::with_capacity(n_nodes as usize);
        for _ in 0..n_nodes {
            let action = match r.u8()? {
                0 => Action::Approach,
                1 => Action::Push {
                    target: r.pos()?,
                    heading: r.ang()?,
                },
                2 => Action::Rotate { heading: r.ang()? },
                3 => Action::Hold,
                _ => return None,
            };
            let n_edges = r.u8()?;
            let mut edges = Vec::with_capacity(n_edges as usize);
            for _ in 0..n_edges {
                let tag = r.u8()?;
                let when = match tag & 0x7f {
                    0 => Predicate::InContact,
                    1 => Predicate::ContactLost,
                    2 => Predicate::Reached {
                        target: r.pos()?,
                        radius: r.len()?,
                    },
                    3 => Predicate::HeadingOff {
                        heading: r.ang()?,
                        tol: r.ang()?,
                    },
                    4 => Predicate::HeadingOk {
                        heading: r.ang()?,
                        tol: r.ang()?,
                    },
                    5 => Predicate::Stalled { secs: r.len()? },
                    6 => Predicate::After { secs: r.len()? },
                    _ => return None,
                };
                let to = r.u8()?;
                if to >= n_nodes {
                    return None;
                }
                edges.push(Edge {
                    when,
                    to,
                    speculative: tag & 0x80 != 0,
                });
            }
            nodes.push(Node { action, edges });
        }
        Some(Self {
            version,
            valid_until,
            role,
            team,
            slot,
            nodes,
        })
    }
}

// Centimetres, +-327 m.
fn put_pos(w: &mut Vec<u8>, p: V2) {
    w.extend_from_slice(&((p.x * 100.0).round() as i16).to_le_bytes());
    w.extend_from_slice(&((p.y * 100.0).round() as i16).to_le_bytes());
}

// 0.1 mrad, +-3.27 rad.
fn put_ang(w: &mut Vec<u8>, a: f32) {
    w.extend_from_slice(&((a * 10_000.0).round() as i16).to_le_bytes());
}

// Unsigned centi-units (m or s), up to 655.35.
fn put_len(w: &mut Vec<u8>, v: f32) {
    w.extend_from_slice(&((v * 100.0).round() as u16).to_le_bytes());
}

struct Reader<'a> {
    b: &'a [u8],
    i: usize,
}

impl Reader<'_> {
    fn take(&mut self, n: usize) -> Option<&[u8]> {
        let s = self.b.get(self.i..self.i + n)?;
        self.i += n;
        Some(s)
    }
    fn u8(&mut self) -> Option<u8> {
        Some(self.take(1)?[0])
    }
    fn i16(&mut self) -> Option<i16> {
        Some(i16::from_le_bytes(self.take(2)?.try_into().ok()?))
    }
    fn pos(&mut self) -> Option<V2> {
        Some(V2::new(
            self.i16()? as f32 / 100.0,
            self.i16()? as f32 / 100.0,
        ))
    }
    fn ang(&mut self) -> Option<f32> {
        Some(self.i16()? as f32 / 10_000.0)
    }
    fn len(&mut self) -> Option<f32> {
        Some(u16::from_le_bytes(self.take(2)?.try_into().ok()?) as f32 / 100.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> ContingencyTree {
        let target = V2::new(12.34, 5.67);
        ContingencyTree {
            version: 7,
            valid_until: 42.5,
            role: Role::Steerer,
            team: 2,
            slot: V2::new(-2.0, 0.85),
            nodes: vec![
                Node {
                    action: Action::Approach,
                    edges: vec![Edge {
                        when: Predicate::InContact,
                        to: 1,
                        speculative: false,
                    }],
                },
                Node {
                    action: Action::Push {
                        target,
                        heading: 1.2345,
                    },
                    edges: vec![
                        Edge {
                            when: Predicate::Reached {
                                target,
                                radius: 0.8,
                            },
                            to: 2,
                            speculative: false,
                        },
                        Edge {
                            when: Predicate::Stalled { secs: 1.5 },
                            to: 2,
                            speculative: true,
                        },
                    ],
                },
                Node {
                    action: Action::Hold,
                    edges: vec![Edge {
                        when: Predicate::After { secs: 1.0 },
                        to: 1,
                        speculative: true,
                    }],
                },
            ],
        }
    }

    #[test]
    fn roundtrip_within_quantization() {
        let t = sample();
        let bytes = t.encode();
        let d = ContingencyTree::decode(&bytes).expect("decodes");
        assert_eq!(d.version, t.version);
        assert_eq!(d.role, t.role);
        assert_eq!(d.team, 2);
        assert_eq!(d.nodes.len(), 3);
        match d.nodes[1].action {
            Action::Push { target, heading } => {
                assert!(target.dist(V2::new(12.34, 5.67)) < 0.01);
                assert!((heading - 1.2345).abs() < 1e-3);
            }
            other => panic!("wrong action {other:?}"),
        }
        assert!(d.nodes[1].edges[1].speculative);
        assert!(d.has_stall_edge(1));
        assert!(!d.has_stall_edge(0));
    }

    #[test]
    fn encoding_is_compact() {
        // Header 16 bytes plus a handful of bytes per node/edge.
        assert!(sample().encode().len() < 64, "{}", sample().encode().len());
    }

    #[test]
    fn rejects_truncated_and_dangling() {
        let bytes = sample().encode();
        assert!(ContingencyTree::decode(&bytes[..bytes.len() - 1]).is_none());
        let mut bad = sample();
        bad.nodes[0].edges[0].to = 9;
        assert!(ContingencyTree::decode(&bad.encode()).is_none());
    }
}
