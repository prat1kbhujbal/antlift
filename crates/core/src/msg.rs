//! Messages that cross the simulated radio between robots and the edge.

use crate::geom::{Aabb, V2};
use crate::tree::NodeId;

pub type RobotId = u16;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ObjectState {
    pub pos: V2,
    pub angle: f32,
    pub vel: V2,
    pub angvel: f32,
    /// Half extents in the object's local frame (x = long axis).
    pub half: V2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotObs {
    pub id: RobotId,
    pub pos: V2,
    pub active: bool,
    pub attached: bool,
    /// Which payload it is holding, when attached.
    pub holding: u8,
}

/// What the edge's overhead camera sees. Infrastructure traffic, not robot radio.
#[derive(Clone, Debug, PartialEq)]
pub struct Snapshot {
    pub t: f32,
    /// Every payload, in layout order.
    pub objects: Vec<ObjectState>,
    pub robots: Vec<RobotObs>,
    /// Obstacles that are not part of the static layout (e.g. dropped crates).
    pub obstacles: Vec<Aabb>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Uplink {
    /// Periodic, tiny: lets the edge notice dead or detached robots.
    Heartbeat {
        robot: RobotId,
        node: NodeId,
        attached: bool,
    },
    /// A tree edge fired. Speculative ones are prediction hits.
    BranchFired {
        robot: RobotId,
        version: u32,
        node: NodeId,
        speculative: bool,
    },
    /// The robot is stuck in a situation its tree has no branch for.
    Surprise {
        robot: RobotId,
        version: u32,
        node: NodeId,
        at: V2,
    },
    Camera(Snapshot),
    /// A tree (or lease renewal) arrived and was installed; lets the edge
    /// retransmit lost ones.
    TreeAck {
        robot: RobotId,
        version: u32,
        valid_until: f32,
    },
    /// Centralized baseline only: the robot's own state, streamed at 20 Hz.
    State {
        robot: RobotId,
        pos: V2,
        vel: V2,
        /// Grip point in the payload frame, when gripping (the robot knows
        /// its own joint anchor exactly).
        grip: Option<V2>,
    },
}

impl Uplink {
    /// Bytes this message would take on the wire.
    pub fn wire_len(&self) -> usize {
        match self {
            Uplink::Heartbeat { .. } => 1 + 2 + 1 + 1,
            Uplink::BranchFired { .. } => 1 + 2 + 4 + 1 + 1,
            Uplink::Surprise { .. } => 1 + 2 + 4 + 1 + 4,
            Uplink::TreeAck { .. } => 1 + 2 + 4 + 4,
            Uplink::Camera(s) => {
                8 + s.objects.len() * 16 + s.robots.len() * 6 + s.obstacles.len() * 8
            }
            // Position in cm and velocity in cm/s as i16 pairs, plus the
            // grip point in cm when gripping.
            Uplink::State { grip, .. } => 1 + 2 + 4 + 4 + 1 + if grip.is_some() { 4 } else { 0 },
        }
    }

    /// The robot that sent this, if any (the camera feed has none).
    pub fn robot(&self) -> Option<RobotId> {
        match self {
            Uplink::Heartbeat { robot, .. }
            | Uplink::BranchFired { robot, .. }
            | Uplink::Surprise { robot, .. }
            | Uplink::TreeAck { robot, .. }
            | Uplink::State { robot, .. } => Some(*robot),
            Uplink::Camera(_) => None,
        }
    }

    /// Robot radio traffic, as opposed to the infrastructure camera feed.
    pub fn is_robot_radio(&self) -> bool {
        !matches!(self, Uplink::Camera(_))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Downlink {
    pub robot: RobotId,
    pub body: DownBody,
}

#[derive(Clone, Debug, PartialEq)]
pub enum DownBody {
    /// An encoded [`crate::ContingencyTree`] (speculative mode).
    Tree(Vec<u8>),
    /// What to do until the next command (centralized baseline): either a
    /// raw force or, when `vel` is set, a velocity setpoint the robot tracks
    /// with its own fast local loop.
    Command {
        force: V2,
        vel: Option<V2>,
        grip: bool,
        release: bool,
    },
}

impl Downlink {
    pub fn wire_len(&self) -> usize {
        2 + match &self.body {
            DownBody::Tree(t) => t.len(),
            // One vector (force in 0.01 N or velocity in mm/s) as two i16,
            // plus a flags byte saying which, and grip/release.
            DownBody::Command { .. } => 4 + 1,
        }
    }
}

/// Where the swarm's decisions are made.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ControlMode {
    /// Robots run their own brains on edge-issued contingency trees.
    Speculative,
    /// The edge runs every brain and streams force commands (baseline).
    Centralized,
}
