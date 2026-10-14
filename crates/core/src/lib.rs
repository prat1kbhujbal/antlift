//! Shared vocabulary between the swarm (robots) and the edge server.
//!
//! Nothing in here knows about physics or threads: it is the "protocol" layer.

pub mod brain;
pub mod geom;
pub mod layout;
pub mod msg;
pub mod tree;

pub use brain::{track, Brain, Command, Event, Mode, Sense, NO_LIMIT};
pub use geom::{axis_error, wrap_angle, Aabb, V2};
pub use layout::{Layout, DELIVERED_RADIUS, ROBOT_RADIUS, SLOT_OFFSET};
pub use msg::{ControlMode, DownBody, Downlink, ObjectState, RobotId, RobotObs, Snapshot, Uplink};
pub use tree::{Action, ContingencyTree, Edge, Node, NodeId, Predicate, Role};
