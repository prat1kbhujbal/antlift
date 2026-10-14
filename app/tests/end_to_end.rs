use antlift_app::{Session, SessionConfig};
use antlift_core::ControlMode;

fn run(control: ControlMode, edge_online: bool, limit: f32) -> (Session, Option<f32>) {
    let mut s = Session::new(SessionConfig::new(control));
    s.set_edge_up(edge_online);
    while s.t() < limit {
        s.step(1.0 / 60.0);
        if s.swarm.at_goal() {
            let t = s.t();
            return (s, Some(t));
        }
    }
    (s, None)
}

#[test]
fn speculative_swarm_delivers_through_the_hall() {
    let (s, reached) = run(ControlMode::Speculative, true, 150.0);
    assert!(
        reached.is_some(),
        "payload stuck at {:?}",
        s.swarm.world.object_state(0).pos
    );
    assert!(
        s.swarm.stats.hits > 0,
        "expected some predicted branches to fire"
    );
}

#[test]
fn reflexes_alone_do_not_get_through() {
    let (_, reached) = run(ControlMode::Speculative, false, 150.0);
    assert!(reached.is_none());
}

#[test]
fn centralized_baseline_also_delivers_on_a_clean_link() {
    let (s, reached) = run(ControlMode::Centralized, true, 150.0);
    assert!(
        reached.is_some(),
        "payload stuck at {:?}",
        s.swarm.world.object_state(0).pos
    );
    assert!(s.swarm.stats.commands_received > 0);
    assert_eq!(
        s.swarm.stats.trees_received, 0,
        "baseline must not ship trees"
    );
}

#[test]
fn switching_modes_mid_run_still_delivers() {
    let mut s = Session::new(SessionConfig::new(ControlMode::Speculative));
    let mut reached = false;
    while s.t() < 180.0 && !reached {
        let t = s.t();
        if (20.0..20.02).contains(&t) {
            s.set_control(ControlMode::Centralized);
        }
        if (45.0..45.02).contains(&t) {
            s.set_control(ControlMode::Speculative);
        }
        s.step(1.0 / 60.0);
        reached = s.swarm.at_goal();
    }
    assert!(
        reached,
        "payload stuck at {:?}",
        s.swarm.world.object_state(0).pos
    );
    assert!(
        s.swarm.stats.commands_received > 0,
        "centralized phase never ran"
    );
}
