# AntLift design

Status: v0.2. The swarm, the edge server and the network all run on one laptop, and the method is evaluated against a centralized baseline.

## 1. Problem

A swarm of small robots has to carry a large, awkward payload (a 4 m sofa) through a cluttered building. The usual architectures each fail in their own way:

| Architecture | Strength | Failure mode |
|---|---|---|
| Central server sends commands every tick | Global view, good decisions | Every reaction waits a network round trip, and the swarm freezes if the server dies |
| Fully decentralized (ant rules) | Instant reactions, no single point of failure | Only local view, so it jams in doorways the global map says need a turn first |

## 2. Core idea: branch prediction for swarms

CPUs don't stall on an `if`: they predict the branch and have both outcomes ready. AntLift applies the same idea to robot coordination.

1. The **edge server** keeps a global model and **speculates** on what could go wrong on each leg of the route (jam in a doorway, heading drifting, a robot losing its grip).
2. It compiles the nominal plan plus the most valuable contingencies into a **contingency tree**: a small guarded state machine.
3. Each robot gets **its own slice**, pruned to what its role can act on and capped at a **byte budget**.
4. Robots evaluate the guards **locally, every tick**. When a predicted contingency happens, the robot switches branch immediately with no network round trip. That's a **prediction hit**.
5. When something happens that the tree has no branch for, the robot reports a **surprise**. The edge raises its prior for that place and situation (it learns where its predictions fail) and replans.
6. Trees are **leases**. If the edge goes quiet, robots keep running their cached tree until it expires, then fall back to reflexes.

```
                 ┌──────────────────────── EDGE SERVER (own thread) ─────────────────────────┐
 camera 2 Hz ──▶ │ SE(2) planner ─▶ waypoints ─▶ contingency candidates ─▶ per-robot trees    │
                 │      ▲                         (scored p × impact,     (role-pruned,       │
                 │      └── surprise heat map ◀── surprises               ≤ 256 B each)       │
                 └───────────────▲─────────────────────────────────────────────┬─────────────┘
                     heartbeats, │ branch-fired, surprises        trees (only on │ change / lease renewal)
                       (≤ 12 B)  │                                              ▼
                 ┌───────────────┴──────────── SIMULATED RADIO ───────────────────────────────┐
                 │   latency, jitter, loss, outage, byte accounting  (sim-time driven)         │
                 └───────────────▲─────────────────────────────────────────────┬─────────────┘
                                 │                                              ▼
                 ┌───────────────┴──────────────── SWARM (Rapier 2D) ─────────────────────────┐
                 │  robot brains: follow tree edges on local sensing → control law → force    │
                 │  no valid tree → reflex rules                                               │
                 └────────────────────────────────────────────────────────────────────────────┘
```

## 3. Components

### 3.1 `antlift-core`: protocol
- `ContingencyTree`: nodes hold an `Action` (`Approach`, `Push{target, heading}`, `Rotate{heading}`, `Hold`) and ordered guarded `Edge`s. Each edge has a `Predicate` (`InContact`, `ContactLost`, `Reached`, `HeadingOff`, `HeadingOk`, `Stalled`, `After`) and a `speculative` flag.
- **Wire format**: hand-rolled binary. Positions are quantized to cm (i16), angles to 0.1 mrad (i16) and durations to 10 ms (u16). Typical trees are 100-250 B. Decoding rejects truncated input and dangling node references.
- Messages: `Uplink` (`Heartbeat`, `BranchFired`, `Surprise`, `TreeAck`, `Camera`, and the baseline's `State`) and `Downlink` (a tree, or the baseline's `Command`), each with a `wire_len()`.
- `Brain` (decision logic) lives here too, so robots and the centralized baseline run identical code.

### 3.2 `antlift-net`: simulated radio
One-way `Link<T>` driven by **sim time**, so results don't depend on how fast the laptop is. It models latency, jitter, Bernoulli loss and outages (an outage also drops packets already in flight), and counts bytes even for lost packets, since the airtime was still spent.

### 3.3 `antlift-sim`: swarm and physics
- Rapier 2D, top-down, no gravity. Floor friction is modelled as damping. The payload is a 4 × 1.2 m box. Robots are 0.25 m discs with a 25 N force limit that grip the payload through revolute joints.
- **Brain** (one per robot, sees only its own `Sense`):
  - follows tree edges, reports `BranchFired` (speculative ones count as hits);
  - **stall detection**: "stalled" means no net progress (under 15 cm and 3°) since a progress anchor was set. If the current node has no `Stalled` edge, the robot reports a `Surprise` every 3 s until it gets help;
  - **re-localization**: after (re)gripping, it jumps to the plan leg the payload is actually closest to;
  - **escalation**: if a planned jam recovery fires 3 times without any nominal progress, the prediction was right but the fix isn't working, so that also becomes a surprise;
  - approach controller that matches the slot's velocity and orbits around the payload when it's in the way;
  - **carry law**: every gripping robot runs the same law on the same estimate, so the swarm agrees without talking. Translation uses a feed-forward + P term toward the waypoint, scaled down while the heading error is large (turn first, then move). Steerers (grip points far from the centre) add a tangential term for heading control.
  - **reflex mode** (no valid tree): grab the nearest free point and push straight at the goal beacon.

### 3.4 `antlift-edge`: the edge server
- **Planner**: holonomic SE(2) A* over (0.25 m cell, 16 headings over π). Clearance is computed exactly from obstacle geometry. The footprint (payload plus gripping robots) is a chain of circles. A 2D geometric distance-to-goal field is the admissible heuristic and also prunes cells that can't reach the goal. Moves that increase clearance are allowed even from a colliding pose, so a payload shoved against a wall can still be planned out. Cells with little slack cost extra, which keeps plans centred in gaps. Typical plan: about 30 ms.
- Movable crates block the first planning attempt. If no route exists, it plans through them and relies on jam contingencies.
- **Waypoints**: every 2 m in the open, every 1 m in tight spots, and wherever the heading changes by more than 20°. In-place turns become `Rotate` nodes.
- **Speculation**: for each waypoint within the horizon (8) it scores candidates by `p × impact`:
  - Jam: `p = 0.05 + 0.8·tightness + heat(location)`, recovery = back off → square up → retry
  - Misalign (pushers only): `p ∝ heading change`, recovery = pause until aligned
  - Grip loss: `p = 0.03 + 0.05·observed_grip_losses`, recovery = re-approach and re-grip
  Candidates are added greedily while the encoded tree fits the byte budget.
- **Roles and slots**: robots are spread evenly around the perimeter, and robots already gripping keep their spot. Robots far from the centre become steerers. Steerer trees don't get misalignment branches, because steerers are the ones fixing it.
- **Reliable delivery**: robots acknowledge each tree and lease. The edge retransmits byte-identical copies after an RTT-adaptive timeout with exponential backoff.
- **Event-driven replanning**: it replans on a surprise, a change in obstacles, the payload drifting more than 2.5 m off the path, or the payload nearing the end of the horizon (seen by the camera, so it still works if the robots' reports are lost). Otherwise it only renews leases (every 8 s, lease 12 s).

### 3.5 `antlift` app: one-laptop runtime
- `Session` wires swarm ↔ links ↔ edge at a fixed 60 Hz sim tick.
- `EdgeRuntime::Inline` runs the edge in the same thread, deterministically (bench, tests). `EdgeRuntime::Thread` puts the edge on its own thread, talking only over channels, so planning time is real and the UI never blocks.
- Interactive UI (macroquad): drop crates, kick the payload, shake robots loose, kill robots, take the edge offline, change latency and loss, and switch between speculative and centralized control (`C`).

## 4. Evaluation

### 4.1 Method
- **Two control modes, same brain.** In *speculative* mode each robot runs its brain on its own sensing and an edge-issued tree. In *centralized* mode the edge runs **the identical brain code** for every robot, using streamed robot state (20 Hz) and a 20 Hz camera feed (dead-reckoned forward to cover camera latency), and streams commands back at 20 Hz. To give the baseline a fair shot:
  - while a robot is approaching the payload, the edge sends **velocity setpoints** and the robot closes its velocity loop locally at 60 Hz;
  - robots report their exact grip point;
  - commands are held until the next one arrives, and a 0.3 s watchdog stops a robot if commands stop coming.
  So the only difference between the modes is *where decisions are made*.
- **Conditions:** clean (40 ms), slow (150 ± 50 ms), lossy (20% loss), slow+lossy, disrupted (two kicks, a crate dropped in the doorway, front robots shaken loose), and a 10 s edge outage.
- **Sweeps:** one-way latency from 0 to 1000 ms (10% loss), and edge outage length from 0 to 40 s.
- **Seeds:** 10 per cell. Each seed randomizes the payload's start pose (±1 m, ±0.4 rad) and the radio's random generator. 380 runs take a few seconds on a multi-core laptop.
- **Metrics:** success within 240 s, time to goal (mean ± 95% CI over successes), and robot radio bytes per robot per second in both directions (the camera feed is infrastructure and isn't counted).

Reproduce with `cargo run --release --bin bench && uv run scripts/plot.py`.

### 4.2 Results

| Condition | Speculative success / time (s) / radio (B/robot/s) | Centralized success / time (s) / radio (B/robot/s) |
|---|---|---|
| clean | 10/10 · 63.2 ± 5.5 · 50 | 10/10 · 58.4 ± 2.0 · 439 |
| slow (150 ms) | 10/10 · 61.1 ± 2.8 · 51 | 10/10 · 61.0 ± 1.9 · 439 |
| lossy (20%) | 10/10 · 61.3 ± 2.8 · 70 | 10/10 · 59.4 ± 1.8 · 439 |
| slow + lossy | 10/10 · 65.8 ± 6.4 · 67 | 10/10 · 65.3 ± 2.2 · 440 |
| disrupted | 10/10 · 76.5 ± 3.1 · 51 | 10/10 · 72.6 ± 0.6 · 444 |
| 10 s edge outage | 10/10 · 62.0 ± 2.0 · 49 | 10/10 · 69.2 ± 3.3 · 422 |

![Latency sweep](img/latency.png)
![Bandwidth](img/bandwidth.png)
![Outage sweep](img/outage.png)

### 4.3 What the data supports
1. **6-9× less radio traffic for the same task.** Speculative uses 50-70 B/robot/s against about 440 for centralized, with time to goal statistically indistinguishable on clean, slow and lossy links.
2. **Latency tolerance is the decisive difference.** Speculative time to goal is flat from 0 to 1000 ms of one-way latency (60-66 s, 10/10). Centralized slows from 200 ms (74 s), takes almost 3× as long at 500 ms (165 s, 9/10), and fails every run at 1000 ms (0/10).
3. **Outages:** both modes always recover. Speculative is a few seconds faster, because robots keep working on their cached leases while centralized robots stop on the watchdog.
4. **Centralized is slightly better when disrupted** (72.6 vs 76.5 s) and has lower variance on a clean link. With 20 Hz state it reacts to kicks with full information, while speculative robots only react to the contingencies they were given.

### 4.4 What the data does *not* support (yet)
- Effects here are for **one level, one payload, 16 robots**. Generalization to other layouts and scales is untested (M4).
- Contingency probabilities are heuristics plus the learned surprise map, not computed from rollouts (M3).
- The radio model is Bernoulli loss with uniform jitter: no bursty loss, no bandwidth cap, no contention.
- The 1 s latency case is extreme for a local edge. It stands in for cellular or long-haul links rather than Wi-Fi.

### 4.5 Bugs the baseline exposed
Building a fair baseline and sweeping conditions found real defects. Several of them were hurting the method itself:

| Symptom | Root cause | Fix |
|---|---|---|
| Speculative 6/10 on lossy links | Trees sent once, with no acknowledgement | Tree acks + retransmission |
| Still failing after acks | Acks covered the plan version, not lease renewals, so leases silently expired | Acknowledge the lease (`valid_until`) |
| 650 B/robot/s at 200 ms latency | Retransmissions were rebuilt with a fresh lease, so acks never matched (a retransmit storm) | Byte-identical retransmits, RTT-adaptive timeout (Karn's rule), exponential backoff |
| Centralized 3× slower on some seeds | Late-gripping robots started at waypoint 1 and dragged the payload backwards (a bug in the *shared* brain) | Re-localize in the plan after gripping |
| Wedged payload after long outages, 200 replans | Planner refused colliding start poses; failed plans re-triggered every camera frame | Planner may move out of collision; failed plans retry on a 2 s timer |
| Payload jammed at the door forever | Stall detection used instantaneous speed, and a jiggling payload never counted as stalled | Stall = no net progress (15 cm or 3°) over a window |

Early results (before these fixes) suggested a large speed advantage for speculative control. That turned out to be mostly the shared re-localization bug, which hurt the remote loop more. The numbers above are after all fixes, for both modes.

## 5. Roadmap

**M2: make the claim measurable (done).** Centralized baseline, seed sweeps with CIs, latency and outage sweeps, plots.

**M3: real speculation**
- Replace heuristic `p` with **physics rollouts** on the edge: clone the Rapier world, inject each candidate disruption, and measure whether the nominal plan survives.
- Persist learned priors across runs (heat map, grip-loss stats).
- Smarter fallback once a lease has expired (follow the cached path cautiously rather than pure reflexes).
- Close the "disrupted" gap: speculate on lateral shoves near walls.

**M4: scale and scenarios**
- 64-256 robots, several payloads with resource conflicts, a library of layouts.
- Bursty-loss and bandwidth-capped radio models (Gilbert-Elliott).
- Byte-budget sweep: hit rate and success against `tree_budget`.

**M5: portfolio polish**
- WASM build so the demo runs in a browser.
- Edge as its own process over UDP, using the same `Link` semantics.
- Optional ROS 2 bridge.
- Workshop paper built on sections 4-5.

## 6. Open questions
- What's the best byte budget against hit rate? Sweep `tree_budget` from 64 to 512 B.
- Should robots share hits with neighbours (gossip) so one robot's surprise updates the others before the edge replies?
- Is the lease length a good knob for trading resilience against staleness?
