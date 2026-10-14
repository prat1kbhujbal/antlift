# AntLift design

Status: v0.4. The swarm, the edge server and the network all run on one laptop, and the method is evaluated against a centralized baseline.

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
 camera 2 Hz ──▶ │ SE(2) planner ─▶ waypoints ─▶ candidates ─▶ ghost futures ─▶ per-robot trees │
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
- **Ghost futures** (`rollout.rs`): after each plan, the edge rebuilds the physics world from its camera snapshot and runs 7 futures in parallel for 10 s. Each future uses the exact trees the robots would receive: as planned, kicks of 80 and 150 N·s from either side, front robots losing grip, and 3 robots dying. A future "jams" if the payload makes no net progress for 2 s. Jam and misalignment probabilities per waypoint are then updated as a Beta-style blend, `p = (prior·2 + futures that jammed there) / (2 + futures)`. The futures are kept with the plan and drawn in the demo.
- **Roles and slots**: robots are spread evenly around the perimeter, and robots already gripping keep their spot. Robots far from the centre become steerers. Steerer trees don't get misalignment branches, because steerers are the ones fixing it.
- **Reliable delivery**: robots acknowledge each tree and lease. The edge retransmits byte-identical copies after an RTT-adaptive timeout with exponential backoff.
- **Event-driven replanning**: it replans on a surprise, a change in obstacles, the payload drifting more than 2.5 m off the path, or the payload nearing the end of the horizon (seen by the camera, so it still works if the robots' reports are lost). Otherwise it only renews leases (every 8 s, lease 12 s).

### 3.5 Multiple teams (the warehouse)
A level is a list of payloads, each with a goal and a team of robots, plus authored **choke points** (doorways only one payload may use at a time). On top of the single-team machinery, the edge:
- **plans each team separately**, treating the other payloads (plus a margin for their robots) as obstacles. It replans a team when another payload has moved more than 3 m since its last plan.
- **reserves choke points.** A team claims the next choke point on its route once it is within 20 m of it, together with any choke points chained too closely behind it to stop between, so nobody ever waits *inside* a corridor. If another team holds any of them, the plan stops a body length in front and the team queues. A reservation is released when the owner's payload has passed through, and queued teams replan straight away.
- **lends robots.** When a payload is delivered, its robots move to the undelivered teams with the fewest robots per square metre of payload, but only up to what fits around that payload. Their next tree carries the new team id, and the robot lets go and walks over.
- **plans around trouble.** The learned surprise map is a route-cost penalty, not just a contingency prior. When a team's own "as planned" ghost future jams, that spot becomes costly for the team and it replans immediately, before anything is sent.
- In-place turns check the space swept *between* planner headings, so a long payload can't plan to rotate through a gap narrower than its length.

### 3.6 `antlift` app: one-laptop runtime
- `Session` wires swarm ↔ links ↔ edge at a fixed 60 Hz sim tick.
- `EdgeRuntime::Inline` runs the edge in the same thread, deterministically (bench, tests). `EdgeRuntime::Thread` puts the edge on its own thread, talking only over channels, so planning time is real and the UI never blocks.
- Interactive UI (macroquad): drop crates, kick payloads, shake robots loose, kill robots, take the edge offline, change latency and loss, switch between speculative and centralized control (`C`), compare both side by side (`V`), and switch between the doorway and warehouse levels (`W`). Radio packets, contingency trees, risks, ghost futures and choke-point reservations are all drawn live.

## 4. Evaluation

### 4.1 Method
- **Two control modes, same brain.** In *speculative* mode each robot runs its brain on its own sensing and an edge-issued tree. In *centralized* mode the edge runs **the identical brain code** for every robot, using streamed robot state (20 Hz) and a 20 Hz camera feed (dead-reckoned forward to cover camera latency), and streams commands back at 20 Hz. To give the baseline a fair shot:
  - while a robot is approaching the payload, the edge sends **velocity setpoints** and the robot closes its velocity loop locally at 60 Hz;
  - robots report their exact grip point;
  - commands are held until the next one arrives, and a 0.3 s watchdog stops a robot if commands stop coming.
  So the only difference between the modes is *where decisions are made*.
- **Conditions:** clean (40 ms), slow (150 ± 50 ms), lossy (20% loss), slow+lossy, disrupted (two kicks, a crate dropped in the doorway, front robots shaken loose), and a 10 s edge outage.
- **Sweeps:** one-way latency from 0 to 1000 ms (10% loss), and edge outage length from 0 to 40 s.
- **Seeds:** 10 per cell. Each seed randomizes the payload's start pose (±1 m, ±0.4 rad) and the radio's random generator. 720 runs take about 4 minutes on 20 cores (ghost futures with 44 robots dominate).
- **Metrics:** success within 240 s, time to goal (mean ± 95% CI over successes), and robot radio bytes per robot per second in both directions (the camera feed is infrastructure and isn't counted).

Reproduce with `cargo run --release --bin bench && uv run scripts/plot.py`.

### 4.2 Results

Three variants: **speculative** (ghost futures), **speculative-heuristic** (the same, with ghost futures off), and **centralized**. Cells are success · time to goal (s) · radio (B/robot/s).

| Condition | Speculative | Speculative, heuristic | Centralized |
|---|---|---|---|
| clean | 10/10 · 67.4 ± 4.2 · 47 | 10/10 · 66.4 ± 3.1 · 49 | 10/10 · 61.3 ± 1.4 · 442 |
| slow (150 ms) | 10/10 · 68.7 ± 5.4 · 47 | 9/10 · 66.8 ± 2.9 · 49 | 10/10 · 68.6 ± 2.3 · 444 |
| lossy (20%) | 10/10 · 66.6 ± 4.3 · 66 | 10/10 · 66.7 ± 2.5 · 69 | 10/10 · 61.2 ± 2.5 · 442 |
| slow + lossy | 10/10 · 73.3 ± 11.4 · 63 | 9/10 · 71.1 ± 6.4 · 66 | 10/10 · 71.0 ± 1.1 · 444 |
| disrupted | 10/10 · 78.0 ± 0.8 · 48 | 10/10 · 81.2 ± 4.2 · 51 | 10/10 · 75.6 ± 1.3 · 446 |
| 10 s edge outage | 10/10 · 67.3 ± 3.8 · 45 | 10/10 · 68.5 ± 1.9 · 47 | 10/10 · 70.6 ± 1.5 · 424 |

Full tables, including both sweeps for all three variants, are in `results/summary.md`.

![Latency sweep](img/latency.png)
![Bandwidth](img/bandwidth.png)
![Outage sweep](img/outage.png)

### 4.3 What the data supports
1. **6-9× less radio traffic for the same task.** Speculative uses 50-70 B/robot/s against about 440 for centralized, with time to goal statistically indistinguishable on clean, slow and lossy links.
2. **Latency tolerance is the decisive difference.** Speculative time to goal stays within 64-75 s from 0 to 1000 ms of one-way latency (10/10 at every point). Centralized slows from 200 ms (81 s), takes about 2.5× as long at 500 ms (164 s), and succeeds in only 5/10 runs at 750 ms and 1/10 at 1000 ms.
3. **Outages:** both modes always recover. Speculative is a few seconds faster, because robots keep working on their cached leases while centralized robots stop on the watchdog.
4. **Ghost futures did not reliably change outcomes, on either level.** On the doorway, time and success are within noise of heuristic priors. Ghost futures have fewer surprises on the slow links (0 vs 12.6 and 3.2 vs 12.8), but the success gap there is one run. In the warehouse (section 4.6) results go both ways. Simulating futures costs compute: about 25 ms per plan per team, and most of the benchmark's CPU.
5. **Centralized is slightly better when disrupted** (75.6 vs 78.0 s) and has lower variance on a clean link. With 20 Hz state it reacts to kicks with full information, while speculative robots only react to the contingencies they were given.

### 4.4 What the data does *not* support (yet)
- Two levels: one doorway with one payload, and one warehouse with three. Larger swarms and other layouts are untested.
- Ghost futures are 7 hand-chosen disruptions, simulated with perfect knowledge of the snapshot. Real perception noise and model mismatch between the edge's simulator and the world aren't modelled, because the "real" world here is the same simulator.
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

The warehouse exposed more:

| Symptom | Root cause | Fix |
|---|---|---|
| Pallet never moved | Level bug: the 3.5 m staging doors were too narrow for the pallet's circle-chain footprint (3.44 m needed) | 4 m staging doors |
| 27 robots stuck on reflexes | Lent robots exceeded the space around their new payload, so they never got a grip slot (and so never a tree) | Capacity-limited lending; gap-filling slot search |
| Beam jammed for 200 s, all 7 ghost futures jamming | Planner let a 7 m payload rotate in place through a 3.7 m gap (it only checked discrete headings), and surprise-triggered replans reset every recovery | Swept-turn checks; replan around a jam the "as planned" future shows; surprises replan at most every 2 s |
| Teams queuing for doors they didn't use | Choke-point zones extended 3 m either side of the wall, so passing routes clipped them | Zones cover just the doorway (±1.5 m) |

The last fix also removed an apparent ghost-futures advantage (48 vs 44 of 50) that came from the over-wide zones causing extra jams, not from the method.

Early results (before these fixes) suggested a large speed advantage for speculative control. That turned out to be mostly the shared re-localization bug, which hurt the remote loop more. The numbers above are after all fixes, for both modes.

### 4.6 Warehouse: three teams, shared doors

Same three variants, 10 seeds per condition (start poses jittered ±0.5 m, ±0.2 rad), 420 s limit. Success means all three payloads were delivered, and time is when the last one arrived.

| Condition | Speculative | Speculative, heuristic | Centralized |
|---|---|---|---|
| clean | 10/10 · 130.8 ± 10.1 · 55 | 10/10 · 126.2 ± 12.5 · 58 | 10/10 · 118.3 ± 8.4 · 448 |
| slow (150 ms) | 9/10 · 137.8 ± 15.0 · 54 | 9/10 · 125.5 ± 5.9 · 58 | 10/10 · 148.1 ± 14.1 · 446 |
| lossy (20%) | 10/10 · 132.6 ± 17.6 · 77 | 10/10 · 134.7 ± 14.3 · 82 | 10/10 · 125.6 ± 17.5 · 447 |
| attrition (1 in 4 robots die at 40 s) | 9/10 · 140.3 ± 8.5 · 42 | 10/10 · 148.0 ± 15.5 · 47 | 10/10 · 134.3 ± 7.4 · 367 |
| 15 s edge outage at 40 s | 9/10 · 136.4 ± 11.3 · 50 | 9/10 · 136.0 ± 13.3 · 52 | 10/10 · 134.7 ± 10.8 · 432 |

![Warehouse](img/warehouse.png)

What it shows:
1. **Coordination works in every variant.** Teams queue for doors (20-50 waits per run, mostly short re-checks), nobody deadlocks, and delivered teams lend 8-15 robots.
2. **Bandwidth still favours speculative by about 8×** (42-82 vs 367-448 B/robot/s).
3. **Centralized is the most reliable here (50/50, against 47/50 and 48/50).** All five speculative failures are the same: a payload wedges inside the dispatch doorway slightly misaligned, and the robots' back-off and square-up recoveries can't free it. Centralized re-reads the full state at 20 Hz and doesn't wedge. Recovering from a wedge is the main weakness the warehouse exposed.
4. **Ghost futures:** no reliable effect on success or time, and in four of five conditions they had *more* surprises than heuristic priors (for example 65 vs 28 per run on the slow link). Only the clean run favours them (0 vs 3). Replanning around predicted jams sends payloads on longer, less familiar routes, and that doesn't pay off here.

### 4.7 Route smoothing: an A/B test that said no

The grid planner's routes zigzag: on the doorway level, 18 direction changes and 4 stops to turn in place for a 31.8 m route. `Grid::smooth` shortcuts them in open space and turns while moving. It never shortcuts stretches with under 0.3 m of slack, and never gives up clearance the grid path had (up to 0.5 m). It cuts that route to 3 direction changes and no turn stops, and the warehouse routes from 25-39 direction changes to 9-15.

It does not help delivery. Warehouse, 5 conditions × 30 seeds per variant (`cargo run --release --example ab_smooth -- 30`):

| Variant | Smoothing off | Smoothing on |
|---|---|---|
| Speculative, ghost futures | 144/150 · 137.9 s · 11.9 surprises | 140/150 · 139.0 s · 42.8 surprises |
| Speculative, heuristic | 144/150 · 133.8 s · 13.7 surprises | 133/150 · 134.9 s · 39.5 surprises |
| Centralized | 149/150 · 131.7 s · 3.1 surprises | 149/150 · 128.3 s · 1.4 surprises |

Smoothing roughly triples surprises for speculative control and lowers its success (clearly for heuristic priors, within noise for ghost futures), with no speed gain. Only the centralized baseline gets slightly faster. So route geometry wasn't what limited performance: speculative trees appear to rely on the grid path's denser waypoints and explicit turns. Smoothing stays available (`EdgeConfig::smooth`) but is off by default, and every other result in this document is with it off. Two early attempts were worse still: a 0.15 m margin pulled routes against door frames, and a margin taken from a wedged start let shortcuts pass through walls.

## 5. Roadmap

**M2: make the claim measurable (done).** Centralized baseline, seed sweeps with CIs, latency and outage sweeps, plots.

**M3: real speculation (in progress)**
- Done: **ghost futures**, physics rollouts on the edge that set contingency probabilities. Measured: no outcome change on the doorway level (section 4.3).
- Next: harder layouts where heuristics are wrong; sim-to-real gap (perturb the edge's simulator parameters relative to the world's); calibration metrics (Brier score of predicted vs observed jams).
- Persist learned priors across runs (heat map, grip-loss stats).
- Smarter fallback once a lease has expired (follow the cached path cautiously rather than pure reflexes).
- Close the "disrupted" gap: speculate on lateral shoves near walls.

**M4: scale and scenarios (in progress)**
- Done: the three-team warehouse with choke-point reservations, deadlock-free chaining and robot lending.
- Next: wedge recovery in doorways (robots release and re-grip to squeeze through, or a larger planned back-off); 64-256 robots; more layouts.
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
