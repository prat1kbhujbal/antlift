| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 82.7 ± 10.5 | 48.4 ± 1.9 | 50.5 | 0.7 |
| clean | speculative-heuristic | 10/10 | 82.9 ± 10.4 | 49.4 ± 1.8 | 59.9 | 0.7 |
| clean | centralized | 10/10 | 72.9 ± 2.3 | 438.1 ± 0.8 | 56.9 | 0.4 |
| slow | speculative | 10/10 | 78.3 ± 3.0 | 47.3 ± 1.3 | 38.9 | 0.2 |
| slow | speculative-heuristic | 10/10 | 78.5 ± 2.9 | 48.8 ± 1.3 | 53.1 | 0.2 |
| slow | centralized | 10/10 | 75.5 ± 4.4 | 438.0 ± 1.2 | 40.2 | 0.2 |
| lossy | speculative | 10/10 | 75.2 ± 2.0 | 67.1 ± 1.8 | 50.2 | 0.1 |
| lossy | speculative-heuristic | 10/10 | 75.3 ± 1.9 | 68.0 ± 1.8 | 60.4 | 0.1 |
| lossy | centralized | 10/10 | 73.8 ± 3.2 | 438.0 ± 0.7 | 51.8 | 0.1 |
| slow+lossy | speculative | 10/10 | 81.0 ± 4.2 | 64.6 ± 1.0 | 64.3 | 0.2 |
| slow+lossy | speculative-heuristic | 10/10 | 81.3 ± 4.2 | 66.1 ± 1.1 | 72.6 | 0.2 |
| slow+lossy | centralized | 10/10 | 77.0 ± 2.4 | 438.3 ± 0.6 | 44.7 | 0.0 |
| disrupted | speculative | 10/10 | 86.1 ± 7.4 | 46.8 ± 1.3 | 60.4 | 0.0 |
| disrupted | speculative-heuristic | 10/10 | 87.5 ± 8.4 | 47.6 ± 1.3 | 46.1 | 0.1 |
| disrupted | centralized | 10/10 | 77.0 ± 0.9 | 439.3 ± 0.2 | 57.2 | 0.0 |
| outage 20-30s | speculative | 10/10 | 84.2 ± 6.5 | 45.2 ± 1.0 | 63.4 | 1.6 |
| outage 20-30s | speculative-heuristic | 10/10 | 83.7 ± 7.0 | 46.5 ± 1.4 | 74.0 | 1.6 |
| outage 20-30s | centralized | 10/10 | 82.6 ± 2.3 | 390.0 ± 2.0 | 58.2 | 0.4 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 82.5 ± 6.0 | 10/10 | 82.6 ± 6.0 | 10/10 | 75.2 ± 3.8 |
| 100 | 10/10 | 75.9 ± 2.9 | 10/10 | 75.8 ± 2.6 | 10/10 | 73.6 ± 5.1 |
| 200 | 10/10 | 77.1 ± 3.8 | 10/10 | 77.0 ± 3.5 | 10/10 | 82.7 ± 1.7 |
| 300 | 10/10 | 78.9 ± 5.7 | 10/10 | 78.5 ± 4.7 | 10/10 | 135.7 ± 11.0 |
| 500 | 10/10 | 82.5 ± 11.2 | 10/10 | 82.4 ± 11.1 | 6/10 | 214.9 ± 16.7 |
| 750 | 10/10 | 81.0 ± 2.9 | 10/10 | 80.9 ± 2.9 | 2/10 | 214.5 ± 1.5 |
| 1000 | 10/10 | 86.6 ± 5.8 | 10/10 | 86.5 ± 5.8 | 0/10 | - |

| Edge outage at t=20 s (s) | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 82.7 ± 10.5 | 10/10 | 82.9 ± 10.4 | 10/10 | 72.9 ± 2.3 |
| 5 | 10/10 | 82.7 ± 10.5 | 10/10 | 82.9 ± 10.4 | 10/10 | 77.7 ± 2.3 |
| 10 | 10/10 | 84.2 ± 6.5 | 10/10 | 83.7 ± 7.0 | 10/10 | 82.6 ± 2.3 |
| 20 | 10/10 | 93.8 ± 6.1 | 10/10 | 93.6 ± 5.9 | 10/10 | 92.6 ± 2.3 |
| 30 | 10/10 | 102.3 ± 5.5 | 10/10 | 102.4 ± 5.5 | 10/10 | 102.5 ± 2.3 |
| 40 | 10/10 | 115.1 ± 8.7 | 10/10 | 114.7 ± 8.1 | 10/10 | 112.4 ± 2.3 |

### Warehouse (3 payloads, 44 robots, shared doors)

| Condition | Control | All delivered | Time (s) | Payloads delivered | Radio (B/robot/s) | Door waits | Robots lent | Surprises |
|---|---|---|---|---|---|---|---|---|
| warehouse | speculative | 10/10 | 138.3 ± 12.3 | 3.00 / 3 | 46 | 8.5 | 3.4 | 0.1 |
| warehouse | speculative-heuristic | 10/10 | 161.2 ± 20.8 | 3.00 / 3 | 44 | 28.2 | 5.7 | 0.1 |
| warehouse | centralized | 10/10 | 138.7 ± 8.0 | 3.00 / 3 | 354 | 3.9 | 1.1 | 0.0 |
| warehouse slow | speculative | 10/10 | 137.3 ± 15.4 | 3.00 / 3 | 45 | 8.6 | 5.0 | 0.2 |
| warehouse slow | speculative-heuristic | 10/10 | 152.6 ± 18.4 | 3.00 / 3 | 46 | 23.0 | 5.6 | 1.4 |
| warehouse slow | centralized | 10/10 | 168.5 ± 19.0 | 3.00 / 3 | 333 | 3.4 | 2.8 | 0.0 |
| warehouse lossy | speculative | 10/10 | 139.2 ± 12.8 | 3.00 / 3 | 64 | 10.8 | 4.4 | 0.4 |
| warehouse lossy | speculative-heuristic | 10/10 | 173.6 ± 16.7 | 3.00 / 3 | 61 | 39.0 | 5.8 | 0.2 |
| warehouse lossy | centralized | 10/10 | 127.3 ± 4.3 | 3.00 / 3 | 354 | 3.1 | 0.0 | 0.0 |
| warehouse attrition | speculative | 10/10 | 153.9 ± 17.2 | 3.00 / 3 | 41 | 7.3 | 8.2 | 0.1 |
| warehouse attrition | speculative-heuristic | 10/10 | 146.4 ± 13.0 | 3.00 / 3 | 41 | 19.6 | 13.4 | 0.6 |
| warehouse attrition | centralized | 10/10 | 150.9 ± 8.8 | 3.00 / 3 | 312 | 10.3 | 7.7 | 0.0 |
| warehouse outage | speculative | 10/10 | 150.2 ± 12.5 | 3.00 / 3 | 43 | 12.7 | 4.1 | 0.1 |
| warehouse outage | speculative-heuristic | 10/10 | 130.5 ± 5.3 | 3.00 / 3 | 46 | 18.6 | 5.5 | 0.0 |
| warehouse outage | centralized | 10/10 | 152.5 ± 9.0 | 3.00 / 3 | 320 | 4.3 | 1.3 | 0.0 |

10 seeds per cell, 420 s limit. Time is until every payload is at its goal, over runs where that happened.
