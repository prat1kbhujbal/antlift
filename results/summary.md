| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 82.8 ± 10.4 | 47.6 ± 1.3 | 49.9 | 0.0 |
| clean | speculative-heuristic | 10/10 | 83.0 ± 10.4 | 48.7 ± 1.3 | 60.5 | 0.0 |
| clean | centralized | 10/10 | 72.9 ± 2.3 | 438.1 ± 0.8 | 56.9 | 0.4 |
| slow | speculative | 10/10 | 78.3 ± 3.0 | 47.1 ± 1.3 | 38.9 | 0.2 |
| slow | speculative-heuristic | 10/10 | 78.5 ± 2.9 | 48.6 ± 1.3 | 53.1 | 0.2 |
| slow | centralized | 10/10 | 75.5 ± 4.4 | 438.0 ± 1.2 | 40.2 | 0.2 |
| lossy | speculative | 10/10 | 75.2 ± 2.0 | 66.8 ± 1.8 | 52.6 | 0.1 |
| lossy | speculative-heuristic | 10/10 | 75.2 ± 1.9 | 68.0 ± 1.7 | 60.4 | 0.1 |
| lossy | centralized | 10/10 | 73.8 ± 3.2 | 438.0 ± 0.7 | 51.8 | 0.1 |
| slow+lossy | speculative | 10/10 | 81.0 ± 4.2 | 64.4 ± 1.0 | 64.3 | 0.2 |
| slow+lossy | speculative-heuristic | 10/10 | 81.3 ± 4.2 | 65.9 ± 1.1 | 72.6 | 0.2 |
| slow+lossy | centralized | 10/10 | 77.0 ± 2.4 | 438.3 ± 0.6 | 44.7 | 0.0 |
| disrupted | speculative | 10/10 | 85.9 ± 7.4 | 46.5 ± 1.1 | 57.4 | 0.0 |
| disrupted | speculative-heuristic | 10/10 | 87.3 ± 8.5 | 47.4 ± 1.2 | 45.5 | 0.1 |
| disrupted | centralized | 10/10 | 77.0 ± 0.9 | 439.3 ± 0.2 | 53.6 | 0.0 |
| outage 20-30s | speculative | 10/10 | 84.1 ± 6.5 | 45.1 ± 1.0 | 67.0 | 0.6 |
| outage 20-30s | speculative-heuristic | 10/10 | 83.6 ± 7.1 | 46.5 ± 1.3 | 77.6 | 0.6 |
| outage 20-30s | centralized | 10/10 | 82.6 ± 2.3 | 390.0 ± 2.0 | 58.2 | 0.4 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 81.6 ± 6.1 | 10/10 | 81.8 ± 6.3 | 10/10 | 75.2 ± 3.8 |
| 100 | 10/10 | 75.9 ± 2.9 | 10/10 | 75.8 ± 2.6 | 10/10 | 73.6 ± 5.1 |
| 200 | 10/10 | 76.6 ± 3.8 | 10/10 | 76.4 ± 3.6 | 10/10 | 83.6 ± 2.1 |
| 300 | 10/10 | 78.9 ± 5.7 | 10/10 | 78.5 ± 4.7 | 10/10 | 139.1 ± 12.8 |
| 500 | 10/10 | 82.5 ± 11.2 | 10/10 | 82.4 ± 11.1 | 4/10 | 215.4 ± 23.4 |
| 750 | 10/10 | 81.0 ± 2.9 | 10/10 | 80.8 ± 2.9 | 2/10 | 225.6 ± 21.0 |
| 1000 | 10/10 | 86.6 ± 5.8 | 10/10 | 86.5 ± 5.8 | 0/10 | - |

| Edge outage at t=20 s (s) | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 82.8 ± 10.4 | 10/10 | 83.0 ± 10.4 | 10/10 | 72.9 ± 2.3 |
| 5 | 10/10 | 82.8 ± 10.4 | 10/10 | 83.0 ± 10.4 | 10/10 | 77.7 ± 2.3 |
| 10 | 10/10 | 84.1 ± 6.5 | 10/10 | 83.6 ± 7.1 | 10/10 | 82.6 ± 2.3 |
| 20 | 10/10 | 93.9 ± 6.1 | 10/10 | 93.8 ± 5.8 | 10/10 | 92.6 ± 2.3 |
| 30 | 10/10 | 102.3 ± 5.5 | 10/10 | 102.4 ± 5.5 | 10/10 | 102.5 ± 2.3 |
| 40 | 10/10 | 115.1 ± 8.7 | 10/10 | 114.7 ± 8.1 | 10/10 | 112.4 ± 2.3 |

### Warehouse (3 payloads, 44 robots, shared doors)

| Condition | Control | All delivered | Time (s) | Payloads delivered | Radio (B/robot/s) | Door waits | Robots lent | Surprises |
|---|---|---|---|---|---|---|---|---|
| warehouse | speculative | 10/10 | 137.1 ± 12.2 | 3.00 / 3 | 46 | 8.5 | 3.4 | 0.1 |
| warehouse | speculative-heuristic | 10/10 | 160.9 ± 19.8 | 3.00 / 3 | 44 | 28.3 | 5.7 | 1.2 |
| warehouse | centralized | 10/10 | 144.0 ± 14.8 | 3.00 / 3 | 346 | 11.0 | 0.8 | 0.0 |
| warehouse slow | speculative | 10/10 | 139.1 ± 13.0 | 3.00 / 3 | 45 | 8.4 | 4.8 | 0.0 |
| warehouse slow | speculative-heuristic | 10/10 | 154.4 ± 17.1 | 3.00 / 3 | 45 | 23.5 | 5.6 | 0.7 |
| warehouse slow | centralized | 10/10 | 168.5 ± 19.0 | 3.00 / 3 | 333 | 3.4 | 2.8 | 0.0 |
| warehouse lossy | speculative | 10/10 | 141.0 ± 12.4 | 3.00 / 3 | 63 | 10.0 | 3.9 | 0.6 |
| warehouse lossy | speculative-heuristic | 10/10 | 175.6 ± 16.9 | 3.00 / 3 | 60 | 39.4 | 5.8 | 1.1 |
| warehouse lossy | centralized | 10/10 | 127.3 ± 4.3 | 3.00 / 3 | 354 | 3.1 | 0.0 | 0.0 |
| warehouse attrition | speculative | 10/10 | 153.8 ± 17.9 | 3.00 / 3 | 41 | 6.9 | 8.3 | 0.1 |
| warehouse attrition | speculative-heuristic | 10/10 | 148.3 ± 14.0 | 3.00 / 3 | 41 | 19.9 | 13.3 | 0.6 |
| warehouse attrition | centralized | 10/10 | 152.7 ± 9.3 | 3.00 / 3 | 311 | 11.1 | 7.9 | 0.0 |
| warehouse outage | speculative | 10/10 | 151.6 ± 13.7 | 3.00 / 3 | 43 | 12.4 | 4.1 | 0.1 |
| warehouse outage | speculative-heuristic | 10/10 | 131.1 ± 5.4 | 3.00 / 3 | 46 | 18.7 | 5.5 | 19.2 |
| warehouse outage | centralized | 10/10 | 155.0 ± 9.7 | 3.00 / 3 | 315 | 10.5 | 1.0 | 0.0 |

10 seeds per cell, 420 s limit. Time is until every payload is at its goal, over runs where that happened.
