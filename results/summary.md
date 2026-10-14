| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 66.6 ± 8.0 | 46.9 ± 1.8 | 75.2 | 3.0 |
| clean | speculative-heuristic | 10/10 | 60.4 ± 2.7 | 50.6 ± 1.1 | 50.9 | 0.0 |
| clean | centralized | 10/10 | 57.4 ± 1.3 | 438.7 ± 1.0 | 52.5 | 0.0 |
| slow | speculative | 10/10 | 64.2 ± 7.1 | 46.5 ± 2.0 | 51.1 | 0.0 |
| slow | speculative-heuristic | 10/10 | 61.0 ± 2.8 | 51.0 ± 2.0 | 50.3 | 0.0 |
| slow | centralized | 10/10 | 61.8 ± 2.4 | 439.6 ± 0.7 | 41.5 | 0.0 |
| lossy | speculative | 10/10 | 63.7 ± 4.1 | 63.7 ± 2.3 | 65.1 | 1.7 |
| lossy | speculative-heuristic | 10/10 | 61.4 ± 2.9 | 69.0 ± 2.2 | 48.8 | 0.0 |
| lossy | centralized | 10/10 | 60.4 ± 3.0 | 439.3 ± 1.3 | 48.8 | 0.2 |
| slow+lossy | speculative | 10/10 | 67.9 ± 5.8 | 62.4 ± 2.6 | 55.7 | 0.0 |
| slow+lossy | speculative-heuristic | 10/10 | 65.9 ± 6.4 | 66.5 ± 1.4 | 58.3 | 0.1 |
| slow+lossy | centralized | 10/10 | 63.8 ± 1.7 | 439.9 ± 0.8 | 34.3 | 0.0 |
| disrupted | speculative | 10/10 | 76.6 ± 3.0 | 48.0 ± 1.0 | 61.1 | 0.2 |
| disrupted | speculative-heuristic | 10/10 | 78.3 ± 2.4 | 50.2 ± 0.9 | 43.0 | 0.0 |
| disrupted | centralized | 10/10 | 75.2 ± 1.4 | 443.9 ± 0.5 | 43.3 | 0.0 |
| outage 20-30s | speculative | 10/10 | 61.7 ± 2.1 | 46.5 ± 1.5 | 67.9 | 0.0 |
| outage 20-30s | speculative-heuristic | 10/10 | 62.0 ± 2.0 | 48.5 ± 1.6 | 60.7 | 0.0 |
| outage 20-30s | centralized | 10/10 | 67.3 ± 1.3 | 420.4 ± 1.2 | 51.9 | 0.0 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 62.8 ± 3.3 | 10/10 | 61.0 ± 2.4 | 10/10 | 67.3 ± 8.4 |
| 100 | 10/10 | 65.7 ± 9.6 | 10/10 | 59.8 ± 1.6 | 10/10 | 59.6 ± 1.5 |
| 200 | 10/10 | 61.0 ± 1.8 | 10/10 | 61.5 ± 1.9 | 10/10 | 71.8 ± 2.1 |
| 300 | 10/10 | 63.4 ± 2.9 | 10/10 | 66.2 ± 7.3 | 10/10 | 109.4 ± 8.7 |
| 500 | 10/10 | 62.5 ± 1.9 | 10/10 | 62.1 ± 2.1 | 10/10 | 149.8 ± 21.7 |
| 750 | 10/10 | 65.7 ± 4.8 | 10/10 | 65.3 ± 2.8 | 5/10 | 190.6 ± 30.8 |
| 1000 | 10/10 | 65.6 ± 2.2 | 10/10 | 65.9 ± 2.1 | 0/10 | - |

| Edge outage at t=20 s (s) | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 66.6 ± 8.0 | 10/10 | 60.4 ± 2.7 | 10/10 | 57.4 ± 1.3 |
| 5 | 10/10 | 63.3 ± 6.9 | 10/10 | 60.5 ± 2.6 | 10/10 | 62.3 ± 1.3 |
| 10 | 10/10 | 61.7 ± 2.1 | 10/10 | 62.0 ± 2.0 | 10/10 | 67.3 ± 1.3 |
| 20 | 10/10 | 71.8 ± 2.5 | 10/10 | 73.6 ± 1.4 | 10/10 | 77.3 ± 1.3 |
| 30 | 10/10 | 82.2 ± 2.5 | 10/10 | 83.4 ± 1.7 | 10/10 | 87.3 ± 1.3 |
| 40 | 10/10 | 92.2 ± 2.4 | 10/10 | 93.9 ± 1.5 | 10/10 | 97.3 ± 1.3 |

### Warehouse (3 payloads, 44 robots, shared doors)

| Condition | Control | All delivered | Time (s) | Payloads delivered | Radio (B/robot/s) | Door waits | Robots lent | Surprises |
|---|---|---|---|---|---|---|---|---|
| warehouse | speculative | 10/10 | 139.2 ± 14.7 | 3.00 / 3 | 47 | 17.1 | 8.8 | 4.8 |
| warehouse | speculative-heuristic | 10/10 | 128.3 ± 15.6 | 3.00 / 3 | 48 | 16.8 | 8.9 | 5.2 |
| warehouse | centralized | 10/10 | 131.5 ± 13.4 | 3.00 / 3 | 449 | 14.7 | 8.7 | 44.5 |
| warehouse slow | speculative | 10/10 | 153.4 ± 26.2 | 3.00 / 3 | 47 | 23.9 | 9.3 | 12.6 |
| warehouse slow | speculative-heuristic | 10/10 | 116.6 ± 9.6 | 3.00 / 3 | 50 | 18.7 | 9.0 | 0.0 |
| warehouse slow | centralized | 9/10 | 166.8 ± 15.3 | 2.80 / 3 | 445 | 23.5 | 9.4 | 14.4 |
| warehouse lossy | speculative | 10/10 | 163.0 ± 55.9 | 3.00 / 3 | 63 | 21.6 | 8.8 | 64.0 |
| warehouse lossy | speculative-heuristic | 10/10 | 111.9 ± 5.0 | 3.00 / 3 | 71 | 27.2 | 8.9 | 0.0 |
| warehouse lossy | centralized | 10/10 | 142.2 ± 19.0 | 3.00 / 3 | 448 | 30.9 | 8.9 | 47.8 |
| warehouse attrition | speculative | 9/10 | 166.2 ± 33.9 | 2.70 / 3 | 40 | 27.0 | 11.6 | 53.3 |
| warehouse attrition | speculative-heuristic | 10/10 | 124.3 ± 10.3 | 3.00 / 3 | 42 | 17.6 | 9.9 | 2.6 |
| warehouse attrition | centralized | 10/10 | 132.7 ± 7.2 | 3.00 / 3 | 368 | 21.5 | 11.6 | 0.0 |
| warehouse outage | speculative | 10/10 | 147.3 ± 25.9 | 3.00 / 3 | 44 | 11.8 | 9.0 | 6.5 |
| warehouse outage | speculative-heuristic | 10/10 | 120.4 ± 9.3 | 3.00 / 3 | 46 | 15.7 | 8.9 | 0.0 |
| warehouse outage | centralized | 10/10 | 139.0 ± 7.5 | 3.00 / 3 | 434 | 21.5 | 8.8 | 37.5 |

10 seeds per cell, 420 s limit. Time is until every payload is at its goal, over runs where that happened.
