| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 67.4 ± 4.2 | 47.2 ± 0.9 | 60.3 | 0.0 |
| clean | speculative-heuristic | 10/10 | 66.4 ± 3.1 | 48.7 ± 1.0 | 59.5 | 0.0 |
| clean | centralized | 10/10 | 61.3 ± 1.4 | 442.0 ± 0.8 | 61.0 | 0.1 |
| slow | speculative | 10/10 | 68.7 ± 5.4 | 46.7 ± 1.1 | 56.9 | 0.0 |
| slow | speculative-heuristic | 9/10 | 66.8 ± 2.9 | 49.3 ± 2.2 | 153.9 | 12.6 |
| slow | centralized | 10/10 | 68.6 ± 2.3 | 443.8 ± 0.8 | 57.5 | 0.0 |
| lossy | speculative | 10/10 | 66.6 ± 4.3 | 66.1 ± 2.3 | 60.0 | 0.0 |
| lossy | speculative-heuristic | 10/10 | 66.7 ± 2.5 | 68.5 ± 1.3 | 69.9 | 0.0 |
| lossy | centralized | 10/10 | 61.2 ± 2.5 | 441.8 ± 0.7 | 69.3 | 0.0 |
| slow+lossy | speculative | 10/10 | 73.3 ± 11.4 | 63.1 ± 1.7 | 81.0 | 3.2 |
| slow+lossy | speculative-heuristic | 9/10 | 71.1 ± 6.4 | 65.6 ± 2.0 | 161.9 | 12.8 |
| slow+lossy | centralized | 10/10 | 71.0 ± 1.1 | 443.9 ± 0.6 | 50.1 | 0.0 |
| disrupted | speculative | 10/10 | 78.0 ± 0.8 | 47.5 ± 0.4 | 61.6 | 0.0 |
| disrupted | speculative-heuristic | 10/10 | 81.2 ± 4.2 | 50.6 ± 0.9 | 72.2 | 0.0 |
| disrupted | centralized | 10/10 | 75.6 ± 1.3 | 445.6 ± 0.5 | 58.8 | 0.0 |
| outage 20-30s | speculative | 10/10 | 67.3 ± 3.8 | 45.3 ± 1.1 | 57.0 | 0.0 |
| outage 20-30s | speculative-heuristic | 10/10 | 68.5 ± 1.9 | 46.5 ± 1.1 | 59.0 | 0.8 |
| outage 20-30s | centralized | 10/10 | 70.6 ± 1.5 | 424.1 ± 1.1 | 62.6 | 0.0 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 69.3 ± 6.4 | 10/10 | 67.4 ± 2.7 | 10/10 | 67.1 ± 1.9 |
| 100 | 10/10 | 64.1 ± 2.4 | 10/10 | 64.3 ± 2.2 | 10/10 | 59.6 ± 1.4 |
| 200 | 10/10 | 66.1 ± 2.8 | 10/10 | 65.9 ± 2.5 | 10/10 | 80.5 ± 1.9 |
| 300 | 10/10 | 75.1 ± 8.8 | 10/10 | 71.6 ± 6.0 | 10/10 | 116.7 ± 7.0 |
| 500 | 10/10 | 70.4 ± 7.0 | 10/10 | 67.7 ± 6.4 | 10/10 | 164.3 ± 14.5 |
| 750 | 10/10 | 70.8 ± 3.2 | 9/10 | 69.1 ± 2.9 | 5/10 | 200.1 ± 19.1 |
| 1000 | 10/10 | 73.7 ± 3.3 | 9/10 | 74.3 ± 3.8 | 1/10 | 171.4 ± 0.0 |

| Edge outage at t=20 s (s) | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 67.4 ± 4.2 | 10/10 | 66.4 ± 3.1 | 10/10 | 61.3 ± 1.4 |
| 5 | 10/10 | 67.4 ± 4.2 | 10/10 | 66.4 ± 3.1 | 10/10 | 65.6 ± 1.5 |
| 10 | 10/10 | 67.3 ± 3.8 | 10/10 | 68.5 ± 1.9 | 10/10 | 70.6 ± 1.5 |
| 20 | 10/10 | 75.8 ± 2.7 | 10/10 | 89.4 ± 15.2 | 10/10 | 80.6 ± 1.5 |
| 30 | 10/10 | 84.0 ± 2.2 | 9/10 | 90.5 ± 9.4 | 10/10 | 90.6 ± 1.5 |
| 40 | 10/10 | 94.8 ± 2.1 | 9/10 | 100.8 ± 8.7 | 10/10 | 100.6 ± 1.5 |

### Warehouse (3 payloads, 44 robots, shared doors)

| Condition | Control | All delivered | Time (s) | Payloads delivered | Radio (B/robot/s) | Door waits | Robots lent | Surprises |
|---|---|---|---|---|---|---|---|---|
| warehouse | speculative | 10/10 | 130.8 ± 10.1 | 3.00 / 3 | 55 | 27.6 | 8.6 | 0.0 |
| warehouse | speculative-heuristic | 10/10 | 126.2 ± 12.5 | 3.00 / 3 | 58 | 22.2 | 8.2 | 3.0 |
| warehouse | centralized | 10/10 | 118.3 ± 8.4 | 3.00 / 3 | 448 | 42.2 | 8.5 | 0.0 |
| warehouse slow | speculative | 9/10 | 137.8 ± 15.0 | 2.90 / 3 | 54 | 29.4 | 8.6 | 65.2 |
| warehouse slow | speculative-heuristic | 9/10 | 125.5 ± 5.9 | 2.90 / 3 | 58 | 21.2 | 8.8 | 27.9 |
| warehouse slow | centralized | 10/10 | 148.1 ± 14.1 | 3.00 / 3 | 446 | 35.4 | 8.1 | 0.0 |
| warehouse lossy | speculative | 10/10 | 132.6 ± 17.6 | 3.00 / 3 | 77 | 39.1 | 8.8 | 7.0 |
| warehouse lossy | speculative-heuristic | 10/10 | 134.7 ± 14.3 | 3.00 / 3 | 82 | 23.6 | 8.6 | 5.4 |
| warehouse lossy | centralized | 10/10 | 125.6 ± 17.5 | 3.00 / 3 | 447 | 36.9 | 8.4 | 3.7 |
| warehouse attrition | speculative | 9/10 | 140.3 ± 8.5 | 2.90 / 3 | 42 | 26.3 | 14.1 | 17.0 |
| warehouse attrition | speculative-heuristic | 10/10 | 148.0 ± 15.5 | 3.00 / 3 | 47 | 21.5 | 14.4 | 7.0 |
| warehouse attrition | centralized | 10/10 | 134.3 ± 7.4 | 3.00 / 3 | 367 | 27.1 | 12.8 | 0.0 |
| warehouse outage | speculative | 9/10 | 136.4 ± 11.3 | 2.90 / 3 | 50 | 29.8 | 8.8 | 27.3 |
| warehouse outage | speculative-heuristic | 9/10 | 136.0 ± 13.3 | 2.90 / 3 | 52 | 24.1 | 8.9 | 15.2 |
| warehouse outage | centralized | 10/10 | 134.7 ± 10.8 | 3.00 / 3 | 432 | 49.9 | 8.8 | 0.1 |

10 seeds per cell, 420 s limit. Time is until every payload is at its goal, over runs where that happened.
