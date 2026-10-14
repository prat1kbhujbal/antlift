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
| warehouse | speculative | 10/10 | 148.1 ± 20.9 | 3.00 / 3 | 48 | 16.3 | 3.9 | 8.5 |
| warehouse | speculative-heuristic | 10/10 | 116.3 ± 8.2 | 3.00 / 3 | 50 | 16.8 | 4.2 | 5.2 |
| warehouse | centralized | 10/10 | 132.2 ± 10.8 | 3.00 / 3 | 449 | 14.7 | 1.1 | 3.6 |
| warehouse slow | speculative | 10/10 | 141.7 ± 21.4 | 3.00 / 3 | 48 | 20.5 | 5.1 | 1.8 |
| warehouse slow | speculative-heuristic | 10/10 | 114.5 ± 7.0 | 3.00 / 3 | 51 | 18.7 | 4.4 | 0.1 |
| warehouse slow | centralized | 10/10 | 174.6 ± 18.7 | 3.00 / 3 | 444 | 19.3 | 2.3 | 0.0 |
| warehouse lossy | speculative | 10/10 | 152.2 ± 35.0 | 3.00 / 3 | 63 | 21.6 | 5.0 | 48.7 |
| warehouse lossy | speculative-heuristic | 10/10 | 110.9 ± 4.4 | 3.00 / 3 | 71 | 27.2 | 4.3 | 0.0 |
| warehouse lossy | centralized | 10/10 | 147.5 ± 19.8 | 3.00 / 3 | 447 | 27.9 | 0.4 | 13.4 |
| warehouse attrition | speculative | 8/10 | 144.0 ± 19.2 | 2.60 / 3 | 40 | 22.8 | 11.2 | 109.3 |
| warehouse attrition | speculative-heuristic | 10/10 | 128.5 ± 10.2 | 3.00 / 3 | 42 | 18.0 | 8.9 | 2.6 |
| warehouse attrition | centralized | 10/10 | 137.2 ± 12.3 | 3.00 / 3 | 366 | 15.9 | 8.5 | 3.2 |
| warehouse outage | speculative | 10/10 | 148.2 ± 23.1 | 3.00 / 3 | 44 | 11.8 | 4.3 | 4.4 |
| warehouse outage | speculative-heuristic | 10/10 | 122.4 ± 11.8 | 3.00 / 3 | 46 | 15.7 | 4.1 | 0.0 |
| warehouse outage | centralized | 10/10 | 142.4 ± 10.0 | 3.00 / 3 | 434 | 25.3 | 0.7 | 0.0 |

10 seeds per cell, 420 s limit. Time is until every payload is at its goal, over runs where that happened.
