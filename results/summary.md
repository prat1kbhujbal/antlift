| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 82.8 ± 10.4 | 47.4 ± 1.3 | 49.9 | 0.0 |
| clean | speculative-heuristic | 10/10 | 83.0 ± 10.4 | 48.5 ± 1.2 | 60.5 | 0.0 |
| clean | centralized | 10/10 | 72.9 ± 2.3 | 438.1 ± 0.8 | 56.9 | 0.4 |
| slow | speculative | 10/10 | 78.3 ± 3.0 | 47.0 ± 1.3 | 38.9 | 0.2 |
| slow | speculative-heuristic | 10/10 | 78.5 ± 2.9 | 48.5 ± 1.3 | 53.1 | 0.2 |
| slow | centralized | 10/10 | 75.5 ± 4.4 | 438.0 ± 1.2 | 40.2 | 0.2 |
| lossy | speculative | 10/10 | 75.1 ± 2.1 | 66.7 ± 2.1 | 51.1 | 0.1 |
| lossy | speculative-heuristic | 10/10 | 75.2 ± 2.0 | 67.7 ± 1.8 | 60.4 | 0.1 |
| lossy | centralized | 10/10 | 73.8 ± 3.2 | 438.0 ± 0.7 | 51.8 | 0.1 |
| slow+lossy | speculative | 10/10 | 81.0 ± 4.3 | 64.4 ± 1.2 | 63.6 | 0.2 |
| slow+lossy | speculative-heuristic | 10/10 | 81.5 ± 4.2 | 66.1 ± 1.2 | 72.1 | 0.2 |
| slow+lossy | centralized | 10/10 | 77.0 ± 2.4 | 438.3 ± 0.6 | 44.7 | 0.0 |
| disrupted | speculative | 10/10 | 85.9 ± 7.4 | 46.4 ± 1.1 | 57.4 | 0.0 |
| disrupted | speculative-heuristic | 10/10 | 87.3 ± 8.5 | 47.3 ± 1.2 | 45.5 | 0.1 |
| disrupted | centralized | 10/10 | 77.0 ± 0.9 | 439.3 ± 0.2 | 53.6 | 0.0 |
| outage 20-30s | speculative | 10/10 | 84.1 ± 6.5 | 45.0 ± 1.0 | 67.0 | 0.6 |
| outage 20-30s | speculative-heuristic | 10/10 | 83.6 ± 7.1 | 46.3 ± 1.3 | 77.6 | 0.6 |
| outage 20-30s | centralized | 10/10 | 82.6 ± 2.3 | 390.0 ± 2.0 | 58.2 | 0.4 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 81.6 ± 6.1 | 10/10 | 81.8 ± 6.3 | 10/10 | 75.2 ± 3.8 |
| 100 | 10/10 | 75.7 ± 2.5 | 10/10 | 75.7 ± 2.5 | 10/10 | 73.6 ± 5.1 |
| 200 | 10/10 | 76.4 ± 3.7 | 10/10 | 76.3 ± 3.4 | 10/10 | 83.6 ± 2.1 |
| 300 | 10/10 | 78.8 ± 5.7 | 10/10 | 78.4 ± 4.7 | 10/10 | 139.1 ± 12.8 |
| 500 | 10/10 | 79.7 ± 6.5 | 10/10 | 79.6 ± 6.5 | 4/10 | 215.4 ± 23.4 |
| 750 | 10/10 | 80.8 ± 2.9 | 10/10 | 80.6 ± 3.0 | 2/10 | 226.2 ± 19.7 |
| 1000 | 10/10 | 86.8 ± 5.8 | 10/10 | 86.7 ± 5.8 | 0/10 | - |

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
| warehouse | speculative | 10/10 | 121.9 ± 8.6 | 3.00 / 3 | 45 | 28.1 | 4.3 | 9.6 |
| warehouse | speculative-heuristic | 10/10 | 116.0 ± 8.1 | 3.00 / 3 | 48 | 16.8 | 4.2 | 5.2 |
| warehouse | centralized | 10/10 | 118.8 ± 9.6 | 3.00 / 3 | 386 | 27.5 | 1.1 | 7.5 |
| warehouse slow | speculative | 10/10 | 129.5 ± 11.3 | 3.00 / 3 | 43 | 22.5 | 4.6 | 10.9 |
| warehouse slow | speculative-heuristic | 10/10 | 113.3 ± 6.0 | 3.00 / 3 | 48 | 18.7 | 4.4 | 2.4 |
| warehouse slow | centralized | 10/10 | 154.2 ± 17.0 | 3.00 / 3 | 371 | 21.2 | 3.2 | 0.0 |
| warehouse lossy | speculative | 10/10 | 129.8 ± 16.2 | 3.00 / 3 | 60 | 26.9 | 4.6 | 11.6 |
| warehouse lossy | speculative-heuristic | 10/10 | 111.3 ± 4.5 | 3.00 / 3 | 68 | 27.2 | 4.3 | 0.1 |
| warehouse lossy | centralized | 10/10 | 114.8 ± 8.4 | 3.00 / 3 | 386 | 24.4 | 0.5 | 0.0 |
| warehouse attrition | speculative | 10/10 | 135.0 ± 15.6 | 3.00 / 3 | 39 | 31.0 | 9.5 | 4.7 |
| warehouse attrition | speculative-heuristic | 10/10 | 128.5 ± 10.2 | 3.00 / 3 | 41 | 18.0 | 8.9 | 2.9 |
| warehouse attrition | centralized | 10/10 | 126.2 ± 10.9 | 3.00 / 3 | 343 | 23.7 | 9.1 | 4.8 |
| warehouse outage | speculative | 10/10 | 134.9 ± 12.9 | 3.00 / 3 | 40 | 22.5 | 4.3 | 3.7 |
| warehouse outage | speculative-heuristic | 10/10 | 122.5 ± 11.9 | 3.00 / 3 | 43 | 15.7 | 4.1 | 0.0 |
| warehouse outage | centralized | 10/10 | 128.9 ± 7.8 | 3.00 / 3 | 346 | 24.2 | 1.1 | 1.6 |

10 seeds per cell, 420 s limit. Time is until every payload is at its goal, over runs where that happened.
