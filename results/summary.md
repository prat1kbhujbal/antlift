| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 82.8 ± 10.4 | 47.4 ± 1.3 | 49.9 | 0.0 |
| clean | speculative-heuristic | 10/10 | 83.0 ± 10.4 | 48.5 ± 1.2 | 60.5 | 0.0 |
| clean | centralized | 10/10 | 74.3 ± 3.2 | 446.9 ± 0.6 | 51.6 | 0.5 |
| slow | speculative | 10/10 | 78.3 ± 3.0 | 47.0 ± 1.3 | 38.9 | 0.2 |
| slow | speculative-heuristic | 10/10 | 78.5 ± 2.9 | 48.5 ± 1.3 | 53.1 | 0.2 |
| slow | centralized | 10/10 | 74.8 ± 3.3 | 446.5 ± 0.6 | 46.0 | 0.2 |
| lossy | speculative | 10/10 | 75.1 ± 2.1 | 66.7 ± 2.1 | 51.1 | 0.1 |
| lossy | speculative-heuristic | 10/10 | 75.2 ± 2.0 | 67.7 ± 1.8 | 60.4 | 0.1 |
| lossy | centralized | 10/10 | 71.9 ± 2.5 | 447.1 ± 0.5 | 46.0 | 0.2 |
| slow+lossy | speculative | 10/10 | 81.0 ± 4.3 | 64.4 ± 1.2 | 63.6 | 0.2 |
| slow+lossy | speculative-heuristic | 10/10 | 81.5 ± 4.2 | 66.1 ± 1.2 | 72.1 | 0.2 |
| slow+lossy | centralized | 10/10 | 78.5 ± 3.8 | 447.0 ± 0.6 | 47.0 | 0.3 |
| disrupted | speculative | 10/10 | 85.9 ± 7.4 | 46.4 ± 1.1 | 57.4 | 0.0 |
| disrupted | speculative-heuristic | 10/10 | 87.3 ± 8.5 | 47.3 ± 1.2 | 45.5 | 0.1 |
| disrupted | centralized | 10/10 | 78.1 ± 2.4 | 447.5 ± 0.5 | 45.4 | 0.0 |
| outage 20-30s | speculative | 10/10 | 84.1 ± 6.5 | 45.0 ± 1.0 | 67.0 | 0.6 |
| outage 20-30s | speculative-heuristic | 10/10 | 83.6 ± 7.1 | 46.3 ± 1.3 | 77.6 | 0.6 |
| outage 20-30s | centralized | 10/10 | 84.8 ± 4.2 | 431.9 ± 1.0 | 57.3 | 0.2 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 81.6 ± 6.1 | 10/10 | 81.8 ± 6.3 | 10/10 | 73.7 ± 3.9 |
| 100 | 10/10 | 75.7 ± 2.5 | 10/10 | 75.7 ± 2.5 | 10/10 | 75.2 ± 5.7 |
| 200 | 10/10 | 76.4 ± 3.7 | 10/10 | 76.3 ± 3.4 | 10/10 | 84.3 ± 3.3 |
| 300 | 10/10 | 78.8 ± 5.7 | 10/10 | 78.4 ± 4.7 | 10/10 | 136.1 ± 7.7 |
| 500 | 10/10 | 79.7 ± 6.5 | 10/10 | 79.6 ± 6.5 | 5/10 | 210.9 ± 13.3 |
| 750 | 10/10 | 80.8 ± 2.9 | 10/10 | 80.6 ± 3.0 | 2/10 | 239.1 ± 0.4 |
| 1000 | 10/10 | 86.8 ± 5.8 | 10/10 | 86.7 ± 5.8 | 0/10 | - |

| Edge outage at t=20 s (s) | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 82.8 ± 10.4 | 10/10 | 83.0 ± 10.4 | 10/10 | 74.3 ± 3.2 |
| 5 | 10/10 | 82.8 ± 10.4 | 10/10 | 83.0 ± 10.4 | 10/10 | 79.8 ± 4.2 |
| 10 | 10/10 | 84.1 ± 6.5 | 10/10 | 83.6 ± 7.1 | 10/10 | 84.8 ± 4.2 |
| 20 | 10/10 | 93.9 ± 6.1 | 10/10 | 93.8 ± 5.8 | 10/10 | 94.8 ± 4.2 |
| 30 | 10/10 | 102.3 ± 5.5 | 10/10 | 102.4 ± 5.5 | 10/10 | 104.8 ± 4.2 |
| 40 | 10/10 | 115.1 ± 8.7 | 10/10 | 114.7 ± 8.1 | 10/10 | 115.4 ± 4.8 |

### Warehouse (3 payloads, 44 robots, shared doors)

| Condition | Control | All delivered | Time (s) | Payloads delivered | Radio (B/robot/s) | Door waits | Robots lent | Surprises |
|---|---|---|---|---|---|---|---|---|
| warehouse | speculative | 10/10 | 122.0 ± 8.5 | 3.00 / 3 | 47 | 28.1 | 4.3 | 8.2 |
| warehouse | speculative-heuristic | 10/10 | 116.3 ± 8.2 | 3.00 / 3 | 50 | 16.8 | 4.2 | 5.2 |
| warehouse | centralized | 9/10 | 119.0 ± 11.8 | 2.80 / 3 | 448 | 25.2 | 1.6 | 93.0 |
| warehouse slow | speculative | 10/10 | 126.7 ± 9.8 | 3.00 / 3 | 47 | 22.5 | 4.6 | 7.6 |
| warehouse slow | speculative-heuristic | 10/10 | 114.5 ± 7.0 | 3.00 / 3 | 51 | 18.7 | 4.4 | 0.1 |
| warehouse slow | centralized | 10/10 | 165.6 ± 20.4 | 3.00 / 3 | 445 | 17.0 | 2.8 | 0.0 |
| warehouse lossy | speculative | 10/10 | 128.3 ± 14.2 | 3.00 / 3 | 65 | 27.7 | 4.6 | 8.5 |
| warehouse lossy | speculative-heuristic | 10/10 | 110.9 ± 4.4 | 3.00 / 3 | 71 | 27.2 | 4.3 | 0.0 |
| warehouse lossy | centralized | 10/10 | 150.0 ± 54.9 | 3.00 / 3 | 449 | 30.4 | 0.5 | 33.6 |
| warehouse attrition | speculative | 10/10 | 135.0 ± 15.5 | 3.00 / 3 | 40 | 31.0 | 9.5 | 4.7 |
| warehouse attrition | speculative-heuristic | 10/10 | 128.5 ± 10.2 | 3.00 / 3 | 42 | 18.0 | 8.9 | 2.6 |
| warehouse attrition | centralized | 10/10 | 140.3 ± 22.3 | 3.00 / 3 | 366 | 19.9 | 9.0 | 2.4 |
| warehouse outage | speculative | 10/10 | 134.2 ± 11.9 | 3.00 / 3 | 43 | 22.5 | 4.3 | 3.7 |
| warehouse outage | speculative-heuristic | 10/10 | 122.4 ± 11.8 | 3.00 / 3 | 46 | 15.7 | 4.1 | 0.0 |
| warehouse outage | centralized | 10/10 | 140.8 ± 18.0 | 3.00 / 3 | 432 | 26.9 | 1.6 | 6.0 |

10 seeds per cell, 420 s limit. Time is until every payload is at its goal, over runs where that happened.
