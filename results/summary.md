| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 63.9 ± 5.4 | 48.0 ± 1.1 | 73.3 | 3.0 |
| clean | speculative-heuristic | 10/10 | 63.2 ± 5.5 | 50.4 ± 0.8 | 68.3 | 3.1 |
| clean | centralized | 10/10 | 58.4 ± 2.0 | 439.3 ± 1.2 | 37.9 | 0.0 |
| slow | speculative | 10/10 | 63.1 ± 5.3 | 50.1 ± 2.8 | 65.4 | 3.0 |
| slow | speculative-heuristic | 10/10 | 61.1 ± 2.8 | 50.9 ± 2.0 | 43.3 | 0.0 |
| slow | centralized | 10/10 | 60.8 ± 1.6 | 439.4 ± 0.6 | 42.2 | 0.0 |
| lossy | speculative | 10/10 | 61.6 ± 3.3 | 66.2 ± 2.1 | 49.2 | 0.0 |
| lossy | speculative-heuristic | 10/10 | 61.3 ± 2.8 | 70.3 ± 1.9 | 42.6 | 0.0 |
| lossy | centralized | 10/10 | 59.3 ± 1.8 | 439.4 ± 1.2 | 40.7 | 0.0 |
| slow+lossy | speculative | 10/10 | 65.7 ± 6.7 | 64.5 ± 1.8 | 49.2 | 0.1 |
| slow+lossy | speculative-heuristic | 10/10 | 65.8 ± 6.4 | 67.1 ± 1.0 | 57.5 | 0.1 |
| slow+lossy | centralized | 10/10 | 65.3 ± 2.4 | 440.3 ± 0.7 | 40.0 | 0.0 |
| disrupted | speculative | 10/10 | 76.6 ± 3.2 | 47.5 ± 1.1 | 67.6 | 0.0 |
| disrupted | speculative-heuristic | 10/10 | 76.5 ± 3.1 | 50.8 ± 1.3 | 50.4 | 0.0 |
| disrupted | centralized | 10/10 | 72.8 ± 0.6 | 443.6 ± 0.4 | 58.2 | 0.0 |
| outage 20-30s | speculative | 10/10 | 62.3 ± 2.2 | 47.1 ± 1.3 | 64.1 | 0.0 |
| outage 20-30s | speculative-heuristic | 10/10 | 62.0 ± 2.0 | 49.2 ± 1.7 | 60.7 | 0.0 |
| outage 20-30s | centralized | 10/10 | 76.5 ± 17.0 | 422.4 ± 3.5 | 34.2 | 0.0 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 60.5 ± 2.5 | 10/10 | 60.7 ± 2.5 | 10/10 | 72.0 ± 25.6 |
| 100 | 10/10 | 60.3 ± 2.3 | 10/10 | 59.7 ± 1.6 | 10/10 | 59.1 ± 1.3 |
| 200 | 10/10 | 65.8 ± 9.2 | 10/10 | 66.3 ± 10.1 | 10/10 | 73.4 ± 3.4 |
| 300 | 10/10 | 66.2 ± 7.3 | 10/10 | 66.0 ± 7.3 | 10/10 | 109.5 ± 7.6 |
| 500 | 10/10 | 62.1 ± 2.1 | 10/10 | 62.2 ± 2.2 | 10/10 | 154.3 ± 12.7 |
| 750 | 10/10 | 66.3 ± 3.7 | 10/10 | 66.4 ± 3.6 | 6/10 | 195.8 ± 26.6 |
| 1000 | 10/10 | 65.9 ± 2.3 | 10/10 | 66.0 ± 2.2 | 0/10 | - |

| Edge outage at t=20 s (s) | speculative success | speculative time (s) | speculative-heuristic success | speculative-heuristic time (s) | centralized success | centralized time (s) |
|---|---|---|---|---|---|---|
| 0 | 10/10 | 63.9 ± 5.4 | 10/10 | 63.2 ± 5.5 | 10/10 | 58.4 ± 2.0 |
| 5 | 10/10 | 61.3 ± 3.0 | 10/10 | 60.4 ± 2.6 | 10/10 | 71.5 ± 17.0 |
| 10 | 10/10 | 62.3 ± 2.2 | 10/10 | 62.0 ± 2.0 | 10/10 | 76.5 ± 17.0 |
| 20 | 10/10 | 75.6 ± 4.7 | 10/10 | 76.8 ± 4.7 | 10/10 | 86.5 ± 17.0 |
| 30 | 10/10 | 85.5 ± 4.6 | 10/10 | 86.8 ± 4.6 | 10/10 | 96.5 ± 17.0 |
| 40 | 10/10 | 95.8 ± 4.5 | 10/10 | 97.5 ± 4.8 | 10/10 | 106.5 ± 17.0 |
