| Condition | Control | Success | Time to goal (s) | Radio (B/robot/s) | Hits | Surprises |
|---|---|---|---|---|---|---|
| clean | speculative | 10/10 | 63.2 ± 5.5 | 50.4 ± 0.8 | 68.3 | 3.1 |
| clean | centralized | 10/10 | 58.4 ± 2.0 | 439.4 ± 1.2 | 36.6 | 0.0 |
| slow | speculative | 10/10 | 61.1 ± 2.8 | 50.9 ± 2.0 | 43.3 | 0.0 |
| slow | centralized | 10/10 | 61.0 ± 1.9 | 439.4 ± 0.7 | 35.1 | 0.0 |
| lossy | speculative | 10/10 | 61.3 ± 2.8 | 70.3 ± 1.9 | 42.6 | 0.0 |
| lossy | centralized | 10/10 | 59.4 ± 1.8 | 439.4 ± 1.2 | 39.3 | 0.0 |
| slow+lossy | speculative | 10/10 | 65.8 ± 6.4 | 67.1 ± 1.0 | 57.5 | 0.1 |
| slow+lossy | centralized | 10/10 | 65.3 ± 2.2 | 440.4 ± 0.7 | 39.4 | 0.0 |
| disrupted | speculative | 10/10 | 76.5 ± 3.1 | 50.8 ± 1.3 | 50.4 | 0.0 |
| disrupted | centralized | 10/10 | 72.6 ± 0.6 | 443.6 ± 0.4 | 28.2 | 0.0 |
| outage 20-30s | speculative | 10/10 | 62.0 ± 2.0 | 49.2 ± 1.7 | 60.7 | 0.0 |
| outage 20-30s | centralized | 10/10 | 69.2 ± 3.3 | 421.5 ± 2.1 | 35.9 | 0.1 |

10 seeds per cell (randomized start pose and radio RNG), 240 s limit. Time is over successful runs; ± is a 95% CI.

| One-way latency (ms), 10% loss | Speculative success | Speculative time (s) | Centralized success | Centralized time (s) |
|---|---|---|---|---|
| 0 | 10/10 | 60.7 ± 2.5 | 10/10 | 60.8 ± 4.2 |
| 100 | 10/10 | 59.7 ± 1.6 | 10/10 | 59.1 ± 1.3 |
| 200 | 10/10 | 66.3 ± 10.1 | 10/10 | 73.7 ± 3.3 |
| 300 | 10/10 | 66.0 ± 7.3 | 10/10 | 113.4 ± 5.3 |
| 500 | 10/10 | 62.2 ± 2.2 | 9/10 | 165.3 ± 25.3 |
| 750 | 10/10 | 66.4 ± 3.6 | 6/10 | 211.5 ± 16.2 |
| 1000 | 10/10 | 66.0 ± 2.2 | 0/10 | - |

| Edge outage at t=20 s (s) | Speculative success | Speculative time (s) | Centralized success | Centralized time (s) |
|---|---|---|---|---|
| 0 | 10/10 | 63.2 ± 5.5 | 10/10 | 58.4 ± 2.0 |
| 5 | 10/10 | 60.4 ± 2.6 | 10/10 | 64.2 ± 3.3 |
| 10 | 10/10 | 62.0 ± 2.0 | 10/10 | 69.2 ± 3.3 |
| 20 | 10/10 | 76.8 ± 4.7 | 10/10 | 79.2 ± 3.3 |
| 30 | 10/10 | 86.8 ± 4.6 | 10/10 | 89.2 ± 3.3 |
| 40 | 10/10 | 97.5 ± 4.8 | 10/10 | 99.2 ± 3.3 |
