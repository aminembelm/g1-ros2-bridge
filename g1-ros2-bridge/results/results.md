# Results

Conditions: simulation only, Ubuntu 22.04 on WSL2, pre-trained policy `motion.pt`, `g1.yaml` as documented.
Tests run in September 2026 on a laptop under WSL2 (loaded: simulation, viewer and recorder at the same time).

## Forward walk, command (0.5, 0, 0)

Raw output of `tools/analyze_bag.py test_forward --out tracking_vx.png`:

```text
Telemetry samples: 238  ->  4.7 s of simulated time in 100.1 s of wall-clock time
Real-time factor: 0.05  (telemetry rate 2.4 Hz; 50 Hz = real time)

Window: 3.3 s -> 4.7 s of simulated time  (71 samples)
  vx: command +0.500 m/s | measured +0.476 +/- 0.026 m/s | RMSE 0.035 m/s
  wz: command +0.000 rad/s | measured -0.020 +/- 0.080 rad/s | RMSE 0.082 rad/s

Inference time (218 samples after 20 warm-up): mean 8.55 ms | median 4.41 | p95 26.31 | p99 38.85 | max 62.77 | > 20 ms: 7.8 %
```

Reading: the mean speed is about 5 % below the command; the standard deviation is the ripple of the gait;
the mean yaw rate is close to zero (the robot walks nearly straight).


## Limits of these results

- One run, a window of only 1.4 s of simulated time (fewer than two gait periods): indicative, not statistical.
- Inference times were measured on a loaded machine (simulation, viewer, recorder, Windows); they say nothing about
  the robot's onboard computer.
- Standing and turning were only observed visually.
