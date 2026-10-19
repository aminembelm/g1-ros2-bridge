# Reproducing the results

## 1. Record a run

Three terminals, ROS 2 sourced in each.

```bash
# Terminal 1: simulation
cd ~/unitree_rl_gym/deploy/deploy_mujoco && python3 deploy_mujoco_ros.py g1.yaml

# Terminal 2: start recording, then wait about 3 s
ros2 bag record -o test_forward /cmd_vel /g1/base_twist /g1/policy_time_ms

# Terminal 3: publish the command and leave it running
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop with Ctrl+C in terminals 3, 2, 1. Use a new bag name for each run (ROS refuses to overwrite).

## 2. Analyze

```bash
source /opt/ros/humble/setup.bash
python3 tools/analyze_bag.py test_forward --out results/tracking_vx.png
```

The script prints:
- the real-time factor (simulated seconds per wall-clock second);
- mean, standard deviation and RMSE of `vx` and `wz` over a steady-state window;
- inference-time statistics (mean, median, 95th/99th percentile, maximum, share above 20 ms);
and saves `tracking_vx.png` and `tracking_vx_inference.png`.

## 3. How to read the numbers

- **Simulated time.** The simulation can run much slower than real time (about 5 % under WSL2 in our test).
  The script rebuilds the time axis from the number of telemetry messages (20 ms each).
- **Window.** The first 2 simulated seconds after the first non-zero command are discarded (`--skip`).
  Use `--t0` and `--t1` to choose a window by hand.
- **Mean vs ripple.** Walking is periodic (0.8 s): the instantaneous velocity oscillates around its mean.
  The mean is the meaningful number; the standard deviation is the ripple.
- **Inference time** is wall-clock and depends on the machine; it was not measured on the robot's computer.

## 4. Our results

See [`../results/results.md`](../results/results.md).
