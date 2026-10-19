# Known issues and limitations

| # | Issue | Impact | Possible fix |
|---|---|---|---|
| 1 | **Simulation much slower than real time** under WSL2 (about 5 % in our test: about 2.4 policy steps per second instead of 50). Likely cause: viewer rendering (`viewer.sync()` at every physics step); not investigated. | Long runs; any claim about real-time behavior is unsupported. | Call `viewer.sync()` only at policy steps; test on native Linux or with a GPU. |
| 2 | **World-frame linear velocity** in `/g1/base_twist` by default (MuJoCo `qvel[0:3]` of a free joint), while `/cmd_vel` is a body-frame command. | Comparison is valid only for straight walking; wrong during turns. | Run with `--body-frame` (added after the report tests; not yet validated on a turning run). |
| 3 | **No command timeout.** | If the publisher stops, the robot keeps walking. | Zero the command if no message arrives for ~0.5 s. |
| 4 | `/joint_states` has **no joint names** and `/g1/base_twist` has no timestamp. | RViz / `robot_state_publisher` cannot use it; rely on bag timestamps. | Add `name` list; use `TwistStamped`. |
| 5 | **Single-threaded loop.** | A slow callback or inference delays the next physics step. | Multi-threaded executor (the lock is already in place). |
| 6 | **Ideal sensing:** the observation uses exact simulator states. | Optimistic compared with the real robot. | Add noise / delay. |
| 7 | **Inference time has a heavy tail** on a loaded PC (about 8 % of steps above 20 ms in our test). | Would matter on a real robot. | Measure on the robot's computer; limit threads (`torch.set_num_threads(1)`). |
| 8 | Hard-coded gait clock period (0.8 s) and no action clipping (inherited from the upstream script). | - | - |
| 9 | Simulation only; the physical robot was never used. | - | `unitree_sdk2` / `unitree_ros2` for the real robot. |
