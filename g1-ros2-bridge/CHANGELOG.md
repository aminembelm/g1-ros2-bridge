# Changelog

## 0.1.0 (2026-10-19)
- ROS 2 node `G1BridgeNode`: `/cmd_vel` subscriber; `/g1/base_twist`, `/g1/policy_time_ms`, `/joint_states` publishers.
- `LEGGED_GYM_ROOT_DIR` read from the environment (default `~/unitree_rl_gym`) instead of a hard-coded path.
- Optional `--body-frame` flag: expresses the measured base linear velocity in the body frame
  (off by default, i.e. identical to the version used for the results of the report).
- `tools/analyze_bag.py`: tracking and inference-time statistics from a rosbag2.
