# ROS 2 interface

Node: `g1_bridge_node` (class `G1BridgeNode`, in `src/deploy_mujoco_ros.py`).

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | input | Uses `linear.x`, `linear.y`, `angular.z`. Body-frame command. |
| `/g1/base_twist` | `geometry_msgs/msg/Twist` | output, once per policy step | Measured base velocity. Linear part: world frame by default, body frame with `--body-frame`. Angular part: body frame. |
| `/g1/policy_time_ms` | `std_msgs/msg/Float32` | output, once per policy step | Wall-clock time of `policy(obs).detach().numpy().squeeze()`. Does not include observation assembly. |
| `/joint_states` | `sensor_msgs/msg/JointState` | output, once per policy step | 12 leg joints, positions and velocities. **No `name` field yet.** |

QoS: default (reliable, depth 10).

## Useful commands

```bash
ros2 topic list
ros2 topic echo /g1/base_twist
ros2 topic hz /g1/policy_time_ms        # about 50 Hz would mean real time; see known_issues.md
rqt_plot /cmd_vel/linear/x /g1/base_twist/linear/x
rqt_graph
```

## Command conventions

- `vx`: forward (m/s), `vy`: lateral (m/s), `wz`: yaw rate (rad/s).
- The last received command stays active: there is **no timeout**. Send a zero `Twist` to stop.
- The command is scaled inside the observation by `(2.0, 2.0, 0.25)`.
