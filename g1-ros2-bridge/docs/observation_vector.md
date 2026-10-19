# Observation vector (47 elements)

Built every 20 ms from the simulator state. Scale factors come from `g1.yaml`.

| Index | Component | Meaning | Scale | Dim. |
|---|---|---|---|---|
| 0-2 | Base angular velocity | Base rotation rate (body frame); IMU gyroscope on the real robot | x 0.25 (`ang_vel_scale`) | 3 |
| 3-5 | Projected gravity | Gravity direction in the base frame, from the base quaternion | - | 3 |
| 6-8 | Velocity command | `(vx, vy, wz)` | x (2.0, 2.0, 0.25) (`cmd_scale`) | 3 |
| 9-20 | Joint position offsets | `q - q_default`, 12 leg joints | x 1.0 (`dof_pos_scale`) | 12 |
| 21-32 | Joint velocities | 12 leg joints | x 0.05 (`dof_vel_scale`) | 12 |
| 33-44 | Previous action | Raw output of the previous policy step | - | 12 |
| 45-46 | Gait clock | `[sin(2*pi*phase), cos(2*pi*phase)]`, period 0.8 s (hard-coded) | - | 2 |

Total: 3 + 3 + 3 + 12 + 12 + 12 + 2 = **47**.

Why these inputs?
- The network is a feed-forward MLP **without memory**: the previous action gives it context and promotes smooth motion;
  the clock gives it a notion of time, from which a periodic gait can emerge.
- The scale factors keep every input in the numeric range used during training.
- In simulation the state is exact (no sensor noise or delay).

## Joint order and default pose

Order (per leg): hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll (left leg first, then right leg).

| Joint | Kp | Kd | Default angle (rad) |
|---|---|---|---|
| Hip pitch | 100 | 2 | -0.1 |
| Hip roll | 100 | 2 | 0.0 |
| Hip yaw | 100 | 2 | 0.0 |
| Knee | 150 | 4 | 0.3 |
| Ankle pitch | 40 | 2 | -0.2 |
| Ankle roll | 40 | 2 | 0.0 |

Kp, Kd, default angles, scales and `num_obs/num_actions` were checked against Unitree's `g1.yaml` (`unitree_rl_gym`) and are identical.
Only difference: Unitree's file has `cmd_init: [0.5, 0, 0]`; here the command starts at zero and is set by `/cmd_vel`
(`cmd_init` is not used by `deploy_mujoco_ros.py`).
