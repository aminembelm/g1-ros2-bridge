"""
ROS 2 version of the MuJoCo deployment script of Unitree's `unitree_rl_gym`.

Derived from deploy/deploy_mujoco/deploy_mujoco.py (Unitree Robotics).
Upstream license (BSD 3-Clause): Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD.
("Unitree Robotics"). All rights reserved. Redistribution and use in source form, with or without
modification, must retain this notice, the BSD 3-Clause conditions and the disclaimer
(see https://github.com/unitreerobotics/unitree_rl_gym/blob/main/LICENSE).
Modifications (ROS 2 node, telemetry): Belmiloud Mohamed Amine, 2026.

What was added to the original script
  - G1BridgeNode: subscribes to /cmd_vel (geometry_msgs/Twist) and publishes
        /g1/base_twist      measured base velocity
        /g1/policy_time_ms  duration of the policy forward pass (ms)
        /joint_states       positions and velocities of the 12 leg joints
  - the velocity command is no longer fixed in the YAML file: it is updated by /cmd_vel
  - `rclpy.spin_once(timeout_sec=0.0)` at every physics step (single thread, non-blocking)

Usage (from unitree_rl_gym/deploy/deploy_mujoco):
    source /opt/ros/humble/setup.bash
    python3 deploy_mujoco_ros.py g1.yaml [--body-frame]
"""
import argparse
import os
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# Folder of the unitree_rl_gym repository. Override with:  export LEGGED_GYM_ROOT_DIR=/path/to/unitree_rl_gym
LEGGED_GYM_ROOT_DIR = os.environ.get(
    "LEGGED_GYM_ROOT_DIR", os.path.expanduser("~/unitree_rl_gym")
)


def get_gravity_orientation(quaternion):
    """Gravity direction expressed in the base frame, from the base quaternion (w, x, y, z)."""
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2.0 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2.0 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1.0 - 2.0 * (qw * qw + qz * qz)

    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


def world_to_body(vec_world, quat_wxyz):
    """Rotate a world-frame vector into the base (body) frame."""
    neg_quat = np.zeros(4)
    mujoco.mju_negQuat(neg_quat, quat_wxyz)
    vec_body = np.zeros(3)
    mujoco.mju_rotVecQuat(vec_body, vec_world, neg_quat)
    return vec_body


class G1BridgeNode(Node):

    def __init__(self, cmd_array, lock):
        super().__init__("g1_bridge_node")
        self.cmd_array = cmd_array
        self.lock = lock

        # 1. Subscriber: listens for velocity commands from ROS 2
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # 2. Publisher: measured robot velocity from MuJoCo
        self.twist_pub = self.create_publisher(Twist, "/g1/base_twist", 10)

        # 3. Publisher: policy neural network latency in milliseconds
        self.time_pub = self.create_publisher(Float32, "/g1/policy_time_ms", 10)

        # 4. Publisher: joint positions and velocities
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.get_logger().info("ROS 2 G1 Bridge Node (Control + Telemetry) initialized.")

    def cmd_vel_callback(self, msg):
        """Thread-safe update of the command array when /cmd_vel arrives"""
        with self.lock:
            self.cmd_array[0] = msg.linear.x
            self.cmd_array[1] = msg.linear.y
            self.cmd_array[2] = msg.angular.z

    def publish_telemetry(self, base_lin_vel, base_ang_vel, inference_time_ms, qpos, qvel):
        """Publishes real-time simulation data back to ROS 2 for monitoring"""
        twist_msg = Twist()
        twist_msg.linear.x = float(base_lin_vel[0])
        twist_msg.linear.y = float(base_lin_vel[1])
        twist_msg.linear.z = float(base_lin_vel[2])
        twist_msg.angular.x = float(base_ang_vel[0])
        twist_msg.angular.y = float(base_ang_vel[1])
        twist_msg.angular.z = float(base_ang_vel[2])
        self.twist_pub.publish(twist_msg)

        time_msg = Float32()
        time_msg.data = float(inference_time_ms)
        self.time_pub.publish(time_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.position = [float(x) for x in qpos]
        joint_msg.velocity = [float(x) for x in qvel]
        self.joint_pub.publish(joint_msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "config_file", type=str, help="config file name in the config folder"
    )
    parser.add_argument(
        "--body-frame",
        action="store_true",
        help="publish the base linear velocity in the body frame "
             "(default: world frame, as in the version used for the report results)",
    )
    args = parser.parse_args()
    config_file = args.config_file

    with open(
        f"{LEGGED_GYM_ROOT_DIR}/deploy/deploy_mujoco/configs/{config_file}", "r"
    ) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    policy_path = config["policy_path"].replace(
        "{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR
    )
    xml_path = config["xml_path"].replace(
        "{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR
    )

    simulation_duration = config["simulation_duration"]
    simulation_dt = config["simulation_dt"]
    control_decimation = config["control_decimation"]

    kps = np.array(config["kps"], dtype=np.float32)
    kds = np.array(config["kds"], dtype=np.float32)

    default_angles = np.array(config["default_angles"], dtype=np.float32)

    ang_vel_scale = config["ang_vel_scale"]
    dof_pos_scale = config["dof_pos_scale"]
    dof_vel_scale = config["dof_vel_scale"]
    action_scale = config["action_scale"]
    cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

    num_actions = config["num_actions"]
    num_obs = config["num_obs"]

    # Initialize zero velocity command (updated later by /cmd_vel)
    cmd = np.zeros(3, dtype=np.float32)
    cmd_lock = threading.Lock()

    # Initialize ROS 2 node
    rclpy.init()
    ros_node = G1BridgeNode(cmd, cmd_lock)

    # Context variables
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)

    counter = 0

    # Load robot model & policy
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt

    policy = torch.jit.load(policy_path)

    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            while viewer.is_running():
                step_start = time.time()

                # Non-blocking spin: process pending ROS callbacks on the main thread
                rclpy.spin_once(ros_node, timeout_sec=0.0)

                tau = pd_control(
                    target_dof_pos,
                    d.qpos[7:],
                    kps,
                    np.zeros_like(kds),
                    d.qvel[6:],
                    kds,
                )
                d.ctrl[:] = tau

                mujoco.mj_step(m, d)

                counter += 1
                if counter % control_decimation == 0:
                    qj = d.qpos[7:]
                    dqj = d.qvel[6:]
                    quat = d.qpos[3:7]
                    omega = d.qvel[3:6]

                    qj = (qj - default_angles) * dof_pos_scale
                    dqj = dqj * dof_vel_scale
                    gravity_orientation = get_gravity_orientation(quat)
                    omega = omega * ang_vel_scale

                    period = 0.8
                    count = counter * simulation_dt
                    phase = (count % period) / period
                    sin_phase = np.sin(2.0 * np.pi * phase)
                    cos_phase = np.cos(2.0 * np.pi * phase)

                    # Read the shared velocity command safely
                    with cmd_lock:
                        current_cmd = cmd.copy()

                    obs[:3] = omega
                    obs[3:6] = gravity_orientation
                    obs[6:9] = current_cmd * cmd_scale
                    obs[9 : 9 + num_actions] = qj
                    obs[9 + num_actions : 9 + 2 * num_actions] = dqj
                    obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action
                    obs[9 + 3 * num_actions : 9 + 3 * num_actions + 2] = (
                        np.array([sin_phase, cos_phase])
                    )

                    obs_tensor = torch.from_numpy(obs).unsqueeze(0)

                    # Measure policy inference time (forward pass + conversion to NumPy)
                    t_infer_start = time.time()
                    action = policy(obs_tensor).detach().numpy().squeeze()
                    inference_time_ms = (time.time() - t_infer_start) * 1000.0

                    target_dof_pos = action * action_scale + default_angles

                    # Measured robot velocities from the physics state
                    base_lin_vel = d.qvel[0:3]  # world frame (MuJoCo free joint)
                    if args.body_frame:
                        base_lin_vel = world_to_body(base_lin_vel, d.qpos[3:7])
                    base_ang_vel = d.qvel[3:6]  # already in the body frame

                    # Publish monitoring telemetry to ROS 2
                    ros_node.publish_telemetry(
                        base_lin_vel=base_lin_vel,
                        base_ang_vel=base_ang_vel,
                        inference_time_ms=inference_time_ms,
                        qpos=d.qpos[7:],
                        qvel=d.qvel[6:],
                    )

                viewer.sync()

                time_until_next_step = m.opt.timestep - (
                    time.time() - step_start
                )
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
