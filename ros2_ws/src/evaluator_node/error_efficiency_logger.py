#!/usr/bin/env python3
import rclpy
import numpy as np
import csv
import os
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.qos import QoSProfile
from datetime import datetime


class EvaluatorNode(Node):
    def __init__(self):
        self.run_id = datetime.now().strftime("throw_%Y%m%d_%H%M%S")

        super().__init__('evaluator_node')

        # Declare and retrieve the controller name parameter
        self.declare_parameter('controller_name', 'unknown_controller')
        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value

        qos = QoSProfile(depth=10)

        self.ee_sub = self.create_subscription(
            PoseStamped,
            '/end_effector_pose',
            self.ee_callback,
            qos)

        self.ball_sub = self.create_subscription(
            PointStamped,
            '/ball/centroid',
            self.ball_callback,
            qos)

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            qos)

        self.last_time = None
        self.energy = 0.0
        self.last_effort = None

        self.ee_pos = None
        self.ball_pos = None

        self.rows = []

        self.get_logger().info(f"Evaluator node started for controller: {self.controller_name}")


    def ee_callback(self, msg):
        self.ee_pos = np.array([msg.pose.position.x,
                                msg.pose.position.y,
                                msg.pose.position.z])
        self.log_metrics(msg.header.stamp)

    def ball_callback(self, msg):
        self.ball_pos = np.array([msg.point.x,
                                  msg.point.y,
                                  msg.point.z])

    def joint_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is not None:
            dt = t - self.last_time
            if msg.effort:
                tau = np.array(msg.effort)
                self.energy += np.sum(tau ** 2) * dt
        self.last_time = t
        self.last_effort = msg.effort

    def log_metrics(self, stamp):
        if self.ee_pos is not None and self.ball_pos is not None:
            error = np.linalg.norm(self.ee_pos - self.ball_pos)
            now = self.get_clock().now().to_msg()
            row = {
                'time_sec': now.sec + now.nanosec * 1e-9,
                'error': error,
                'energy': self.energy
            }
            self.rows.append(row)
            self.get_logger().info(f"t={row['time_sec']:.2f}s | error={error:.4f} | energy={self.energy:.4f}")

    def destroy_node(self):
        super().destroy_node()

        # Create a folder named after the controller
        save_dir = os.path.join(os.path.expanduser("~"), f"ros2_eval_logs/{self.controller_name}")
        os.makedirs(save_dir, exist_ok=True)

        # Save log to that folder
        filename = f"{self.run_id}.csv"
        save_path = os.path.join(save_dir, filename)

        with open(save_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['time_sec', 'error', 'energy'])
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(f"Saved evaluation log to {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
