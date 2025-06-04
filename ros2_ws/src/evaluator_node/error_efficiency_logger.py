import os
import csv
import math
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped


class ErrorEfficiencyLogger(Node):
    def __init__(self):
        super().__init__('error_efficiency_logger')
        self.joint_states = None
        self.end_effector_position = None
        self.ball_position = None
        self.start_time = self.get_clock().now()
        self.prev_time = self.start_time

        # Initialize effort calculations
        self.effort_torque_squared = 0.0
        self.effort_torque_velocity = 0.0

        # Folder setup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.controller_type = os.environ.get('CONTROLLER_TYPE', 'default_controller')
        self.output_folder = os.path.join(os.path.expanduser('~'), 'controller_logs', self.controller_type, timestamp)
        os.makedirs(self.output_folder, exist_ok=True)
        self.csv_file_path = os.path.join(self.output_folder, 'log.csv')

        # CSV header
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'time', 'joint_positions', 'joint_velocities', 'joint_efforts',
                'end_effector_position', 'ball_position', 'position_error',
                'effort_torque_squared', 'effort_torque_velocity'
            ])

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(PointStamped, '/end_effector_position', self.ee_position_callback, 10)
        self.create_subscription(PointStamped, '/ball_position', self.ball_position_callback, 10)

        self.timer = self.create_timer(0.01, self.log_data)  # 100 Hz

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def ee_position_callback(self, msg):
        self.end_effector_position = msg.point

    def ball_position_callback(self, msg):
        self.ball_position = msg.point

    def log_data(self):
        if (self.joint_states is None
            or not self.joint_states.effort
            or not self.joint_states.velocity
            or self.end_effector_position is None
            or self.ball_position is None):
            return
        if len(self.joint_states.effort) != len(self.joint_states.velocity):
            self.get_logger().warn("effort/velocity length mismatch – skipping sample")
            return

        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        position_error = math.sqrt(
            (self.end_effector_position.x - self.ball_position.x) ** 2 +
            (self.end_effector_position.y - self.ball_position.y) ** 2 +
            (self.end_effector_position.z - self.ball_position.z) ** 2
        )

        torque_squared = sum([eff ** 2 for eff in self.joint_states.effort])
        torque_velocity = sum([
            eff * vel for eff, vel in zip(self.joint_states.effort, self.joint_states.velocity)
        ])

        self.effort_torque_squared += torque_squared * dt
        self.effort_torque_velocity += torque_velocity * dt

        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                elapsed_time,
                self.joint_states.position,
                self.joint_states.velocity,
                self.joint_states.effort,
                (self.end_effector_position.x, self.end_effector_position.y, self.end_effector_position.z),
                (self.ball_position.x, self.ball_position.y, self.ball_position.z),
                position_error,
                self.effort_torque_squared,
                self.effort_torque_velocity
            ])


def main(args=None):
    rclpy.init(args=args)
    node = ErrorEfficiencyLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
