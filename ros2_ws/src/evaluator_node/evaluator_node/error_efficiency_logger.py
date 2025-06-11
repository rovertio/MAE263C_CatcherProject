import os
import csv
import math
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from controller_msgs.msg import JointPlotData as jp

class ErrorEfficiencyLogger(Node):
    def __init__(self):
        super().__init__('error_efficiency_logger')
        self.get_logger().info('✅ Evaluator Node has started successfully.')
        self.joint_states = None
        self.end_effector_position = None
        self.ball_position = None
        self.start_time = self.get_clock().now()
        self.prev_time = self.start_time
        # new members to track data updates and whether data has been received
        self.last_update = self.start_time
        self.has_data = False
        self.joint_pwm = [0, 0]
        self.joint_angles_current = [0, 0]
        self.joint_angles_desired = [0, 0]
        self.controller_name = None
        self.effort_PWM = 0.00

        # Folder setup: use an absolute path that's volume-mounted.
        # For example, if your workspace is at "/workspaces/ros2_ws", then:
        base_dir = os.path.join("/workspaces/ros2_ws", "src", "evaluator_node")
        # If no controller name yet, default to "default_controller"
        controller = "default_controller" if self.controller_name is None else self.controller_name
        self.output_folder = os.path.join(base_dir, "Data Set", controller)
        os.makedirs(self.output_folder, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file_path = os.path.join(self.output_folder, f'{controller}_DataSet[{timestamp}].csv')
        self.get_logger().info(f"CSV file will be saved to: {self.csv_file_path}")

        # Initialize the CSV file with headers
        try:
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'time', 'PWM1', 'PWM2', 'PWM Efforts', "Joint Angle Current 1", "Joint Angle Current 2",
                    "Joint Angle Desired 1", "Joint Angle Desired 2", "Controller Name"
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to create CSV file: {e}")

        self.create_subscription(jp, "pwm/joint_plot_data", self.pwm_callback, 10)
        self.create_subscription(jp, "idc_joint/joint_plot_data", self.pwm_callback, 10)
        self.create_subscription(jp, "idc/joint_plot_data", self.pwm_callback, 10)
        self.create_subscription(jp, "robust_joint/joint_plot_data", self.pwm_callback, 10)

        self.timer = self.create_timer(0.01, self.log_data)  # 100 Hz

    def pwm_callback(self, msg):
        if self.controller_name is None and hasattr(msg, "controller_name"):
            self.controller_name = msg.controller_name if msg.controller_name else "unknown_controller"
            self.get_logger().info(f"Controller name set to: {self.controller_name}")
        self.joint_pwm = [getattr(msg, "pwm1", 0.0), getattr(msg, "pwm2", 0.0)]
        self.joint_angles_current = [getattr(msg, "measured_q1", 0.0), getattr(msg, "measured_q2", 0.0)]
        self.joint_angles_desired = [getattr(msg, "desired_q1", 0.0), getattr(msg, "desired_q2", 0.0)]
        self.has_data = True
        self.last_update = self.get_clock().now()

    def log_data(self):
        if not self.has_data:
            return

        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        # Format time values to 4 decimal places for output
        elapsed_time_fmt = f"{elapsed_time:.4f}"
        dt_fmt = f"{dt:.4f}"

        if (now - self.last_update).nanoseconds / 1e9 > 1.0:
            self.get_logger().info("No updated data received in over 1 second. Finalizing CSV file.")
            self.get_logger().info(f"CSV file saved at: {os.path.abspath(self.csv_file_path)}")
            self.timer.cancel()
            return

        self.effort_PWM += (abs(self.joint_pwm[0])**2 + abs(self.joint_pwm[1])**2) * dt

        Message = f"Effort: {round(self.effort_PWM, 3)}"
        self.get_logger().info(Message)
        Message2 = (
            f"\n{round(self.joint_angles_current[0], 2)} {round(self.joint_angles_current[1], 2)} "
            f"{round(self.joint_angles_desired[0], 2)} {round(self.joint_angles_desired[1], 2)} "
            f"t={elapsed_time_fmt}"
        )
        self.get_logger().info(Message2)

        try:
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    elapsed_time_fmt,
                    self.joint_pwm[0],
                    self.joint_pwm[1],
                    self.effort_PWM,
                    self.joint_angles_current[0],
                    self.joint_angles_current[1],
                    self.joint_angles_desired[0],
                    self.joint_angles_desired[1],
                    self.controller_name
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to write to CSV file: {e}")

def main():
    rclpy.init()
    node = ErrorEfficiencyLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    node.destroy_node()
    rclpy.shutdown()
    node.get_logger().info("Node has been shut down successfully.")

if __name__ == '__main__':
    main()