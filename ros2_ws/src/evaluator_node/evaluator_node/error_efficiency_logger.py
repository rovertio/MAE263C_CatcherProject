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
        self.joint_pwm = [0,0]
        self.joint_angles_current = [0,0]
        self.joint_angles_desired =[0,0]
        # Initialize effort calculations
        self.effort_PWM = 0.00

        # Folder setup
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.controller_type = os.environ.get('CONTROLLER_TYPE', 'default_controller')
        self.output_folder = os.path.join(os.path.expanduser('~'), 'controller_logs', self.controller_type, timestamp)
        os.makedirs(self.output_folder, exist_ok=True)
        self.csv_file_path = os.path.join(self.output_folder, 'log.csv')
        self.get_logger().info(self.csv_file_path)


        # CSV header
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'time', 'PWM1', 'PWM2', 'PWM Efforts', "Joint Angle Current 1", "Joint Angle Current 2",
                "Joint Angle Desired 1", "Joint Angle Desired 2"
            ])

        # Subscriptions
        # self.joint_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        self.get_logger().info('🔧 Subscribed to /joint_states')

        # self.create_subscription(PointStamped, '/ball_position', self.ball_callback, 10)
        
        
        self.create_subscription(jp, "pwm/joint_plot_data", self.pwm_callback,10)
        
        
        self.timer = self.create_timer(0.01, self.log_data)  # 100 Hz

    # def joint_state_callback(self, msg):
    #     self.latest_joint_msg = msg
    #     self.try_log_data()

    # def ball_callback(self, msg):
    #     self.latest_ball_msg = msg
    #     self.try_log_data()

    def pwm_callback(self, msg):
        self.joint_pwm = [msg.pwm1,msg.pwm2]
        self.joint_angles_current = [msg.q1_deg, msg.q2_deg]
        self.joint_angles_desired = [msg.q1_des_deg, msg.q2_des_deg]
        Message = "Joint PWM: [" + str(self.joint_pwm[0]) + ", " + str(self.joint_pwm[1]) + "]"
        # self.get_logger().info(Message)



    # def try_log_data(self):
    #     if self.latest_joint_msg and self.latest_ball_msg and self.latest_pwm_msg:
    #     # Extract timestamp (you can improve this based on sync logic)
    #         stamp = self.latest_joint_msg.header.stamp

    #         joint_pos = self.latest_joint_msg.position
    #         joint_vel = self.latest_joint_msg.velocity
    #         effort = self.latest_joint_msg.effort

    #         ball_x = self.latest_ball_msg.x
    #         ball_y = self.latest_ball_msg.y

    #         pwm = self.latest_pwm_msg.data

    #     # Dummy placeholder: you should replace with real effort calculations
    #         effort_torque_squared = sum([t**2 for t in effort]) if effort else 0.0
    #         effort_pwm_squared = sum([p**2 for p in pwm]) if pwm else 0.0

    #         self.writer.writerow([
    #             stamp.sec + stamp.nanosec * 1e-9,
    #             joint_pos,
    #             joint_vel,
    #             effort,
    #             ball_x,
    #             ball_y,
    #             pwm,
    #             effort_torque_squared,
    #             effort_pwm_squared
    #     ])

    #         self.get_logger().info("✅ Logged one row of synchronized data.")

    #     # Reset after logging
    #         self.latest_joint_msg = None
    #         self.latest_ball_msg = None
    #     self.latest_pwm_msg = None



    def log_data(self):
        # if (self.joint_states is None
        #     or not self.joint_states.effort
        #     or not self.joint_states.velocity
        #     or self.end_effector_position is None
        #     or self.ball_position is None):
        #     return
        # if len(self.joint_states.effort) != len(self.joint_states.velocity):
        #     self.get_logger().warn("effort/velocity length mismatch – skipping sample")
        #     return

        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        # position_error = math.sqrt(
        #     (self.end_effector_position.x - self.ball_position.x) ** 2 +
        #     (self.end_effector_position.y - self.ball_position.y) ** 2 
        # )

        # torque_squared = sum([eff ** 2 for eff in self.joint_states.effort])
        # torque_velocity = sum([
        #     eff * vel for eff, vel in zip(self.joint_states.effort, self.joint_states.velocity)
        # ])

        # self.effort_torque_squared += torque_squared * dt
        # self.effort_torque_velocity += torque_velocity * dt

        self.effort_PWM += (abs(self.joint_pwm[0])**2 + abs(self.joint_pwm[1])**2)*dt

        Message = "Effort: " + str(self.effort_PWM)
        self.get_logger().info(Message)

        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                elapsed_time, 
                self.joint_pwm[0],
                self.joint_pwm[1],
                self.effort_PWM,
                self.joint_angles_current[0],
                self.joint_angles_current[1],
                self.joint_angles_desired[0],
                self.joint_angles_desired[1]
            ])




def main():
    rclpy.init()
    node = ErrorEfficiencyLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
