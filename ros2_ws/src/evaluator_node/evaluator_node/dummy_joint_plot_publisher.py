import math
import rclpy
from rclpy.node import Node
from controller_msgs.msg import JointPlotData
import matplotlib.pyplot as plt

class DummyJointPlotPublisher(Node):
    def __init__(self):
        super().__init__('dummy_joint_plot_publisher')
        self.pub = self.create_publisher(JointPlotData, 'pwm/joint_plot_data', 10)
        self.timer = self.create_timer(0.01, self.publish_dummy)
        self.get_logger().info('Dummy JointPlotData publisher started.')
        # Storing time series data for plotting
        self.start_time = self.get_clock().now()
        self.times = []
        self.desired_q1_list = []
        self.measured_q1_list = []
        self.desired_q2_list = []
        self.measured_q2_list = []
        self.pwm1_list = []
        self.pwm2_list = []

    def publish_dummy(self):
        now = self.get_clock().now()
        # Obtain elapsed time in seconds
        t = (now - self.start_time).nanoseconds / 1e9
        # Sinusoidal dummy signals:
        # For joint1: desired is 30*sin(1*t) deg; measured is same plus a 5 deg amplitude offset oscillation.
        desired_q1 = 30.0 * math.sin(t)
        measured_q1 = desired_q1 + 5.0 * math.sin(0.5 * t)
        # For joint2: desired is 45*cos(0.5*t) deg; measured is same plus a 3 deg offset oscillation.
        desired_q2 = 45.0 * math.cos(0.5 * t)
        measured_q2 = desired_q2 + 3.0 * math.cos(t)
        
        # Compute angular accelerations (second derivatives)
        ddq1 = -30.0 * math.sin(t)  # Second derivative of desired_q1
        ddq2 = -22.5 * math.cos(0.5 * t)  # Second derivative of desired_q2
        
        # Dummy PWM proportional to angular acceleration
        pwm1 = ddq1 * 10.0  # Scale factor for PWM
        pwm2 = ddq2 * 10.0  # Scale factor for PWM
        
        # Save data into lists for later plotting
        self.times.append(t)
        self.desired_q1_list.append(desired_q1)
        self.measured_q1_list.append(measured_q1)
        self.desired_q2_list.append(desired_q2)
        self.measured_q2_list.append(measured_q2)
        self.pwm1_list.append(pwm1)
        self.pwm2_list.append(pwm2)
        
        msg = JointPlotData()
        # Set dummy joint angles (in degrees)
        msg.q1_deg = measured_q1
        msg.q2_deg = measured_q2
        msg.q1_des_deg = desired_q1
        msg.q2_des_deg = desired_q2
        # Set dummy PWM values
        msg.pwm1 = pwm1
        msg.pwm2 = pwm2
        msg.measured_q1 = measured_q1
        msg.desired_q1 = desired_q1
        msg.measured_q2 = measured_q2
        msg.desired_q2 = desired_q2
        msg.controller_name = "dummy"
        self.pub.publish(msg)
    
    def plot_data(self):
        # Plot Joint 1: Desired and Measured
        plt.figure()
        plt.plot(self.times, self.desired_q1_list, label="Joint1 Desired (deg)")
        plt.plot(self.times, self.measured_q1_list, label="Joint1 Measured (deg)")
        plt.title("Dummy Joint 1 Signal")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.legend()
        plt.grid(True)
        
        # Plot Joint 2: Desired and Measured
        plt.figure()
        plt.plot(self.times, self.desired_q2_list, label="Joint2 Desired (deg)")
        plt.plot(self.times, self.measured_q2_list, label="Joint2 Measured (deg)")
        plt.title("Dummy Joint 2 Signal")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.legend()
        plt.grid(True)
        
        # Plot PWM values
        plt.figure()
        plt.plot(self.times, self.pwm1_list, label="PWM1")
        plt.plot(self.times, self.pwm2_list, label="PWM2")
        plt.title("Dummy PWM Signals")
        plt.xlabel("Time (s)")
        plt.ylabel("PWM")
        plt.legend()
        plt.grid(True)
        
        plt.show()


def main():
    rclpy.init()
    node = DummyJointPlotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.plot_data()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
