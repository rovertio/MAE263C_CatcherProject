/* ====================================================================
 *  Subscribes   :  /set_xy  (controller_msgs/SetXY, cm in camera plane)
 *  Computes     :  planar 2-DOF Independent PID Joint Control ➜ pwm signal
 *  Writes to    :  Dynamixel PWM Registers
 *  Publishes on :  /pwm/joint_plot_data  (controller_msgs/JointPlotData)
 * ==================================================================== */

#include <rclcpp/rclcpp.hpp>
#include <controller_msgs/msg/set_xy.hpp>
#include <controller_msgs/msg/joint_plot_data.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "control_common/control_params.hpp"
#include "control_common/control_utils.hpp"
#include <cmath>

using SetXY = controller_msgs::msg::SetXY;
using Plot = controller_msgs::msg::JointPlotData;
using std::placeholders::_1;

class PwmPositionNode : public rclcpp::Node
{
public:
  PwmPositionNode() : Node("pwm_position_node")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baud", 1000000);
    declare_parameter<double>("kp", 50.0);
    declare_parameter<double>("ki", 0.1);
    declare_parameter<double>("kd", 5.0);
    declare_parameter<int>("max_pwm", 885);
    declare_parameter<int>("ctrl_rate_hz", 1000);

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    max_pwm_ = get_parameter("max_pwm").as_int();
    dt_s_ = 1.0 / get_parameter("ctrl_rate_hz").as_int();

    port_ = dynamixel::PortHandler::getPortHandler(get_parameter("port").as_string().c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort() || !port_->setBaudRate(get_parameter("baud").as_int()))
      RCLCPP_FATAL(get_logger(), "Serial open failed");

    for (uint8_t id : control::SERVO_IDS)
    {
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
      write8(id, control::ADDR_OPERATING_MODE, control::MODE_PWM);
      write8(id, control::ADDR_TORQUE_ENABLE, 1);
    }

    sub_xy_ = create_subscription<SetXY>("set_xy", 10, std::bind(&PwmPositionNode::cb_xy, this, _1));
    pub_plot_ = create_publisher<Plot>("pwm/joint_plot_data", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_s_), std::bind(&PwmPositionNode::control_loop, this));
    last_good_cmd_time_ = now();

    RCLCPP_INFO(get_logger(), "PWM node ready (kp=%.1f ki=%.1f kd=%.1f)", kp_, ki_, kd_);
  }

  ~PwmPositionNode() override
  {
    for (uint8_t id : control::SERVO_IDS)
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
    port_->closePort();
  }

private:
  void write8(uint8_t id, uint16_t addr, uint8_t d)
  {
    uint8_t e{};
    int rc = packet_->write1ByteTxRx(port_, id, addr, d, &e);
    if (rc != COMM_SUCCESS || e)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "DXL %u err=%d rc=%d", id, e, rc);
  }

  // Use GroupSyncWrite to set both motors' PWMs at once
  void send_pwms(int pwm1, int pwm2)
  {
    dynamixel::GroupSyncWrite group_writer(port_, packet_, control::ADDR_GOAL_PWM, 2);
    uint8_t b1[2] = {DXL_LOBYTE(pwm1), DXL_HIBYTE(pwm1)};
    uint8_t b2[2] = {DXL_LOBYTE(pwm2), DXL_HIBYTE(pwm2)};
    bool ok1 = group_writer.addParam(control::ID1, b1);
    bool ok2 = group_writer.addParam(control::ID2, b2);
    if (!ok1 || !ok2)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "GroupSyncWrite addParam failed! (ok1=%d ok2=%d)", ok1, ok2);
      return;
    }
    int rc = group_writer.txPacket();
    if (rc != COMM_SUCCESS)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "GroupSyncWrite PWM failed: %s", packet_->getTxRxResult(rc));
    }
  }

  void cb_xy(const SetXY::SharedPtr msg)
  {
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "set_xy contains NaN/Inf — ignoring");
      return;
    }
    double j1, j2;
    RCLCPP_INFO(get_logger(), "PWM HERE->>>>> (kp=%.1f ki=%.1f kd=%.1f)", kp_, ki_, kd_);
    if (control::ik_xy(msg->x * 0.01, msg->y * 0.01, j1, j2))
    {
      tgt_j1_ = j1 * 180.0 / M_PI;
      tgt_j2_ = j2 * 180.0 / M_PI;
      last_good_cmd_time_ = now();
      has_target_ = true;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IK failed for (%.1f, %.1f)", msg->x, msg->y);
    }
  }

  void control_loop()
  {
    if (!has_target_ || (now() - last_good_cmd_time_).seconds() > 3.0)
      return;
    int32_t tick1{}, tick2{};
    control::read_two_positions(port_, packet_, tick1, tick2);
    double cur1 = control::tick2deg(0, tick1);
    double cur2 = control::tick2deg(1, tick2);
    RCLCPP_INFO(get_logger(), "PWM gains (kp=%.1f ki=%.1f kd=%.1f)", kp_, ki_, kd_);
    double pwm1 = pid1_.step(tgt_j1_ - cur1, kp_, ki_, kd_, max_pwm_);
    double pwm2 = pid2_.step(tgt_j2_ - cur2, kp_, ki_, kd_, max_pwm_);
    send_pwms(int(pwm1), int(pwm2));

    Plot p;
    p.stamp = now();
    p.q1_deg = cur1;
    p.q2_deg = cur2;
    p.q1_des_deg = tgt_j1_;
    p.q2_des_deg = tgt_j2_;
    p.e1 = tgt_j1_ - cur1;
    p.e2 = tgt_j2_ - cur2;
    p.pwm1 = pwm1;
    p.pwm2 = pwm2;
    pub_plot_->publish(p);
  }

  rclcpp::Subscription<SetXY>::SharedPtr sub_xy_;
  rclcpp::Publisher<Plot>::SharedPtr pub_plot_;
  rclcpp::TimerBase::SharedPtr timer_;
  dynamixel::PortHandler *port_{};
  dynamixel::PacketHandler *packet_{};
  control::PidState pid1_, pid2_;
  double kp_, ki_, kd_, dt_s_;
  int max_pwm_;
  bool has_target_{false};
  double tgt_j1_{}, tgt_j2_{};
  rclcpp::Time last_good_cmd_time_;
};

/* ---- main --------------------------------------------------------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PwmPositionNode>());
  rclcpp::shutdown();
  return 0;
}
