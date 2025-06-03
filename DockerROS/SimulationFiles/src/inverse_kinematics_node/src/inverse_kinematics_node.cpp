/* ====================================================================
 * inverse_kinematics_node.cpp
 * --------------------------------------------------------------------
 *  Subscribes   :  /set_xy  (controller_msgs/SetXY, cm in camera plane)
 *  Computes     :  planar 2-DOF IK  ➜ joint goal ticks
 *  Writes to    :  Dynamixel Goal Position registers
 *  Publishes on :  /ik/joint_plot_data  (controller_msgs/JointPlotData)
 * ==================================================================== */
#include <rclcpp/rclcpp.hpp>
#include <controller_msgs/msg/set_xy.hpp>
#include <controller_msgs/msg/joint_plot_data.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "control_common/control_params.hpp"
#include "control_common/control_utils.hpp"

using SetXY  = controller_msgs::msg::SetXY;
using Plot   = controller_msgs::msg::JointPlotData;
using std::placeholders::_1;

/* ------------------------------------------------------------------ */
class InverseKinematicsNode : public rclcpp::Node
{
public:
  InverseKinematicsNode() : Node("inverse_kinematics_node")
  {
    /* ---- parameters ------------------------------------------------ */
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baud", 1'000'000);

    port_name_ = get_parameter("port").as_string();
    baud_      = get_parameter("baud").as_int();

    /* ---- Dynamixel init ------------------------------------------- */
    port_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort() || !port_->setBaudRate(baud_))
      RCLCPP_FATAL(get_logger(), "Failed to open %s @ %d", port_name_.c_str(), baud_);

    for (uint8_t id : control::SERVO_IDS)
    {
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
      write8(id, control::ADDR_OPERATING_MODE, control::MODE_POSITION);
      write8(id, control::ADDR_TORQUE_ENABLE, 1);
    }

    /* ---- ROS I/O --------------------------------------------------- */
    sub_xy_ = create_subscription<SetXY>(
        "set_xy", 10, std::bind(&InverseKinematicsNode::cb_xy, this, _1));

    pub_plot_ = create_publisher<Plot>("ik/joint_plot_data", 10);

    RCLCPP_INFO(get_logger(), "IK node ready on %s (%d baud).", port_name_.c_str(), baud_);
  }

  ~InverseKinematicsNode() override
  {
    for (uint8_t id : control::SERVO_IDS)
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
    port_->closePort();
  }

private:
  /* ---- helpers ---------------------------------------------------- */
  void write8(uint8_t id, uint16_t addr, uint8_t data)
  {
    uint8_t err{}; int rc = packet_->write1ByteTxRx(port_, id, addr, data, &err);
    if (rc != COMM_SUCCESS || err)
      RCLCPP_ERROR(get_logger(), "DXL %u write1 err=%d rc=%d", id, err, rc);
  }
  void write32(uint8_t id, uint16_t addr, uint32_t data)
  {
    uint8_t err{}; int rc = packet_->write4ByteTxRx(port_, id, addr, data, &err);
    if (rc != COMM_SUCCESS || err)
      RCLCPP_ERROR(get_logger(), "DXL %u write4 err=%d rc=%d", id, err, rc);
  }

  /* ---- callback --------------------------------------------------- */
  void cb_xy(const SetXY::SharedPtr msg)
  {
    double j1_deg{}, j2_deg{};
    if (!control::ik_xy(msg->x, msg->y, j1_deg, j2_deg))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "XY target (%.1f, %.1f) cm unreachable", msg->x, msg->y);
      return;
    }

    write32(control::ID1, control::ADDR_GOAL_POSITION, control::deg2tick(0, j1_deg));
    write32(control::ID2, control::ADDR_GOAL_POSITION, control::deg2tick(1, j2_deg));

    Plot p;
    p.stamp = now();
    p.q1_deg = j1_deg;
    p.q2_deg = j2_deg;
    pub_plot_->publish(p);
  }

  /* ---- members ---------------------------------------------------- */
  std::string port_name_; int baud_;
  dynamixel::PortHandler*  port_{};
  dynamixel::PacketHandler* packet_{};

  rclcpp::Subscription<SetXY>::SharedPtr sub_xy_;
  rclcpp::Publisher<Plot>::SharedPtr     pub_plot_;
};

/* ---- main --------------------------------------------------------- */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}

