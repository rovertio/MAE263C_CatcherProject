#include <memory>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "joint_positions_node/msg/set_joint_degrees.hpp"
#include "joint_positions_node/srv/get_joint_degrees.hpp"

using SetJointDegrees = joint_positions_node::msg::SetJointDegrees;
using GetJointDegrees = joint_positions_node::srv::GetJointDegrees;

/* ---------- Constants ---------- */
constexpr uint8_t  ID_1 = 4;
constexpr uint8_t  ID_2 = 1;

constexpr double   ZERO_TICK_1 = 691.0;
constexpr double   ZERO_TICK_2 = 2530.0;
constexpr double   TICKS_PER_DEG = 4096.0 / 360.0;

constexpr double   MIN_DEG_1 = 0, MAX_DEG_1 = 70;
constexpr double   MIN_DEG_2 = 0, MAX_DEG_2 = 135.0;

constexpr uint8_t  ADDR_GOAL_POSITION    = 116;
constexpr uint8_t  ADDR_PRESENT_POSITION = 132;
constexpr uint8_t  ADDR_TORQUE_ENABLE    = 64;
constexpr uint8_t  ADDR_OPERATING_MODE   = 11;

constexpr uint8_t  OPERATING_MODE_POSITION = 3;

/* ---------- Helper conversion ---------- */
inline uint32_t deg_to_tick_1(double deg)
{ return static_cast<uint32_t>(ZERO_TICK_1 + deg * TICKS_PER_DEG); }

inline uint32_t deg_to_tick_2(double deg)
{ return static_cast<uint32_t>(ZERO_TICK_2 + deg * TICKS_PER_DEG); }

inline double tick_to_deg_1(uint32_t tick)
{ return (static_cast<int32_t>(tick) - ZERO_TICK_1) / TICKS_PER_DEG; }

inline double tick_to_deg_2(uint32_t tick)
{ return (static_cast<int32_t>(tick) - ZERO_TICK_2) / TICKS_PER_DEG; }

/* ---------- Node class ---------- */
class JointPosNode : public rclcpp::Node
{
public:
  JointPosNode() : Node("joint_positions_node")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 57600);

    port_name_ = get_parameter("port").as_string();
    baud_      = get_parameter("baudrate").as_int();

    // --- open serial port ---
    port_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if(!port_->openPort() || !port_->setBaudRate(baud_))
      throw std::runtime_error("Failed to open or set baudrate on port.");

    setup_servo(ID_1);
    setup_servo(ID_2);

    set_sub_ = create_subscription<SetJointDegrees>(
      "set_joint_degrees", 10,
      [this](const SetJointDegrees::SharedPtr msg) { on_set(msg); });

    get_srv_ = create_service<GetJointDegrees>(
      "get_joint_degrees",
      [this](const std::shared_ptr<GetJointDegrees::Request> req,
             std::shared_ptr<GetJointDegrees::Response>      res)
      { on_get(req, res); });

    RCLCPP_INFO(get_logger(), "joint_positions_node ready.");
  }

  ~JointPosNode() override
  {
    std::lock_guard<std::mutex> lock(mx_);
    packet_->write1ByteTxRx(port_, 0xFE, ADDR_TORQUE_ENABLE, 0, &dxl_err_);
    port_->closePort();
  }

private:
  /* ----------- init helpers ---------- */
  void setup_servo(uint8_t id)
  {
    std::lock_guard<std::mutex> lock(mx_);
    write_byte(id, ADDR_TORQUE_ENABLE, 0);                     // torque off
    write_byte(id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION);
    write_byte(id, ADDR_TORQUE_ENABLE, 1);                     // torque on
  }

  void write_byte(uint8_t id, uint8_t addr, uint8_t value)
  {
    int comm = packet_->write1ByteTxRx(port_, id, addr, value, &dxl_err_);
    if(comm != COMM_SUCCESS || dxl_err_ != 0)
      RCLCPP_ERROR(get_logger(),
        "Write failed id=%u addr=%u err=%s/%s",
        id, addr,
        packet_->getTxRxResult(comm),
        packet_->getRxPacketError(dxl_err_));
  }

  /* ----------- callbacks ------------- */
  void on_set(const SetJointDegrees::SharedPtr msg)
  {
    if(msg->degrees.size() != 2) {
      RCLCPP_ERROR(get_logger(), "Need exactly 2 degree values.");
      return;
    }

    double d1 = std::clamp(msg->degrees[0], MIN_DEG_1, MAX_DEG_1);
    double d2 = std::clamp(msg->degrees[1], MIN_DEG_2, MAX_DEG_2);

    uint32_t t1 = deg_to_tick_1(d1);
    uint32_t t2 = deg_to_tick_2(d2);

    dynamixel::GroupSyncWrite sync(port_, packet_, ADDR_GOAL_POSITION, 4);
    uint8_t buf1[4] = {uint8_t(t1), uint8_t(t1>>8), uint8_t(t1>>16), uint8_t(t1>>24)};
    uint8_t buf2[4] = {uint8_t(t2), uint8_t(t2>>8), uint8_t(t2>>16), uint8_t(t2>>24)};

    sync.addParam(ID_1, buf1);
    sync.addParam(ID_2, buf2);

    std::lock_guard<std::mutex> lock(mx_);
    int comm = sync.txPacket();
    if(comm != COMM_SUCCESS)
      RCLCPP_ERROR(get_logger(), "SyncWrite failed: %s",
                   packet_->getTxRxResult(comm));
  }

  void on_get(const std::shared_ptr<GetJointDegrees::Request>,
              std::shared_ptr<GetJointDegrees::Response> res)
  {
    dynamixel::GroupSyncRead sync(port_, packet_, ADDR_PRESENT_POSITION, 4);
    sync.addParam(ID_1);
    sync.addParam(ID_2);

    std::lock_guard<std::mutex> lock(mx_);
    int comm = sync.txRxPacket();
    if(comm != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncRead comm err: %s",
                   packet_->getTxRxResult(comm));
      res->degrees = {0.0,0.0};
      return;
    }

    uint32_t tick1 = sync.getData(ID_1, ADDR_PRESENT_POSITION, 4);
    uint32_t tick2 = sync.getData(ID_2, ADDR_PRESENT_POSITION, 4);


    res->degrees[0] = tick_to_deg_1(tick1);
    res->degrees[1] = tick_to_deg_2(tick2);
  }

  /* ----------- members -------------- */
  std::string port_name_;
  int         baud_;
  dynamixel::PortHandler   *port_{nullptr};
  dynamixel::PacketHandler *packet_{nullptr};
  uint8_t dxl_err_{0};
  std::mutex mx_;

  rclcpp::Subscription<SetJointDegrees>::SharedPtr set_sub_;
  rclcpp::Service<GetJointDegrees>::SharedPtr      get_srv_;
};

/* ----------- main ------------------- */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPosNode>());
  rclcpp::shutdown();
  return 0;
}

