// pwm_position_node.cpp (simplified)
// Author: <you>
// ----------------------------------------------------------------------------
#include <memory>
#include <cmath>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "inverse_kinematics_node/msg/set_xy.hpp"
#include "inverse_kinematics_node/srv/get_xy.hpp"

using SetXY = inverse_kinematics_node::msg::SetXY;
using GetXY = inverse_kinematics_node::srv::GetXY;

/* ---------------- Hardware & encoder constants ---------------------------- */
constexpr uint8_t  ID_1 = 4;           // shoulder
constexpr uint8_t  ID_2 = 1;           // elbow
constexpr double   ZERO_TICK_1 = 2048;
constexpr double   ZERO_TICK_2 = 2048;
constexpr double   TICKS_PER_DEG = 4096.0 / 360.0;

inline double  tick_to_deg_1(uint32_t t){ return (static_cast<int32_t>(t)-ZERO_TICK_1)/TICKS_PER_DEG; }
inline double  tick_to_deg_2(uint32_t t){ return (static_cast<int32_t>(t)-ZERO_TICK_2)/TICKS_PER_DEG; }

/* ---------------- Kinematics (same numbers as IK node) -------------------- */
constexpr double L1 = 30.0, L2 = 30.0;
constexpr double BASE_X = -41.0, BASE_Y = -1.0;

constexpr double MIN_J1 = -90.0 , MAX_J1 =  20.0;
constexpr double MIN_J2 =   0.0 , MAX_J2 = 140.0;

/* ---------------- Dynamixel registers (XM430‑W210) ----------------------- */
constexpr uint8_t ADDR_OPERATING_MODE   =  11;
constexpr uint8_t ADDR_TORQUE_ENABLE    =  64;
constexpr uint8_t ADDR_PRESENT_POSITION = 132;
constexpr uint8_t ADDR_GOAL_PWM         = 100;
constexpr uint8_t MODE_PWM              = 16;          // current‑based PWM
//constexpr uint8_t BROADCAST_ID          = 0xFE;

/*----------------------- IK helper (elbow-down) -----------------------------*/
bool cartesian_to_joint(double x, double y,
                        double &j1_deg, double &j2_deg)
{
  /* translate world → robot frame */
  double xr = x - BASE_X;   // world origin –42 → xr = x+42
  double yr = y - BASE_Y;

  double r2 = xr*xr + yr*yr;
  double c2 = (r2 - L1*L1 - L2*L2)/(2*L1*L2);
  if(c2 < -1.0 || c2 > 1.0) return false;           // unreachable
  double theta2 = std::abs(std::acos(c2));                          

  double s2 = std::sin(theta2);              // elbow-down (negative sin)
  //double theta2 = M_PI -  std::atan2(s2, c2);               // math CCW (+)

  double k1 = L1 + L2 * c2;
  double k2 = L2 * s2;
  double theta1 = std::atan2(yr, xr) - std::atan2(k2, k1);

  /* convert to motor conventions */
  j1_deg = theta1 * 180.0/M_PI;    // motor positive CW
  j2_deg = theta2 * 180.0/M_PI;    // motor positive CCW

  /* limits */
  if(j1_deg < MIN_J1 || j1_deg > MAX_J1) return false;
  if(j2_deg < MIN_J2 || j2_deg > MAX_J2) return false;
  return true;
}

/* ========================================================================= */
class PWMNode : public rclcpp::Node
{
public:
  PWMNode() : Node("pwm_position_node")
  {
    /* parameters --------------------------------------------------------- */
    declare_parameter<std::string>("port",     "/dev/ttyUSB0");
    declare_parameter<int>       ("baudrate",  57600);
    declare_parameter<double>    ("J1.kp",        30.0);   // proportional gains
    declare_parameter<double>    ("J2.kp",        30.0);
    declare_parameter<int>       ("pwm_limit", 600);

    /* serial ------------------------------------------------------------- */
    port_   = dynamixel::PortHandler::getPortHandler(
                get_parameter("port").as_string().c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if(!port_->openPort() ||
       !port_->setBaudRate(get_parameter("baudrate").as_int()))
      throw std::runtime_error("Failed to open/baudrate serial port");

    setup_servo(ID_1);
    setup_servo(ID_2);

    /* ROS interfaces ----------------------------------------------------- */
    sub_ = create_subscription<SetXY>(
      "set_xy", 10,
      [this](SetXY::SharedPtr msg){ handle_set_xy(std::move(msg)); });

    srv_ = create_service<GetXY>(
      "get_xy",
      [this](GetXY::Request::SharedPtr,
             GetXY::Response::SharedPtr res){ handle_get_xy(res); });

    RCLCPP_INFO(get_logger(), "Simple PWM node ready.");
  }

  ~PWMNode() override
  {
    std::lock_guard<std::mutex> lk(mux_);
    packet_->write1ByteTxRx(port_, BROADCAST_ID,
                            ADDR_TORQUE_ENABLE, 0, &dxl_err_);
    port_->closePort();
  }

private:
  /* ------ low‑level helpers ------------------------------------------- */
  void setup_servo(uint8_t id)
  {
    packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 0, &dxl_err_);
    packet_->write1ByteTxRx(port_, id, ADDR_OPERATING_MODE, MODE_PWM, &dxl_err_);
    packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 1, &dxl_err_);
  }

  /* ------ subscriber callback ----------------------------------------- */
  void handle_set_xy(SetXY::SharedPtr msg)
  {
    /* 1. IK ------------------------------------------------------------- */
    double j1_des, j2_des;
    if(!cartesian_to_joint(msg->x, msg->y, j1_des, j2_des)) {
      RCLCPP_WARN(get_logger(), "IK failed for (%.2f, %.2f)", msg->x, msg->y);
      return;
    }

    /* 2. read current joint angles ------------------------------------- */
    dynamixel::GroupSyncRead sr(port_, packet_, ADDR_PRESENT_POSITION, 4);
    sr.addParam(ID_1);
    sr.addParam(ID_2);

    {
      std::lock_guard<std::mutex> lk(mux_);
      if(sr.txRxPacket()!=COMM_SUCCESS){
        RCLCPP_ERROR(get_logger(),"SyncRead failed");
        return;
      }
    }
    uint32_t t1 = sr.getData(ID_1, ADDR_PRESENT_POSITION, 4);
    uint32_t t2 = sr.getData(ID_2, ADDR_PRESENT_POSITION, 4);
    double j1_cur = tick_to_deg_1(t1);
    double j2_cur = tick_to_deg_2(t2);

    /* 3. proportional control ------------------------------------------ */
    double kp1 = get_parameter("J1.kp").as_double();
    double kp2 = get_parameter("J2.kp").as_double();
    int pwm_lim = get_parameter("pwm_limit").as_int();

    int pwm1 = static_cast<int>(kp1 * (j1_des - j1_cur));
    int pwm2 = static_cast<int>(kp2 * (j2_des - j2_cur));

    pwm1 = std::clamp(pwm1, -pwm_lim, pwm_lim);
    pwm2 = std::clamp(pwm2, -pwm_lim, pwm_lim);

    /* 4. pack & send ---------------------------------------------------- */
    uint8_t b1[2] = { DXL_LOBYTE(static_cast<uint16_t>(pwm1)),
                      DXL_HIBYTE(static_cast<uint16_t>(pwm1)) };
    uint8_t b2[2] = { DXL_LOBYTE(static_cast<uint16_t>(pwm2)),
                      DXL_HIBYTE(static_cast<uint16_t>(pwm2)) };

    dynamixel::GroupSyncWrite sw(port_, packet_, ADDR_GOAL_PWM, 2);
    sw.addParam(ID_1, b1);
    sw.addParam(ID_2, b2);

    {
      std::lock_guard<std::mutex> lk(mux_);
      if(sw.txPacket()!=COMM_SUCCESS)
        RCLCPP_ERROR(get_logger(),"SyncWrite PWM failed");
    }

    RCLCPP_INFO(get_logger(),
      "XY(%.2f, %.2f) → (j1=%.1f°, j2=%.1f°) | cur=(%.1f°, %.1f°)"
      " → pwm=(%d, %d)",
      msg->x, msg->y, j1_des, j2_des, j1_cur, j2_cur, pwm1, pwm2);
  }

  /* ------ service: return current EE position ------------------------- */
  void handle_get_xy(GetXY::Response::SharedPtr res)
  {
    dynamixel::GroupSyncRead sr(port_, packet_, ADDR_PRESENT_POSITION, 4);
    sr.addParam(ID_1); sr.addParam(ID_2);

    std::lock_guard<std::mutex> lk(mux_);
    if(sr.txRxPacket()!=COMM_SUCCESS){
      res->x = res->y = 0.0;
      return;
    }
    uint32_t t1 = sr.getData(ID_1, ADDR_PRESENT_POSITION, 4);
    uint32_t t2 = sr.getData(ID_2, ADDR_PRESENT_POSITION, 4);

    double j1 = tick_to_deg_1(t1);
    double j2 = tick_to_deg_2(t2);

    /* forward kinematics (elbow‑down) */
    double th1 = -j1*M_PI/180.0;
    double th2 =  j2*M_PI/180.0;
    res->x = BASE_X + L1*std::cos(th1) + L2*std::cos(th1+th2);
    res->y = BASE_Y + L1*std::sin(th1) + L2*std::sin(th1+th2);
  }

  /* ------ members ------------------------------------------------------ */
  dynamixel::PortHandler   *port_{nullptr};
  dynamixel::PacketHandler *packet_{nullptr};
  uint8_t dxl_err_{0};
  std::mutex mux_;

  rclcpp::Subscription<SetXY>::SharedPtr sub_;
  rclcpp::Service<GetXY>::SharedPtr      srv_;
};

/* ---------------- main ---------------------------------------------------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PWMNode>());
  rclcpp::shutdown();
  return 0;
}

