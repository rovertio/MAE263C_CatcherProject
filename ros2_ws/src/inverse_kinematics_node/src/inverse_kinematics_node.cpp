#include <memory>
#include <cmath>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "inverse_kinematics_node/msg/set_xy.hpp"
#include "inverse_kinematics_node/srv/get_xy.hpp"

using SetXY = inverse_kinematics_node::msg::SetXY;
using GetXY = inverse_kinematics_node::srv::GetXY;

/*----------------------- hardware constants (from joint_positions_node) ----*/
constexpr uint8_t  ID_1 = 4;           // shoulder
constexpr uint8_t  ID_2 = 1;           // elbow
constexpr double   ZERO_TICK_1 = 691;
constexpr double   ZERO_TICK_2 = 2530;
constexpr double   TICKS_PER_DEG = 4096.0/360.0;

inline uint32_t deg_to_tick_1(double d){return uint32_t(ZERO_TICK_1+d*TICKS_PER_DEG);}
inline uint32_t deg_to_tick_2(double d){return uint32_t(ZERO_TICK_2+d*TICKS_PER_DEG);}
inline double tick_to_deg_1(uint32_t tick)
{ return (static_cast<int32_t>(tick) - ZERO_TICK_1) / TICKS_PER_DEG; }
inline double tick_to_deg_2(uint32_t tick)
{ return (static_cast<int32_t>(tick) - ZERO_TICK_2) / TICKS_PER_DEG; }

/*----------------------- kinematic constants --------------------------------*/
constexpr double L1 = 30.0, L2 = 30.0;       // link lengths
constexpr double BASE_X = -42.0, BASE_Y = 0; // base in world

/* limits */
constexpr double MIN_J1 = 0.0 , MAX_J1 = 70.0;
constexpr double MIN_J2 = 0.0 , MAX_J2 = 135.0;

/* Dynamixel addresses */
constexpr uint8_t ADDR_GOAL_POSITION    = 116;
constexpr uint8_t ADDR_PRESENT_POSITION = 132;
constexpr uint8_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint8_t ADDR_OPERATING_MODE   = 11;
constexpr uint8_t MODE_POSITION         = 3;

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
  j1_deg = -theta1 * 180.0/M_PI;    // motor positive CW
  j2_deg =  theta2 * 180.0/M_PI;    // motor positive CCW

  /* limits */
  if(j1_deg < MIN_J1 || j1_deg > MAX_J1) return false;
  if(j2_deg < MIN_J2 || j2_deg > MAX_J2) return false;
  return true;
}

/*----------------------- Node class -----------------------------------------*/
class IKNode : public rclcpp::Node
{
public:
  IKNode(): Node("inverse_kinematics_node")
  {
    declare_parameter<std::string>("port","/dev/ttyUSB0");
    declare_parameter<int>("baudrate",57600);
    port_   = dynamixel::PortHandler::getPortHandler(
                get_parameter("port").as_string().c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if(!port_->openPort() || !port_->setBaudRate(get_parameter("baudrate").as_int()))
      throw std::runtime_error("Serial open failed");

    setup_servo(ID_1);
    setup_servo(ID_2);

    sub_ = create_subscription<SetXY>("set_xy",10,
      [this](SetXY::SharedPtr m){handle_set(m);} );

    srv_ = create_service<GetXY>("get_xy",
      [this](GetXY::Request::SharedPtr,const GetXY::Response::SharedPtr r){
          handle_get(r);} );

    RCLCPP_INFO(get_logger(),"IK node ready.");
  }
  ~IKNode() override {
    std::lock_guard<std::mutex> lk(mx_);
    packet_->write1ByteTxRx(port_,0xFE,ADDR_TORQUE_ENABLE,0,&dxl_err_);
    port_->closePort();
  }

private:
  void setup_servo(uint8_t id){
    packet_->write1ByteTxRx(port_,id,ADDR_TORQUE_ENABLE,0,&dxl_err_);
    packet_->write1ByteTxRx(port_,id,ADDR_OPERATING_MODE,MODE_POSITION,&dxl_err_);
    packet_->write1ByteTxRx(port_,id,ADDR_TORQUE_ENABLE,1,&dxl_err_);
  }

  void handle_set(SetXY::SharedPtr msg)
  {
    double j1,j2;
    if(!cartesian_to_joint(msg->x,msg->y,j1,j2)){
      RCLCPP_ERROR(get_logger(),"Target (%.1f,%.1f) unreachable",msg->x,msg->y);
      return;
    }
    uint32_t t1=deg_to_tick_1(j1), t2=deg_to_tick_2(j2);

    dynamixel::GroupSyncWrite sw(port_,packet_,ADDR_GOAL_POSITION,4);
    uint8_t buf1[4]={uint8_t(t1),uint8_t(t1>>8),uint8_t(t1>>16),uint8_t(t1>>24)};
    uint8_t buf2[4]={uint8_t(t2),uint8_t(t2>>8),uint8_t(t2>>16),uint8_t(t2>>24)};
    sw.addParam(ID_1,buf1); sw.addParam(ID_2,buf2);

    std::lock_guard<std::mutex> lk(mx_);
    if(sw.txPacket()!=COMM_SUCCESS)
      RCLCPP_ERROR(get_logger(),"SyncWrite failed");
  }

  void handle_get(GetXY::Response::SharedPtr res)
  {
    dynamixel::GroupSyncRead sr(port_,packet_,ADDR_PRESENT_POSITION,4);
    sr.addParam(ID_1); sr.addParam(ID_2);

    std::lock_guard<std::mutex> lock(mx_);
    if(sr.txRxPacket()!=COMM_SUCCESS){
      res->x=res->y=0; return;
    }
    uint32_t t1=sr.getData(ID_1,ADDR_PRESENT_POSITION,4);
    uint32_t t2=sr.getData(ID_2,ADDR_PRESENT_POSITION,4);

    double j1 = tick_to_deg_1(t1); // math θ1
    double j2 = tick_to_deg_2(t2); // θ2 math

    /* forward kinematics (elbow-down) */
    double theta1 = -j1*M_PI/180.0;           // back to math sign
    double theta2 =  j2*M_PI/180.0;
    double x = BASE_X + L1*std::cos(theta1)+L2*std::cos(theta1+theta2);
    double y = BASE_Y + L1*std::sin(theta1)+L2*std::sin(theta1+theta2);

    res->x = x; res->y = y;
  }

  /* data */
  dynamixel::PortHandler   *port_;
  dynamixel::PacketHandler *packet_;
  uint8_t dxl_err_{0};
  std::mutex mx_;

  rclcpp::Subscription<SetXY>::SharedPtr sub_;
  rclcpp::Service<GetXY>::SharedPtr      srv_;
};

/*--- main ---*/
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}
