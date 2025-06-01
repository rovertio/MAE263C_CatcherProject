/* =======================================================================
 * pwm_position_node.cpp – very-small PID → PWM joint controller
 *   • extra diagnostics on every Dynamixel call
 *   • param-driven port & baud
 *   • robust SyncRead (checks addParam / TxRx result / availability)
 * ===================================================================== */
#include <cmath>
#include <mutex>
#include <memory>
#include <chrono>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "inverse_kinematics_node/msg/set_xy.hpp"

using namespace std::chrono_literals;
using SetXY = inverse_kinematics_node::msg::SetXY;

/* ── hardware constants ------------------------------------------------ */
constexpr uint8_t ID_1 = 4 , ID_2 = 1;
constexpr uint8_t MODE_PWM = 16;
constexpr uint8_t ADDR_TORQUE = 64, ADDR_MODE = 11,
                  ADDR_POS    = 132, ADDR_PWM  = 100;

constexpr double ZERO_1 = 2048 , ZERO_2 = 2048;
constexpr double TICK2DEG = 360.0/4096.0;
inline double tick2deg(uint32_t t,double z){return (int(t)-z)*TICK2DEG;}

constexpr double L1=30,L2=30, BASE_X=-42, BASE_Y=1;
constexpr double REACH2=(L1+L2)*(L1+L2);
constexpr double J1_MIN=-80,J1_MAX=-10,J2_MIN=20,J2_MAX=140;

/* ── tiny IK (elbow-down) --------------------------------------------- */
bool ik(double x,double y,double& j1,double& j2)
{
  double xr=x-BASE_X, yr=y-BASE_Y, r2=xr*xr+yr*yr;
  if(r2>REACH2) return false;
  double c2=(r2-L1*L1-L2*L2)/(2*L1*L2); if(std::abs(c2)>1) return false;
  j2=std::acos(c2)*180/M_PI;
  double s2=std::sin(j2*M_PI/180);
  j1=std::atan2(yr,xr)-std::atan2(L2*s2,L1+L2*c2); j1*=180/M_PI;
  return (j1>J1_MIN&&j1<J1_MAX&&j2>J2_MIN&&j2<J2_MAX);
}

/* ── minimal PID ------------------------------------------------------- */
struct PID {
  double kp,ki,kd; double integ{},prev{};
  int compute(double err,double dt,int lim){
    double limf=double(lim);
    integ=std::clamp(integ+err*dt,-limf/kp,limf/kp);
    double deriv=(dt>1e-6)?(err-prev)/dt:0;
    prev=err;
    int out=int(std::clamp(kp*err+ki*integ+kd*deriv,-limf,limf));
    return out;
  }
};

/* ===================================================================== */
class PWMNode : public rclcpp::Node
{
public:
  PWMNode():Node("pwm_position_node")
  {
    /* parameters ----------------------------------------------------- */
    declare_parameter<std::string>("port","/dev/ttyUSB0");
    declare_parameter<int>("baud",1000000);
    const auto port_name = get_parameter("port").as_string();
    int baud = get_parameter("baud").as_int();

    /* open serial ---------------------------------------------------- */
    port_  = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    packet_= dynamixel::PacketHandler::getPacketHandler(2.0);
    if(!port_->openPort() || !port_->setBaudRate(baud))
      throw std::runtime_error("Cannot open "+port_name);

    init_servo(ID_1,"ID_1"); init_servo(ID_2,"ID_2");

    sub_=create_subscription<SetXY>("set_xy",10,
      std::bind(&PWMNode::cb_xy,this,std::placeholders::_1));

    timer_=create_wall_timer(10ms,std::bind(&PWMNode::loop,this));

    RCLCPP_INFO(get_logger(),"PWM PID node: port=%s baud=%d",port_name.c_str(),baud);
  }

  ~PWMNode() override{
    packet_->write1ByteTxRx(port_,0xFE,ADDR_TORQUE,0,&dxl_err_);
    port_->closePort();
  }

private:
/* ---- Dynamixel helpers --------------------------------------------- */
  void init_servo(uint8_t id,const char* tag)
  {
    auto tx=[&](uint8_t addr,uint8_t data){
      int res=packet_->write1ByteTxRx(port_,id,addr,data,&dxl_err_);
      if(res!=COMM_SUCCESS)
        RCLCPP_ERROR(get_logger(),"%s: write 0x%02X → %s",
                     tag,addr,packet_->getTxRxResult(res));
    };
    tx(ADDR_TORQUE,0);
    tx(ADDR_MODE ,MODE_PWM);
    tx(ADDR_TORQUE,1);
  }

  bool read_pos(double& j1,double& j2)
  {
    dynamixel::GroupSyncRead sr(port_,packet_,ADDR_POS,4);
    if(!sr.addParam(ID_1) || !sr.addParam(ID_2)){
      RCLCPP_ERROR(get_logger(),"addParam failed");
      return false;
    }
    int res=sr.txRxPacket();
    if(res!=COMM_SUCCESS){
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,
        "SyncRead error: %s",packet_->getTxRxResult(res));
      return false;
    }
    if(!sr.isAvailable(ID_1,ADDR_POS,4)||!sr.isAvailable(ID_2,ADDR_POS,4)){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"data not available");
      return false;
    }
    j1=tick2deg(sr.getData(ID_1,ADDR_POS,4),ZERO_1);
    j2=tick2deg(sr.getData(ID_2,ADDR_POS,4),ZERO_2);
    return true;
  }

/* ---- ROS callbacks -------------------------------------------------- */
  void cb_xy(SetXY::SharedPtr m)
  {
    double j1,j2;
    bool ok=ik(m->x,m->y,j1,j2);
    std::lock_guard<std::mutex> lk(mut_);
    if(ok){
      goal_j1_=j1; goal_j2_=j2; goal_update_=true;
      RCLCPP_INFO(get_logger(),"XY(%.1f,%.1f) → J(%.1f,%.1f)",m->x,m->y,j1,j2);
    }else{
      RCLCPP_WARN(get_logger(),"IK fail for (%.1f,%.1f)",m->x,m->y);
    }
  }

/* ---- control loop --------------------------------------------------- */
  void loop()
  {
    double cur1,cur2;
    if(!read_pos(cur1,cur2)) return;

    double g1,g2;
    { std::lock_guard<std::mutex> lk(mut_);
      g1=goal_j1_; g2=goal_j2_;
      if(goal_update_){ pid1_={pid1_.kp,pid1_.ki,pid1_.kd};
                        pid2_={pid2_.kp,pid2_.ki,pid2_.kd};
                        goal_update_=false; }
    }

    double e1=g1-cur1, e2=g2-cur2;
    int pwm1=pid1_.compute(e1,DT,PWM_MAX);
    int pwm2=pid2_.compute(e2,DT,PWM_MAX);

    dynamixel::GroupSyncWrite sw(port_,packet_,ADDR_PWM,2);
    uint8_t b1[2]={DXL_LOBYTE(pwm1),DXL_HIBYTE(pwm1)};
    uint8_t b2[2]={DXL_LOBYTE(pwm2),DXL_HIBYTE(pwm2)};
    sw.addParam(ID_1,b1); sw.addParam(ID_2,b2);
    int res=sw.txPacket();
    if(res!=COMM_SUCCESS)
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,
        "SyncWrite error: %s",packet_->getTxRxResult(res));

    RCLCPP_DEBUG_THROTTLE(get_logger(),*get_clock(),200,
      "J (%.1f→%.1f / %.1f→%.1f)  PWM (%d,%d)",
      cur1,g1,cur2,g2,pwm1,pwm2);
  }

/* ---- data members --------------------------------------------------- */
  dynamixel::PortHandler*  port_{};
  dynamixel::PacketHandler* packet_{};
  uint8_t dxl_err_{0};

  rclcpp::Subscription<SetXY>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr           timer_;
  std::mutex mut_;

  /* goals & PID */
  double goal_j1_{0},goal_j2_{0}; bool goal_update_{false};
  static constexpr double DT = 0.01;
  static constexpr int    PWM_MAX = 600;
  PID pid1_{50,1,0.01}, pid2_{50,1,0.01};
};

/* --------------------------------------------------------------------- */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PWMNode>());
  rclcpp::shutdown();
  return 0;
}


