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
#include "ball_detector/msg/set_xy.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using namespace std::chrono_literals;
using SetXY = ball_detector::msg::SetXY;

/* ── hardware constants ------------------------------------------------ */
// constexpr uint8_t ID_1 = 4 , ID_2 = 1;
// constexpr uint8_t MODE_PWM = 16;
// constexpr uint8_t ADDR_TORQUE = 64, ADDR_MODE = 11,
//                   ADDR_POS    = 132, ADDR_PWM  = 100;

// constexpr double ZERO_1 = 2048 , ZERO_2 = 2048;
// constexpr double TICK2DEG = 360.0/4096.0;
// inline double tick2deg(uint32_t t,double z){return (int(t)-z)*TICK2DEG;}

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
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    subscriber_Cen_.subscribe(this, "set_xy", rmw_qos_profile);
    subscriber_Sta_.subscribe(this, "joint_state", rmw_qos_profile);

    pos_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::JointState, SetXY>>(subscriber_Sta_, subscriber_Cen_, 10);
    pos_sync_->registerCallback(std::bind(&PWMNode::cb_xy, this, std::placeholders::_1, std::placeholders::_2));

    pubSh_ = create_publisher<std_msgs::msg::Float64>("velS",10);
    pubEl_ = create_publisher<std_msgs::msg::Float64>("velE",10);

    timer_=create_wall_timer(10ms,std::bind(&PWMNode::loop,this));

    // RCLCPP_INFO(get_logger(),"PWM PID node: port=%s baud=%d",port_name.c_str(),baud);
  }

private:

/* ---- ROS callbacks -------------------------------------------------- */
  void cb_xy(const sensor_msgs::msg::JointState::ConstSharedPtr& msg_js, 
              const SetXY::ConstSharedPtr& msg_cen)
  {

    /* Processing data from centroid */
    double j1,j2;
    bool ok=ik(msg_cen->x,msg_cen->y,j1,j2);
    std::lock_guard<std::mutex> lk(mut_);
    if(ok){
      goal_j1_=j1; goal_j2_=j2; goal_update_=true;
      /* Processing data from joint states */
      pos_S = msg_js->position[0];
      pos_E = msg_js->position[1];
      RCLCPP_INFO(get_logger(),"XY(%.1f,%.1f) → J(%.1f,%.1f)",msg_cen->x,msg_cen->y,j1,j2);
    }else{
      RCLCPP_WARN(get_logger(),"IK fail for (%.1f,%.1f)",msg_cen->x,msg_cen->y);
    }

  }

/* ---- control loop --------------------------------------------------- */
  void loop()
  {
    // double cur1,cur2;
    // if(!read_pos(cur1,cur2)) return;

    double cur1,cur2,g1,g2;
    { std::lock_guard<std::mutex> lk(mut_);
      g1=goal_j1_; g2=goal_j2_;
      cur1 = pos_S; cur2 = pos_E;
      if(goal_update_){ pid1_={pid1_.kp,pid1_.ki,pid1_.kd};
                        pid2_={pid2_.kp,pid2_.ki,pid2_.kd};
                        goal_update_=false; }
    }

    double e1=g1-cur1, e2=g2-cur2;
    int pwm1=pid1_.compute(e1,DT,PWM_MAX);
    int pwm2=pid2_.compute(e2,DT,PWM_MAX);

    auto message_shoulder = std_msgs::msg::Float64();
    auto message_elbow = std_msgs::msg::Float64();
    message_shoulder.data = pwm1;
    message_shoulder.data = pwm2;

    pubSh_->publish(message_shoulder);
    pubSh_->publish(message_elbow);

    RCLCPP_DEBUG_THROTTLE(get_logger(),*get_clock(),200,
      "J (%.1f→%.1f / %.1f→%.1f)  PWM (%d,%d)",
      cur1,g1,cur2,g2,pwm1,pwm2);
  }

/* ---- data members --------------------------------------------------- */

  // rclcpp::Subscription<SetXY>::SharedPtr sub_;
  
  message_filters::Subscriber<sensor_msgs::msg::JointState> subscriber_Sta_;
  message_filters::Subscriber<SetXY> subscriber_Cen_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::JointState, SetXY>> pos_sync_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubSh_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubEl_;

  rclcpp::TimerBase::SharedPtr           timer_;
  std::mutex mut_;

  /* goals & PID */
  double goal_j1_{0},goal_j2_{0}; bool goal_update_{false};
  double pos_S{0},pos_E{0};
  static constexpr double DT = 0.01;
  static constexpr double    PWM_MAX = 5.236;
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


