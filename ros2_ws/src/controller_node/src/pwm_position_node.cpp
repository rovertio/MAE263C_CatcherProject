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
#include "ball_detector/msg/set_xy.hpp"
#include "controller_node/msg/joint_plot_data.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using namespace std::chrono_literals;
using SetXY = ball_detector::msg::SetXY;
using Plot  = controller_node::msg::JointPlotData;

/* ── hardware constants ------------------------------------------------ */

constexpr double L1=30,L2=30, BASE_X=-42, BASE_Y=1;
constexpr double REACH2=(L1+L2)*(L1+L2);
constexpr double J1_MIN=-80,J1_MAX=-10,J2_MIN=20,J2_MAX=140;
// double P_GAIN=125;
// double I_GAIN=0.25;
// double D_GAIN=30;


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
  double kp{}, ki{}, kd{}; 
  double integ{}, prev{};

  double compute(double err,double dt,double lim){
    double limf=double(lim);

    integ=std::clamp(integ+err*dt,-limf/kp,limf/kp);
    double deriv=(dt>1e-6)?(err-prev)/dt:0;
    prev=err;

    double out= double(std::clamp(kp*err + ki*integ + kd*deriv, -limf,limf));
    return out;
  }
};

/* ===================================================================== */
class PWMNode : public rclcpp::Node
{
public:
  PWMNode():Node("pwm_position_node")
  {
    
    declare_parameter<double>("kp", 50.0);
    declare_parameter<double>("ki",  0.1);
    declare_parameter<double>("kd",   5.0);
    // declare_parameter<double>("max_pwm", 5.236);
    // declare_parameter<double>("max_pwm", 100.0); 
    //    P: 35, I: 2.15, D: 10.0
    declare_parameter<double>("max_pwm", 1000000.0);
    //    P: 75, I: 0.12, D: 0.1

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    max_pwm_ = get_parameter("max_pwm").as_double();
    // dt_s_ = 1.0 / get_parameter("ctrl_rate_hz").as_int();
    // static constexpr double DT = 0.001;

    // Initialize PID controllers with the parameters
    pid1_.kp = kp_; pid1_.ki = ki_; pid1_.kd = kd_;
    pid2_.kp = kp_; pid2_.ki = ki_; pid2_.kd = kd_;
    

    subCen_ = create_subscription<SetXY>(
				"/set_xy", 10, std::bind(&PWMNode::cb_xy, this, std::placeholders::_1));
    subSta_ = create_subscription<sensor_msgs::msg::JointState>(
				"/joint_state", 10, std::bind(&PWMNode::cb_state, this, std::placeholders::_1));

    pubSh_ = create_publisher<std_msgs::msg::Float64>("/velS",10);
    pubEl_ = create_publisher<std_msgs::msg::Float64>("/velE",10);
    pub_plot_ = create_publisher<Plot>("pwm/joint_plot_data", 10);

    timer_=create_wall_timer(10ms,std::bind(&PWMNode::loop,this));
    last_good_cmd_time_ = now();
    
    RCLCPP_INFO(get_logger(), "PWM node ready (kp=%.1f ki=%.1f kd=%.1f)", kp_, ki_, kd_);
    // RCLCPP_INFO(get_logger(),"PWM PID node: port=%s baud=%d",port_name.c_str(),baud);
  }

private:

/* ---- ROS callbacks -------------------------------------------------- */

  void cb_state(const sensor_msgs::msg::JointState::ConstSharedPtr& msg_js)
  {
    pos_S = msg_js->position[0] * 180/M_PI;
    pos_E = msg_js->position[1] * 180/M_PI;
    // pos_S = msg_js->position[0] * 180/M_PI;
    // pos_E = msg_js->position[1] * 180/M_PI;

    // this->get_parameter("kp", kp_);
    // this->get_parameter("ki", ki_);
    // this->get_parameter("kd", kd_);

    // double kp_ = get_parameter("kp").as_double();
    // double ki_ = get_parameter("ki").as_double();
    // double kd_ = get_parameter("kd").as_double();
  }

  void cb_xy(const SetXY::ConstSharedPtr& msg_cen)
  {

    /* Processing data from centroid */
    double j1,j2;
    bool ok=ik(msg_cen->x,msg_cen->y,j1,j2);
    std::lock_guard<std::mutex> lk(mut_);
    if(ok){
      goal_j1_=j1; goal_j2_=j2; goal_update_=true;
      last_good_cmd_time_ = now();
      RCLCPP_INFO(get_logger(),"XY(%.1f,%.1f) → J(%.1f,%.1f)",msg_cen->x,msg_cen->y,j1,j2);
    }
    else{
      RCLCPP_WARN(get_logger(),"IK fail for (%.1f,%.1f)",msg_cen->x,msg_cen->y);
    }

  }

/* ---- control loop --------------------------------------------------- */
  void loop()
  {
    // double cur1,cur2;
    // if(!read_pos(cur1,cur2)) return;
    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    pid1_.kp = kp_; pid2_.kp = kp_;
    pid1_.ki = ki_; pid2_.ki = ki_;
    pid1_.kd = kd_; pid2_.kd = kd_;


    if (!goal_update_ || (now() - last_good_cmd_time_).seconds() > 0.1){
      // RCLCPP_INFO(get_logger(),"waiting for new command input...");
      RCLCPP_INFO(get_logger(),"Pgain(%.2f) | Igain(%.2f) | Dgain(%.2f)",
      kp_, ki_, kd_);

      auto message_shoulder = std_msgs::msg::Float64();
      auto message_elbow = std_msgs::msg::Float64();
      message_shoulder.data = 0.0;
      message_elbow.data = 0.0;
  
      pubSh_->publish(message_shoulder);
      pubEl_->publish(message_elbow);
    }
    else{
      double cur1,cur2,g1,g2;
      { std::lock_guard<std::mutex> lk(mut_);
        g1=goal_j1_; g2=goal_j2_;
        cur1 = pos_S; cur2 = pos_E;
        if(goal_update_){ 
          pid1_.integ = 0.0; pid1_.prev = 0.0;
          pid2_.integ = 0.0; pid2_.prev = 0.0;
          // pid1_={pid1_.kp,pid1_.ki,pid1_.kd};
          // pid2_={pid2_.kp,pid2_.ki,pid2_.kd};
          goal_update_=false; 
        }
      }
  
      double e1=g1-cur1, e2=g2-cur2;
      double pwm1 = pid1_.compute(e1, DT_, max_pwm_);
      double pwm2 = pid2_.compute(e2, DT_, max_pwm_);
  
      RCLCPP_INFO(get_logger(),"Error1(%.1f) | Error2(%.1f) | PWM1(%.1f) | PWM2(%.1f)",
      e1, e2, pwm1, pwm2);
      // RCLCPP_INFO(get_logger(),"Pgain(%.1f) | Igain(%.1f) | Dgain(%.1f)",
      // kp_, ki_, kd_);
      RCLCPP_INFO(get_logger(),"MaxCom(%.1f) | timeStep(%.1f)",
      max_pwm_, DT_);

      Plot p;
      p.stamp = now();
      p.q1_deg = cur1;  p.q2_deg = cur2;
      p.q1_des_deg = g1; p.q2_des_deg = g2;
      p.e1 = g1 - cur1;  p.e2 = g2 - cur2;
      pub_plot_->publish(p);
      
  
      auto message_shoulder = std_msgs::msg::Float64();
      auto message_elbow = std_msgs::msg::Float64();
      message_shoulder.data = pwm1;
      message_elbow.data = pwm2;
  
      pubSh_->publish(message_shoulder);
      pubEl_->publish(message_elbow);

      goal_update_=false;
    }

    // auto message_shoulder = std_msgs::msg::Float64();
    // auto message_elbow = std_msgs::msg::Float64();
    // message_shoulder.data = pwm1;
    // message_elbow.data = pwm2;

    // pubSh_->publish(message_shoulder);
    // pubEl_->publish(message_elbow);

  }

/* ---- data members --------------------------------------------------- */

  rclcpp::Subscription<SetXY>::SharedPtr subCen_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subSta_;
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubSh_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubEl_;

  rclcpp::Publisher<Plot>::SharedPtr pub_plot_;
  rclcpp::TimerBase::SharedPtr           timer_;
  std::mutex mut_;
  rclcpp::Time last_good_cmd_time_;

  static constexpr double DT_ = 0.001;
  /* goals & PID */
  double goal_j1_{0},goal_j2_{0}; bool goal_update_{false};
  double pos_S{0},pos_E{0};
  double kp_{}, ki_{}, kd_{};
  double max_pwm_{};
  PID pid1_{}, pid2_{};
};

/* --------------------------------------------------------------------- */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PWMNode>());
  rclcpp::shutdown();
  return 0;
}

