/* =======================================================================
 * adaptive_operational_control.cpp — Slotine–Li adaptive OSC
 *  for a planar 2-DOF arm, 500 Hz, bulk+sync.
 * ======================================================================= */
#include <chrono>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <controller_msgs/msg/set_xy.hpp>
#include <controller_msgs/msg/joint_plot_data.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <Eigen/Dense>

#include "control_common/control_params.hpp"
#include "control_common/control_utils.hpp"

using SetXY = controller_msgs::msg::SetXY;
using Plot  = controller_msgs::msg::JointPlotData;
namespace ch = std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

/* hard joint limits (deg) */
constexpr double Q1_MIN_DEG=-90.0, Q1_MAX_DEG=20.0;
constexpr double Q2_MIN_DEG= 10.0, Q2_MAX_DEG=140.0;

/* number of base dynamic parameters in regressor */
static constexpr int P = 10;

/* ===================================================================== */
class AdaptiveOSC final : public rclcpp::Node
{
public:
  AdaptiveOSC() : Node("adaptive_operational_control")
  {
    /* parameters ---------------------------------------------------- */
    declare_parameter("port",        std::string{"/dev/ttyUSB0"});
    declare_parameter("baud",        1'000'000);
    declare_parameter("kd_diag",     std::vector<double>{25,25});
    declare_parameter("lambda",      6.0);
    declare_parameter("kpi_diag",    std::vector<double>(P,1.0));
    declare_parameter("pi_init",     std::vector<double>(P,0.0));
    declare_parameter("ctrl_rate_hz",500);

    auto kd_v = get_parameter("kd_diag").as_double_array();
    KD_ << kd_v[0],0,
           0,      kd_v[1];

    lambda_ = get_parameter("lambda").as_double();

    auto kpi_v = get_parameter("kpi_diag").as_double_array();
    KpiInv_.setZero();
    for(int i=0;i<P;++i) KpiInv_(i,i)=1.0/std::max(1e-6,kpi_v[i]);

    auto pi0_v = get_parameter("pi_init").as_double_array();
    for(int i=0;i<P;++i) pi_hat_[i]=pi0_v[i];

    dt_ = 1.0/get_parameter("ctrl_rate_hz").as_int();

    /* Dynamixel ----------------------------------------------------- */
    port_=dynamixel::PortHandler::getPortHandler(
            get_parameter("port").as_string().c_str());
    packet_=dynamixel::PacketHandler::getPacketHandler(2.0);
    if(!port_->openPort() ||
       !port_->setBaudRate(get_parameter("baud").as_int()))
      throw std::runtime_error("DXL port");

    bulk_=new dynamixel::GroupBulkRead(port_,packet_);
    sync_=new dynamixel::GroupSyncWrite(port_,packet_,
                                        control::ADDR_GOAL_POSITION,4);
    constexpr uint16_t VEL_ADDR=control::ADDR_PRESENT_VELOCITY;
    if(!bulk_->addParam(control::ID1,VEL_ADDR,8)||
       !bulk_->addParam(control::ID2,VEL_ADDR,8))
      throw std::runtime_error("bulk addParam");

    for(uint8_t id: control::SERVO_IDS){
      write8(id,control::ADDR_TORQUE_ENABLE,0);
      write8(id,control::ADDR_OPERATING_MODE,control::MODE_POSITION);
      write8(id,control::ADDR_TORQUE_ENABLE,1);
    }

    /* ROS I/O ------------------------------------------------------- */
    sub_xy_=create_subscription<SetXY>("set_xy",10,
              std::bind(&AdaptiveOSC::cb_target,this,_1));
    pub_plot_=create_publisher<Plot>("adaptive_osc/joint_plot_data",10);
    timer_=create_wall_timer(ch::duration<double>(dt_),
              std::bind(&AdaptiveOSC::cb_loop,this));

    RCLCPP_INFO(get_logger(),
      "Adaptive OSC ready  dt=%g  λ=%g",dt_,lambda_);
  }

  ~AdaptiveOSC() override
  {
    for(uint8_t id: control::SERVO_IDS)
      write8(id,control::ADDR_TORQUE_ENABLE,0);
    delete bulk_; delete sync_; port_->closePort();
  }

private:
  /* low-level helpers ---------------------------------------------- */
  void write8(uint8_t id,uint16_t a,uint8_t d)
  { uint8_t err{}; packet_->write1ByteTxRx(port_,id,a,d,&err); }

  bool read_state(double &q1,double &q2,double &dq1,double &dq2)
  {
    if(bulk_->txRxPacket()!=COMM_SUCCESS) return false;
    auto ok=[&](uint8_t id,uint16_t a){return bulk_->isAvailable(id,a,4);};
    if(!(ok(control::ID1,control::ADDR_PRESENT_POSITION)&&
         ok(control::ID1,control::ADDR_PRESENT_VELOCITY)&&
         ok(control::ID2,control::ADDR_PRESENT_POSITION)&&
         ok(control::ID2,control::ADDR_PRESENT_VELOCITY))) return false;

    int32_t v1=bulk_->getData(control::ID1,control::ADDR_PRESENT_VELOCITY,4);
    int32_t p1=bulk_->getData(control::ID1,control::ADDR_PRESENT_POSITION,4);
    int32_t v2=bulk_->getData(control::ID2,control::ADDR_PRESENT_VELOCITY,4);
    int32_t p2=bulk_->getData(control::ID2,control::ADDR_PRESENT_POSITION,4);

    dq1=control::vel_tick2rad(v1); dq2=control::vel_tick2rad(v2);
    q1 =control::tick2deg(0,p1)*M_PI/180.0;
    q2 =control::tick2deg(1,p2)*M_PI/180.0;
    return true;
  }

  void send_goal(uint32_t t1,uint32_t t2)
  {
    uint8_t b1[4]={DXL_LOBYTE(DXL_LOWORD(t1)),DXL_HIBYTE(DXL_LOWORD(t1)),
                   DXL_LOBYTE(DXL_HIWORD(t1)),DXL_HIBYTE(DXL_HIWORD(t1))};
    uint8_t b2[4]={DXL_LOBYTE(DXL_LOWORD(t2)),DXL_HIBYTE(DXL_LOWORD(t2)),
                   DXL_LOBYTE(DXL_HIWORD(t2)),DXL_HIBYTE(DXL_HIWORD(t2))};
    sync_->clearParam();
    sync_->addParam(control::ID1,b1);
    sync_->addParam(control::ID2,b2);
    sync_->txPacket();
  }

  /* target --------------------------------------------------------- */
  void cb_target(const SetXY::SharedPtr m)
  {
    if(!std::isfinite(m->x)||!std::isfinite(m->y)) return;
    target_xy_ = {m->x*0.01, m->y*0.01};
    has_target_=true;
  }

  /* main loop ------------------------------------------------------ */
  void cb_loop()
  {
    if(!has_target_) return;

    /* 1 ▸ read state ---------------------------------------------- */
    double q1,q2,dq1,dq2;
    if(!read_state(q1,q2,dq1,dq2)) return;
    Eigen::Vector2d q{q1,q2}, dq{dq1,dq2};

    /* 2 ▸ task kinematics ----------------------------------------- */
    Eigen::Vector2d x  = control::fk_xy(q);
    Eigen::Matrix2d J  = control::jacobian(q);
    Eigen::Vector2d xd = J*dq;

    Eigen::Vector2d e  = target_xy_ - x;
    Eigen::Vector2d x_r_dot  = lambda_ * e;           // desired xd = 0
    Eigen::Vector2d x_r_ddot = -lambda_ * xd;

    /* 3 ▸ map reference to joint space ---------------------------- */
    Eigen::Vector2d q_r_dot  = J.inverse() * x_r_dot;
    Eigen::Vector2d Jdqdot   = control::jdot_qdot(q,dq);
    Eigen::Vector2d q_r_ddot = J.inverse() * (x_r_ddot - Jdqdot);

    Eigen::Vector2d sigma = dq - q_r_dot;

    /* 4 ▸ parameter update --------------------------------------- */
    auto Y = control::detail::regressor(q,dq,q_r_ddot);
    pi_hat_ += dt_ * KpiInv_ * (Y.transpose()*sigma);
    control::clamp_vec<P>(pi_hat_, 20.0);

    /* 5 ▸ control torque ----------------------------------------- */
    Eigen::Vector2d tau = Y * pi_hat_ + KD_ * sigma;

    /* 6 ▸ torque→position command -------------------------------- */
    Eigen::Vector2d qdeg = (q*180.0/M_PI) + control::tau_to_deg(tau);
    qdeg[0]=std::clamp(qdeg[0],Q1_MIN_DEG,Q1_MAX_DEG);
    qdeg[1]=std::clamp(qdeg[1],Q2_MIN_DEG,Q2_MAX_DEG);
    send_goal(control::deg2tick(0,qdeg[0]),control::deg2tick(1,qdeg[1]));

    /* 7 ▸ publish ------------------------------------------------- */
    Plot p; p.stamp=now();
    p.q1_deg=qdeg[0]; p.q2_deg=qdeg[1];
    p.q1_des_deg=qdeg[0]+e[0]*180.0/M_PI;  // quick visual
    p.q2_des_deg=qdeg[1]+e[1]*180.0/M_PI;
    p.e1=e[0]; p.e2=e[1];
    pub_plot_->publish(p);
  }

  /* members ------------------------------------------------------- */
  rclcpp::Subscription<SetXY>::SharedPtr sub_xy_;
  rclcpp::Publisher<Plot>::SharedPtr     pub_plot_;
  rclcpp::TimerBase::SharedPtr           timer_;

  dynamixel::PortHandler*   port_{};
  dynamixel::PacketHandler* packet_{};
  dynamixel::GroupBulkRead*  bulk_{};
  dynamixel::GroupSyncWrite* sync_{};

  Eigen::Vector2d target_xy_{0,0};
  bool   has_target_{false};

  Eigen::Matrix2d KD_{Eigen::Matrix2d::Identity()*25.0};
  double lambda_{6.0};
  Eigen::Matrix<double,P,1>  pi_hat_{Eigen::Matrix<double,P,1>::Zero()};
  Eigen::Matrix<double,P,P>  KpiInv_{Eigen::Matrix<double,P,P>::Identity()};
  double dt_{0.002};
};

/* main -------------------------------------------------------------- */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<AdaptiveOSC>());
  rclcpp::shutdown();
  return 0;
}

