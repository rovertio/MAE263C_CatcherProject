/* =======================================================================
 * robust_joint_control.cpp  —  Robust Joint-Space Inverse-Dynamics Control
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

/* ---- joint hard limits (deg) ---------------------------------------- */
constexpr double Q1_MIN_DEG = -90.0, Q1_MAX_DEG =  20.0;
constexpr double Q2_MIN_DEG =  10.0, Q2_MAX_DEG = 140.0;

/* ===================================================================== */
class RobustJointControl : public rclcpp::Node
{
public:
  RobustJointControl() : Node("robust_joint_control")
  {
    /* ---- parameters -------------------------------------------------- */
    declare_parameter("port",          std::string{"/dev/ttyUSB0"});
    declare_parameter("baud",          1'000'000);
    declare_parameter("kp1_joint",     50.0);
    declare_parameter("kd1_joint",      1.0);
    declare_parameter("kp2_joint",     50.0);
    declare_parameter("kd2_joint",      1.0);
    declare_parameter("rho",            5.0);
    declare_parameter("epsilon",        0.01);
    declare_parameter("ctrl_rate_hz",  500);

    kp1_ = get_parameter("kp1_joint").as_double();
    kd1_ = get_parameter("kd1_joint").as_double();
    kp2_ = get_parameter("kp2_joint").as_double();
    kd2_ = get_parameter("kd2_joint").as_double();
    rho_ = get_parameter("rho").as_double();
    eps_ = get_parameter("epsilon").as_double();
    dt_  = 1.0 / get_parameter("ctrl_rate_hz").as_int();

    /* ---- Dynamixel init (identical to joint_space_idc) --------------- */
    port_   = dynamixel::PortHandler::getPortHandler(
                get_parameter("port").as_string().c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort() || !port_->setBaudRate(get_parameter("baud").as_int()))
      throw std::runtime_error("DXL: cannot open / set baud");

    bulk_ = new dynamixel::GroupBulkRead(port_, packet_);
    sync_ = new dynamixel::GroupSyncWrite(port_, packet_,
                                          control::ADDR_GOAL_POSITION, 4);
    constexpr uint16_t VEL_ADDR = control::ADDR_PRESENT_VELOCITY;
    if (!bulk_->addParam(control::ID1, VEL_ADDR, 8) ||
        !bulk_->addParam(control::ID2, VEL_ADDR, 8))
      throw std::runtime_error("DXL: GroupBulkRead addParam failed");

    for (uint8_t id : control::SERVO_IDS) {
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
      write8(id, control::ADDR_OPERATING_MODE, control::MODE_POSITION);
      write8(id, control::ADDR_TORQUE_ENABLE, 1);
    }

    /* ---- ROS I/O ----------------------------------------------------- */
    sub_xy_   = create_subscription<SetXY>("set_xy", 10,
                  std::bind(&RobustJointControl::cb_target, this, _1));
    pub_plot_ = create_publisher<Plot>("robust_joint/joint_plot_data", 10);
    timer_    = create_wall_timer(ch::duration<double>(dt_),
                  std::bind(&RobustJointControl::cb_loop, this));

    RCLCPP_INFO(get_logger(),
      "Robust IDC ready  dt=%g s | rho=%g eps=%g", dt_, rho_, eps_);
  }

  ~RobustJointControl() override
  {
    for (uint8_t id : control::SERVO_IDS)
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
    delete bulk_; delete sync_; port_->closePort();
  }

private:
  /* ---- low-level helpers ------------------------------------------- */
  void write8(uint8_t id, uint16_t addr, uint8_t data)
  { uint8_t err{}; packet_->write1ByteTxRx(port_, id, addr, data, &err); }

  bool read_state(double &q1,double &q2,double &dq1,double &dq2)
  {
    if (bulk_->txRxPacket() != COMM_SUCCESS) return false;
    auto ok=[&](uint8_t id,uint16_t a){return bulk_->isAvailable(id,a,4);};
    if(!(ok(control::ID1, control::ADDR_PRESENT_POSITION) &&
         ok(control::ID1, control::ADDR_PRESENT_VELOCITY) &&
         ok(control::ID2, control::ADDR_PRESENT_POSITION) &&
         ok(control::ID2, control::ADDR_PRESENT_VELOCITY))) return false;

    int32_t v1=bulk_->getData(control::ID1, control::ADDR_PRESENT_VELOCITY,4);
    int32_t p1=bulk_->getData(control::ID1, control::ADDR_PRESENT_POSITION,4);
    int32_t v2=bulk_->getData(control::ID2, control::ADDR_PRESENT_VELOCITY,4);
    int32_t p2=bulk_->getData(control::ID2, control::ADDR_PRESENT_POSITION,4);

    dq1 = control::vel_tick2rad(v1);
    dq2 = control::vel_tick2rad(v2);
    q1  = control::tick2deg(0,p1)*M_PI/180.0;
    q2  = control::tick2deg(1,p2)*M_PI/180.0;
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

  /* ---- callbacks ---------------------------------------------------- */
  void cb_target(const SetXY::SharedPtr m)
  {
    if(!std::isfinite(m->x)||!std::isfinite(m->y)) return;
    double j1,j2;
    if(!control::ik_xy(m->x*0.01, m->y*0.01, j1, j2)){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IK fail (%.1f,%.1f)", m->x, m->y);
      return;
    }
    qd_ = {j1,j2};  has_target_ = true;
  }

  void cb_loop()
  {
    if(!has_target_) return;

    double q1,q2,dq1,dq2;
    if(!read_state(q1,q2,dq1,dq2)) return;

    Eigen::Vector2d q{q1,q2},               dq{dq1,dq2};
    Eigen::Vector2d qd = qd_,               dqd = {0,0},  ddqd = {0,0};
    Eigen::Vector2d q_tilde  = qd - q;
    Eigen::Vector2d dq_tilde = dqd - dq;

    /* ---- error state ξ & sliding vector z --------------------------- */
    Eigen::Vector4d xi; xi << q_tilde, dq_tilde;
    Eigen::Matrix<double,2,4> D_T;  // actually Dᵀ with rows that pick velocities
    D_T.setZero();  D_T(0,2)=1; D_T(1,3)=1;
    Eigen::Vector2d z = D_T * xi;   // Q = I so Dᵀ Q ξ = Dᵀ ξ

    /* ---- robust term ------------------------------------------------ */
    Eigen::Vector2d w = control::robust_w(z, rho_, eps_);

    /* ---- desired accel (PD+robust) ---------------------------------- */
    Eigen::Vector2d qdd_star;
    qdd_star[0] = ddqd[0] + kd1_*dq_tilde[0] + kp1_*q_tilde[0] + w[0];
    qdd_star[1] = ddqd[1] + kd2_*dq_tilde[1] + kp2_*q_tilde[1] + w[1];

    Eigen::Vector2d tau = control::mass_matrix(q)*qdd_star +
                          control::coriolis(q,dq) +
                          control::gravity(q);

    /* ---- position-mode conversion ---------------------------------- */
    Eigen::Vector2d qdeg = (q*180.0/M_PI) + control::tau_to_deg(tau);
    qdeg[0]=std::clamp(qdeg[0],Q1_MIN_DEG,Q1_MAX_DEG);
    qdeg[1]=std::clamp(qdeg[1],Q2_MIN_DEG,Q2_MAX_DEG);
    send_goal(control::deg2tick(0,qdeg[0]), control::deg2tick(1,qdeg[1]));

    /* ---- publish plot ---------------------------------------------- */
    Plot p;
    p.stamp=now();
    p.q1_deg=qdeg[0]; p.q2_deg=qdeg[1];
    p.q1_des_deg=qd[0]*180.0/M_PI;
    p.q2_des_deg=qd[1]*180.0/M_PI;
    p.e1=p.q1_des_deg-p.q1_deg;
    p.e2=p.q2_des_deg-p.q2_deg;
    auto xy = control::fk_xy(q);
    p.x=xy[0]; p.y=xy[1];
    pub_plot_->publish(p);
  }

  /* ---- members ----------------------------------------------------- */
  rclcpp::Subscription<SetXY>::SharedPtr sub_xy_;
  rclcpp::Publisher<Plot>::SharedPtr     pub_plot_;
  rclcpp::TimerBase::SharedPtr           timer_;

  dynamixel::PortHandler*   port_{};
  dynamixel::PacketHandler* packet_{};
  dynamixel::GroupBulkRead*  bulk_{};
  dynamixel::GroupSyncWrite* sync_{};

  Eigen::Vector2d qd_{0,0};
  bool   has_target_{false};
  double kp1_,kd1_,kp2_,kd2_,rho_,eps_,dt_;
};

/* main ------------------------------------------------------------------ */
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<RobustJointControl>());
  rclcpp::shutdown();
  return 0;
}

