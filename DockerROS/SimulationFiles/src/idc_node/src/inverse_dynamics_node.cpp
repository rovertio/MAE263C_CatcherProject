/* ========================================================================
 * inverse_dynamics_node.cpp — latency-optimised  +  joint-limit clamp
 * ------------------------------------------------------------------------
 *  Subscribes   :  /set_xy  (controller_msgs/SetXY, cm in camera plane)
 *  Computes     :  planar 2-DOF Operational Space ID  ➜ pwm signal
 *  Writes to    :  Dynamixel PWM Registers
 *  Publishes on :  /idc/joint_plot_data  (controller_msgs/JointPlotData)
 * ======================================================================== */
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

#ifndef PROFILING
#define PROFILING 1
#endif

using SetXY = controller_msgs::msg::SetXY;
using Plot  = controller_msgs::msg::JointPlotData;
namespace ch = std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

/* ── Joint hard-limits (deg) ───────────────────────────────────── */
constexpr double Q1_MIN_DEG = -90.0;
constexpr double Q1_MAX_DEG =  20.0;
constexpr double Q2_MIN_DEG =  10.0;
constexpr double Q2_MAX_DEG = 140.0;

/* ════════════════════════════════════════════════════════════════ */
class InverseDynamicsNode final : public rclcpp::Node
{
public:
  InverseDynamicsNode() : Node("inverse_dynamics_node")
  {
    /* parameters */
    declare_parameter("port",          std::string{"/dev/ttyUSB0"});
    declare_parameter("baud",          1'000'000);
    declare_parameter("kp1_task",      50.0);
    declare_parameter("kd1_task",       1.0);
    declare_parameter("kp2_task",      50.0);
    declare_parameter("kd2_task",       1.0);
    declare_parameter("ctrl_rate_hz",  500);

    kp1_ = get_parameter("kp1_task").as_double();
    kd1_ = get_parameter("kd1_task").as_double();
    kp2_ = get_parameter("kp2_task").as_double();
    kd2_ = get_parameter("kd2_task").as_double();
    dt_  = 1.0 / get_parameter("ctrl_rate_hz").as_int();

    /* Dynamixel init */
    port_   = dynamixel::PortHandler::getPortHandler(
                get_parameter("port").as_string().c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort() ||
        !port_->setBaudRate(get_parameter("baud").as_int()))
      throw std::runtime_error("DXL: cannot open port or set baud");

    /* bulk read: 8 bytes per ID  (vel-pos) */
    bulk_ = new dynamixel::GroupBulkRead(port_, packet_);
    sync_ = new dynamixel::GroupSyncWrite(port_, packet_,
                                          control::ADDR_GOAL_POSITION, 4);
    constexpr uint16_t VEL_ADDR = control::ADDR_PRESENT_VELOCITY; // 128
    if (!bulk_->addParam(control::ID1, VEL_ADDR, 8) ||
        !bulk_->addParam(control::ID2, VEL_ADDR, 8))
      throw std::runtime_error("DXL: GroupBulkRead addParam failed");

    for (uint8_t id : control::SERVO_IDS) {
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
      write8(id, control::ADDR_OPERATING_MODE, control::MODE_POSITION);
      write8(id, control::ADDR_TORQUE_ENABLE, 1);
    }

    /* ROS 2 I/O */
    sub_xy_   = create_subscription<SetXY>(
                  "set_xy", 10, std::bind(&InverseDynamicsNode::cb_target, this, _1));
    pub_plot_ = create_publisher<Plot>("idc/joint_plot_data", 10);
    timer_    = create_wall_timer(
                  ch::duration<double>(dt_), std::bind(&InverseDynamicsNode::cb_loop, this));

    RCLCPP_INFO(get_logger(), "IDC ready | dt=%.4f s (%.0f Hz)", dt_, 1.0/dt_);
  }

  ~InverseDynamicsNode() override
  {
    for (uint8_t id : control::SERVO_IDS)
      write8(id, control::ADDR_TORQUE_ENABLE, 0);
    delete bulk_;
    delete sync_;
    port_->closePort();
  }

private:
  /* low-level helpers */
  void write8(uint8_t id,uint16_t addr,uint8_t data)
  { uint8_t err=0; packet_->write1ByteTxRx(port_,id,addr,data,&err); }

  bool read_states(double &q1,double &q2,double &dq1,double &dq2)
  {
    if (bulk_->txRxPacket() != COMM_SUCCESS) return false;
    auto ok=[&](uint8_t id,uint16_t a){return bulk_->isAvailable(id,a,4);};
    if(!(ok(control::ID1, control::ADDR_PRESENT_POSITION ) &&
         ok(control::ID1, control::ADDR_PRESENT_VELOCITY ) &&
         ok(control::ID2, control::ADDR_PRESENT_POSITION ) &&
         ok(control::ID2, control::ADDR_PRESENT_VELOCITY ))) return false;

    int32_t v1 = bulk_->getData(control::ID1, control::ADDR_PRESENT_VELOCITY, 4);
    int32_t t1 = bulk_->getData(control::ID1, control::ADDR_PRESENT_POSITION, 4);
    int32_t v2 = bulk_->getData(control::ID2, control::ADDR_PRESENT_VELOCITY, 4);
    int32_t t2 = bulk_->getData(control::ID2, control::ADDR_PRESENT_POSITION, 4);

    dq1 = control::vel_tick2rad(v1);
    dq2 = control::vel_tick2rad(v2);
    q1  = control::tick2deg(0, t1) * M_PI/180.0;
    q2  = control::tick2deg(1, t2) * M_PI/180.0;
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

#ifdef PROFILING
  struct Stat{double avg=0,worst=0;void u(double s){const double k=.01;avg=(1-k)*avg+k*s;worst=std::max(worst,s);} } st_[5];
  uint64_t loops_=0;
#endif

  /* callbacks */
  void cb_target(const SetXY::SharedPtr m)
  {
    if(!std::isfinite(m->x)||!std::isfinite(m->y)) return;
    target_xy_ = { m->x*0.01, m->y*0.01 };
    has_target_= true;
  }

  void cb_loop()
  {
    auto t0 = ch::steady_clock::now();
    if(!has_target_) return;

    /* 1 ▸ read */
    double q1,q2,dq1,dq2;
    if(!read_states(q1,q2,dq1,dq2)) return;
    auto t1 = ch::steady_clock::now();

    /* 2 ▸ dynamics */
    Eigen::Vector2d q{q1,q2}, dq{dq1,dq2};
    Eigen::Vector2d x  = control::fk_xy(q);
    Eigen::Matrix2d J  = control::jacobian(q);
    Eigen::Vector2d xd = J*dq;
    Eigen::Vector2d e  = target_xy_ - x;
    Eigen::Vector2d ed = -xd;
    Eigen::Vector2d xdd{ kp1_*e[0]+kd1_*ed[0], kp2_*e[1]+kd2_*ed[1] };
    Eigen::Vector2d qdd = J.inverse() * (xdd - control::jdot_qdot(q,dq));
    Eigen::Vector2d tau = control::mass_matrix(q)*qdd +
                          control::coriolis(q,dq) +
                          control::gravity(q);
    auto t2 = ch::steady_clock::now();

    /* 3 ▸ write (with clamp) */
    Eigen::Vector2d dqdeg = control::tau_to_deg(tau);
    Eigen::Vector2d qdeg  = (q*180.0/M_PI) + dqdeg;

    qdeg[0] = std::clamp(qdeg[0], Q1_MIN_DEG, Q1_MAX_DEG);
    qdeg[1] = std::clamp(qdeg[1], Q2_MIN_DEG, Q2_MAX_DEG);

    send_goal(control::deg2tick(0,qdeg[0]), control::deg2tick(1,qdeg[1]));
    auto t3 = ch::steady_clock::now();

    /* 4 ▸ publish */
    Plot p; p.stamp=now(); p.q1_deg=qdeg[0]; p.q2_deg=qdeg[1];
    p.x = x[0]; p.y = x[1]; pub_plot_->publish(p);
    auto t4 = ch::steady_clock::now();

#ifdef PROFILING
    double us_r=ch::duration_cast<ch::microseconds>(t1-t0).count();
    double us_d=ch::duration_cast<ch::microseconds>(t2-t1).count();
    double us_w=ch::duration_cast<ch::microseconds>(t3-t2).count();
    double us_p=ch::duration_cast<ch::microseconds>(t4-t3).count();
    double us_t=ch::duration_cast<ch::microseconds>(t4-t0).count();
    st_[0].u(us_r); st_[1].u(us_d); st_[2].u(us_w); st_[3].u(us_p); st_[4].u(us_t);
    if(++loops_%100==0 || us_t>dt_*1e6)
      RCLCPP_INFO(get_logger(),
        "LOOP #%llu  avg_us [read %.1f | dyn %.1f | write %.1f | pub %.1f | total %.1f]  worst_total %.1f",
        static_cast<unsigned long long>(loops_),
        st_[0].avg, st_[1].avg, st_[2].avg, st_[3].avg, st_[4].avg, st_[4].worst);
#endif
  }

  /* members */
  rclcpp::Subscription<SetXY>::SharedPtr sub_xy_;
  rclcpp::Publisher<Plot>::SharedPtr     pub_plot_;
  rclcpp::TimerBase::SharedPtr           timer_;

  dynamixel::PortHandler*   port_{};
  dynamixel::PacketHandler* packet_{};
  dynamixel::GroupBulkRead*  bulk_{};
  dynamixel::GroupSyncWrite* sync_{};

  Eigen::Vector2d target_xy_{0,0};
  bool            has_target_{false};

  double kp1_,kd1_,kp2_,kd2_,dt_;
};

/* ── main ──────────────────────────────────────────────────────── */
int main(int argc,char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}


