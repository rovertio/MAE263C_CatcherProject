// inverse_dynamics_node.cppinverse_dynamics_node.cpp
#include <cmath>
#include <memory>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <optional>
#include <array>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "inverse_kinematics_node/msg/set_xy.hpp"
#include "idc_node/msg/joint_plot_data.hpp"

using SetXY = inverse_kinematics_node::msg::SetXY;
using JointPlotData = idc_node::msg::JointPlotData;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
inline double deg2rad(double d){ return d*M_PI/180.0; }
inline double rad2deg(double r){ return r*180.0/M_PI; }

constexpr uint8_t ID1 = 4, ID2 = 1;
constexpr uint8_t MODE_PWM = 16;
constexpr uint8_t ADDR_TORQUE = 64, ADDR_MODE = 11;
constexpr uint16_t ADDR_POS = 132, ADDR_VEL = 128, ADDR_PWM = 100;

constexpr double ZERO1 = 2048, ZERO2 = 2048;
constexpr double TICK2RAD = (360.0/4096.0)*M_PI/180.0;
constexpr double SPEED2RAD = 0.229*(2*M_PI/60.0);
inline double pos2rad(uint32_t t,double z){ return (int(t)-z)*TICK2RAD; }
inline double vel2rad(int32_t v){ return v*SPEED2RAD; }

constexpr double Infill = 52.0/88.0;
constexpr double L1 = 0.30, L2 = 0.30, LC1 = L1*0.7, LC2 = L2*0.5;
constexpr double M1 = (0.098 + 0.08) * Infill + 0.077, M2 = 0.088 * Infill;
constexpr double I1 = 0.001105+ (0.077 * (0.09*L1*L1)), I2 = 0.000300;

constexpr double BLX = -42, BLY = 1, L1d = 30, L2d = 30;
constexpr double J1min = -85, J1max = -10, J2min = 20, J2max = 140;

struct Coef{double a0,a1,a2,a3,a4,a5;};

bool ik(double x,double y,double& q1,double& q2){
    double xr = x-BLX, yr = y-BLY, r2 = xr*xr+yr*yr, maxR = L1d+L2d;
    if(r2>maxR*maxR) return false;
    double c2 = (r2-L1d*L1d-L2d*L2d)/(2*L1d*L2d);
    c2 = std::clamp(c2,-1.0,1.0);
    double s2 = std::sqrt(1-c2*c2);
    q2 = std::atan2(s2,c2);
    q1 = std::atan2(yr,xr)-std::atan2(L2d*s2,L1d+L2d*c2);
    double d1 = rad2deg(q1), d2 = rad2deg(q2);
    return d1>=J1min&&d1<=J1max && d2>=J2min&&d2<=J2max;
}

Coef quintic(double qi,double qdi,double qddi,
             double qf,double qdf,double qddf,double T){
    Coef c; c.a0=qi; c.a1=qdi; c.a2=qddi/2.0;
    if (T <= 1e-6) {
        c.a3=0; c.a4=0; c.a5=0; return c;
    }
    double T2=T*T,T3=T2*T,T4=T3*T,T5=T4*T;
    c.a3=(20*(qf-qi)-(8*qdf+12*qdi)*T-(3*qddf-qddi)*T2)/(2*T3);
    c.a4=(30*(qi-qf)+(14*qdf+16*qdi)*T+(3*qddf-2*qddi)*T2)/(2*T4);
    c.a5=(12*(qf-qi)-(6*qdf+6*qdi)*T-(qddf-qddi)*T2)/(2*T5); return c;
}

void eval(const Coef& c,double t,double& q,double&qd,double&qdd){
    double t2=t*t,t3=t2*t,t4=t3*t, t5=t4*t;
    q   =c.a0+c.a1*t+c.a2*t2+c.a3*t3+c.a4*t4+c.a5*t5;
    qd  =c.a1+2*c.a2*t+3*c.a3*t2+4*c.a4*t3+5*c.a5*t4;
    qdd =2*c.a2+6*c.a3*t+12*c.a4*t2+20*c.a5*t3;
}


std::array<double, 2> computeEndEffector(double q1, double q2) {
    constexpr double L1 = 0.30, L2 = 0.30;
    double x = L1 * std::cos(q1) + L2 * std::cos(q1 + q2);
    double y = L1 * std::sin(q1) + L2 * std::sin(q1 + q2);
    return {x, y};
}



std::array<std::array<double, 2>, 2> computeJacobian(double q1, double q2) {
    constexpr double L1 = 0.30;
    constexpr double L2 = 0.30;

    double s1 = std::sin(q1);
    double s12 = std::sin(q1 + q2);
    double c1 = std::cos(q1);
    double c12 = std::cos(q1 + q2);

    return {{
        { -L1 * s1 - L2 * s12, -L2 * s12 },
        {  L1 * c1 + L2 * c12,  L2 * c12 }
    }};
}


std::array<double, 2> computeJdotTimesQdot(double q1, double q2, double dq1, double dq2) {
    constexpr double L1 = 0.30, L2 = 0.30;
    double dq12 = dq1 + dq2;
    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2), c12 = std::cos(q1 + q2);

    double j11_dot = -L1 * c1 * dq1 - L2 * c12 * dq12;
    double j12_dot = -L2 * c12 * dq12;
    double j21_dot = -L1 * s1 * dq1 - L2 * s12 * dq12;
    double j22_dot = -L2 * s12 * dq12;

    double dJdq_x = j11_dot * dq1 + j12_dot * dq2;
    double dJdq_y = j21_dot * dq1 + j22_dot * dq2;

    return { dJdq_x, dJdq_y };
}




std::array<std::array<double, 2>, 2> invert2x2(const std::array<std::array<double, 2>, 2>& J) {
    double a = J[0][0], b = J[0][1];
    double c = J[1][0], d = J[1][1];
    double det = a * d - b * c;

    if (std::abs(det) < 1e-6) throw std::runtime_error("Jacobian is singular!");

    return {{
        {  d / det, -b / det },
        { -c / det,  a / det }
    }};
}

struct OperationalTrajectory {
    std::array<double, 2> pos;      // x_d
    std::array<double, 2> vel;      // ẋ_d
    std::array<double, 2> accel;    // ẍ_d
};

OperationalTrajectory computeOperationalTrajectory(
    const Coef& cf1, const Coef& cf2, double t) 
{
    // Joint space trajectory
    double q1, dq1, ddq1;
    double q2, dq2, ddq2;
    eval(cf1, t, q1, dq1, ddq1);
    eval(cf2, t, q2, dq2, ddq2);

    std::array<double, 2> q  = {q1, q2};
    std::array<double, 2> dq = {dq1, dq2};
    std::array<double, 2> ddq = {ddq1, ddq2};

    // Forward Kinematics for xd
    auto xd = computeEndEffector(q1, q2);

    // Jacobian for ẋ_d
    auto J = computeJacobian(q1, q2);
    std::array<double, 2> dxd = {
        J[0][0] * dq1 + J[0][1] * dq2,
        J[1][0] * dq1 + J[1][1] * dq2
    };

    // Jacobian derivative for ẍ_d
    std::array<double, 2> dJdq = computeJdotTimesQdot(q1, q2, dq1, dq2);
    std::array<double, 2> ddxd = {
        J[0][0] * ddq1 + J[0][1] * ddq2 + dJdq[0],
        J[1][0] * ddq1 + J[1][1] * ddq2 + dJdq[1]
    };

    return OperationalTrajectory{xd, dxd, ddxd};
}

constexpr double DT = 0.001;
constexpr double TRAJECTORY_DURATION = 0.2;
constexpr double COOLDOWN_DURATION = 4.8;
constexpr double KP1 = 300, KD1 = 30;
constexpr double KP2 = 2000, KD2 = 50;
constexpr double TAUMAX1 = 2.5, TAUMAX2 = 2.5;
constexpr int    PWM_LIM = 885, PWM_BOOST2 = 15;

class IDCNode : public rclcpp::Node{
public:
    IDCNode() : Node("inverse_dynamics_node"),
                current_goal_q1_(0.0), current_goal_q2_(0.0),
                trajectory_active_(false) {
        busy_until_time_ = this->get_clock()->now();

        port_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
        pkt_  = dynamixel::PacketHandler::getPacketHandler(2.0);
        if(!port_->openPort() || !port_->setBaudRate(1000000)) {
            RCLCPP_FATAL(get_logger(), "Failed to open port or set baud rate.");
            throw std::runtime_error("Serial port initialization failed.");
        }

        init_servo(ID1); init_servo(ID2);

        double initial_q1, initial_q2, initial_dq1, initial_dq2;
        if (read_state(initial_q1, initial_q2, initial_dq1, initial_dq2)) {
            current_goal_q1_ = initial_q1;
            current_goal_q2_ = initial_q2;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to read initial state. Assuming (0,0).");
            current_goal_q1_ = 0.0; current_goal_q2_ = 0.0;
        }

        sub_ = create_subscription<SetXY>(
            "set_xy",10,[this](SetXY::SharedPtr m){
                double new_target_q1_rad, new_target_q2_rad;
                if(ik(m->x,m->y,new_target_q1_rad,new_target_q2_rad)){
                    rclcpp::Time time_of_request = this->get_clock()->now();
                    bool plan_this_trajectory = false;
                    {
                        std::scoped_lock l(mx_);
                        if (!trajectory_active_ && time_of_request >= busy_until_time_) {
                            plan_this_trajectory = true;
                            busy_until_time_ = time_of_request + rclcpp::Duration::from_seconds(TRAJECTORY_DURATION + COOLDOWN_DURATION);
                        } else {
                            if (trajectory_active_){
                                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,"SetXY: Ignoring; quintic trajectory active (current goal q1=%.2f, q2=%.2f).", current_goal_q1_, current_goal_q2_);
                            } else { 
                                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,"SetXY: Ignoring; in cooldown period until %.2fs from now.", (busy_until_time_ - time_of_request).seconds());
                            }
                        }
                    }

                    if (plan_this_trajectory) {
                        double current_q1_val, current_q2_val, current_dq1_val, current_dq2_val;
                        if(!read_state(current_q1_val, current_q2_val, current_dq1_val, current_dq2_val)){
                            RCLCPP_ERROR(get_logger(), "SetXY: Failed to read state for new trajectory. Reverting busy_until_time_.");
                            {
                                std::scoped_lock l(mx_);
                                busy_until_time_ = this->get_clock()->now(); 
                            }
                            return;
                        }
                        
                        RCLCPP_INFO(get_logger(), "Planning new trajectory from (q1m=%.2f, q2m=%.2f) to (q1t=%.2f, q2t=%.2f)",
                                    current_q1_val, current_q2_val, new_target_q1_rad, new_target_q2_rad);
                        {
                            std::scoped_lock l(mx_);
                            active_cf1_ = quintic(current_q1_val, current_dq1_val, 0.0, new_target_q1_rad, 0.0, 0.0, TRAJECTORY_DURATION);
                            active_cf2_ = quintic(current_q2_val, current_dq2_val, 0.0, new_target_q2_rad, 0.0, 0.0, TRAJECTORY_DURATION);
                            trajectory_start_time_ = time_of_request; 
                            trajectory_active_ = true;                
                            current_goal_q1_ = new_target_q1_rad; 
                            current_goal_q2_ = new_target_q2_rad;
                        }
                    }
                }
            });
        plot_data_pub_ = create_publisher<JointPlotData>("joint_plot_data", 10);
        timer_ = create_wall_timer(std::chrono::duration<double>(DT), std::bind(&IDCNode::tick,this));
        RCLCPP_INFO(get_logger(),"IDC stream online (dt %.3f s, traj_T %.2f s, cooldown %.2f s)", DT, TRAJECTORY_DURATION, COOLDOWN_DURATION);
    }

    ~IDCNode() override {
        RCLCPP_INFO(get_logger(), "Disabling torque and closing port.");
        uint8_t dxl_error = 0;
        pkt_->write1ByteTxRx(port_, ID1, ADDR_TORQUE, 0, &dxl_error);
        pkt_->write1ByteTxRx(port_, ID2, ADDR_TORQUE, 0, &dxl_error);
        port_->closePort();
    }

private:
    void init_servo(uint8_t id){
        uint8_t dxl_error = 0;
        int comm_result = pkt_->write1ByteTxRx(port_, id, ADDR_TORQUE, 0, &dxl_error);
        if (comm_result != COMM_SUCCESS) RCLCPP_ERROR(get_logger(), "Torque disable ID %d fail: %s", id, pkt_->getTxRxResult(comm_result));
        if (dxl_error != 0) RCLCPP_ERROR(get_logger(), "DXL error torque disable ID %d: %s", id, pkt_->getRxPacketError(dxl_error));

        comm_result = pkt_->write1ByteTxRx(port_, id, ADDR_MODE, MODE_PWM, &dxl_error);
        if (comm_result != COMM_SUCCESS) RCLCPP_ERROR(get_logger(), "Mode set ID %d fail: %s", id, pkt_->getTxRxResult(comm_result));
        if (dxl_error != 0) RCLCPP_ERROR(get_logger(), "DXL error mode set ID %d: %s", id, pkt_->getRxPacketError(dxl_error));
        
        comm_result = pkt_->write1ByteTxRx(port_, id, ADDR_TORQUE, 1, &dxl_error);
        if (comm_result != COMM_SUCCESS) RCLCPP_ERROR(get_logger(), "Torque enable ID %d fail: %s", id, pkt_->getTxRxResult(comm_result));
        if (dxl_error != 0) RCLCPP_ERROR(get_logger(), "DXL error torque enable ID %d: %s", id, pkt_->getRxPacketError(dxl_error));
    }

    bool read_state(double& q1,double& q2,double& dq1,double& dq2){
        dynamixel::GroupSyncRead p(port_,pkt_,ADDR_POS,4);
        dynamixel::GroupSyncRead v(port_,pkt_,ADDR_VEL,4);
        p.addParam(ID1); p.addParam(ID2);
        v.addParam(ID1); v.addParam(ID2);

        int dxl_comm_result_pos = p.txRxPacket();
        if (dxl_comm_result_pos != COMM_SUCCESS) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSRead Pos failed: %s", pkt_->getTxRxResult(dxl_comm_result_pos));
            return false;
        }
        int dxl_comm_result_vel = v.txRxPacket();
        if (dxl_comm_result_vel != COMM_SUCCESS) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSRead Vel failed: %s", pkt_->getTxRxResult(dxl_comm_result_vel));
            return false;
        }

        bool p_ok_id1 = p.isAvailable(ID1, ADDR_POS, 4);
        bool p_ok_id2 = p.isAvailable(ID2, ADDR_POS, 4);
        bool v_ok_id1 = v.isAvailable(ID1, ADDR_VEL, 4);
        bool v_ok_id2 = v.isAvailable(ID2, ADDR_VEL, 4);

        if (!p_ok_id1 || !p_ok_id2 || !v_ok_id1 || !v_ok_id2) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSRead data not available. P1:%d,P2:%d,V1:%d,V2:%d", p_ok_id1,p_ok_id2,v_ok_id1,v_ok_id2);
            return false;
        }

        q1 = pos2rad(p.getData(ID1,ADDR_POS,4),ZERO1);
        q2 = pos2rad(p.getData(ID2,ADDR_POS,4),ZERO2);
        dq1= vel2rad(int32_t(v.getData(ID1,ADDR_VEL,4)));
        dq2= vel2rad(int32_t(v.getData(ID2,ADDR_VEL,4)));
        return true;
    }

    void send_pwm(int pwm1,int pwm2){
        uint8_t b1[2]={DXL_LOBYTE(int16_t(pwm1)),DXL_HIBYTE(int16_t(pwm1))};
        uint8_t b2[2]={DXL_LOBYTE(int16_t(pwm2)),DXL_HIBYTE(int16_t(pwm2))};
        dynamixel::GroupSyncWrite sw(port_,pkt_,ADDR_PWM,2);
        sw.addParam(ID1,b1); sw.addParam(ID2,b2);
        int comm_result = sw.txPacket();
        if (comm_result != COMM_SUCCESS) {
             RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSWrite PWM failed: %s", pkt_->getTxRxResult(comm_result));
        }
    }

    void tick(){
        double q1_m, q2_m, dq1_m, dq2_m;
        if(!read_state(q1_m, q2_m, dq1_m, dq2_m)) {
            return;
        }

        double q1d = current_goal_q1_;
        double q2d = current_goal_q2_;
<<<<<<< HEAD
        // double q1dd = 0.0, q2dd = 0.0;
        // double q1ddd = 0.0, q2ddd = 0.0;

=======
        double q1dd = 0.0, q2dd = 0.0;
        double q1ddd = 0.0, q2ddd = 0.0;
        bool just_finished_quintic = false;

        {
            std::scoped_lock l(mx_);
            if(trajectory_active_){
                double t_elapsed = (this->get_clock()->now() - trajectory_start_time_).seconds();
                if(t_elapsed < TRAJECTORY_DURATION){
                    if (active_cf1_ && active_cf2_) {
                        eval(active_cf1_.value(), t_elapsed, q1d, q1dd, q1ddd);
                        eval(active_cf2_.value(), t_elapsed, q2d, q2dd, q2ddd);
                    } else {
                        trajectory_active_ = false;
                         RCLCPP_ERROR(get_logger(), "Bug: Trajectory active but coefficients not set!");
                    }
                } else { 
                    q1d = current_goal_q1_; q1dd = 0.0; q1ddd = 0.0;
                    q2d = current_goal_q2_; q2dd = 0.0; q2ddd = 0.0;
                    trajectory_active_ = false; 
                    just_finished_quintic = true;
                }
            }
        } 

        if (just_finished_quintic) {
             RCLCPP_INFO(get_logger(), "Quintic trajectory finished. Entering cooldown. Holding (%.2f, %.2f)", current_goal_q1_, current_goal_q2_);
        }
>>>>>>> 7ff5822 (gitignore)

       
if (active_cf1_ && active_cf2_) {
    double t_elapsed = (this->get_clock()->now() - trajectory_start_time_).seconds();

    auto traj = computeOperationalTrajectory(active_cf1_.value(), active_cf2_.value(), t_elapsed);
    auto x_d = traj.pos;
    auto xd_d = traj.vel;
    auto xdd_d = traj.accel;

    // Get measured state
    auto x = computeEndEffector(q1_m, q2_m);
    auto J = computeJacobian(q1_m, q2_m);
    std::array<double, 2> dq_m = { dq1_m, dq2_m };
    std::array<double, 2> xd = {
        J[0][0]*dq1_m + J[0][1]*dq2_m,
        J[1][0]*dq1_m + J[1][1]*dq2_m
    };

    // Task-space errors
    std::array<double, 2> e_x = { x_d[0] - x[0], x_d[1] - x[1] };
    std::array<double, 2> e_xdot = { xd_d[0] - xd[0], xd_d[1] - xd[1] };

    // Task-space gains
    constexpr double KP = 500.0, KD = 50.0;

    // dJ·dq
    auto dJdq = computeJdotTimesQdot(q1_m, q2_m, dq1_m, dq2_m);

    // Desired Cartesian acceleration
    std::array<double, 2> xdd_des = {
        xdd_d[0] + KD * e_xdot[0] + KP * e_x[0] - dJdq[0],
        xdd_d[1] + KD * e_xdot[1] + KP * e_x[1] - dJdq[1]
    };

    // Joint-space mapping
    auto Jinv = invert2x2(J);
    std::array<double, 2> y = {
        Jinv[0][0] * xdd_des[0] + Jinv[0][1] * xdd_des[1],
        Jinv[1][0] * xdd_des[0] + Jinv[1][1] * xdd_des[1]
    };

    double cos_q2_m = std::cos(q2_m), sin_q2_m = std::sin(q2_m);
    double h   = -M2 * L1 * LC2 * sin_q2_m;
    double b11 = I1 + I2 + M1 * LC1 * LC1 + M2 * (L1 * L1 + LC2 * LC2 + 2 * L1 * LC2 * cos_q2_m);
    double b12 = I2 + M2 * (LC2 * LC2 + L1 * LC2 * cos_q2_m);
    double b22 = I2 + M2 * LC2 * LC2;

    double tau1 = b11 * y[0] + b12 * y[1] + h * (2 * dq1_m * dq2_m + dq2_m * dq2_m);
    double tau2 = b12 * y[0] + b22 * y[1] - h * dq1_m * dq1_m;

    int pwm1 = int(tau1 / TAUMAX1 * PWM_LIM);
    int pwm2 = int(tau2 / TAUMAX2 * PWM_LIM);
    pwm1 = std::clamp(pwm1, -PWM_LIM, PWM_LIM);
    pwm2 = std::clamp(pwm2, -PWM_LIM, PWM_LIM);
    send_pwm(pwm1, pwm2);
}



<<<<<<< HEAD
=======
        int pwm1 = int(tau1/TAUMAX1 * PWM_LIM);
        int pwm2 = int(tau2/TAUMAX2 * PWM_LIM);
        pwm1 = std::clamp(pwm1,-PWM_LIM,PWM_LIM);
        pwm2 = std::clamp(pwm2,-PWM_LIM,PWM_LIM);
        
        send_pwm(pwm1,pwm2);
        
>>>>>>> 7ff5822 (gitignore)
        auto plot_msg = JointPlotData();
        plot_msg.measured_q1 = q1_m; plot_msg.desired_q1 = q1d;
        plot_msg.measured_q2 = q2_m; plot_msg.desired_q2 = q2d;
        plot_data_pub_->publish(plot_msg);
    }

    dynamixel::PortHandler*   port_{};
    dynamixel::PacketHandler* pkt_{};
    rclcpp::Subscription<SetXY>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr           timer_;
    rclcpp::Publisher<JointPlotData>::SharedPtr plot_data_pub_;
    
    std::mutex mx_;
    double current_goal_q1_;
    double current_goal_q2_;
    std::optional<Coef> active_cf1_;
    std::optional<Coef> active_cf2_;
    rclcpp::Time trajectory_start_time_; 
    bool trajectory_active_;             
    rclcpp::Time busy_until_time_;       
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    auto idc_node = std::make_shared<IDCNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(idc_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
