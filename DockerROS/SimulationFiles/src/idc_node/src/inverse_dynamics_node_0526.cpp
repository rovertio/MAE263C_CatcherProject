// inverse_dynamics_node.cpp
#include <cmath>
#include <memory>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <optional>
#include <array>
// #include <cmath> // Redundant, already included

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
inline double pos2rad(uint32_t t,double z){ return (static_cast<int>(t)-z)*TICK2RAD; } // Used static_cast for clarity
inline double vel2rad(int32_t v){ return v*SPEED2RAD; }

constexpr double Infill = 52.0/88.0;
constexpr double L1_dyn = 0.30, L2_dyn = 0.30, LC1 = L1_dyn*0.7, LC2 = L2_dyn*0.5; // Renamed L1,L2 to L1_dyn, L2_dyn to avoid conflict with IK L1,L2
constexpr double M1 = (0.098 + 0.08) * Infill + 0.077, M2 = 0.088 * Infill;
constexpr double I1_dyn = 0.001105+ (0.077 * (0.09*L1_dyn*L1_dyn)), I2_dyn = 0.000300; // Renamed I1,I2 to I1_dyn, I2_dyn

// IK constants
constexpr double BLX = -42, BLY = 1, L1_ik = 30, L2_ik = 30; // Renamed L1,L2 for IK
constexpr double J1min = -85, J1max = -10, J2min = 20, J2max = 140;

struct Coef{double a0,a1,a2,a3,a4,a5;};

bool ik(double x,double y,double& q1,double& q2){
    double xr = x-BLX, yr = y-BLY, r2 = xr*xr+yr*yr, maxR = L1_ik+L2_ik;
    if(r2>maxR*maxR) return false;
    double c2_val = (r2-L1_ik*L1_ik-L2_ik*L2_ik)/(2*L1_ik*L2_ik); // Renamed c2 to c2_val
    c2_val = std::clamp(c2_val,-1.0,1.0);
    double s2_val = std::sqrt(1-c2_val*c2_val); // Renamed s2 to s2_val
    q2 = std::atan2(s2_val,c2_val);
    q1 = std::atan2(yr,xr)-std::atan2(L2_ik*s2_val,L1_ik+L2_ik*c2_val);
    double d1 = rad2deg(q1), d2 = rad2deg(q2);
    return d1>=J1min&&d1<=J1max && d2>=J2min&&d2<=J2max;
}

Coef quintic(double qi,double qdi,double qddi,
             double qf,double qdf,double qddf,double T){
    Coef c; c.a0=qi; c.a1=qdi; c.a2=qddi/2.0;
    if (T <= 1e-6) { // Avoid division by zero for very small T
        c.a3=0; c.a4=0; c.a5=0; return c;
    }
    double T2=T*T,T3=T2*T,T4=T3*T,T5=T4*T;
    c.a3=(20*(qf-qi)-(8*qdf+12*qdi)*T-(3*qddf-qddi)*T2)/(2*T3);
    c.a4=(30*(qi-qf)+(14*qdf+16*qdi)*T+(3*qddf-2*qddi)*T2)/(2*T4);
    c.a5=(12*(qf-qi)-(6*qdf+6*qdi)*T-(qddf-qddi)*T2)/(2*T5); return c;
}

void eval(const Coef& c,double t,double& q_out,double& qd_out,double& qdd_out){ // Renamed output vars
    double t2=t*t,t3=t2*t,t4=t3*t, t5=t4*t;
    q_out   =c.a0+c.a1*t+c.a2*t2+c.a3*t3+c.a4*t4+c.a5*t5;
    qd_out  =c.a1+2*c.a2*t+3*c.a3*t2+4*c.a4*t3+5*c.a5*t4;
    qdd_out =2*c.a2+6*c.a3*t+12*c.a4*t2+20*c.a5*t3;
}

// Kinematic model constants for Jacobian, FK etc.
// These should match the L1_dyn, L2_dyn if used for dynamic calculations
constexpr double KIN_L1 = 0.30;
constexpr double KIN_L2 = 0.30;

std::array<double, 2> computeEndEffector(double q1, double q2) {
    double x = KIN_L1 * std::cos(q1) + KIN_L2 * std::cos(q1 + q2);
    double y = KIN_L1 * std::sin(q1) + KIN_L2 * std::sin(q1 + q2);
    return {x, y};
}

std::array<std::array<double, 2>, 2> computeJacobian(double q1, double q2) {
    double s1 = std::sin(q1);
    double s12 = std::sin(q1 + q2);
    double c1 = std::cos(q1);
    double c12 = std::cos(q1 + q2);

    return {{
        { -KIN_L1 * s1 - KIN_L2 * s12, -KIN_L2 * s12 },
        {  KIN_L1 * c1 + KIN_L2 * c12,  KIN_L2 * c12 }
    }};
}

std::array<double, 2> computeJdotTimesQdot(double q1, double q2, double dq1, double dq2) {
    double dq12 = dq1 + dq2; // q1_dot + q2_dot
    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s12 = std::sin(q1 + q2), c12 = std::cos(q1 + q2);

    // Elements of J_dot * q_dot
    // J_dot_11*dq1 + J_dot_12*dq2
    // J_dot_21*dq1 + J_dot_22*dq2
    // where J_dot_ij = d(J_ij)/dt = (dJ_ij/dq1)*dq1 + (dJ_ij/dq2)*dq2
    
    // Derivative of J[0][0] = -L1*s1 - L2*s12  w.r.t time
    double J00_dot_t = -KIN_L1*c1*dq1 - KIN_L2*c12*dq12;
    // Derivative of J[0][1] = -L2*s12 w.r.t time
    double J01_dot_t = -KIN_L2*c12*dq12;
    // Derivative of J[1][0] = L1*c1 + L2*c12 w.r.t time
    double J10_dot_t = -KIN_L1*s1*dq1 - KIN_L2*s12*dq12;
    // Derivative of J[1][1] = L2*c12 w.r.t time
    double J11_dot_t = -KIN_L2*s12*dq12;
    
    double dJdq_x = J00_dot_t * dq1 + J01_dot_t * dq2;
    double dJdq_y = J10_dot_t * dq1 + J11_dot_t * dq2;
    
    return { dJdq_x, dJdq_y };
}

std::array<std::array<double, 2>, 2> invert2x2(const std::array<std::array<double, 2>, 2>& J) {
    double a = J[0][0], b = J[0][1];
    double c = J[1][0], d = J[1][1];
    double det = a * d - b * c;

    if (std::abs(det) < 1e-9) { // Slightly more robust check for singularity
        // RCLCPP_WARN(rclcpp::get_logger("global_logger"), "Jacobian determinant close to zero: %f", det); // Consider logging
        throw std::runtime_error("Jacobian is singular or near-singular!");
    }
    double inv_det = 1.0 / det;
    return {{
        {  d * inv_det, -b * inv_det },
        { -c * inv_det,  a * inv_det }
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
    double q1_traj, dq1_traj, ddq1_traj;
    double q2_traj, dq2_traj, ddq2_traj;
    eval(cf1, t, q1_traj, dq1_traj, ddq1_traj);
    eval(cf2, t, q2_traj, dq2_traj, ddq2_traj);

    // Forward Kinematics for x_d
    auto xd_val = computeEndEffector(q1_traj, q2_traj);

    // Jacobian for ẋ_d
    auto J_val = computeJacobian(q1_traj, q2_traj);
    std::array<double, 2> dxd_val = {
        J_val[0][0] * dq1_traj + J_val[0][1] * dq2_traj,
        J_val[1][0] * dq1_traj + J_val[1][1] * dq2_traj
    };

    // Jacobian derivative term for ẍ_d
    std::array<double, 2> dJdq_val = computeJdotTimesQdot(q1_traj, q2_traj, dq1_traj, dq2_traj);
    std::array<double, 2> ddxd_val = {
        J_val[0][0] * ddq1_traj + J_val[0][1] * ddq2_traj + dJdq_val[0],
        J_val[1][0] * ddq1_traj + J_val[1][1] * ddq2_traj + dJdq_val[1]
    };

    return OperationalTrajectory{xd_val, dxd_val, ddxd_val};
}

constexpr double DT = 0.001;
constexpr double TRAJECTORY_DURATION = 0.2;
constexpr double COOLDOWN_DURATION = 2.0; // Reduced for quicker testing, adjust as needed
// Joint space PD gains
constexpr double KP1_joint = 300, KD1_joint = 30;
constexpr double KP2_joint = 2000, KD2_joint = 50; // This was very high, check if intended
// Operational space PD gains (NEEDS TUNING)
constexpr double KP_OP = 300.0, KD_OP = 30.0; 

constexpr double TAUMAX1 = 2.5, TAUMAX2 = 2.5;
constexpr int    PWM_LIM = 885; // PWM_BOOST2 was removed from original quintic block, check if it was used elsewhere

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
            RCLCPP_INFO(get_logger(), "Initial state read: q1=%.2f, q2=%.2f", initial_q1, initial_q2);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to read initial state. Assuming (0,0).");
            // Consider setting to a known safe "home" position in joint space
            current_goal_q1_ = deg2rad(0.0); // Example: 0 degrees for joint 1
            current_goal_q2_ = deg2rad(90.0); // Example: 90 degrees for joint 2
        }

        sub_ = create_subscription<SetXY>(
            "set_xy",10,[this](SetXY::SharedPtr m){
                double new_target_q1_rad, new_target_q2_rad;
                if(ik(m->x,m->y,new_target_q1_rad,new_target_q2_rad)){
                    rclcpp::Time time_of_request = this->get_clock()->now();
                    bool plan_this_trajectory = false;
                    { // Mutex scope
                        std::scoped_lock l(mx_);
                        if (!trajectory_active_ && time_of_request >= busy_until_time_) {
                            plan_this_trajectory = true;
                            // busy_until_time_ will be set after successful state read and planning
                        } else {
                            if (trajectory_active_){
                                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,"SetXY: Ignoring; trajectory active (current goal q1=%.2f, q2=%.2f).", rad2deg(current_goal_q1_), rad2deg(current_goal_q2_));
                            } else { 
                                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,"SetXY: Ignoring; in cooldown period until %.2fs from now.", (busy_until_time_ - time_of_request).seconds());
                            }
                        }
                    } // End mutex scope

                    if (plan_this_trajectory) {
                        double current_q1_val, current_q2_val, current_dq1_val, current_dq2_val;
                        if(!read_state(current_q1_val, current_q2_val, current_dq1_val, current_dq2_val)){
                            RCLCPP_ERROR(get_logger(), "SetXY: Failed to read state for new trajectory. Aborting plan.");
                            // No need to revert busy_until_time_ here as it wasn't definitively set for this trajectory yet
                            return;
                        }
                        
                        RCLCPP_INFO(get_logger(), "Planning new trajectory from (q1m=%.2f, q2m=%.2f) to (q1t=%.2f, q2t=%.2f) (degrees)",
                                    rad2deg(current_q1_val), rad2deg(current_q2_val), rad2deg(new_target_q1_rad), rad2deg(new_target_q2_rad));
                        { // Mutex scope
                            std::scoped_lock l(mx_);
                            active_cf1_ = quintic(current_q1_val, current_dq1_val, 0.0, new_target_q1_rad, 0.0, 0.0, TRAJECTORY_DURATION);
                            active_cf2_ = quintic(current_q2_val, current_dq2_val, 0.0, new_target_q2_rad, 0.0, 0.0, TRAJECTORY_DURATION);
                            trajectory_start_time_ = time_of_request; 
                            trajectory_active_ = true;                
                            current_goal_q1_ = new_target_q1_rad; // This is the final target of the quintic
                            current_goal_q2_ = new_target_q2_rad; // This is the final target of the quintic
                            busy_until_time_ = time_of_request + rclcpp::Duration::from_seconds(TRAJECTORY_DURATION + COOLDOWN_DURATION);
                        } // End mutex scope
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "SetXY: IK failed for (%.1f, %.1f)", m->x, m->y);
                }
            });
        plot_data_pub_ = create_publisher<JointPlotData>("idc_joint_plot_data", 10); // Renamed topic for clarity
        timer_ = create_wall_timer(std::chrono::duration<double>(DT), std::bind(&IDCNode::tick,this));
        RCLCPP_INFO(get_logger(),"IDC stream online (dt %.3f s, traj_T %.2f s, cooldown %.2f s)", DT, TRAJECTORY_DURATION, COOLDOWN_DURATION);
    }

    ~IDCNode() override {
        RCLCPP_INFO(get_logger(), "Disabling torque and closing port.");
        uint8_t dxl_error = 0;
        // Consider setting torque off for broadcast ID if supported, or individual IDs
        pkt_->write1ByteTxRx(port_, ID1, ADDR_TORQUE, 0, &dxl_error);
        pkt_->write1ByteTxRx(port_, ID2, ADDR_TORQUE, 0, &dxl_error);
        port_->closePort();
    }

private:
    void init_servo(uint8_t id){
        uint8_t dxl_error = 0;
        int comm_result = pkt_->write1ByteTxRx(port_, id, ADDR_TORQUE, 0, &dxl_error); // Torque Off
        if (comm_result != COMM_SUCCESS) RCLCPP_ERROR(get_logger(), "Torque disable ID %d fail: %s", id, pkt_->getTxRxResult(comm_result));
        else if (dxl_error != 0) RCLCPP_ERROR(get_logger(), "DXL error torque disable ID %d: %s", id, pkt_->getRxPacketError(dxl_error));

        comm_result = pkt_->write1ByteTxRx(port_, id, ADDR_MODE, MODE_PWM, &dxl_error); // Set PWM Mode
        if (comm_result != COMM_SUCCESS) RCLCPP_ERROR(get_logger(), "Mode set ID %d fail: %s", id, pkt_->getTxRxResult(comm_result));
        else if (dxl_error != 0) RCLCPP_ERROR(get_logger(), "DXL error mode set ID %d: %s", id, pkt_->getRxPacketError(dxl_error));
        
        comm_result = pkt_->write1ByteTxRx(port_, id, ADDR_TORQUE, 1, &dxl_error); // Torque On
        if (comm_result != COMM_SUCCESS) RCLCPP_ERROR(get_logger(), "Torque enable ID %d fail: %s", id, pkt_->getTxRxResult(comm_result));
        else if (dxl_error != 0) RCLCPP_ERROR(get_logger(), "DXL error torque enable ID %d: %s", id, pkt_->getRxPacketError(dxl_error));
        RCLCPP_INFO(get_logger(), "Servo ID %d initialized.", id);
    }

    bool read_state(double& q1_out,double& q2_out,double& dq1_out,double& dq2_out){
        dynamixel::GroupSyncRead pos_reader(port_,pkt_,ADDR_POS,4); // pos is 4 bytes (Present Position)
        dynamixel::GroupSyncRead vel_reader(port_,pkt_,ADDR_VEL,4); // vel is 4 bytes (Present Velocity)
        
        bool p_param_ok = pos_reader.addParam(ID1) && pos_reader.addParam(ID2);
        bool v_param_ok = vel_reader.addParam(ID1) && vel_reader.addParam(ID2);

        if (!p_param_ok || !v_param_ok) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSRead addParam failed. P:%d, V:%d", p_param_ok, v_param_ok);
            return false;
        }

        int dxl_comm_result_pos = pos_reader.txRxPacket();
        if (dxl_comm_result_pos != COMM_SUCCESS) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSRead Pos failed: %s", pkt_->getTxRxResult(dxl_comm_result_pos));
            return false;
        }
        int dxl_comm_result_vel = vel_reader.txRxPacket();
        if (dxl_comm_result_vel != COMM_SUCCESS) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSRead Vel failed: %s", pkt_->getTxRxResult(dxl_comm_result_vel));
            return false;
        }

        bool p_ok_id1 = pos_reader.isAvailable(ID1, ADDR_POS, 4);
        bool p_ok_id2 = pos_reader.isAvailable(ID2, ADDR_POS, 4);
        bool v_ok_id1 = vel_reader.isAvailable(ID1, ADDR_VEL, 4);
        bool v_ok_id2 = vel_reader.isAvailable(ID2, ADDR_VEL, 4);

        if (!p_ok_id1 || !p_ok_id2 || !v_ok_id1 || !v_ok_id2) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSRead data not available. P1:%d,P2:%d,V1:%d,V2:%d", p_ok_id1,p_ok_id2,v_ok_id1,v_ok_id2);
            return false;
        }

        q1_out = pos2rad(pos_reader.getData(ID1,ADDR_POS,4),ZERO1);
        q2_out = pos2rad(pos_reader.getData(ID2,ADDR_POS,4),ZERO2);
        dq1_out= vel2rad(static_cast<int32_t>(vel_reader.getData(ID1,ADDR_VEL,4)));
        dq2_out= vel2rad(static_cast<int32_t>(vel_reader.getData(ID2,ADDR_VEL,4)));
        return true;
    }

    void send_pwm(int pwm1_val, int pwm2_val){ // Renamed args
        uint8_t pwm1_bytes[2]={DXL_LOBYTE(static_cast<int16_t>(pwm1_val)),DXL_HIBYTE(static_cast<int16_t>(pwm1_val))};
        uint8_t pwm2_bytes[2]={DXL_LOBYTE(static_cast<int16_t>(pwm2_val)),DXL_HIBYTE(static_cast<int16_t>(pwm2_val))};
        dynamixel::GroupSyncWrite pwm_writer(port_,pkt_,ADDR_PWM,2); // PWM is 2 bytes
        
        bool param1_ok = pwm_writer.addParam(ID1,pwm1_bytes);
        bool param2_ok = pwm_writer.addParam(ID2,pwm2_bytes);

        if(!param1_ok || !param2_ok){
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSWrite PWM addParam failed. P1:%d, P2:%d", param1_ok, param2_ok);
            return;
        }
        int comm_result = pwm_writer.txPacket();
        if (comm_result != COMM_SUCCESS) {
             RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GSWrite PWM failed: %s", pkt_->getTxRxResult(comm_result));
        }
    }

    void tick(){
        double q1_m, q2_m, dq1_m, dq2_m; // Measured states
        if(!read_state(q1_m, q2_m, dq1_m, dq2_m)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Tick: Failed to read state. Commanding zero PWM.");
            send_pwm(0,0); // Send zero PWM if state read fails
            return;
        }

        // These are for plotting: either the final goal or intermediate quintic targets
        double q1_target_for_plot; 
        double q2_target_for_plot;

        // These are for the joint-space PD controller if it's used
        double q1_desired_joint;
        double q2_desired_joint;
        double dq1_desired_joint = 0.0;
        double dq2_desired_joint = 0.0;
        double ddq1_feedforward_joint = 0.0; 
        double ddq2_feedforward_joint = 0.0; 

        bool perform_operational_space_control = false;

        { // Mutex scope for accessing trajectory state
            std::scoped_lock l(mx_);
            q1_target_for_plot = current_goal_q1_; // Default for plotting is final goal
            q2_target_for_plot = current_goal_q2_;
            q1_desired_joint = current_goal_q1_;   // Default for joint controller is final goal
            q2_desired_joint = current_goal_q2_;

            if(trajectory_active_){
                double t_elapsed = (this->get_clock()->now() - trajectory_start_time_).seconds();
                if(t_elapsed < TRAJECTORY_DURATION){
                    if (active_cf1_ && active_cf2_) {
                        perform_operational_space_control = true;
                        // Get instantaneous joint targets from quintic for plotting and feedforward
                        eval(active_cf1_.value(), t_elapsed, q1_target_for_plot, dq1_desired_joint, ddq1_feedforward_joint);
                        eval(active_cf2_.value(), t_elapsed, q2_target_for_plot, dq2_desired_joint, ddq2_feedforward_joint);
                    } else {
                        trajectory_active_ = false; 
                        RCLCPP_ERROR(get_logger(), "Bug: Trajectory active but coefficients not set! Reverting to hold.");
                    }
                } else { // Quintic trajectory duration finished
                    trajectory_active_ = false;
                    // Targets for holding are already set (current_goal_q1_, current_goal_q2_)
                    // Desired velocities and accelerations for holding are 0.0 (already set as default)
                    RCLCPP_INFO_ONCE(get_logger(), "Quintic trajectory finished. Holding (q1=%.2f, q2=%.2f) deg.", rad2deg(current_goal_q1_), rad2deg(current_goal_q2_));
                }
            }
        } // End Mutex scope

        int final_pwm1 = 0;
        int final_pwm2 = 0;

        if (perform_operational_space_control && active_cf1_ && active_cf2_) {
            // --- OPERATIONAL SPACE CONTROL (during active quintic-defined trajectory) ---
            double t_elapsed_op = (this->get_clock()->now() - trajectory_start_time_).seconds();
             // Clamp time to avoid issues if tick is slightly late
            if (t_elapsed_op >= TRAJECTORY_DURATION) t_elapsed_op = TRAJECTORY_DURATION - 1e-6; // just before end

            OperationalTrajectory current_op_setpoints;
            try {
                 current_op_setpoints = computeOperationalTrajectory(active_cf1_.value(), active_cf2_.value(), t_elapsed_op);
            } catch (const std::runtime_error& e) {
                RCLCPP_ERROR(get_logger(), "Error in computeOperationalTrajectory: %s. Switching to joint space hold.", e.what());
                perform_operational_space_control = false; 
                std::scoped_lock l(mx_); trajectory_active_ = false;
            }

            if(perform_operational_space_control) { 
                auto x_d_op = current_op_setpoints.pos;    // Desired task-space pos from quintic
                auto xd_d_op = current_op_setpoints.vel;   // Desired task-space vel from quintic
                auto xdd_d_op = current_op_setpoints.accel; // Desired task-space accel from quintic (feedforward)

                auto x_m_op = computeEndEffector(q1_m, q2_m); // Measured task-space pos
                auto J_m_op = computeJacobian(q1_m, q2_m);     // Measured Jacobian
                std::array<double, 2> xd_m_op = {             // Measured task-space vel
                    J_m_op[0][0] * dq1_m + J_m_op[0][1] * dq2_m,
                    J_m_op[1][0] * dq1_m + J_m_op[1][1] * dq2_m
                };

                std::array<double, 2> e_x_op = { x_d_op[0] - x_m_op[0], x_d_op[1] - x_m_op[1] };
                std::array<double, 2> e_xd_op = { xd_d_op[0] - xd_m_op[0], xd_d_op[1] - xd_m_op[1] };

                // Commanded task-space acceleration (PD feedback + feedforward acceleration)
                std::array<double, 2> xdd_cmd_op = {
                    xdd_d_op[0] + KD_OP * e_xd_op[0] + KP_OP * e_x_op[0],
                    xdd_d_op[1] + KD_OP * e_xd_op[1] + KP_OP * e_x_op[1]
                };

                auto dJdq_m_op = computeJdotTimesQdot(q1_m, q2_m, dq1_m, dq2_m); // J_dot * q_dot term

                // Desired joint accelerations: y_op_ddq = J_inv * (xdd_cmd_op - dJdq_m_op)
                std::array<double, 2> term_for_jinv = {
                    xdd_cmd_op[0] - dJdq_m_op[0],
                    xdd_cmd_op[1] - dJdq_m_op[1]
                };
                
                std::array<double, 2> y_op_ddq_desired; // Desired joint accelerations
                try {
                    auto Jinv_m_op = invert2x2(J_m_op);
                    y_op_ddq_desired = {
                        Jinv_m_op[0][0] * term_for_jinv[0] + Jinv_m_op[0][1] * term_for_jinv[1],
                        Jinv_m_op[1][0] * term_for_jinv[0] + Jinv_m_op[1][1] * term_for_jinv[1]
                    };
                } catch (const std::runtime_error& e) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Jacobian singular during op-space: %s. Falling back to joint space.", e.what());
                    perform_operational_space_control = false; 
                    std::scoped_lock l(mx_); trajectory_active_ = false;
                }

                if(perform_operational_space_control){ // Check again if Jinv failed
                    // Inverse Dynamics (using y_op_ddq_desired)
                    double cos_q2_m = std::cos(q2_m), sin_q2_m = std::sin(q2_m);
                    double h_op   = -M2 * KIN_L1 * LC2 * sin_q2_m; // Use KIN_L1 for consistency with Jacobian
                    double b11_op = I1_dyn + I2_dyn + M1 * LC1 * LC1 + M2 * (KIN_L1 * KIN_L1 + LC2 * LC2 + 2 * KIN_L1 * LC2 * cos_q2_m);
                    double b12_op = I2_dyn + M2 * (LC2 * LC2 + KIN_L1 * LC2 * cos_q2_m);
                    double b22_op = I2_dyn + M2 * LC2 * LC2;

                    double tau1_op = b11_op * y_op_ddq_desired[0] + b12_op * y_op_ddq_desired[1] + h_op * (2 * dq1_m * dq2_m + dq2_m * dq2_m);
                    double tau2_op = b12_op * y_op_ddq_desired[0] + b22_op * y_op_ddq_desired[1] - h_op * dq1_m * dq1_m;

                    final_pwm1 = static_cast<int>(tau1_op / TAUMAX1 * PWM_LIM);
                    final_pwm2 = static_cast<int>(tau2_op / TAUMAX2 * PWM_LIM);
                }
            }
        }
        
        if (!perform_operational_space_control) { // Fallback or default joint space control
            // --- JOINT SPACE PD CONTROL (for holding position or if operational space failed) ---
            double e1_joint = q1_desired_joint - q1_m;
            double e2_joint = q2_desired_joint - q2_m;
            double de1_joint = dq1_desired_joint - dq1_m; 
            double de2_joint = dq2_desired_joint - dq2_m; 

            double y1_joint_ddq_desired = KP1_joint * e1_joint + KD1_joint * de1_joint + ddq1_feedforward_joint;
            double y2_joint_ddq_desired = KP2_joint * e2_joint + KD2_joint * de2_joint + ddq2_feedforward_joint;

            double cos_q2_m = std::cos(q2_m), sin_q2_m = std::sin(q2_m);
            double h_joint   = -M2 * KIN_L1 * LC2 * sin_q2_m;
            double b11_joint = I1_dyn + I2_dyn + M1 * LC1 * LC1 + M2 * (KIN_L1 * KIN_L1 + LC2 * LC2 + 2 * KIN_L1 * LC2 * cos_q2_m);
            double b12_joint = I2_dyn + M2 * (LC2 * LC2 + KIN_L1 * LC2 * cos_q2_m);
            double b22_joint = I2_dyn + M2 * LC2 * LC2;

            double tau1_joint = b11_joint * y1_joint_ddq_desired + b12_joint * y2_joint_ddq_desired + h_joint * (2 * dq1_m * dq2_m + dq2_m * dq2_m);
            double tau2_joint = b12_joint * y1_joint_ddq_desired + b22_joint * y2_joint_ddq_desired - h_joint * dq1_m * dq1_m;
            
            final_pwm1 = static_cast<int>(tau1_joint / TAUMAX1 * PWM_LIM);
            final_pwm2 = static_cast<int>(tau2_joint / TAUMAX2 * PWM_LIM);
        }

        final_pwm1 = std::clamp(final_pwm1, -PWM_LIM, PWM_LIM);
        final_pwm2 = std::clamp(final_pwm2, -PWM_LIM, PWM_LIM);
        send_pwm(final_pwm1, final_pwm2);
        
        auto plot_msg = JointPlotData();
        plot_msg.measured_q1 = q1_m; 
        plot_msg.desired_q1 = q1_target_for_plot; 
        plot_msg.measured_q2 = q2_m; 
        plot_msg.desired_q2 = q2_target_for_plot; 
        plot_data_pub_->publish(plot_msg);
    }

    dynamixel::PortHandler*   port_{nullptr}; // Initialize to nullptr
    dynamixel::PacketHandler* pkt_{nullptr};  // Initialize to nullptr
    rclcpp::Subscription<SetXY>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr           timer_;
    rclcpp::Publisher<JointPlotData>::SharedPtr plot_data_pub_;
    
    std::mutex mx_;
    double current_goal_q1_; // Final target of the current/last trajectory
    double current_goal_q2_; // Final target of the current/last trajectory
    std::optional<Coef> active_cf1_;
    std::optional<Coef> active_cf2_;
    rclcpp::Time trajectory_start_time_; 
    bool trajectory_active_;             
    rclcpp::Time busy_until_time_;       
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    try {
        auto idc_node = std::make_shared<IDCNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(idc_node);
        executor.spin();
    } catch (const std::runtime_error& e) {
        RCLCPP_FATAL(rclcpp::get_logger("idc_node_main"), "Runtime error in main: %s", e.what());
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("idc_node_main"), "Unknown exception in main.");
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
