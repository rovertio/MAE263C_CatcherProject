#pragma once
#include <array>
#include <cstdint>

namespace control
{
/* ================== Hardware =================== */
constexpr std::array<int,2> SERVO_IDS{4,1};   // ID1, ID2
constexpr uint8_t ID1 = 4, ID2 = 1;

/* Dynamixel registers (XM430-W210) */
constexpr uint16_t ADDR_TORQUE_ENABLE   = 64;
constexpr uint16_t ADDR_OPERATING_MODE  = 11;
constexpr uint16_t ADDR_GOAL_POSITION   = 116;
constexpr uint16_t ADDR_GOAL_PWM        = 100;
constexpr uint16_t ADDR_PRESENT_POSITION= 132;
constexpr uint16_t ADDR_PRESENT_VELOCITY= 128;
constexpr uint16_t ADDR_PRESENT_PWM     = 124; // Present PWM value

/* Operating-mode codes */
constexpr uint8_t MODE_POSITION = 3;
constexpr uint8_t MODE_PWM      = 16;

/* ================== Geometry =================== */
constexpr double PI = 3.14159265358979323846;
constexpr double L1 = 0.30;                 // [m]
constexpr double L2 = 0.30;                 // [m]
constexpr double BASE_X = -0.42;            // [m] world frame
constexpr double BASE_Y =  0.01;            // [m]
constexpr double Q1_MIN = -80.0 * PI / 180.0;
constexpr double Q1_MAX = -10 * PI / 180.0;
constexpr double Q2_MIN =  20.0 * PI / 180.0;
constexpr double Q2_MAX =  140 * PI / 180.0;

/* ================== Servo calibration ========= */
constexpr double ZERO_TICK_1 = 2048;        // ticks when q1 = 0 deg
constexpr double ZERO_TICK_2 = 2048;
constexpr double TICKS_PER_DEG = 4096.0/360.0;

/* Velocity conversion (tick/s → rad/s) */
constexpr double VELOCITY_SCALE = 0.229 * 2*PI / 60.0;

/* ================== Dynamics ================== */
constexpr double M1 = 0.17;                 // [kg] link 1
constexpr double M2 = 0.088;                // [kg] link 2
constexpr double LC1 = L1*0.7;              // COM offsets
constexpr double LC2 = L2*0.5;
constexpr double I1 = 0.00110;              // [kg·m²]
constexpr double I2 = 0.00030;

/* PWM / torque limits */
constexpr double TAUMAX1 = 2.5;             // [N·m]
constexpr double TAUMAX2 = 2.5;
constexpr int    PWM_LIM = 885;             // full-scale
} // namespace control