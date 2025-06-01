#pragma once
#include "control_params.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace control
{
/* ---------- tick/deg helpers ------------------------------------ */
inline uint32_t deg2tick(int j,double deg){
  const double z = (j==0?ZERO_TICK_1:ZERO_TICK_2);
  return static_cast<uint32_t>(z + deg*TICKS_PER_DEG);
}
inline double tick2deg(int j,int32_t t){
  const double z = (j==0?ZERO_TICK_1:ZERO_TICK_2);
  return (t - z)/TICKS_PER_DEG;
}
inline double vel_tick2rad(int32_t v){ return v*VELOCITY_SCALE; }

/* ---------- FK  (planar) ---------------------------------------- */
inline Eigen::Vector2d fk_xy(const Eigen::Vector2d& q){
  double c1=std::cos(q[0]), s1=std::sin(q[0]);
  double c12=std::cos(q[0]+q[1]), s12=std::sin(q[0]+q[1]);
  return { BASE_X + L1*c1 + L2*c12,
           BASE_Y + L1*s1 + L2*s12 };
}

/* ---------- Planar IK (camera XY → joint rad) ------------------- */
inline bool ik_xy(double x,double y,double& q1,double& q2)
{
  const double dx = x - BASE_X;
  const double dy = y - BASE_Y;
  const double r2 = dx*dx + dy*dy;
  const double L12 = L1 + L2;
  if (r2 > L12*L12) return false;                 // unreachable

  double c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2);
  c2 = std::clamp(c2, -1.0, 1.0);
  double s2 = std::sqrt(1.0 - c2*c2);
  q2 = std::atan2(s2, c2);
  if (q2 < Q2_MIN || q2 > Q2_MAX) return false;
  
  q1 = std::atan2(dy, dx) - std::atan2(L2*s2, L1 + L2*c2);
  if (q1 < Q1_MIN || q1 > Q1_MAX) return false;
  return true;                                    // always inside limits for now
}

/* ---------- simple PID struct (for PWM node) -------------------- */
struct PidState
{
  double i = 0, prev = 0;
  double step(double err, double kp, double ki, double kd, int max_out)
  {
    i += ki * err;
    double d = err - prev;  prev = err;
    double u = kp * err + i + kd * d;
    u = std::clamp(u, -double(max_out), double(max_out));
    if (std::abs(u) >= max_out) i = 0;      // crude anti-windup
    return u;
  }
};

/* ---------- blocking read of the two servos (helper) ------------ */
inline void read_two_positions(dynamixel::PortHandler* port,
                               dynamixel::PacketHandler* pkt,
                               int32_t& t1, int32_t& t2)
{
  uint8_t err{};
  pkt->read4ByteTxRx(port, ID1, ADDR_PRESENT_POSITION, (uint32_t*)&t1, &err);
  pkt->read4ByteTxRx(port, ID2, ADDR_PRESENT_POSITION, (uint32_t*)&t2, &err);
}


/* ---------- Jacobian -------------------------------------------- */
inline Eigen::Matrix2d jacobian(const Eigen::Vector2d& q){
  double s1=std::sin(q[0]), c1=std::cos(q[0]);
  double s12=std::sin(q[0]+q[1]), c12=std::cos(q[0]+q[1]);
  Eigen::Matrix2d J;
  J << -L1*s1 - L2*s12 ,  -L2*s12,
        L1*c1 + L2*c12 ,   L2*c12;
  return J;
}

/* J̇·q̇ ------------------------------------------------------------ */
inline Eigen::Vector2d jdot_qdot(const Eigen::Vector2d& q,
                                 const Eigen::Vector2d& dq){
  double dq12=dq[0]+dq[1];
  double c1=std::cos(q[0]), s1=std::sin(q[0]);
  double c12=std::cos(q[0]+q[1]), s12=std::sin(q[0]+q[1]);

  Eigen::Vector2d v;
  v[0]=(-L1*c1*dq[0]-L2*c12*dq12)*dq[0] + (-L2*c12*dq12)*dq[1];
  v[1]=(-L1*s1*dq[0]-L2*s12*dq12)*dq[0] + (-L2*s12*dq12)*dq[1];
  return v;
}

/* ---------- Mass matrix, Coriolis, Gravity ---------------------- */
inline Eigen::Matrix2d mass_matrix(const Eigen::Vector2d& q){
  double c2=std::cos(q[1]);
  double m11 = I1 + I2 +
               M1*LC1*LC1 +
               M2*(L1*L1 + LC2*LC2 + 2*L1*LC2*c2);
  double m12 = I2 + M2*(LC2*LC2 + L1*LC2*c2);
  double m22 = I2 + M2*LC2*LC2;
  Eigen::Matrix2d M; M << m11, m12, m12, m22; return M;
}
inline Eigen::Vector2d coriolis(const Eigen::Vector2d& q,
                                const Eigen::Vector2d& dq){
  double s2=std::sin(q[1]);
  double h = -M2*L1*LC2*s2;
  return { h*(2*dq[0]*dq[1] + dq[1]*dq[1]),
          -h*dq[0]*dq[0] };
}
inline Eigen::Vector2d gravity(const Eigen::Vector2d& q){
  /* ignored (arm horizontal) → return zeros */
  return {0,0};
}

/* ---------- Torque → small position step (heuristic) ------------ */
inline Eigen::Vector2d tau_to_deg(const Eigen::Vector2d& tau){
  return { tau[0]/TAUMAX1 * (PWM_LIM/20.0),   //  ~±45 deg FS
           tau[1]/TAUMAX2 * (PWM_LIM/2.0) };
}

/* ---------- Robust Control Parameters ------------ */
inline Eigen::Vector2d robust_w(const Eigen::Vector2d& z,
                                double rho, double eps)
{
  const double norm_z = z.norm();
  const double gain   = (norm_z >= eps) ? (rho / norm_z)
                                        : (rho / eps);
  return gain * z;
}

/* ================================================================
 *  Adaptive-control helpers  (planar 2-DOF arm)
 * ================================================================ */
namespace detail
{
/* dynamic regressor  Y(q,q̇,q̈r)  — 2×10, fixed-size, no heap
   π = [α1 α2 α3 α4 α5  α6 α7 α8 α9 α10]^T encodes link masses,
       inertias, COM offsets.  Expressions follow Spong ’06.          */
inline Eigen::Matrix<double,2,10> regressor(
        const Eigen::Vector2d& q,
        const Eigen::Vector2d& dq,
        const Eigen::Vector2d& ddqr)
{
  const double c2 = std::cos(q[1]),
               s2 = std::sin(q[1]);

  /* shorthand */
  const double q1dd = ddqr[0],  q2dd = ddqr[1];
  const double q1d  = dq[0],    q2d  = dq[1];

  Eigen::Matrix<double,2,10> Y; Y.setZero();

  /* Row 1 ---------------------------------------------------------- */
  Y(0,0) = q1dd;
  Y(0,1) = c2*q1dd - s2*q2d*q1d;
  Y(0,2) = q2dd;
  Y(0,3) = q1dd;
  Y(0,4) = 0.0;
  Y(0,5) = 2*c2*q1dd + c2*q2dd - 2*s2*q1d*q2d - s2*q2d*q2d;
  Y(0,6) = q2dd;
  Y(0,7) = 0.0;
  Y(0,8) = 0.0;
  Y(0,9) = 0.0;

  /* Row 2 ---------------------------------------------------------- */
  Y(1,0) = 0.0;
  Y(1,1) = s2*q1d*q1d + c2*q2dd;
  Y(1,2) = 0.0;
  Y(1,3) = 0.0;
  Y(1,4) = q1dd + q2dd;
  Y(1,5) = s2*q1d*q1d + c2*q1dd;
  Y(1,6) = q1dd + q2dd;
  Y(1,7) = 0.0;
  Y(1,8) = 0.0;
  Y(1,9) = 0.0;

  return Y;
}
} // namespace detail

/* ======== small helper that clamps a vector entry-wise =========== */
template<int N>
inline void clamp_vec(Eigen::Matrix<double,N,1>& v, double lim)
{
  for(int i=0;i<N;++i) v[i] = std::clamp(v[i], -lim, lim);
}


} // namespace control





