/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

ImuKinematicsJacobian F_c(const RobotState &state, const ImuMeasurement &z)
{
  auto R_WS = state.q_WS.matrix();
  ImuKinematicsJacobian F_c;
  F_c.setZero();
  F_c.block(0,6,3,3) = Eigen::Matrix3d::Identity();
  F_c.block(3,3,3,3) = -crossMx(R_WS*(z.omega_S - state.b_g));
  F_c.block(6,3,3,3) = -crossMx(R_WS*(z.acc_S - state.b_a));
  F_c.block(3,9,3,3) = -R_WS;
  F_c.block(6,12,3,3) = -R_WS;
  return F_c;
}

DeltaRobotState delta_f_c(const float dt, const RobotState &state, const ImuMeasurement &z, const Eigen::Vector3d &g_W)
{
  auto R_WS = state.q_WS.matrix();
  DeltaRobotState delta_chi;
  delta_chi.delta_t_WS = dt * state.v_W;
  delta_chi.delta_alpha_WS = dt * R_WS*(z.omega_S - state.b_g);
  delta_chi.delta_v_W = dt * (R_WS*(z.acc_S - state.b_a) + g_W);
  delta_chi.delta_b_g.setZero();
  delta_chi.delta_b_a.setZero();
  return delta_chi;
}

RobotState boxPlus(const RobotState &state, const DeltaRobotState &delta_state)
{
  RobotState resState;
  resState.t_WS = state.t_WS + delta_state.delta_t_WS;
  resState.q_WS = (deltaQ(delta_state.delta_alpha_WS) * state.q_WS).normalized();
  resState.v_W = state.v_W + delta_state.delta_v_W;
  resState.b_g = state.b_g + delta_state.delta_b_g;
  resState.b_a = state.b_a + delta_state.delta_b_a;
  return resState;
}

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  //// TRAPEZOIDAL INTEGRATION ////
  // Create gravity vector
  Eigen::Vector3d g_W{0, 0, -9.81};
  
  // Create the two delta_chi states
  DeltaRobotState delta_chi_1, delta_chi_2;
  
  // Chi_1
  delta_chi_1 = delta_f_c(dt, state_k_minus_1, z_k_minus_1, g_W);
  
  // x_k-1 [+] delta_chi_1
  RobotState state_kMinus1_plus_delta_chi_1;
  state_kMinus1_plus_delta_chi_1 = boxPlus(state_k_minus_1, delta_chi_1);

  // Chi_2
  delta_chi_2 = delta_f_c(dt, state_kMinus1_plus_delta_chi_1, z_k, g_W);

  // deltaStep = 1/2 * (Chi_1 + Chi_2)
  DeltaRobotState deltaStep;
  deltaStep.delta_t_WS = 0.5*(delta_chi_1.delta_t_WS + delta_chi_2.delta_t_WS);
  deltaStep.delta_alpha_WS = 0.5*(delta_chi_1.delta_alpha_WS + delta_chi_2.delta_alpha_WS);
  deltaStep.delta_v_W = 0.5*(delta_chi_1.delta_v_W + delta_chi_2.delta_v_W);
  deltaStep.delta_b_g = 0.5*(delta_chi_1.delta_b_g + delta_chi_2.delta_b_g);
  deltaStep.delta_b_a = 0.5*(delta_chi_1.delta_b_a + delta_chi_2.delta_b_a);

  // x_k = x_k-1 [+] deltaStep
  state_k = boxPlus(state_k_minus_1, deltaStep);
  
  //// JACOBIAN ////
  if (jacobian) {
    // Get the rotation matrices
    Eigen::Matrix3d R_WS_k = state_k.q_WS.matrix();
    Eigen::Matrix3d R_WS_kMinus1 = state_k_minus_1.q_WS.matrix();
  
    ImuKinematicsJacobian F_c_xkMinus1_tkMinus1;
    F_c_xkMinus1_tkMinus1 = F_c(state_k_minus_1, z_k_minus_1);
    
    ImuKinematicsJacobian F_c_xkMinus1_plus_delta_x1_tkMinus1;
    F_c_xkMinus1_plus_delta_x1_tkMinus1 = F_c(state_kMinus1_plus_delta_chi_1, z_k);
    
    ImuKinematicsJacobian I_15;
    I_15.setIdentity();
    *jacobian = I_15 + 0.5*dt*F_c_xkMinus1_tkMinus1 + 0.5*dt*F_c_xkMinus1_plus_delta_x1_tkMinus1*(I_15 + dt*F_c_xkMinus1_tkMinus1);
    jacobian->block(3,3,3,3).setIdentity();
    jacobian->block(3,9,3,3) = -0.5*dt*(R_WS_kMinus1 + R_WS_k);   // are we allowed to use the components of state_k here??? 
    jacobian->block(6,3,3,3) = -0.5*dt*crossMx(R_WS_kMinus1*(z_k_minus_1.acc_S - state_k_minus_1.b_a) + R_WS_k*(z_k.acc_S - state_k.b_a));  // maybe need to get state_k.q_WS.matrix() from somewhere else. Can we use "z_k.acc_S" and "state_k.b_a" here, since we get them from the non-linear computation. 
  }
  return true;
}

}
}  // namespace arp

