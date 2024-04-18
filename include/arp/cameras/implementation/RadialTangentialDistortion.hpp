/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Feb 3, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */


#include <Eigen/LU>
#include <iostream>
#include <stdexcept>

/// \brief arp Main namespace of this package.
namespace arp {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// The default constructor with all zero ki
RadialTangentialDistortion::RadialTangentialDistortion()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0)
{
}

// Constructor initialising ki
RadialTangentialDistortion::RadialTangentialDistortion(double k1, double k2,
                                                       double p1, double p2)
{
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted,
    Eigen::Vector2d * pointDistorted) const
{
  const double& u = pointUndistorted(0);
  const double& v = pointUndistorted(1);
  double rsq = pow(u, 2) + pow(v, 2);
  double numerator = 1 + k1_*rsq + k2_*pow(rsq, 2);
  Eigen::Vector2d offset;
  offset << 2*p1_*u*v + p2_*(rsq + 2*pow(u, 2)),
            p1_*(rsq + 2*pow(v, 2)) + 2*p2_*u*v;
  *pointDistorted = numerator * pointUndistorted + offset;
  return true;
}
bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian) const
{
  distort(pointUndistorted, pointDistorted);
  // common expressions for jacobian entries
  const double& u = pointUndistorted(0);
  const double& v = pointUndistorted(1);
  double uSq_vSq = pow(u,2) + pow(v,2);
  double _2k1u_plus_4k2u_uvSq = (2*k1_*u + 4*k2_*u*uSq_vSq);
  double _2k1v_plus_4k2v_uvSq = (2*k1_*v + 4*k2_*v*uSq_vSq);
  // jacobian entries
  double d11 = 1 + 6*p2_*u + 2*p1_*v + k1_*uSq_vSq + k2_*pow(uSq_vSq,2) + u*_2k1u_plus_4k2u_uvSq;
  double d12 = 2*p1_*u + 2*p2_*v + u*_2k1v_plus_4k2v_uvSq;
  double d21 = 2*p1_*u + 2*p2_*v + v*_2k1u_plus_4k2u_uvSq;
  double d22 = 1 + 2*p2_*u + 6*p1_*v + k1_*uSq_vSq + k2_*pow(uSq_vSq,2) + v*_2k1v_plus_4k2v_uvSq;
  *pointJacobian << d11, d12, 
                    d21, d22;
  return true;
}

bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2d & pointDistorted,
    Eigen::Vector2d * pointUndistorted) const
{
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted; // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {

    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Matrix2d E2 = (E.transpose() * E);
    Eigen::Vector2d du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }

  }
  *pointUndistorted = x_bar;

  return success;
}

}  // namespace cameras
}  // namespace arp
