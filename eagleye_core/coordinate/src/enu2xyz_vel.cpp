// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <eigen3/Eigen/StdVector>

void enu2xyz_vel(double enu_vel[3], double ecef_base_pos[3], double xyz_vel[3])
{
  using namespace GeographicLib;
  Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

  std::vector<double> rotation(9);
  double llh[3];
  earth.Reverse(ecef_base_pos[0], ecef_base_pos[1], ecef_base_pos[2], llh[0], llh[1], llh[2], rotation);

  Eigen::Matrix3d R(rotation.data());
  Eigen::Vector3d v_enu(enu_vel);

  Eigen::Map<Eigen::Vector3d> v_xyz(xyz_vel);
  v_xyz = R.transpose() * v_enu;
}