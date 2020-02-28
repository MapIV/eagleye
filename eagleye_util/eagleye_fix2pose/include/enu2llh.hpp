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

void enu2llh(double enu_pos[3], double ecef_base_pos[3], double llh_pos[3])
{
  // ecef2llh
  double semi_major_axis = 6378137.0000;
  double semi_minor_axis = 6356752.3142;
  double a1 = sqrt(1 - pow(semi_minor_axis / semi_major_axis, 2.0));
  double a2 = sqrt((ecef_base_pos[0] * ecef_base_pos[0]) + (ecef_base_pos[1] * ecef_base_pos[1]));
  double a3 = 54 * (semi_minor_axis * semi_minor_axis) * (ecef_base_pos[2] * ecef_base_pos[2]);
  double a4 = (a2 * a2) + (1 - (a1 * a1)) * (ecef_base_pos[2] * ecef_base_pos[2]) - (a1 * a1) * (semi_major_axis * semi_major_axis - semi_minor_axis * semi_minor_axis);
  double a5 = ((a1 * a1) * (a1 * a1) * a3 * (a2 * a2)) / (a4 * a4 * a4);
  double a6 = pow((1 + a5 + sqrt(a5 * a5 + 2 * a5)), 1.0 / 3.0);
  double a7 = a3 / (3 * pow((a6 + 1 / a6 + 1), 2.0) * a4 * a4);
  double a8 = sqrt(1 + 2 * (a1 * a1) * (a1 * a1) * a7);
  double a9 = -(a7 * (a1 * a1) * a2) / (1 + a8) + sqrt((semi_major_axis * semi_major_axis / 2) * (1 + 1 / a8) - (a7 * (1 - (a1 * a1)) * (ecef_base_pos[2] * ecef_base_pos[2])) / (a8 * (1 + a8)) - a7 * (a2 * a2) / 2);
  double a10 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (ecef_base_pos[2] * ecef_base_pos[2]));
  double a11 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (1 - (a1 * a1)) * (ecef_base_pos[2] * ecef_base_pos[2]));
  double a12 = ((semi_minor_axis * semi_minor_axis) * ecef_base_pos[2]) / (semi_major_axis * a11);
  double base_latitude = atan((ecef_base_pos[2] + (a1 * (semi_major_axis / semi_minor_axis)) * (a1 * (semi_major_axis / semi_minor_axis)) * a12) / a2);
  double base_longitude = 0;

  if (ecef_base_pos[0] >= 0)
  {
    base_longitude = (atan(ecef_base_pos[1] / ecef_base_pos[0]));
  }
  else
  {
    if (ecef_base_pos[0] < 0 && ecef_base_pos[1] >= 0)
    {
      base_longitude = M_PI + (atan(ecef_base_pos[1] / ecef_base_pos[0]));
    }
    else
    {
      base_longitude = (atan(ecef_base_pos[1] / ecef_base_pos[0])) - M_PI;
    }
  }

  double base_altitude = a10 * (1 - (semi_minor_axis * semi_minor_axis) / (semi_major_axis * a11));

  //enu2ecef
  double ecef_pos_x = ecef_base_pos[0] + ((-(sin(base_longitude)) * enu_pos[0]) + (-(cos(base_longitude)) * (sin(base_latitude)) * enu_pos[1]) + ((cos(base_longitude)) * (cos(base_latitude)) * enu_pos[2]));
  double ecef_pos_y = ecef_base_pos[1] + (((cos(base_longitude)) * enu_pos[0]) + (-(sin(base_longitude)) * (sin(base_latitude)) * enu_pos[1]) + ((sin(base_longitude)) * (cos(base_latitude)) * enu_pos[2]));
  double ecef_pos_z = ecef_base_pos[2] + ((0 * enu_pos[0]) + ((cos(base_latitude)) * enu_pos[1]) + ((sin(base_latitude)) * enu_pos[2]));

  //ecef2llh
  a1 = sqrt(1 - pow(semi_minor_axis / semi_major_axis, 2.0));
  a2 = sqrt((ecef_pos_x * ecef_pos_x) + (ecef_pos_y * ecef_pos_y));
  a3 = 54 * (semi_minor_axis * semi_minor_axis) * (ecef_pos_z * ecef_pos_z);
  a4 = (a2 * a2) + (1 - (a1 * a1)) * (ecef_pos_z * ecef_pos_z) - (a1 * a1) * (semi_major_axis * semi_major_axis - semi_minor_axis * semi_minor_axis);
  a5 = ((a1 * a1) * (a1 * a1) * a3 * (a2 * a2)) / (a4 * a4 * a4);
  a6 = pow((1 + a5 + sqrt(a5 * a5 + 2 * a5)), 1.0 / 3.0);
  a7 = a3 / (3 * pow((a6 + 1 / a6 + 1), 2.0) * a4 * a4);
  a8 = sqrt(1 + 2 * (a1 * a1) * (a1 * a1) * a7);
  a9 = -(a7 * (a1 * a1) * a2) / (1 + a8) + sqrt((semi_major_axis * semi_major_axis / 2) * (1 + 1 / a8) - (a7 * (1 - (a1 * a1)) * (ecef_pos_z * ecef_pos_z)) / (a8 * (1 + a8)) - a7 * (a2 * a2) / 2);
  a10 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (ecef_pos_z * ecef_pos_z));
  a11 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (1 - (a1 * a1)) * (ecef_pos_z * ecef_pos_z));
  a12 = ((semi_minor_axis * semi_minor_axis) * ecef_pos_z) / (semi_major_axis * a11);
  llh_pos[1] = (atan((ecef_pos_z + (a1 * (semi_major_axis / semi_minor_axis)) * (a1 * (semi_major_axis / semi_minor_axis)) * a12) / a2)) * 180 / M_PI;

  if (ecef_pos_x >= 0)
  {
    llh_pos[0] = ((atan(ecef_pos_y / ecef_pos_x)))*180 / M_PI;
  }
  else
  {
    if (ecef_pos_x < 0 && ecef_pos_y >= 0)
    {
      llh_pos[0] = (M_PI + (atan(ecef_pos_y / ecef_pos_x))) * 180 / M_PI;
    }
    else
    {
      llh_pos[0] = ((atan(ecef_pos_y / ecef_pos_x)) - M_PI) * 180 / M_PI;
    }
  }

  llh_pos[2] = a10 * (1 - (semi_minor_axis * semi_minor_axis) / (semi_major_axis * a11));
}