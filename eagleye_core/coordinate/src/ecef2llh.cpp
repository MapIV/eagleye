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
#include <math.h>


void ecef2llh(double ecef_pos[3], double llh_pos[3])
{
  double semi_major_axis = 6378137.0000;
  double semi_minor_axis = 6356752.3142;
  double a1 = sqrt(1 - pow(semi_minor_axis / semi_major_axis, 2.0));
  double a2 = sqrt((ecef_pos[0] * ecef_pos[0]) + (ecef_pos[1] * ecef_pos[1]));
  double a3 = 54 * (semi_minor_axis * semi_minor_axis) * (ecef_pos[2] * ecef_pos[2]);
  double a4 = (a2 * a2) + (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2]) - (a1 * a1) * (semi_major_axis * semi_major_axis - semi_minor_axis * semi_minor_axis);
  double a5 = ((a1 * a1) * (a1 * a1) * a3 * (a2 * a2)) / (a4 * a4 * a4);
  double a6 = pow((1 + a5 + sqrt(a5 * a5 + 2 * a5)), 1.0 / 3.0);
  double a7 = a3 / (3 * pow((a6 + 1 / a6 + 1), 2.0) * a4 * a4);
  double a8 = sqrt(1 + 2 * (a1 * a1) * (a1 * a1) * a7);
  double a9 = -(a7 * (a1 * a1) * a2) / (1 + a8) + sqrt((semi_major_axis * semi_major_axis / 2) * (1 + 1 / a8) - (a7 * (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2])) / (a8 * (1 + a8)) - a7 * (a2 * a2) / 2);
  double a10 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (ecef_pos[2] * ecef_pos[2]));
  double a11 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2]));
  double a12 = ((semi_minor_axis * semi_minor_axis) * ecef_pos[2]) / (semi_major_axis * a11);
  llh_pos[0] = atan((ecef_pos[2] + (a1 * (semi_major_axis / semi_minor_axis)) * (a1 * (semi_major_axis / semi_minor_axis)) * a12) / a2);
  llh_pos[1] = 0;

  if (ecef_pos[0] >= 0)
  {
    llh_pos[1] = (atan(ecef_pos[1] / ecef_pos[0]));
  }
  else
  {
    if (ecef_pos[0] < 0 && ecef_pos[1] >= 0)
    {
      llh_pos[1] = M_PI + (atan(ecef_pos[1] / ecef_pos[0]));
    }
    else
    {
      llh_pos[1] = (atan(ecef_pos[1] / ecef_pos[0])) - M_PI;
    }
  }

  llh_pos[2] = a10 * (1 - (semi_minor_axis * semi_minor_axis) / (semi_major_axis * a11));
}
