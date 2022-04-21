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

void llh2xyz(double llh_pos[3], double ecef_pos[3])
{
  double semi_major_axis = 6378137.0000;
  double semi_minor_axis = 6356752.3142;
  double a1 = sqrt(1 - pow((semi_minor_axis / semi_major_axis), 2.0));
  double a2 = a1 * a1;

  double phi = llh_pos[0];
  double lam = llh_pos[1];
  double hei = llh_pos[2];

  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double cos_lam = cos(lam);
  double sin_lam = sin(lam);

  double tmp1 = 1 - a2;
  double tmp2 = sqrt(1 - a2 * sin_phi * sin_phi);

  ecef_pos[0] = (semi_major_axis / tmp2 + hei) * cos_lam * cos_phi;
  ecef_pos[1] = (semi_major_axis / tmp2 + hei) * sin_lam * cos_phi;
  ecef_pos[2] = (semi_major_axis / tmp2 * tmp1 + hei) * sin_phi;
}
