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

#include "coordinate/coordinate.hpp"
#include <math.h>

void enu2llh(double enu_pos[3], double ecef_base_pos[3], double llh_pos[3])
{
  double llh_base_pos[3];
  double ecef_pos[3];
  ecef2llh(ecef_base_pos,llh_base_pos);
  ecef_pos[0] = ecef_base_pos[0] + ((-(sin(llh_base_pos[1])) * enu_pos[0]) + (-(cos(llh_base_pos[1])) * (sin(llh_base_pos[0])) * enu_pos[1]) + ((cos(llh_base_pos[1])) * (cos(llh_base_pos[0])) * enu_pos[2]));
  ecef_pos[1] = ecef_base_pos[1] + (((cos(llh_base_pos[1])) * enu_pos[0]) + (-(sin(llh_base_pos[1])) * (sin(llh_base_pos[0])) * enu_pos[1]) + ((sin(llh_base_pos[1])) * (cos(llh_base_pos[0])) * enu_pos[2]));
  ecef_pos[2] = ecef_base_pos[2] + ((0 * enu_pos[0]) + ((cos(llh_base_pos[0])) * enu_pos[1]) + ((sin(llh_base_pos[0])) * enu_pos[2]));
  ecef2llh(ecef_pos,llh_pos);
}
