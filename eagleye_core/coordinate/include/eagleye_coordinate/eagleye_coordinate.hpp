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

#ifndef COORDINATE_H
#define COORDINATE_H

#include <cmath>
#include "geographic_msgs/msg/geo_point.hpp"
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <iostream>

class ConvertHeight
{
public:
  ConvertHeight();

  double convert2altitude();
  double convert2ellipsoid();
  double getGeoidPerMinute();
  double getGeoidPerDegree();
  void setLLH(double, double, double);

private:
  double _latitude;
  double _longitude;
  double _height;
  double geoid;
  double converted_height;
  double** geoid_map_data;
};

extern void ll2xy(int, double*, double*);
extern void ecef2llh(double*, double*);
extern void enu2llh(double*, double*, double*);
extern void enu2xyz_vel(double*, double*, double*);
extern void llh2xyz(double*, double*);
extern void xyz2enu(double*, double*, double*);
extern void xyz2enu_vel(double*, double*, double*);
extern double geoid_per_degree(double, double);
extern double geoid_per_minute(double, double, double**);
extern double** read_geoid_map();


#endif /*COORDINATE_H */
