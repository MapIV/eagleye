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

/*
 * convertheight.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"

ConvertHeight::ConvertHeight()
{
}

void ConvertHeight::setLLH(double latitude,double longitude,double height)
{
  _latitude  = latitude;
  _longitude = longitude;
  _height    = height;
}

double ConvertHeight::convert2altitude()
{
  try
  {
    GeographicLib::Geoid egm2008("egm2008-1");
    converted_height = egm2008.ConvertHeight(_latitude, _longitude, _height, GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  }
  catch (const GeographicLib::GeographicErr err)
  {
    std::cerr << "\033[31;1mError: Failed to convert height from Ellipsoid to Altitude. " << err.what() << std::endl;
    exit(4);
  }

  return converted_height;
}

double ConvertHeight::convert2ellipsoid()
{
  try
  {
    GeographicLib::Geoid egm2008("egm2008-1");
    converted_height = egm2008.ConvertHeight(_latitude, _longitude, _height, GeographicLib::Geoid::GEOIDTOELLIPSOID);
  }
  catch (const GeographicLib::GeographicErr err)
  {
    std::cerr << "\033[31;1mError: Failed to convert height from Ellipsoid to Altitude. " << err.what() << std::endl;
    exit(4);
  }

  return converted_height;
}

double ConvertHeight::getGeoidPerDegree()
{
  geoid = geoid_per_degree(_latitude,_longitude);
  return geoid;
}

double ConvertHeight::getGeoidPerMinute()
{
  geoid = geoid_per_minute(_latitude,_longitude,geoid_map_data);
  return geoid;
}
