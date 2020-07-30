// Copyright (c) 2019, Map IV, Inc.
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
#include <string>
#include <map>
#include <iostream>



int checkCrossBoader(std::string code_origin, std::string code_current, bool is_x)
{
  std::map<std::string, int> mgrs_alphabet{ { "A", 0 },  { "B", 1 },  { "C", 2 },  { "D", 3 },  { "E", 4 },
                                            { "F", 5 },  { "G", 6 },  { "H", 7 },  { "J", 8 },  { "K", 9 },
                                            { "L", 10 }, { "M", 11 }, { "N", 12 }, { "P", 13 }, { "Q", 14 },
                                            { "R", 15 }, { "S", 16 }, { "T", 17 }, { "U", 18 }, { "V", 19 },
                                            { "W", 20 }, { "X", 21 }, { "Y", 22 }, { "Z", 23 } };

  int diff = mgrs_alphabet[code_current] - mgrs_alphabet[code_origin];

  //std::cout << "Straddling over 2 grids is not supported." << std::endl;


  if (is_x)
  {
    if (diff == -23 || diff == 1)
    {
      return 1;
    }
    else if (diff == 23 || diff == -1)
    {
      return -1;
    }
    else if (diff == 0)
    {
      return 0;
    }
    else
    {
      std::cerr << "Straddling over 2 grids is not supported." << std::endl;
      std::cerr << "Straddling: " << diff << std::endl;
      exit(4);
    }
  }
  else
  {
    if (diff == -19 || diff == 1)
    {
      return 1;
    }
    else if (diff == 19 || diff == -1)
    {
      return -1;
    }
    else if (diff == 0)
    {
      return 0;
    }
    else
    {
      std::cerr << "Straddling over 2 grids is not supported." << std::endl;
      std::cerr << "Straddling: " << diff << std::endl;
      exit(4);
    }
  }
}

void ll2xy_mgrs(double llh[3], double xyz[3])
{
  // north/south pole is not supported in MGRS
  if (llh[0] >= 84 || llh[0] <= -80)
  {
    std::cerr << "Error: north and south pole is not supported in MGRS" << std::endl;
    exit(4);
  }

  geographic_msgs::GeoPoint wgs_point;
  wgs_point.latitude = llh[0]*180/M_PI;
  wgs_point.longitude = llh[1]*180/M_PI;
  //wgs_point.latitude = llh[0];
  //wgs_point.longitude = llh[1];
  wgs_point.altitude = llh[2];

  std::cout << wgs_point.latitude << std::endl;
  std::cout << wgs_point.longitude << std::endl;


  geodesy::UTMPoint utm_point;
  geodesy::fromMsg(wgs_point, utm_point);

  std::string easting_letters = "ABCDEFGHJKLMNPQRSTUVWXYZ";
  std::string northing_letters = "ABCDEFGHJKLMNPQRSTUV";

  // first letter
  int group = utm_point.zone % 6;
  int easting_letter_offset = 0;
  int northing_letter_offset = 0;
  switch (group)
  {
    case 1:
      easting_letter_offset = 0;   // A
      northing_letter_offset = 0;  // A
      break;
    case 2:
      easting_letter_offset = 8;   // J
      northing_letter_offset = 5;  // F
      break;
    case 3:
      easting_letter_offset = 16;  // S
      northing_letter_offset = 0;  // A
      break;
    case 4:
      easting_letter_offset = 0;   // A
      northing_letter_offset = 5;  // F
      break;
    case 5:
      easting_letter_offset = 8;   // J
      northing_letter_offset = 0;  // A
      break;
    case 0:
      easting_letter_offset = 16;  // S
      northing_letter_offset = 5;  // F
      break;
  }

  int easting_idx =
      (int)(utm_point.easting / 1e5) + easting_letter_offset - 1;  // subtract -1 so that letter starts from A
  char easting_letter = easting_letters.at(easting_idx);

  int northing_idx = (int)(fmod(utm_point.northing, 2e6)) / 1e5 + northing_letter_offset;
  northing_idx = northing_idx % northing_letters.size();
  char northing_letter = northing_letters.at(northing_idx);

  double m_x,m_y,m_z;

  m_x = fmod(utm_point.easting, 1e5);
  m_y = fmod(utm_point.northing, 1e5);
  m_z = llh[2];

  std::string m_mgrs_zone;
  std::string origin_x_zone;
  std::string origin_y_zone;
  bool use_origin_zone;
  bool is_first;



  std::stringstream ss;
  ss << (int)utm_point.zone << utm_point.band << easting_letter << northing_letter;
  m_mgrs_zone = ss.str();

  if (use_origin_zone)
  {
    if (is_first)
    {
      origin_x_zone = m_mgrs_zone.substr(3, 1);
      origin_y_zone = m_mgrs_zone.substr(4, 1);
      is_first = false;
    }
    else
    {
      std::string mgrs_x_zone = m_mgrs_zone.substr(3, 1);
      std::string mgrs_y_zone = m_mgrs_zone.substr(4, 1);

      m_x += 100000 * checkCrossBoader(origin_x_zone, mgrs_x_zone, true);
      m_y += 100000 * checkCrossBoader(origin_y_zone, mgrs_y_zone, false);
    }
  }

  xyz[1] = m_x;
  xyz[0] = m_y;
  xyz[2] = m_z;


}
