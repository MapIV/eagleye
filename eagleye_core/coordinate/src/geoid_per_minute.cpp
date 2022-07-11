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

// * Created by Meijo University & Map IV, Inc. based on Geoid data (Geospatial
//   Information Authority of Japan) (https://fgd.gsi.go.jp/download/geoid.php)

/*
 * geoid_per_minute.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"

#include <string>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>


double** read_geoid_map()
{

  double** data;

  std::string path = ament_index_cpp::get_package_share_directory("eagleye_coordinate") + "/data/";
  std::string file_name = "gsigeo2011_ver2.asc";
  std::ifstream ifs(path+file_name);

    if (!ifs)
    {
      fprintf(stderr, "[LocalizationTool]: Geoid database is not found\n");
      exit(2);
    }
    size_t row = 1802;
    size_t column = 1202;

    data = (double**)malloc(sizeof(double*) * row + 1);

    data[1] = (double*)malloc(sizeof(double) * column * row);
    for (int i = 2; i < row; i++)
    {
      data[i] = data[1] + i * column;
    }
    std::string str;
    getline(ifs, str);

    int i = 1;
    int j = 1;

    while (getline(ifs, str))
    {
      std::string token;
      std::istringstream stream(str);

      while (getline(stream, token, ' '))
      {
        if (token == " " || token == "" || token == "\n" || token == "\r")
        {
          continue;
        }
        if (j >= column)
        {
          j = 1;
          i += 1;
        }
        data[i][j] = std::stod(token);
        j++;
      }
    }

  return data;
}

double geoid_per_minute(double latitude,double longitude,double** geoid_map_data)
{
  double xmin = 120;
  double ymin = 20;
  double xpt = longitude;
  double ypt = latitude;
  double dx = 1.5 / 60.0;
  double dy = 1.0 / 60.0;

  int ix = (int)((xpt - xmin) / dx) + 1;
  int iy = (int)((ypt - ymin) / dy) + 1;
  int jx = ix + 1;
  int jy = iy + 1;

  double x = ((xpt - xmin) / dx) - (ix - 1);
  double y = ((ypt - ymin) / dy) - (iy - 1);
  double yy = std::fabs(y);
  double xx = std::fabs(x);
  double el2 = 1e-5;

  if (ix < 0 || ix >= 1201 || iy < 0 || iy >= 1801)
  {
    return 0.0;
  }

  int iadx = 99;
  int iady = 99;

  if (yy < el2)
  {
    iady = 0;
  }
  else if ((1 - yy) < el2)
  {
    iady = 1;
  }

  if (xx < el2)
  {
    iadx = 0;
  }
  else if ((1 - xx) < el2)
  {
    iadx = 1;
  }

  if (iady < 10)
  {
    if (iadx < 10)
    {
      return geoid_map_data[iy + iady][ix + iadx];
    }
    if (geoid_map_data[iy + iady][ix] == 999.000 || geoid_map_data[iy + iady][jx] == 999.000)
    {
      return 999.000;
    }
    else
    {
      return (1 - x) * geoid_map_data[iy + iady][ix] + x * geoid_map_data[iy + iady][jx];
    }
  }
  else
  {
    if (iadx < 10)
    {
      if (geoid_map_data[iy][ix + iadx] == 999.000 || geoid_map_data[jy][ix + iadx] == 999.000)
      {
        return 999.000;
      }
      else
      {
        return (1 - y) * geoid_map_data[iy][ix + iadx] + y * geoid_map_data[jy][ix + iadx];
      }
    }
  }

  if (geoid_map_data[jy][ix] == 999.0 || geoid_map_data[jy][jx] == 999.0 || geoid_map_data[iy][ix] == 999.0 || geoid_map_data[iy][jx] == 999.0)
  {
    return 999.0;
  }
  else
  {
    return (1 - x) * (1 - y) * geoid_map_data[iy][ix] + y * (1. - x) * geoid_map_data[jy][ix] + x * (1. - y) * geoid_map_data[iy][jx] +
           geoid_map_data[jy][jx] * x * y;
  }
}
