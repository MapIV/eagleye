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

#include "nmea2fix/nmea2fix.hpp"

double stringToROSTime(std::string& input, double header_time)
{

  time_t time;
  struct tm *tm_localtime;
  struct tm tm_GPSTime;
  double GPSTime, GPSTime_msec;
  int Leaptime = 37;

  time = header_time;
  tm_localtime = localtime(&time);

  tm_GPSTime.tm_year = tm_localtime->tm_year;
  tm_GPSTime.tm_mon = tm_localtime->tm_mon;
  tm_GPSTime.tm_mday = tm_localtime->tm_mday;
  tm_GPSTime.tm_hour = stod(input.substr(0,2)) + 9;
  tm_GPSTime.tm_min = stod(input.substr(2,2));
  tm_GPSTime.tm_sec = stod(input.substr(4,2));
  GPSTime_msec = stod(input.substr(6));

  GPSTime = mktime(&tm_GPSTime) + GPSTime_msec + Leaptime;

  return GPSTime;
}

void nmea2fix_converter(const nmea_msgs::Sentence sentence, sensor_msgs::NavSatFix* fix, nmea_msgs::Gpgga* gga)
{

  std::vector<std::string> linedata,nmea_data;
  std::string token1,token2;
  std::stringstream tmp_ss(sentence.sentence);
  int i;
  int index_length;

  while (getline(tmp_ss, token1, '\n'))
  {
    linedata.push_back(token1);
  }

  index_length = std::distance(linedata.begin(), linedata.end());

  for (i = 0; i < index_length; i++)
  {
    if (linedata[i].compare(3, 3, "GGA") ==0)
    {
      std::stringstream tmp_ss1(linedata[i]);

      while (getline(tmp_ss1, token2, ','))
      {
        nmea_data.push_back(token2);
      }

      if(!nmea_data[2].empty() || !nmea_data[4].empty())
      {
        gga->header = sentence.header;
        gga->message_id = nmea_data[0];
        // gga->utc_seconds = stod(nmea_data[1]);
        if(!nmea_data[1].empty()) gga->utc_seconds = stringToROSTime(nmea_data[1], sentence.header.stamp.toSec());
        gga->lat = floor(stod(nmea_data[2])/100) + fmod(stod(nmea_data[2]),100)/60;
        gga->lat_dir = nmea_data[3];
        gga->lon = floor(stod(nmea_data[4])/100) + fmod(stod(nmea_data[4]),100)/60;
        gga->lon_dir = nmea_data[5];
        if(!nmea_data[6].empty()) gga->gps_qual = stod(nmea_data[6]);
        if(!nmea_data[7].empty()) gga->num_sats = stod(nmea_data[7]);
        if(!nmea_data[8].empty()) gga->hdop = stod(nmea_data[8]);
        if(!nmea_data[9].empty()) gga->alt = stod(nmea_data[9]);
        gga->altitude_units = nmea_data[10];
        if(!nmea_data[11].empty()) gga->undulation = stod(nmea_data[11]);
        gga->undulation_units = nmea_data[12];
        if(!nmea_data[13].empty()) gga->diff_age = stod(nmea_data[13]);
        gga->station_id = nmea_data[14].substr(0, nmea_data[14].find("*"));

        fix->header = sentence.header;
        fix->latitude = gga->lat;
        fix->longitude = gga->lon;
        fix->altitude = gga->alt + gga->undulation;
        fix->status.service = 1;

        if(gga->gps_qual == 4)
        {
          fix->status.status = 0;
        }
        else
        {
          fix->status.status = -1;
        }
      }
    }
  }
}
