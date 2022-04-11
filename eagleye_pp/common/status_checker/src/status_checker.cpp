// Copyright (c) 2022, Map IV, Inc.
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

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <math.h>

std::vector<std::string> split(std::string &input, char delimiter) {
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

int main(int argc, char** argv)
{

  std::string path = argv[1];
  std::ifstream ifs_p(path );

  std::cout << "OpenData: " << path<< std::endl;
  if (!ifs_p) {
      std::cout << "Couldn't open " << path  << "." << std::endl;
      exit(1);
  }

  int count = 0;

  std::string line_p;
  std::vector<std::string> str_vec;

  double time = 0;
  double time_start = 0;
  double time_last = 0;
  double velocity = 0;
  double distance = 0;
  double lat = 0;
  double lat_last = 0;
  double fix_status = 0;
  double num_epoch = 0;
  double num_fix = 0;
  bool flag_start = false;

  while (getline(ifs_p, line_p))
  {
      count++;
      if(count <= 1){
          continue;
      }

       str_vec = split(line_p, ',');
       time = std::stod(str_vec.at(0));
       velocity = std::stod(str_vec.at(14));
       lat = std::stod(str_vec.at(62));
       fix_status = std::stod(str_vec.at(74));

       if(flag_start == false)
       {
         time_start = time;
         time_last = time;
         lat_last = lat;
         flag_start = true;
       }
       else if(flag_start == true)
       {
         if(time != time_last)
         {
           distance += velocity * (time - time_last)/(pow(10,9));
         }
         if(lat_last != lat)
         {
           num_epoch++;
           if(fix_status == 0)
           {
             num_fix++;
           }
         }
         time_last = time;
         lat_last = lat;
       }
  }

  std::cout << std::endl;
  std::cout<<" Time: " << (time - time_start)/(pow(10,9))<< " [s] "  << std::endl;
  std::cout<<" Distance: " << distance<<  " [m] "  << std::endl;
  std::cout<<" Fix rate: " << num_fix/num_epoch*100<< " [%] " << std::endl;
  std::cout << std::endl;

  return 0;
}
