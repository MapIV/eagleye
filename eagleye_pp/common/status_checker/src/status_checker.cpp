
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
  std::cout<<" Time: " << (time - time_start)/(pow(10,9))<< " [s] " <<std::endl;
  std::cout<<" Distance: " << distance<<  " [m] " <<std::endl;
  std::cout<<" Fix rate: " << num_fix/num_epoch*100<< " [%] " << std::endl;
  std::cout << std::endl;

  return 0;
}
