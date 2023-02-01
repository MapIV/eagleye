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

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"
#include "gnss_converter/nmea2fix.hpp"
#include "eagleye_pp.hpp"
#include <multi_rosbag_controller/multi_rosbag_controller.hpp>

int main(int argc, char *argv[])
{
  eagleye_pp eagleye_pp;
  int n_bag;
  std::vector<std::string> rosbags;
  std::string output_path, config_file;

  if (argc > 1)
  {
    n_bag = std::stoi(argv[1]);
    if (argc >= 4 + n_bag)
    {
      for (int bag_id = 0; bag_id < n_bag; bag_id++)
      {
        rosbags.push_back(argv[2 + bag_id]);
      }
      output_path = argv[n_bag + 2];
      config_file = argv[n_bag + 3];
      eagleye_pp.setOutputPath(output_path);
    }
    else
    {
      std::cerr << "\033[31;1mError: Invalid Arguments\033[m" << std::endl;
    }
  }
  else
  {
    std::cerr << "\033[1;31mError: Invalid Arguments\033[m" << std::endl;
    exit(1);
  }

  MultiRosbagController rosbag_controller(rosbags);

  // Reading params from config file
  std::string twist_topic = "/can_twist";
  std::string imu_topic = "/imu/data_raw";
  std::string rtklib_nav_topic = "/rtklib_nav";
  std::string nmea_sentence_topic = "/navsat/nmea_sentence";
  bool nmea_data_flag = false;
  bool use_rtk_navsatfix_topic = false;
  bool use_rtklib_topic = false;
    
  eagleye_pp.setParam(config_file, &twist_topic, &imu_topic, &rtklib_nav_topic, &nmea_sentence_topic);

  std::string use_gnss_mode = eagleye_pp.getUseGNSSMode();

  std::cout << "Estimate mode (GNSS) " << use_gnss_mode << std::endl; 
    
  if (rosbag_controller.setTopic(std::string(twist_topic)) && !eagleye_pp.getUseCanlessMode())
  {
    std::cout << "TwistStamped topic: " << twist_topic << std::endl;
  }
  else if(eagleye_pp.getUseCanlessMode())
  {
    std::cout << "Velocity Estimate mode" << std::endl;
    if(!rosbag_controller.findTopic(std::string(nmea_sentence_topic)))
    {
      std::cerr << "\033[1;31mError: Cannot find the topic for Velocity Estimate mode: " << nmea_sentence_topic << "\033[0m" << std::endl;
      exit(1);
    } else if (!rosbag_controller.findTopic(std::string(rtklib_nav_topic)))
    {
      std::cerr << "\033[1;31mError: Cannot find the topic for Velocity Estimate mode: " << rtklib_nav_topic << "\033[0m" << std::endl;
      exit(1);
    }
  }
  else
  {
    std::cerr << "\033[1;31mError: Cannot find the topic: " << twist_topic << "\033[0m" << std::endl;
    exit(1);
  }

  if (rosbag_controller.setTopic(std::string(nmea_sentence_topic)))
  {
    std::cout << "Sentence topic: " << nmea_sentence_topic << std::endl;
    nmea_data_flag = true;
    use_rtk_navsatfix_topic = true;
  }
  else if (!rosbag_controller.setTopic(std::string(nmea_sentence_topic)) && use_gnss_mode != "rtklib" && use_gnss_mode != "RTKLIB")
  {
    std::cerr << "\033[1;31mError: Cannot find the topic (Please change the Estimation mode): " << nmea_sentence_topic << "\033[0m" << std::endl;
    exit(1);
  }
  else
  {
    std::cout << "\033[1;33mWarn: Cannot find the topic: " << nmea_sentence_topic  << "\033[0m" << std::endl;
    use_rtk_navsatfix_topic = false;
  }

  if (rosbag_controller.setTopic(std::string(imu_topic)))
  {
    std::cout << "Imu topic: " << imu_topic << std::endl;
  }
  else
  {
    std::cerr << "\033[1;31mError: Cannot find the topic: " << imu_topic << "\033[0m" << std::endl;
    exit(1);
  }

  if (rosbag_controller.setTopic(std::string(rtklib_nav_topic)))
  {
    std::cout << "RtklibNav topic: " << rtklib_nav_topic << std::endl;
    use_rtklib_topic = true;
  }
  else
  {
    std::cerr << "\033[1;31mWarning: Some rosbags do not contain the topic: " << rtklib_nav_topic << "\033[0m" << std::endl;
  }

  rosbag::View in_view;
  rosbag_controller.addQueries(in_view);

  // synchronize time
  eagleye_pp.syncTimestamp(nmea_data_flag, in_view);

  std::cout << "start eagleye offline processing!" << std::endl;

  bool forward_flag = true; // Switch between forward and backward
  //forward
  eagleye_pp.estimatingEagleye(forward_flag);
  std::cout << std::endl << "forward estimation finish" <<  std::endl;

  //Backward
  if(eagleye_pp.getUseBackward())
  {
    forward_flag = false;
    eagleye_pp.estimatingEagleye(forward_flag);
    std::cout << std::endl << "backward estimation finish" <<  std::endl;
  }
  
  if(use_rtklib_topic)
  {
    eagleye_pp.smoothingDeadReckoning();
    std::cout << std::endl << "smoothing dead reckoning finish" <<  std::endl;
  }

  // forward/backward combination
  if(eagleye_pp.getUseCombination())
  {
    std::cout << "start eagleye forward/backward combination processing!" << std::endl;
    eagleye_pp.smoothingTrajectory();
  }

  // Convert height
  eagleye_pp.convertHeight();

  //　output process
  eagleye_pp.writeLineKML(use_rtk_navsatfix_topic);
  eagleye_pp.writePointKML(use_rtk_navsatfix_topic);
  eagleye_pp.writeSimpleCSV();
  eagleye_pp.writeDetailCSV();
  return 0;
}
