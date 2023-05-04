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

void print_error_and_exit(const std::string &message) {
  std::cerr << "\033[1;31mError: " << message << "\033[0m" << std::endl;
  exit(1);
}

int main(int argc, char *argv[]) {
  if (argc <= 1) {
    print_error_and_exit("Invalid Arguments");
  }

  int n_bag = std::stoi(argv[1]);
  if (argc < 4 + n_bag) {
    print_error_and_exit("Invalid Arguments");
  }

  eagleye_pp eagleye_pp_instance;
  std::vector<std::string> rosbags(argv + 2, argv + 2 + n_bag);
  std::string output_path = argv[n_bag + 2];
  std::string config_file = argv[n_bag + 3];

  eagleye_pp_instance.setOutputPath(output_path);

  MultiRosbagController rosbag_controller(rosbags);

  // Reading params from config file
  std::string twist_topic = "/can_twist";
  std::string imu_topic = "/imu/data_raw";
  std::string rtklib_nav_topic = "/rtklib_nav";
  std::string nmea_sentence_topic = "/navsat/nmea_sentence";

  eagleye_pp_instance.setParam(config_file, &twist_topic, &imu_topic, &rtklib_nav_topic, &nmea_sentence_topic);

  std::string use_gnss_mode = eagleye_pp_instance.getUseGNSSMode();
  bool use_canless_mode = eagleye_pp_instance.getUseCanlessMode();

  std::cout << "Estimate mode (GNSS) " << use_gnss_mode << std::endl;

  if (!rosbag_controller.setTopic(twist_topic) && !use_canless_mode) {
    print_error_and_exit("Cannot find the topic: " + twist_topic);
  }

  bool nmea_data_flag = rosbag_controller.setTopic(nmea_sentence_topic);
  if (!nmea_data_flag && use_gnss_mode != "rtklib" && use_gnss_mode != "RTKLIB") {
    print_error_and_exit("Cannot find the topic (Please change the Estimation mode): " + nmea_sentence_topic);
  }

  if (!rosbag_controller.setTopic(imu_topic)) {
    print_error_and_exit("Cannot find the topic: " + imu_topic);
  }

  bool use_rtklib_topic = rosbag_controller.setTopic(rtklib_nav_topic);
  if (!use_rtklib_topic) {
    std::cerr << "\033[1;31mWarning: Some rosbags do not contain the topic: " << rtklib_nav_topic << "\033[0m" << std::endl;
  }

  rosbag::View in_view;
  rosbag_controller.addQueries(in_view);

  // synchronize time
  eagleye_pp_instance.syncTimestamp(nmea_data_flag, in_view);

  std::cout << "start eagleye offline processing!" << std::endl;

  bool forward_flag = true; // Switch between forward and backward
  //forward
  eagleye_pp_instance.estimatingEagleye(forward_flag);
  std::cout << std::endl << "forward estimation finish" << std::endl;

  // Backward
  if (eagleye_pp_instance.getUseBackward()) {
    forward_flag = false;
    eagleye_pp_instance.estimatingEagleye(forward_flag);
    std::cout << std::endl << "backward estimation finish" << std::endl;
  }

  if (use_rtklib_topic) {
    eagleye_pp_instance.smoothingDeadReckoning();
    std::cout << "smoothing dead reckoning finish" << std::endl;
  }

  // forward/backward combination
  if (eagleye_pp_instance.getUseCombination()) {
    std::cout << "start eagleye forward/backward combination processing!" << std::endl;
    eagleye_pp_instance.smoothingTrajectory();
  }

  // Convert height
  eagleye_pp_instance.convertHeight();

  // Output process
  eagleye_pp_instance.writeLineKML(nmea_data_flag);
  eagleye_pp_instance.writePointKML(nmea_data_flag);
  eagleye_pp_instance.writeSimpleCSV();
  eagleye_pp_instance.writeDetailCSV();
  return 0;
}