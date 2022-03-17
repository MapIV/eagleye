#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"
#include "nmea2fix/nmea2fix.hpp"
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
  std::string navsatfix_topic = "/navsat/fix";
  std::string nmea_sentence_topic = "/navsat/nmea_sentence";
  bool nmea_data_flag = false;
  bool use_rtk_navsatfix_topic = false;

  YAML::Node conf = YAML::LoadFile(config_file);
    
  eagleye_pp.setParam(conf, &twist_topic, &imu_topic, &rtklib_nav_topic, &navsatfix_topic, &nmea_sentence_topic);

  std::string use_gnss_mode = eagleye_pp.getUseGNSSMode();

  std::cout << "Estimate mode (GNSS) " << use_gnss_mode << std::endl; 
    
  if (rosbag_controller.setTopic(std::string(twist_topic)))
  {
    std::cout << "TwistStamped topic: " << twist_topic << std::endl;
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
  else if (rosbag_controller.setTopic(std::string(navsatfix_topic)))
  {
    std::cout << "NavSatFix topic: " << navsatfix_topic << std::endl;
    use_rtk_navsatfix_topic = true;
  }
  else
  {
    std::cout << "\033[1;33mWarn: Cannot find the topic: " << nmea_sentence_topic << " or " << navsatfix_topic << "\033[0m" << std::endl;
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
  std::cout << std::endl << "forward estimation finish"<< std::endl;

  //Backward
  forward_flag = false;
  eagleye_pp.estimatingEagleye(forward_flag);
  std::cout << std::endl << "backward estimation finish"<< std::endl;
  
  std::size_t data_length = eagleye_pp.getDataLength();
  // Calculate initial azimuth
  double GPSTime[data_length] = {0};

  double *GNSSTime = (double*)malloc(sizeof(double) * data_length);
  std::vector<int>  index_gnsstime;
  std::vector<rtklib_msgs::RtklibNav> rtklib_nav_vector = eagleye_pp.getRtklibNavVector();
  for(int i =0; i < data_length; i++){
    GNSSTime[i] = (double)rtklib_nav_vector[i].tow / 1000;
  }
  for(int i =1; i < data_length; i++){
    if(GNSSTime[i] != GNSSTime[i-1]){
	index_gnsstime.push_back(i);
	GPSTime[i] = GNSSTime[i];
    }
  }
  for(int i =1; i < index_gnsstime.size(); i++){
    double diff_time = GNSSTime[index_gnsstime[i]] - GNSSTime[index_gnsstime[i-1]];
    int diff_cnt = index_gnsstime[i] - index_gnsstime[i-1];
    double time = diff_time/diff_cnt;
    for(int j =0; j < diff_cnt; j++){
      GPSTime[index_gnsstime[i-1]+j+1] = GPSTime[index_gnsstime[i-1]+j] + time;
    }
  }
  for(int i =0; i < index_gnsstime[0]; i++){
    GPSTime[i] = GPSTime[index_gnsstime[0]];
  }
  for(int i = index_gnsstime[index_gnsstime.size()-1]; i < data_length; i++){
    GPSTime[i] = GPSTime[index_gnsstime[index_gnsstime.size()-1]];
  }
  free(GNSSTime);

  std::vector<int> index_DRs;
  std::vector<int> index_DRe;
  std::cout << std::endl << "Start MissPositiveFIX"<< std::endl;
  bool flag_SMRaw_2D[data_length] = {0};
  double TH_POSMAX;
  // if(loop_count == 1)
  // {
  //   TH_POSMAX = 1.5;
  // }else
  // {
    TH_POSMAX = 0.3;
  // }
  eagleye_pp.calcMissPositiveFIX(TH_POSMAX, GPSTime);
  std::cout << std::endl << "Start PickDR"<< std::endl;
  eagleye_pp.calcPickDR(GPSTime, flag_SMRaw_2D, index_DRs, index_DRe);
  std::cout << std::endl << "Start initial azimuth calculation"<< std::endl;
  eagleye_pp.calcInitialHeading(GPSTime, flag_SMRaw_2D, index_DRs, index_DRe);

  // forward/backward combination
  std::cout << "start eagleye forward/backward combination processing!" << std::endl;
  eagleye_pp.smoothingTrajectory();

  // Convert height
  eagleye_pp.convertHeight();

  //　output process
  std::string s_eagleye_line;
  std::string s_eagleye_back_line;
  std::string s_eagleye_pp_line;

  eagleye_pp.writeLineKML(use_rtk_navsatfix_topic, &s_eagleye_line, &s_eagleye_back_line, &s_eagleye_pp_line);
  eagleye_pp.writePointKML(use_rtk_navsatfix_topic, &s_eagleye_line, &s_eagleye_back_line, &s_eagleye_pp_line);
  eagleye_pp.writeSimpleCSV();
  eagleye_pp.writeDetailCSV();
  return 0;
}
