#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <yaml-cpp/yaml.h>

class eagleye_pp
{
  private:
 
  // Frequently used variables
  std::string outputpath_;
  std::size_t data_length_;

  // For inital azimuth calculation
  std::vector<bool>flag_GNSS_;
  
  // Data estimated by eagleye
  std::vector<rtklib_msgs::RtklibNav> rtklib_nav_;
  std::vector<sensor_msgs::NavSatFix> fix_;
  std::vector<sensor_msgs::Imu> imu_;
  std::vector<geometry_msgs::TwistStamped> velocity_;
  std::vector<nmea_msgs::Sentence> nmea_sentence_;
  std::vector<nmea_msgs::Gpgga> gga_;
  std::vector<nmea_msgs::Gprmc> rmc_;

  std::vector<bool> flag_reliability_buffer_,b_flag_reliability_buffer_;
  std::vector<double> enu_smoothing_trajectory_east_, enu_smoothing_trajectory_north_, enu_smoothing_trajectory_height_;
  std::vector<double> smoothing_trajectory_status_;
  std::vector<double> llh_smoothing_trajectory_lat_,llh_smoothing_trajectory_lon_,llh_smoothing_trajectory_hei_;
  std::vector<eagleye_msgs::VelocityScaleFactor> velocity_scale_factor_,b_velocity_scale_factor_;
  std::vector<eagleye_msgs::Distance> distance_,b_distance_;
  std::vector<eagleye_msgs::Heading> heading_1st_,b_heading_1st_; 
  std::vector<eagleye_msgs::Heading> heading_interpolate_1st_,b_heading_interpolate_1st_;
  std::vector<eagleye_msgs::Heading> heading_2nd_,b_heading_2nd_;
  std::vector<eagleye_msgs::Heading> heading_interpolate_2nd_,b_heading_interpolate_2nd_;
  std::vector<eagleye_msgs::Heading> heading_3rd_,b_heading_3rd_;
  std::vector<eagleye_msgs::Heading> heading_interpolate_3rd_,b_heading_interpolate_3rd_;
  std::vector<eagleye_msgs::YawrateOffset> yawrate_offset_stop_,b_yawrate_offset_stop_;
  std::vector<eagleye_msgs::YawrateOffset> yawrate_offset_1st_,b_yawrate_offset_1st_;
  std::vector<eagleye_msgs::YawrateOffset> yawrate_offset_2nd_,b_yawrate_offset_2nd_;
  std::vector<eagleye_msgs::SlipAngle> slip_angle_,b_slip_angle_;
  std::vector<eagleye_msgs::Height> height_,b_height_;
  std::vector<eagleye_msgs::Pitching> pitching_,b_pitching_;
  std::vector<eagleye_msgs::AccXOffset> acc_x_offset_,b_acc_x_offset_;
  std::vector<eagleye_msgs::AccXScaleFactor> acc_x_scale_factor_,b_acc_x_scale_factor_;
  std::vector<eagleye_msgs::Position> enu_relative_pos_,b_enu_relative_pos_;
  std::vector<geometry_msgs::Vector3Stamped> enu_vel_,b_enu_vel_;
  std::vector<eagleye_msgs::Position> enu_absolute_pos_,b_enu_absolute_pos_;
  std::vector<eagleye_msgs::Position> enu_absolute_pos_interpolate_,b_enu_absolute_pos_interpolate_; 
  std::vector<eagleye_msgs::Position> gnss_smooth_pos_enu_,b_gnss_smooth_pos_enu_;
  std::vector<sensor_msgs::NavSatFix> eagleye_fix_,b_eagleye_fix_;
  std::vector<geometry_msgs::TwistStamped> eagleye_twist_,b_eagleye_twist_;

  // Data retrieved from .yaml file
  double interval_plot_;
  double interval_line_;
  // Distance interval in the kml file to output eagleye estimation results [m]

  bool output_log_ = true; 
  bool timestamp_sort_ = true;
  int convert_height_num_;

  bool output_kml_eagleye_forward_plot_;  
  bool output_kml_eagleye_backward_plot_;
  bool output_kml_eagleye_pp_plot_;
  bool output_kml_rtklib_plot_;
  bool output_kml_gnss_plot_;
  bool output_kml_eagleye_forward_line_;
  bool output_kml_eagleye_backward_line_;
  bool output_kml_eagleye_pp_line_;

  std::string use_gnss_mode_;
  bool use_nmea_downsample_;
  double nmea_downsample_freq_;

  struct HeadingParameter heading_parameter_;
  struct HeadingInterpolateParameter heading_interpolate_parameter_;
  struct YawrateOffsetStopParameter yawrate_offset_stop_parameter_;
  struct YawrateOffsetParameter yawrate_offset_1st_parameter_;
  struct YawrateOffsetParameter yawrate_offset_2nd_parameter_;
  struct SlipangleParameter slip_angle_parameter_;
  struct TrajectoryParameter trajectory_parameter_;
  struct VelocityScaleFactorParameter velocity_scale_factor_parameter_;
  struct PositionParameter position_parameter_;
  struct PositionInterpolateParameter position_interpolate_parameter_;
  struct SmoothingParameter smoothing_parameter_;
  struct HeightParameter height_parameter_;

  // Function declaration
  public:

  eagleye_pp();//Constructor

  void setOutputPath(std::string arg_output_path);
  void setParam(YAML::Node arg_conf, std::string *arg_twist_topic, std::string *arg_imu_topic, std::string *arg_rtklib_nav_topic, std::string *arg_navsatfix_topic, std::string *arg_nmea_sentence_topic);
  void setDataLength();
  std::size_t getDataLength();
  std::string getUseGNSSMode();
  std::vector<rtklib_msgs::RtklibNav> getRtklibNavVector();

  void syncTimestamp(bool arg_nmea_data_flag, rosbag::View& arg_in_view);
  void estimatingEagleye(bool arg_forward_flag);

  // Function to calculate the initial azimuth
  void setGPSTime(double arg_GPSTime[]);
  void calcMissPositiveFIX(double arg_TH_POSMAX, double arg_GPSTime[]);
  void calcPickDR(double arg_GPSTime[], bool *arg_flag_SMRaw_2D, std::vector<int> &arg_index_DRs, std::vector<int> &arg_index_DRe);
  void calcInitialHeading(double arg_GPSTime[], bool arg_flag_SMRaw_2D[], std::vector<int> arg_index_DRs, std::vector<int> arg_index_DRe);

  void smoothingTrajectory();
  void convertHeight();
  void writePointKML(bool arg_use_rtk_navsatfix_topic,std::string* arg_s_eagleye_line,std::string* arg_s_eagleye_back_line, std::string* arg_s_eagleye_pp_line);
  void writeLineKML(bool arg_use_rtk_navsatfix_topic,std::string* arg_s_eagleye_line,std::string* arg_s_eagleye_back_line, std::string* arg_s_eagleye_pp_line);
  void writeSimpleCSV();
  void writeDetailCSV();
};
