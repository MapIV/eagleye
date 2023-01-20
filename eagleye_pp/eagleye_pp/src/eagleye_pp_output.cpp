#include "eagleye_pp.hpp"
#include "rtklib_msgs/RtklibNav.h"

/******************************************************************************
writePointKML
Function to output kml(plot)
bool arg_use_rtk_navsatfix_topic : Determine if to use navsatfix
******************************************************************************/
void eagleye_pp::writePointKML(bool arg_use_rtk_navsatfix_topic)
{
  std::cout << "Output file = " << outputpath_ << "eagleye.kml" << std::endl;

  kml_generator_->setIntervalType(KmlGenerator::IntervalType::DISTANCE_INTERBAL);
  kml_generator_->setLineInterval(interval_line_);
  kml_generator_->setPointInterval(interval_plot_);

  if (output_kml_eagleye_forward_plot_)
  {
    int visibility = (!output_kml_eagleye_pp_plot_ ? 1 : 0);
    kml_generator_->addPointVector2PointKML(forward_point_vector_, "Eagleye Forward Point", visibility, KmlGenerator::ColorType::RED);
  }
  if(output_kml_eagleye_backward_plot_)
  {
    int visibility = ((!output_kml_eagleye_pp_plot_ && !output_kml_eagleye_forward_plot_) ? 1 : 0);
    kml_generator_->addPointVector2PointKML(backward_point_vector_, "Eagleye Backword Point", visibility, KmlGenerator::ColorType::GREEN);
  }
  if(output_kml_eagleye_pp_plot_)
  {
    kml_generator_->addPointVector2PointKML(smoothing_point_vector_, "Eagleye Smoothing Point", 0, KmlGenerator::ColorType::BLUE);
  }


  if(output_kml_eagleye_forward_line_)
  {
    int visibility = (!(arg_use_rtk_navsatfix_topic && output_kml_eagleye_pp_line_) ? 1 : 0);
    kml_generator_->addPointVector2LineKML(forward_point_vector_, "Eagleye Forward Line", visibility, KmlGenerator::ColorType::RED);
  }
  if(output_kml_eagleye_backward_line_)
  {
    int visibility = ((!output_kml_eagleye_pp_line_ && !output_kml_eagleye_forward_line_) ? 1 : 0);
    kml_generator_->addPointVector2LineKML(backward_point_vector_, "Eagleye Backword Line", visibility, KmlGenerator::ColorType::GREEN);
  }
  if(arg_use_rtk_navsatfix_topic && output_kml_eagleye_pp_line_)
  {
    kml_generator_->addPointVector2LineKML(smoothing_point_vector_, "Eagleye Smoothing Line", 1, KmlGenerator::ColorType::BLUE);
  }

  if(output_kml_gnss_plot_)
  {
    std::vector<kml_utils::Point> rtklibnav_point_vector = rtklibnavVector2PointVector();
    kml_generator_->addPointVector2PointKML(rtklibnav_point_vector, "RTKLIB", 1, KmlGenerator::ColorType::CYAN);
  }

  if(output_kml_gnss_plot_)
  {
    int visibility = (!arg_use_rtk_navsatfix_topic ? 1 : 0);
    std::pair<std::vector<kml_utils::Point> , std::vector<kml_utils::Point> > gnss_pair_ = ggaVector2PointVectorPair();
    kml_generator_->addPointVector2PointKML(gnss_pair_.first, "GNSS_FIX", visibility, KmlGenerator::ColorType::ORANGE);
    kml_generator_->addPointVector2PointKML(gnss_pair_.second, "GNSS_NO_FIX", visibility, KmlGenerator::ColorType::MAGENTA);
  }

  kml_generator_->outputKml();
}



/************************************************************************************************
writeLineKML
Function to output kml(line)
bool arg_use_rtk_navsatfix_topic : Determine if to use navsatfix
*************************************************************************************************/
void eagleye_pp::writeLineKML(bool arg_use_rtk_navsatfix_topic)
{
  // std::ofstream output_line_kml_file(outputpath_ + "eagleye_line.kml", std::ios::out);
  std::cout << "Output file = " << outputpath_ << "eagleye_line.kml" << std::endl;
  if(!arg_use_rtk_navsatfix_topic)
  {
    std::cout << "\033[1;33mWarn:Forward/backward smoothing is not possible without nmea_msgs/Sentence. Display kml of forward processing results.\033[0m" << std::endl;
  }

  kml_generator_line_->setIntervalType(KmlGenerator::IntervalType::DISTANCE_INTERBAL);
  kml_generator_line_->setLineInterval(interval_line_);

  if(output_kml_eagleye_forward_line_)
  {
    int visibility = (!(arg_use_rtk_navsatfix_topic && output_kml_eagleye_pp_line_) ? 1 : 0);
    forward_point_vector_ = eagleyeStatus2PointVector(eagleye_state_forward_);
    kml_generator_line_->addPointVector2LineKML(forward_point_vector_, "Eagleye Forward Line", visibility, KmlGenerator::ColorType::RED);
  }

  if(output_kml_eagleye_backward_line_)
  {
    int visibility = ((!output_kml_eagleye_pp_line_ && !output_kml_eagleye_forward_line_) ? 1 : 0);
    backward_point_vector_ = eagleyeStatus2PointVector(eagleye_state_backward_);
    kml_generator_line_->addPointVector2LineKML(backward_point_vector_, "Eagleye Backword Line", visibility, KmlGenerator::ColorType::GREEN);
  }

  if(arg_use_rtk_navsatfix_topic && output_kml_eagleye_pp_line_)
  {
    smoothing_point_vector_ = smoothingLLH2PointVector();
    kml_generator_line_->addPointVector2LineKML(smoothing_point_vector_, "Eagleye Smoothing Line", 1, KmlGenerator::ColorType::BLUE);
  }

  kml_generator_line_->outputKml();

}


/************************************************************
writeSimpleCSV
Function to output csv(Simplified version)
************************************************************/
void eagleye_pp::writeSimpleCSV(void)
{
  std::ofstream output_csv_file(outputpath_ + "eagleye.csv", std::ios::out);
  std::cout << "Output file = " << outputpath_ << "eagleye.csv" << std::endl;

  if (!boost::filesystem::is_directory(outputpath_))
  {
    boost::filesystem::create_directory(outputpath_);
    std::cout << "Directory created: " << outputpath_ << std::endl;
  }

  // TODO(Map IV): temporary covariance value
  double std_dev_pos = (!getUseCombination()) ? 100 : 1.5; // [m]
  double std_dev_pos_z = (!getUseCombination()) ? 100 : 1.5; // [m]
  double std_dev_roll; // [rad]
  double std_dev_pitch; // [rad]
  double std_dev_yaw; // [rad]

  // output_csv_file header
output_csv_file << "timestamp,eagleye_llh.latitude,eagleye_llh.longitude,eagleye_llh.altitude\
,eagleye_llh.position_covariance[0],eagleye_llh.position_covariance[1],eagleye_llh.position_covariance[2],eagleye_llh.position_covariance[3],eagleye_llh.position_covariance[4],eagleye_llh.position_covariance[5],eagleye_llh.position_covariance[6],eagleye_llh.position_covariance[7],eagleye_llh.position_covariance[8]\
,eagleye_llh.status,eagleye_twist.linear.x,eagleye_twist.linear.y,eagleye_twist.linear.z,eagleye_twist.angular.x,eagleye_twist.angular.y,eagleye_twist.angular.z\
,eagleye_twist.orientation_covariance[0],eagleye_twist.orientation_covariance[1],eagleye_twist.orientation_covariance[2],eagleye_twist.orientation_covariance[3],eagleye_twist.orientation_covariance[4],eagleye_twist.orientation_covariance[5],eagleye_twist.orientation_covariance[6],eagleye_twist.orientation_covariance[7],eagleye_twist.orientation_covariance[8]\
,eagleye_twist.orientation_covariance[9],eagleye_twist.orientation_covariance[10],eagleye_twist.orientation_covariance[11],eagleye_twist.orientation_covariance[12],eagleye_twist.orientation_covariance[13],eagleye_twist.orientation_covariance[14],eagleye_twist.orientation_covariance[15],eagleye_twist.orientation_covariance[16],eagleye_twist.orientation_covariance[17]\
,eagleye_acceleration.x,eagleye_acceleration.y,eagleye_acceleration.z\
,eagleye_acceleration.orientation_covariance[0],eagleye_acceleration.orientation_covariance[1],eagleye_acceleration.orientation_covariance[2],eagleye_acceleration.orientation_covariance[3],eagleye_acceleration.orientation_covariance[4],eagleye_acceleration.orientation_covariance[5],eagleye_acceleration.orientation_covariance[6],eagleye_acceleration.orientation_covariance[7],eagleye_acceleration.orientation_covariance[8]\
,eagleye_posture.roll,eagleye_posture.pitch,eagleye_posture.yaw\
,eagleye_posture.orientation_covariance[0],eagleye_posture.orientation_covariance[1],eagleye_posture.orientation_covariance[2],eagleye_posture.orientation_covariance[3],eagleye_posture.orientation_covariance[4],eagleye_posture.orientation_covariance[5],eagleye_posture.orientation_covariance[6],eagleye_posture.orientation_covariance[7],eagleye_posture.orientation_covariance[8]\
,navsat_llh.latitude,navsat_llh.longitude,navsat_llh.altitude\
,navsat_llh.orientation_covariance[0],navsat_llh.orientation_covariance[1],navsat_llh.orientation_covariance[2],navsat_llh.orientation_covariance[3],navsat_llh.orientation_covariance[4],navsat_llh.orientation_covariance[5],navsat_llh.orientation_covariance[6],navsat_llh.orientation_covariance[7],navsat_llh.orientation_covariance[8]\
,navsat_llh.gps_qual\
,eagleye_pp_llh.latitude,eagleye_pp_llh.longitude,eagleye_pp_llh.altitude\
,eagleye_pp_llh.position_covariance[0],eagleye_pp_llh.position_covariance[1],eagleye_pp_llh.position_covariance[2],eagleye_pp_llh.position_covariance[3],eagleye_pp_llh.position_covariance[4],eagleye_pp_llh.position_covariance[5],eagleye_pp_llh.position_covariance[6],eagleye_pp_llh.position_covariance[7],eagleye_pp_llh.position_covariance[8]\
,eagleye_pp_llh.status\
" << std::endl;

  std::size_t data_length = std::distance(smoothing_trajectory_status_.begin(), smoothing_trajectory_status_.end());
  std::cout << "data_length: " << data_length << std::endl;

  // output debug file
  for(int i=0; i<data_length_; i++)
  {

    if(smoothing_trajectory_status_.empty() || smoothing_trajectory_status_[i] == -1)
    {
      std_dev_pos = 100.0;
      std_dev_pos_z = 100.0;
    }
    else if(smoothing_trajectory_status_[i] == 0)
    {
      std_dev_pos = 0.3;
    }
    else if(smoothing_trajectory_status_[i] == 1)
    {
      std_dev_pos = 1.5;
    }
    else if(smoothing_trajectory_status_[i] == 2)
    {
      std_dev_pos = 4.0;
    }

    std_dev_roll = (eagleye_state_forward_.rolling[i].status.enabled_status) ? 0.5 / 180 * M_PI : 100;
    std_dev_pitch = (eagleye_state_forward_.pitching[i].status.enabled_status) ? 0.5 / 180 * M_PI : 100;
    std_dev_yaw = (eagleye_state_forward_.heading_interpolate_3rd[i].status.enabled_status) ? 0.2 / 180 * M_PI : 100;

    output_csv_file << std::setprecision(15) << imu_[i].header.stamp.toSec() << ","; // timestamp
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].latitude << ","; // eagleye_llh.latitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].longitude << ","; // eagleye_llh.longitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].altitude << ","; // eagleye_llh.altitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[0] << ","; // eagleye_llh.position_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[1] << ","; // eagleye_llh.position_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[2] << ","; // eagleye_llh.position_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[3] << ","; // eagleye_llh.position_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[4] << ","; // eagleye_llh.position_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[5] << ","; // eagleye_llh.position_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[6] << ","; // eagleye_llh.position_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[7] << ","; // eagleye_llh.position_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[8] << ","; // eagleye_llh.position_covariance[8]
    output_csv_file << bool(eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.estimate_status) << ","; // eagleye_llh.status
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist_with_covariance[i].twist.twist.linear.x << ","; // eagleye_twist.linear.x
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist_with_covariance[i].twist.twist.linear.y << ","; // eagleye_twist.linear.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist_with_covariance[i].twist.twist.linear.z << ","; // eagleye_twist.linear.z
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist_with_covariance[i].twist.twist.angular.x << ","; // eagleye_twist.angular.x
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist_with_covariance[i].twist.twist.angular.y << ","; // eagleye_twist.angular.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist_with_covariance[i].twist.twist.angular.z << ","; // eagleye_twist.angular.z
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[9]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[10]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[11]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[12]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[13]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[14]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[15]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[16]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_twist.covariance[17]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.x << ","; // eagleye_acceleration.x TODO Change to acceleration estimated by eagleye
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.y << ","; // eagleye_acceleration.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.z << ","; // eagleye_acceleration.z
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_acceleration.orientation_covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.rolling[i].rolling_angle << ","; // eagleye_posture.roll
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.pitching[i].pitching_angle << ","; // eagleye_posture.pitch
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_3rd[i].heading_angle << ","; // eagleye_posture.yaw
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << std_dev_roll * std_dev_roll << ","; // eagleye_posture.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << std_dev_pitch * std_dev_pitch << ","; // eagleye_posture.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << std_dev_yaw * std_dev_yaw << ","; // eagleye_posture.orientation_covariance[8]
    double navsat_lat = 0.0, navsat_lon = 0.0;
    if (gga_[i].lat_dir == "N") {
      navsat_lat = gga_[i].lat;
    } else if (gga_[i].lat_dir == "S") {
      navsat_lat = -gga_[i].lat;
    }
    if (gga_[i].lon_dir == "E") {
      navsat_lon = gga_[i].lon;
    } else if (gga_[i].lon_dir == "W") {
      navsat_lon = -gga_[i].lon;
    }
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << navsat_lat << ","; // navsat_llh.latitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << navsat_lon << ","; // navsat_llh.longitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].alt +  gga_[i].undulation<< ","; // navsat_llh.altitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // navsat_llh.position_covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(gga_[i].gps_qual) << ","; // navsat_llh.gps_qual
    if(getUseCombination())
    {
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lat_[i] << ","; // eagleye_pp_llh.latitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lon_[i] << ","; // eagleye_pp_llh.longitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_hei_[i] << ","; // eagleye_pp_llh.altitude
    }
    else
    {
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.latitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.longitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.altitude
    }
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << std_dev_pos * std_dev_pos << ","; // eagleye_pp_llh.position_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.position_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.position_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.position_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << std_dev_pos * std_dev_pos  << ","; // eagleye_pp_llh.position_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.position_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.position_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.position_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << std_dev_pos_z * std_dev_pos_z  << ","; // eagleye_pp_llh.position_covariance[8]
    if(getUseCombination())
    {
      output_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << smoothing_trajectory_status_[i]; // eagleye_pp_llh.status
    }
    else
    {
      output_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0; // eagleye_pp_llh.status
    }
    output_csv_file << "\n";
  }
}


void eagleye_pp::writeDetailCSVOneWay(std::ofstream* output_log_csv_file, const EagleyeStates& eagleye_state)
{
    *output_log_csv_file << "timestamp,imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z,imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z\
,rtklib_nav.tow,rtklib_nav.ecef_pos.x,rtklib_nav.ecef_pos.y,rtklib_nav.ecef_pos.z,rtklib_nav.ecef_vel.x,rtklib_nav.ecef_vel.y,rtklib_nav.ecef_vel.z,rtklib_nav.status.status.status,rtklib_nav.status.status.service,rtklib_nav.status.latitude,rtklib_nav.status.longitude,rtklib_nav.status.altitude\
,velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.linear.z,velocity.twist.angular.x,velocity.twist.angular.y,velocity.twist.angular.z\
,velocity_scale_factor.scale_factor,correction_velocity.twist.linear.x,correction_velocity.twist.linear.y,correction_velocity.twist.linear.z,correction_velocity.twist.angular.x,correction_velocity.twist.angular.y,correction_velocity.twist.angular.z,velocity_scale_factor.status.enabled_status,velocity_scale_factor.status.estimate_status\
,distance.distance,distance.status.enabled_status,distance.status.estimate_status\
,heading_1st.heading_angle,heading_1st.variance,heading_1st.status.enabled_status,heading_1st.status.estimate_status\
,heading_interpolate_1st.heading_angle,heading_interpolate_1st.variance,heading_interpolate_1st.status.enabled_status,heading_interpolate_1st.status.estimate_status\
,heading_2nd.heading_angle,heading_2nd.variance,heading_2nd.status.enabled_status,heading_2nd.status.estimate_status\
,heading_interpolate_2nd.heading_angle,heading_interpolate_2nd.variance,heading_interpolate_2nd.status.enabled_status,heading_interpolate_2nd.status.estimate_status\
,heading_3rd.heading_angle,heading_3rd.variance,heading_3rd.status.enabled_status,heading_3rd.status.estimate_status\
,heading_interpolate_3rd.heading_angle,heading_interpolate_3rd.variance,heading_interpolate_3rd.status.enabled_status,heading_interpolate_3rd.status.estimate_status\
,yawrate_offset_stop.yawrate_offset,yawrate_offset_stop.status.enabled_status,yawrate_offset_stop.status.estimate_status\
,yawrate_offset_1st.yawrate_offset,yawrate_offset_1st.status.enabled_status,yawrate_offset_1st.status.estimate_status\
,yawrate_offset_2nd.yawrate_offset,yawrate_offset_2nd.status.enabled_status,yawrate_offset_2nd.status.estimate_status\
,slip_angle.coefficient,slip_angle.slip_angle,slip_angle.status.enabled_status,slip_angle.status.estimate_status\
,enu_vel.vector.x,enu_vel.vector.y,enu_vel.vector.z\
,enu_absolute_pos.enu_pos.x,enu_absolute_pos.enu_pos.y,enu_absolute_pos.enu_pos.z,enu_absolute_pos.ecef_base_pos.x,enu_absolute_pos.ecef_base_pos.y,enu_absolute_pos.ecef_base_pos.z\
,enu_absolute_pos.covariance[0],enu_absolute_pos.covariance[1],enu_absolute_pos.covariance[2],enu_absolute_pos.covariance[3],enu_absolute_pos.covariance[4],enu_absolute_pos.covariance[5],enu_absolute_pos.covariance[6],enu_absolute_pos.covariance[7],enu_absolute_pos.covariance[8]\
,enu_absolute_pos.status.enabled_status,enu_absolute_pos.status.estimate_status\
,enu_absolute_pos_interpolate.enu_pos.x,enu_absolute_pos_interpolate.enu_pos.y,enu_absolute_pos_interpolate.enu_pos.z,enu_absolute_pos_interpolate.ecef_base_pos.x,enu_absolute_pos_interpolate.ecef_base_pos.y,enu_absolute_pos_interpolate.ecef_base_pos.z\
,enu_absolute_pos_interpolate.covariance[0],enu_absolute_pos_interpolate.covariance[1],enu_absolute_pos_interpolate.covariance[2],enu_absolute_pos_interpolate.covariance[3],enu_absolute_pos_interpolate.covariance[4],enu_absolute_pos_interpolate.covariance[5],enu_absolute_pos_interpolate.covariance[6],enu_absolute_pos_interpolate.covariance[7],enu_absolute_pos_interpolate.covariance[8]\
,enu_absolute_pos_interpolate.status.enabled_status,enu_absolute_pos_interpolate.status.estimate_status\
,height.height,height.status.enabled_status,height.status.estimate_status\
,pitching.pitching_angle,pitching.status.enabled_status,pitching.status.estimate_status\
,acc_x_offset.acc_x_offset,acc_x_offset.status.enabled_status,acc_x_offset.status.estimate_status\
,acc_x_scale_factor.acc_x_scale_factor,acc_x_scale_factor.status.enabled_status,acc_x_scale_factor.status.estimate_status\
,rolling.rolling_angle,rolling.status.enabled_status,rolling.status.estimate_status\
,gga_timestamp\
,gga_llh.latitude,gga_llh.longitude,gga_llh.altitude\
,gga_llh.gps_qual\
,eagleye_pp_llh.latitude,eagleye_pp_llh.longitude,eagleye_pp_llh.altitude\
,eagleye_pp_llh.orientation_covariance[0],eagleye_pp_llh.orientation_covariance[1],eagleye_pp_llh.orientation_covariance[2],eagleye_pp_llh.orientation_covariance[3],eagleye_pp_llh.orientation_covariance[4],eagleye_pp_llh.orientation_covariance[5],eagleye_pp_llh.orientation_covariance[6],eagleye_pp_llh.orientation_covariance[7],eagleye_pp_llh.orientation_covariance[8]\
,eagleye_pp_llh.status\
,eagleye_pp_llh.height_status\
,enu_relative_pos.enu_pos.x,enu_relative_pos.enu_pos.y,enu_relative_pos.enu_pos.z\
,enu_relative_pos.status.enabled_status\
" << std::endl;
    // ,angular_velocity_offset_stop.rollrate_offset,angular_velocity_offset_stop.pitchrate_offset,angular_velocity_offset_stop.yawrate_offset,angular_velocity_offset_stop.status.enabled_status,angular_velocity_offset_stop.status.estimate_status
    for(int i=0; i<data_length_; i++)
    {
      *output_log_csv_file << std::setprecision(15) << imu_[i].header.stamp.toSec() << ","; //timestamp
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].tow << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(rtklib_nav_[i].status.status.status) << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].status.status.service << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.latitude << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.longitude << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.altitude << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.linear.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.linear.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.linear.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.angular.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.angular.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.angular.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.velocity_scale_factor[i].scale_factor << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.correction_velocity[i].twist.linear.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.correction_velocity[i].twist.linear.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.correction_velocity[i].twist.linear.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.correction_velocity[i].twist.angular.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.correction_velocity[i].twist.angular.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.correction_velocity[i].twist.angular.z << ",";
      *output_log_csv_file << (eagleye_state.velocity_scale_factor[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.velocity_scale_factor[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.distance[i].distance << ",";
      *output_log_csv_file << (eagleye_state.distance[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.distance[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_1st[i].heading_angle << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_1st[i].variance << ",";
      *output_log_csv_file << (eagleye_state.heading_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.heading_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_1st[i].heading_angle << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_1st[i].variance << ",";
      *output_log_csv_file << (eagleye_state.heading_interpolate_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.heading_interpolate_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_2nd[i].heading_angle << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_2nd[i].variance << ",";
      *output_log_csv_file << (eagleye_state.heading_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.heading_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_2nd[i].heading_angle << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_2nd[i].variance << ",";
      *output_log_csv_file << (eagleye_state.heading_interpolate_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.heading_interpolate_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_3rd[i].heading_angle << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_3rd[i].variance << ",";
      *output_log_csv_file << (eagleye_state.heading_3rd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.heading_3rd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_3rd[i].heading_angle << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_3rd[i].variance << ",";
      *output_log_csv_file << (eagleye_state.heading_interpolate_3rd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.heading_interpolate_3rd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.yawrate_offset_stop[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state.yawrate_offset_stop[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.yawrate_offset_stop[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.yawrate_offset_1st[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state.yawrate_offset_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.yawrate_offset_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.yawrate_offset_2nd[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state.yawrate_offset_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.yawrate_offset_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.slip_angle[i].coefficient << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.slip_angle[i].slip_angle << ",";
      *output_log_csv_file << (eagleye_state.slip_angle[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.slip_angle[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_vel[i].vector.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_vel[i].vector.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_vel[i].vector.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].enu_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].ecef_base_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].ecef_base_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].ecef_base_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[0] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[1] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[2] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[3] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[4] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[5] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[6] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[7] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos[i].covariance[8] << ",";
      *output_log_csv_file << (eagleye_state.enu_absolute_pos[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.enu_absolute_pos[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].enu_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].ecef_base_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].ecef_base_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].ecef_base_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[0] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[1] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[2] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[3] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[4] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[5] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[6] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[7] << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_absolute_pos_interpolate[i].covariance[8] << ",";
      *output_log_csv_file << (eagleye_state.enu_absolute_pos_interpolate[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.enu_absolute_pos_interpolate[i].status.estimate_status ? "1" : "0") << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.angular_velocity_offset_stop.rollrate_offset << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.angular_velocity_offset_stop.pitchrate_offset << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.angular_velocity_offset_stop.yawrate_offset << ",";
      // *output_log_csv_file << (eagleye_state.angular_velocity_offset_stop.status.enabled_status ? "1" : "0") << ",";
      // *output_log_csv_file << (eagleye_state.angular_velocity_offset_stop.status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.height[i].height << ",";
      *output_log_csv_file << (eagleye_state.height[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.height[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.pitching[i].pitching_angle << ",";
      *output_log_csv_file << (eagleye_state.pitching[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.pitching[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.acc_x_offset[i].acc_x_offset << ",";
      *output_log_csv_file << (eagleye_state.acc_x_offset[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.acc_x_offset[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.acc_x_scale_factor[i].acc_x_scale_factor << ",";
      *output_log_csv_file << (eagleye_state.acc_x_scale_factor[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.acc_x_scale_factor[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.rolling[i].rolling_angle << ",";
      *output_log_csv_file << (eagleye_state.rolling[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state.rolling[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << gga_[i].header.stamp.toNSec() << ","; // timestamp
      double navsat_lat = 0.0, navsat_lon = 0.0;
      if (gga_[i].lat_dir == "N") {
        navsat_lat = gga_[i].lat;
      } else if (gga_[i].lat_dir == "S") {
        navsat_lat = -gga_[i].lat;
      }
      if (gga_[i].lon_dir == "E") {
        navsat_lon = gga_[i].lon;
      } else if (gga_[i].lon_dir == "W") {
        navsat_lon = -gga_[i].lon;
      }
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << navsat_lat << ","; // gga_llh.latitude
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << navsat_lon << ","; // gga_llh.longitude
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].alt +  gga_[i].undulation << ","; // gga_llh.altitude
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(gga_[i].gps_qual) << ","; // gga_llh.gps_qual
      if(getUseCombination())
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lat_[i] << ","; // eagleye_pp_llh.latitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lon_[i] << ","; // eagleye_pp_llh.longitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_hei_[i] << ","; // eagleye_pp_llh.altitude
      }
      else
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].latitude << ","; // eagleye_pp_llh.latitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].longitude << ","; // eagleye_pp_llh.longitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].altitude << ","; // eagleye_pp_llh.altitude     
      }
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[0]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[1]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[2]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[3]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[4]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[5]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[6]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[7]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[8]
      if(getUseCombination())
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << smoothing_trajectory_status_[i] << ","; // eagleye_pp_llh.status
      }
      else
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; // eagleye_pp_llh.status    
      }
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << eagleye_state.flag_reliability_buffer[i] << ","; // eagleye_pp_llh.status
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_relative_pos[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_relative_pos[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.enu_relative_pos[i].enu_pos.z << ",";
      *output_log_csv_file << (eagleye_state.enu_relative_pos[i].status.enabled_status ? "1" : "0");
      *output_log_csv_file << "\n";
    }
}

/************************************************************
writeDetailCSV
Function to output csv(Detailed version)
************************************************************/
void eagleye_pp::writeDetailCSV(void)
{

  if (!boost::filesystem::is_directory(outputpath_))
  {
    boost::filesystem::create_directory(outputpath_);
    std::cout << "Directory created: " << outputpath_ << std::endl;
  }

  if(output_log_)
  {
    std::ofstream output_log_csv_file(outputpath_ + "eagleye_log.csv", std::ios::out);
    std::cout << "Output file = " << outputpath_ << "eagleye_log.csv" << std::endl;
    writeDetailCSVOneWay(&output_log_csv_file, eagleye_state_forward_);

    if(getUseBackward())
    {
      std::ofstream output_log_back_csv_file(outputpath_ + "eagleye_log_back.csv", std::ios::out);
      std::cout << "Output file = " << outputpath_ << "eagleye_log_back.csv" << std::endl;
      writeDetailCSVOneWay(&output_log_back_csv_file, eagleye_state_backward_);
    }
  }
}

std::vector<kml_utils::Point> eagleye_pp::eagleyeStatus2PointVector( const EagleyeStates& eagleye_state)
{
  int num_vector = eagleye_state.velocity_scale_factor.size();
  std::vector<kml_utils::Point> point_vector;
  
  for(int i=0; i< num_vector ; i++)
  {
    kml_utils::Point point;
    point.seq = i;
    point.time = imu_[i].header.stamp.toSec();
    point.latitude = eagleye_state.eagleye_fix[i].latitude;
    point.longitude = eagleye_state.eagleye_fix[i].longitude;
    point.altitude = eagleye_state.eagleye_fix[i].altitude;

    kml_utils::addOtherInformation(point, "CAN Velocity [km/h] ", kml_utils::makeDouble2String(velocity_[i].twist.linear.x * 3.6));
    kml_utils::addOtherInformation(point, "IMU Linear Acceleration X [m/s^2] ", kml_utils::makeDouble2String(imu_[i].linear_acceleration.x));
    kml_utils::addOtherInformation(point, "IMU Linear Acceleration Y [m/s^2] ", kml_utils::makeDouble2String(imu_[i].linear_acceleration.y));
    kml_utils::addOtherInformation(point, "IMU Linear Acceleration Z [m/s^2] ", kml_utils::makeDouble2String(imu_[i].linear_acceleration.z));
    kml_utils::addOtherInformation(point, "IMU Angular Velocity X [rad/s] ", kml_utils::makeDouble2String(imu_[i].angular_velocity.x));
    kml_utils::addOtherInformation(point, "IMU Angular Velocity Y [rad/s] ", kml_utils::makeDouble2String(imu_[i].angular_velocity.y));
    kml_utils::addOtherInformation(point, "IMU Angular Velocity Z [rad/s] ", kml_utils::makeDouble2String(imu_[i].angular_velocity.z));
    kml_utils::addOtherInformation(point, "Estimate Velocity [km/h] ", kml_utils::makeDouble2String(eagleye_state.correction_velocity[i].twist.linear.x * 3.6));
    kml_utils::addOtherInformation(point, "Velocity Scale Factor", kml_utils::makeDouble2String(eagleye_state.velocity_scale_factor[i].scale_factor));
    kml_utils::addOtherInformation(point, "Velocity Scale Factor Flag", kml_utils::makeBool2String(eagleye_state.velocity_scale_factor[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Heading 1st [rad] ", kml_utils::makeDouble2String(eagleye_state.heading_interpolate_1st[i].heading_angle));
    kml_utils::addOtherInformation(point, "Estimate Heading 1st Flag", kml_utils::makeBool2String(eagleye_state.heading_interpolate_1st[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Heading 2nd [rad] ", kml_utils::makeDouble2String(eagleye_state.heading_interpolate_2nd[i].heading_angle));
    kml_utils::addOtherInformation(point, "Estimate Heading 2nd Flag", kml_utils::makeBool2String(eagleye_state.heading_interpolate_2nd[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Heading 3rd [rad] ", kml_utils::makeDouble2String(eagleye_state.heading_interpolate_3rd[i].heading_angle));
    kml_utils::addOtherInformation(point, "Estimate Heading 3rd Flag", kml_utils::makeBool2String(eagleye_state.heading_interpolate_3rd[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Yawrate Offset Stop [rad] ", kml_utils::makeDouble2String(eagleye_state.yawrate_offset_stop[i].yawrate_offset));
    kml_utils::addOtherInformation(point, "Estimate Yawrate Offset Stop Flag", kml_utils::makeBool2String(eagleye_state.yawrate_offset_stop[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Yawrate Offset 1st [rad] ", kml_utils::makeDouble2String(eagleye_state.yawrate_offset_1st[i].yawrate_offset));
    kml_utils::addOtherInformation(point, "Estimate Yawrate Offset 1st Flag", kml_utils::makeBool2String(eagleye_state.yawrate_offset_1st[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Yawrate Offset 2nd [rad] ", kml_utils::makeDouble2String(eagleye_state.yawrate_offset_2nd[i].yawrate_offset));
    kml_utils::addOtherInformation(point, "Estimate Yawrate Offset 2nd Flag", kml_utils::makeBool2String(eagleye_state.yawrate_offset_2nd[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Tire Slip Angle [rad] ", kml_utils::makeDouble2String(eagleye_state.slip_angle[i].slip_angle));
    kml_utils::addOtherInformation(point, "Estimate Tire Slip Angle Flag", kml_utils::makeBool2String(eagleye_state.slip_angle[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Pitch Angle [rad] ", kml_utils::makeDouble2String(eagleye_state.pitching[i].pitching_angle));
    kml_utils::addOtherInformation(point, "Estimate Pitch Angle Flag", kml_utils::makeBool2String(eagleye_state.pitching[i].status.enabled_status));
    kml_utils::addOtherInformation(point, "Estimate Height Angle Flag", kml_utils::makeBool2String(eagleye_state.height[i].status.enabled_status));

    point_vector.push_back(point);
  }
  return point_vector;
}

std::vector<kml_utils::Point> eagleye_pp::smoothingLLH2PointVector()
{
  int num_vector = llh_smoothing_trajectory_lat_.size();
  std::vector<kml_utils::Point> point_vector;
  
  for(int i=0; i< num_vector ; i++)
  {
    kml_utils::Point point;
    point.seq = i;
    point.time = imu_[i].header.stamp.toSec();
    point.latitude = llh_smoothing_trajectory_lat_[i];
    point.longitude = llh_smoothing_trajectory_lon_[i];
    point.altitude =  llh_smoothing_trajectory_hei_[i];
    point_vector.push_back(point);
  }
  return point_vector;
}

std::vector<kml_utils::Point> eagleye_pp::rtklibnavVector2PointVector()
{
  int num_vector = rtklib_nav_.size();
  std::vector<kml_utils::Point> point_vector;
  
  for(int i=0; i< num_vector ; i++)
  {
    kml_utils::Point point;
    point.seq = i;
    point.time = rtklib_nav_[i].header.stamp.toSec();
    point.latitude = rtklib_nav_[i].status.latitude;
    point.longitude = rtklib_nav_[i].status.longitude;
    point.altitude =  rtklib_nav_[i].status.altitude;
    point_vector.push_back(point);
  }
  return point_vector;
}

std::pair<std::vector<kml_utils::Point> , std::vector<kml_utils::Point> > eagleye_pp::ggaVector2PointVectorPair()
{
  int num_vector = gga_.size();
  std::vector<kml_utils::Point> fix_vector;
  std::vector<kml_utils::Point> nofix_vector;
  
  for(int i=0; i< num_vector ; i++)
  {
    kml_utils::Point point;
    point.seq = i;
    point.time = gga_[i].header.stamp.toSec();
    if (gga_[i].lat_dir == "N") {
      point.latitude = gga_[i].lat;
    } else if (gga_[i].lat_dir == "S") {
      point.latitude = -gga_[i].lat;
    }
    if (gga_[i].lon_dir == "E") {
      point.longitude = gga_[i].lon;
    } else if (gga_[i].lon_dir == "W") {
      point.longitude = -gga_[i].lon;
    }
    point.altitude =  gga_[i].alt +  gga_[i].undulation;
    if(int(gga_[i].gps_qual) == 4) 
    {
      fix_vector.push_back(point);
    }
    else
    {
      nofix_vector.push_back(point);
    }
  }
  std::pair<std::vector<kml_utils::Point> , std::vector<kml_utils::Point> > gnss_pair = std::make_pair(fix_vector, nofix_vector);

  return gnss_pair;
}