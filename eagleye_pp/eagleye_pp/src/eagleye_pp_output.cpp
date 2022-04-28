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
    int visibility = (!output_kml_eagleye_pp_line_ ? 1 : 0);
    kml_generator_->addPointVector2LineKML(forward_point_vector_, "Eagleye Forward Line", visibility, KmlGenerator::ColorType::RED);
  }
  if(output_kml_eagleye_backward_line_)
  {
    int visibility = ((!output_kml_eagleye_pp_line_ && !output_kml_eagleye_forward_line_) ? 1 : 0);
    kml_generator_->addPointVector2LineKML(backward_point_vector_, "Eagleye Backword Line", visibility, KmlGenerator::ColorType::GREEN);
  }
  if(output_kml_eagleye_pp_line_)
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
    std::cout << "\033[1;33mWarn: Display kml of forward processing results.\033[0m" << std::endl;
  }

  kml_generator_line_->setIntervalType(KmlGenerator::IntervalType::DISTANCE_INTERBAL);
  kml_generator_line_->setLineInterval(interval_line_);

  if(output_kml_eagleye_forward_line_)
  {
    int visibility = (!output_kml_eagleye_pp_line_ ? 1 : 0);
    forward_point_vector_ =eagleyeStatus2PointVector(eagleye_state_forward_);
    kml_generator_line_->addPointVector2LineKML(forward_point_vector_, "Eagleye Forward Line", visibility, KmlGenerator::ColorType::RED);
  }

  if(output_kml_eagleye_backward_line_)
  {
    int visibility = ((!output_kml_eagleye_pp_line_ && !output_kml_eagleye_forward_line_) ? 1 : 0);
    backward_point_vector_ =eagleyeStatus2PointVector(eagleye_state_backward_);
    kml_generator_line_->addPointVector2LineKML(backward_point_vector_, "Eagleye Backword Line", visibility, KmlGenerator::ColorType::GREEN);
  }

  if(output_kml_eagleye_pp_line_)
  {
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
,eagleye_pp_llh.orientation_covariance[0],eagleye_pp_llh.orientation_covariance[1],eagleye_pp_llh.orientation_covariance[2],eagleye_pp_llh.orientation_covariance[3],eagleye_pp_llh.orientation_covariance[4],eagleye_pp_llh.orientation_covariance[5],eagleye_pp_llh.orientation_covariance[6],eagleye_pp_llh.orientation_covariance[7],eagleye_pp_llh.orientation_covariance[8]\
,eagleye_pp_llh.status\
" << std::endl;

  // output debug file
  for(int i=0; i<data_length_; i++)
  {
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
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.linear.x << ","; // eagleye_twist.linear.x
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.linear.y << ","; // eagleye_twist.linear.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.linear.z << ","; // eagleye_twist.linear.z
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.angular.x << ","; // eagleye_twist.angular.x
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.angular.y << ","; // eagleye_twist.angular.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.angular.z << ","; // eagleye_twist.angular.z
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
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_posture.orientation_covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lat << ","; // navsat_llh.latitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lon << ","; // navsat_llh.longitude
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
    
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.orientation_covariance[8]
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
,velocity_scale_factor.scale_factor,velocity_scale_factor.correction_velocity.linear.x,velocity_scale_factor.correction_velocity.linear.y,velocity_scale_factor.correction_velocity.linear.z,velocity_scale_factor.correction_velocity.angular.x,velocity_scale_factor.correction_velocity.angular.y,velocity_scale_factor.correction_velocity.angular.z,velocity_scale_factor.status.enabled_status,velocity_scale_factor.status.estimate_status\
,distance.distance,distance.status.enabled_status,distance.status.estimate_status\
,heading_1st.heading_angle,heading_1st.status.enabled_status,heading_1st.status.estimate_status\
,heading_interpolate_1st.heading_angle,heading_interpolate_1st.status.enabled_status,heading_interpolate_1st.status.estimate_status\
,heading_2nd.heading_angle,heading_2nd.status.enabled_status,heading_2nd.status.estimate_status\
,heading_interpolate_2nd.heading_angle,heading_interpolate_2nd.status.enabled_status,heading_interpolate_2nd.status.estimate_status\
,heading_3rd.heading_angle,heading_3rd.status.enabled_status,heading_3rd.status.estimate_status\
,heading_interpolate_3rd.heading_angle,heading_interpolate_3rd.status.enabled_status,heading_interpolate_3rd.status.estimate_status\
,yawrate_offset_stop.yawrate_offset,yawrate_offset_stop.status.enabled_status,yawrate_offset_stop.status.estimate_status\
,yawrate_offset_1st.yawrate_offset,yawrate_offset_1st.status.enabled_status,yawrate_offset_1st.status.estimate_status\
,yawrate_offset_2nd.yawrate_offset,yawrate_offset_2nd.status.enabled_status,yawrate_offset_2nd.status.estimate_status\
,slip_angle.coefficient,slip_angle.slip_angle,slip_angle.status.enabled_status,slip_angle.status.estimate_status\
,enu_vel.vector.x,enu_vel.vector.y,enu_vel.vector.z\
,enu_absolute_pos.enu_pos.x,enu_absolute_pos.enu_pos.y,enu_absolute_pos.enu_pos.z,enu_absolute_pos.ecef_base_pos.x,enu_absolute_pos.ecef_base_pos.y,enu_absolute_pos.ecef_base_pos.z,enu_absolute_pos.status.enabled_status,enu_absolute_pos.status.estimate_status\
,enu_absolute_pos_interpolate.enu_pos.x,enu_absolute_pos_interpolate.enu_pos.y,enu_absolute_pos_interpolate.enu_pos.z,enu_absolute_pos_interpolate.ecef_base_pos.x,enu_absolute_pos_interpolate.ecef_base_pos.y,enu_absolute_pos_interpolate.ecef_base_pos.z,enu_absolute_pos_interpolate.status.enabled_status,enu_absolute_pos_interpolate.status.estimate_status\
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
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].scale_factor << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) <<
        eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) <<
        eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) <<
        eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) <<
        eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.angular.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) <<
        eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.angular.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) <<
        eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.angular.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.velocity_scale_factor[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.velocity_scale_factor[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.distance[i].distance << ",";
      *output_log_csv_file << (eagleye_state_forward_.distance[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.distance[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_1st[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_1st[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_2nd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_2nd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_3rd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_3rd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_3rd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_3rd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_3rd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_3rd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.yawrate_offset_stop[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_stop[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_stop[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.yawrate_offset_1st[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.yawrate_offset_2nd[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.slip_angle[i].coefficient << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.slip_angle[i].slip_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.slip_angle[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.slip_angle[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_vel[i].vector.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_vel[i].vector.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_vel[i].vector.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].enu_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].ecef_base_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].ecef_base_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].ecef_base_pos.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].ecef_base_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].ecef_base_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].ecef_base_pos.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.estimate_status ? "1" : "0") << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.angular_velocity_offset_stop.rollrate_offset << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.angular_velocity_offset_stop.pitchrate_offset << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.angular_velocity_offset_stop.yawrate_offset << ",";
      // *output_log_csv_file << (eagleye_state_forward_.angular_velocity_offset_stop.status.enabled_status ? "1" : "0") << ",";
      // *output_log_csv_file << (eagleye_state_forward_.angular_velocity_offset_stop.status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.height[i].height << ",";
      *output_log_csv_file << (eagleye_state_forward_.height[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.height[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.pitching[i].pitching_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.pitching[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.pitching[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.acc_x_offset[i].acc_x_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_offset[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_offset[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.acc_x_scale_factor[i].acc_x_scale_factor << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_scale_factor[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_scale_factor[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.rolling[i].rolling_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.rolling[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.rolling[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << gga_[i].header.stamp.toNSec() << ","; // timestamp
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lat << ","; // gga_llh.latitude
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lon << ","; // gga_llh.longitude
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
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.latitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.longitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // eagleye_pp_llh.altitude     
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
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << eagleye_state_forward_.flag_reliability_buffer[i] << ","; // eagleye_pp_llh.status
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_relative_pos[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_relative_pos[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_relative_pos[i].enu_pos.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_relative_pos[i].status.enabled_status ? "1" : "0");
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

    addOtherInformation(point, "CAN Velocity [km/h] ", kml_generator_->makeDouble2String(velocity_[i].twist.linear.x * 3.6));
    addOtherInformation(point, "IMU Linear Acceleration X [m/s^2] ", kml_generator_->makeDouble2String(imu_[i].linear_acceleration.x));
    addOtherInformation(point, "IMU Linear Acceleration Y [m/s^2] ", kml_generator_->makeDouble2String(imu_[i].linear_acceleration.y));
    addOtherInformation(point, "IMU Linear Acceleration Z [m/s^2] ", kml_generator_->makeDouble2String(imu_[i].linear_acceleration.z));
    addOtherInformation(point, "IMU Angular Velocity X [rad/s] ", kml_generator_->makeDouble2String(imu_[i].angular_velocity.x));
    addOtherInformation(point, "IMU Angular Velocity Y [rad/s] ", kml_generator_->makeDouble2String(imu_[i].angular_velocity.y));
    addOtherInformation(point, "IMU Angular Velocity Z [rad/s] ", kml_generator_->makeDouble2String(imu_[i].angular_velocity.z));
    addOtherInformation(point, "Estimate Velocity [km/h] ", kml_generator_->makeDouble2String(eagleye_state.velocity_scale_factor[i].correction_velocity.linear.x * 3.6));
    addOtherInformation(point, "Velocity Scale Factor", kml_generator_->makeDouble2String(eagleye_state.velocity_scale_factor[i].scale_factor));
    addOtherInformation(point, "Velocity Scale Factor Flag", kml_generator_->makeBool2String(eagleye_state.velocity_scale_factor[i].status.enabled_status));
    addOtherInformation(point, "Estimate Heading 1st [rad] ", kml_generator_->makeDouble2String(eagleye_state.heading_interpolate_1st[i].heading_angle));
    addOtherInformation(point, "Estimate Heading 1st Flag", kml_generator_->makeBool2String(eagleye_state.heading_interpolate_1st[i].status.enabled_status));
    addOtherInformation(point, "Estimate Heading 2nd [rad] ", kml_generator_->makeDouble2String(eagleye_state.heading_interpolate_2nd[i].heading_angle));
    addOtherInformation(point, "Estimate Heading 2nd Flag", kml_generator_->makeBool2String(eagleye_state.heading_interpolate_2nd[i].status.enabled_status));
    addOtherInformation(point, "Estimate Heading 3rd [rad] ", kml_generator_->makeDouble2String(eagleye_state.heading_interpolate_3rd[i].heading_angle));
    addOtherInformation(point, "Estimate Heading 3rd Flag", kml_generator_->makeBool2String(eagleye_state.heading_interpolate_3rd[i].status.enabled_status));
    addOtherInformation(point, "Estimate Yawrate Offset Stop [rad] ", kml_generator_->makeDouble2String(eagleye_state.yawrate_offset_stop[i].yawrate_offset));
    addOtherInformation(point, "Estimate Yawrate Offset Stop Flag", kml_generator_->makeBool2String(eagleye_state.yawrate_offset_stop[i].status.enabled_status));
    addOtherInformation(point, "Estimate Yawrate Offset 1st [rad] ", kml_generator_->makeDouble2String(eagleye_state.yawrate_offset_1st[i].yawrate_offset));
    addOtherInformation(point, "Estimate Yawrate Offset 1st Flag", kml_generator_->makeBool2String(eagleye_state.yawrate_offset_1st[i].status.enabled_status));
    addOtherInformation(point, "Estimate Yawrate Offset 2nd [rad] ", kml_generator_->makeDouble2String(eagleye_state.yawrate_offset_2nd[i].yawrate_offset));
    addOtherInformation(point, "Estimate Yawrate Offset 2nd Flag", kml_generator_->makeBool2String(eagleye_state.yawrate_offset_2nd[i].status.enabled_status));
    addOtherInformation(point, "Estimate Tire Slip Angle [rad] ", kml_generator_->makeDouble2String(eagleye_state.slip_angle[i].slip_angle));
    addOtherInformation(point, "Estimate Tire Slip Angle Flag", kml_generator_->makeBool2String(eagleye_state.slip_angle[i].status.enabled_status));
    addOtherInformation(point, "Estimate Pitch Angle [rad] ", kml_generator_->makeDouble2String(eagleye_state.pitching[i].pitching_angle));
    addOtherInformation(point, "Estimate Pitch Angle Flag", kml_generator_->makeBool2String(eagleye_state.pitching[i].status.enabled_status));
    addOtherInformation(point, "Estimate Height Angle Flag", kml_generator_->makeBool2String(eagleye_state.height[i].status.enabled_status));

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
    point.latitude = gga_[i].lat;
    point.longitude = gga_[i].lon;
    point.altitude =  gga_[i].alt +  gga_[i].undulation;
    if(int(gga_[i].gps_qual) == 4) 
    {
      fix_vector.push_back(point);
    }
    {
      nofix_vector.push_back(point);
    }
  }
  std::pair<std::vector<kml_utils::Point> , std::vector<kml_utils::Point> > gnss_pair = std::make_pair(fix_vector, nofix_vector);

  return gnss_pair;
}

void eagleye_pp::addOtherInformation(kml_utils::Point & point, std::string other_info_name, std::string other_info_value_str)
{
  kml_utils::OtherInfo other_info;
  other_info.name = other_info_name;
  other_info.value_str = other_info_value_str;
  point.other_info_vector.push_back(other_info);
}