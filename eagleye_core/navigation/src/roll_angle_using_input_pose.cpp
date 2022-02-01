#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

#define g 9.80665

bool is_first_imu = true;
bool is_first_rollrate = true;
bool is_first_rollangle = true;
int acc_offset_data_count = 0;

void rolling_estimate_using_input_pose(const eagleye_msgs::VelocityScaleFactor velocity_scale_factor,const eagleye_msgs::YawrateOffset yawrate_offset_2nd,const eagleye_msgs::YawrateOffset yawrate_offset_stop,const eagleye_msgs::Distance distance,const sensor_msgs::Imu imu,const geometry_msgs::PoseStamped localization_pose,const RollangleParameterUsingInputPose rollangle_parameter_using_input_pose,RollangleStatusUsingInputPose* roll_angle_status,eagleye_msgs::Rolling* rolling_angle,eagleye_msgs::AccYOffset* acc_y_offset)
{

bool acc_offset_status = false;
bool rollrad_buffer_status = false;
bool rolling_estimated_buffer_status = false;
bool data_buffer_status = false;
int i,j,k;
double qw,qx,qy,qz;
double input_pose_roll=0.0;
double acc_offset = 0.0;
double correction_roll_rate = 0.0;
double rolling_estimated = 0.0;
double rolling_estimated_sum = 0.0;
double rolling_estimated_average = 0.0;
double diff_imu_time = 0.0;
double rolling_offset_buffer = 0.0;
double rolling_offset_buffer_num = rollangle_parameter_using_input_pose.rolling_buffer_num / 2;

/// reverse_imu ///
  if (!rollangle_parameter_using_input_pose.reverse_imu)
  {
    roll_angle_status->yawrate = imu.angular_velocity.z;
  }
  else if (rollangle_parameter_using_input_pose.reverse_imu)
  {
    roll_angle_status->yawrate = -1 * imu.angular_velocity.z;
  }

  /// reverse_imu rollrate ///
  if (!rollangle_parameter_using_input_pose.reverse_imu_angular_velocity_x)
  {
    roll_angle_status->rollrate = imu.angular_velocity.x;
  }
  else if (rollangle_parameter_using_input_pose.reverse_imu_angular_velocity_x)
  {
    roll_angle_status->rollrate = -1* imu.angular_velocity.x;
  }

  /// reverse_imu y ACC ///
  if (!rollangle_parameter_using_input_pose.reverse_imu_linear_acceleration_y)
  {
    roll_angle_status->imu_acceleration_y = imu.linear_acceleration.y;
  }
  else if (rollangle_parameter_using_input_pose.reverse_imu_linear_acceleration_y)
  {
    roll_angle_status->imu_acceleration_y = -1* imu.linear_acceleration.y;
  }


// data buffer 
// if(roll_angle_status->imu_time_buffer.empty() && is_first_imu == true  && velocity_scale_factor.status.enabled_status == true)
if(roll_angle_status->imu_time_buffer.empty() && is_first_imu  && velocity_scale_factor.status.enabled_status)
  {
    roll_angle_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    roll_angle_status->yawrate_buffer.push_back(roll_angle_status->yawrate);
    roll_angle_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    roll_angle_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    roll_angle_status->acceleration_y_buffer.push_back(roll_angle_status->imu_acceleration_y);
    roll_angle_status->distance_buffer.push_back(distance.distance);
    is_first_imu = false;
  }
  else if (roll_angle_status->imu_time_buffer.size() < rollangle_parameter_using_input_pose.imu_buffer_num && velocity_scale_factor.status.enabled_status)
  {
    roll_angle_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    roll_angle_status->yawrate_buffer.push_back(roll_angle_status->yawrate);
    roll_angle_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    roll_angle_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    roll_angle_status->acceleration_y_buffer.push_back(roll_angle_status->imu_acceleration_y);
    roll_angle_status->distance_buffer.push_back(distance.distance);
  }
  else if (velocity_scale_factor.status.enabled_status)
  {
    roll_angle_status->imu_time_buffer.erase(roll_angle_status->imu_time_buffer.begin());
    roll_angle_status->yawrate_buffer.erase(roll_angle_status->yawrate_buffer.begin());
    roll_angle_status->velocity_buffer.erase(roll_angle_status->velocity_buffer.begin());
    roll_angle_status->yawrate_offset_buffer.erase(roll_angle_status->yawrate_offset_buffer.begin());
    roll_angle_status->acceleration_y_buffer.erase(roll_angle_status->acceleration_y_buffer.begin());
    roll_angle_status->distance_buffer.erase(roll_angle_status->distance_buffer.begin());
    
    roll_angle_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    roll_angle_status->yawrate_buffer.push_back(roll_angle_status->yawrate);
    roll_angle_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    roll_angle_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    roll_angle_status->acceleration_y_buffer.push_back(roll_angle_status->imu_acceleration_y);
    roll_angle_status->distance_buffer.push_back(distance.distance);
    data_buffer_status = true;
  }

/// acc_y_offset ///
  if (data_buffer_status)
  {
    for ( i=0; i < rollangle_parameter_using_input_pose.imu_buffer_num-1; i++ )
    {
      if(abs(roll_angle_status->imu_time_buffer[i]-localization_pose.header.stamp.toSec()) < rollangle_parameter_using_input_pose.link_Time_stamp_parameter)
      {
        if (abs(roll_angle_status->distance_save - roll_angle_status->distance_buffer[i]) >= rollangle_parameter_using_input_pose.matching_update_distance )
        {
         acc_offset_status = true;
         roll_angle_status->distance_save = roll_angle_status->distance_buffer[i];
        }
      }

      if (acc_offset_status)
      {
        if (roll_angle_status->velocity_buffer[i] > rollangle_parameter_using_input_pose.stop_judgment_velocity_threshold)
        {
         qw = localization_pose.pose.orientation.w;
         qx = localization_pose.pose.orientation.x;
         qy = localization_pose.pose.orientation.y;
         qz = localization_pose.pose.orientation.z;

         input_pose_roll = atan((2*(qw*qx+qy*qz))/((qw*qw)-(qx*qx)-(qy*qy)+(qz*qz)));      

         acc_offset = -1*(roll_angle_status->velocity_buffer[i]*(roll_angle_status->yawrate_buffer[i]+roll_angle_status->yawrate_offset_buffer[i])-roll_angle_status->acceleration_y_buffer[i]-g*sin(input_pose_roll));
         roll_angle_status->acc_offset_sum = roll_angle_status->acc_offset_sum + acc_offset;
         acc_offset_data_count += 1;
         acc_y_offset->acc_y_offset = roll_angle_status->acc_offset_sum/acc_offset_data_count;
         acc_offset_status = false;
         acc_y_offset->status.enabled_status=true;
         acc_y_offset->status.estimate_status=true;
         break;
        }
      }
      else
      {
        acc_y_offset->status.estimate_status=false;
      }

    }
    
  }
  else
  {
    acc_y_offset->status.enabled_status=false;
  }


/// estimated rolling angle ///
  if(acc_y_offset->status.enabled_status && velocity_scale_factor.status.enabled_status)
  {
    if (velocity_scale_factor.correction_velocity.linear.x > rollangle_parameter_using_input_pose.stop_judgment_velocity_threshold)
    {
      rolling_estimated = asin((velocity_scale_factor.correction_velocity.linear.x*(roll_angle_status->yawrate+yawrate_offset_2nd.yawrate_offset)/g)-(roll_angle_status->imu_acceleration_y-acc_y_offset->acc_y_offset)/g);
    }
    else
    {
      rolling_estimated = asin((velocity_scale_factor.correction_velocity.linear.x*(roll_angle_status->yawrate+yawrate_offset_stop.yawrate_offset)/g)-(roll_angle_status->imu_acceleration_y-acc_y_offset->acc_y_offset)/g);
    }
  }

/// correction rolling angle offset///
  diff_imu_time = imu.header.stamp.toSec() - roll_angle_status->imu_time_before ;
  roll_angle_status->imu_time_before =imu.header.stamp.toSec();
  if(velocity_scale_factor.correction_velocity.linear.x < rollangle_parameter_using_input_pose.stop_judgment_velocity_threshold)
  {
    roll_angle_status->rollrate_offset = roll_angle_status->rollrate;
  }

  correction_roll_rate = (roll_angle_status->rollrate - roll_angle_status->rollrate_offset)*diff_imu_time;


/// buffering estimated rolling angle offset ///
  if(roll_angle_status->correction_roll_rate_buffer.empty() && is_first_rollrate )
  {
    roll_angle_status->correction_roll_rate_buffer.push_back(correction_roll_rate);
    is_first_rollrate = false;
  }
  else if (roll_angle_status->correction_roll_rate_buffer.size() <rolling_offset_buffer_num)
  {
    roll_angle_status->correction_roll_rate_buffer.push_back(correction_roll_rate);
  }
  else 
  {
    roll_angle_status->correction_roll_rate_buffer.erase(roll_angle_status->correction_roll_rate_buffer.begin());
    roll_angle_status->correction_roll_rate_buffer.push_back(correction_roll_rate);
    rollrad_buffer_status = true;
  }


/// buffering estimated roll angle ///
  if(roll_angle_status->rolling_estimated_buffer.empty() && is_first_rollangle &&  velocity_scale_factor.status.enabled_status && acc_y_offset->status.enabled_status)
  {
    roll_angle_status->rolling_estimated_buffer.push_back(rolling_estimated);
    is_first_rollangle = false;
  }
  else if (roll_angle_status->rolling_estimated_buffer.size() < rollangle_parameter_using_input_pose.rolling_buffer_num && velocity_scale_factor.status.enabled_status && acc_y_offset->status.enabled_status)
  {
    roll_angle_status->rolling_estimated_buffer.push_back(rolling_estimated);
  }
  else if (velocity_scale_factor.status.enabled_status && acc_y_offset->status.enabled_status)
  {
    roll_angle_status->rolling_estimated_buffer.erase(roll_angle_status->rolling_estimated_buffer.begin());
    roll_angle_status->rolling_estimated_buffer.push_back(rolling_estimated);
    rolling_estimated_buffer_status = true;
  }
  else
  {
    rolling_estimated_buffer_status = false;
  }

/// buffering rolling offset ///
  if (rollrad_buffer_status)
  {
    for( j=0; j <rolling_offset_buffer_num-1; j++)
    {
      rolling_offset_buffer += roll_angle_status->correction_roll_rate_buffer[j];
    }
  }

/// Moving average estimation of roll angle ///
  if (rolling_estimated_buffer_status )
  {
    for( k=0; k <rollangle_parameter_using_input_pose.rolling_buffer_num-1; k++ )
    {
      rolling_estimated_sum =rolling_estimated_sum + roll_angle_status->rolling_estimated_buffer[k];
    }
    
  rolling_estimated_average = rolling_estimated_sum/rollangle_parameter_using_input_pose.rolling_buffer_num;
  rolling_angle->rolling_angle = rolling_estimated_average + rolling_offset_buffer;
  rolling_angle->status.enabled_status=true;
  rolling_angle->status.estimate_status=true;
  }
  else
  {
    rolling_angle->status.estimate_status=false;
  }

}