#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"



#define g 9.80665


bool is_first_imu = true;
bool is_first_rollrate = true;
bool is_first_rollangle = true;
int count = 1;

void rolling_estimate(const eagleye_msgs::VelocityScaleFactor velocity_scale_factor,const eagleye_msgs::YawrateOffset yawrate_offset_2nd,const eagleye_msgs::YawrateOffset yawrate_offset_stop,const eagleye_msgs::Distance distance,const sensor_msgs::Imu imu,const geometry_msgs::PoseStamped localization_pose,const RollangleParameter rollangle_parameter,RollangleStatus* rollangle_status,eagleye_msgs::Rolling* rolling_angle,eagleye_msgs::AccYOffset* imu_lateral_accoffset)
{

bool accoffset_status = false;
bool rollrad_buffer_status = false;
bool rolling_est_buffer_status = false;
bool data_buffer_status = false;
int i,j,k;
double q0,q1,q2,q3;
double ndt_roll_rad=0.0;
double acc_offset = 0.0;
double rollrad = 0.0;
double rolling_est = 0.0;
double rolling_est_sum = 0.0;
double rolling_est_av = 0.0;
double IMU_time_dt = 0.0;
double rolling_time_offset = 0.0;

// reverse_imu
  if (rollangle_parameter.reverse_imu == true)
  {
    rollangle_status->yawrate = imu.angular_velocity.z;
    rollangle_status->rollrate = imu.angular_velocity.x;
  }
  else if (rollangle_parameter.reverse_imu == false)
  {
    rollangle_status->yawrate = -1 * imu.angular_velocity.z;
    rollangle_status->rollrate = -1* imu.angular_velocity.x;
  }

  //　reverse_imu rollrate
  if (rollangle_parameter.reverse_imu_angular_velocity_x == false)
  {
    rollangle_status->rollrate = imu.angular_velocity.x;
  }
  else if (rollangle_parameter.reverse_imu_angular_velocity_x == true)
  {
    rollangle_status->rollrate = -1* imu.angular_velocity.x;
  }

  //　reverse_imu yACC
  if (rollangle_parameter.reverse_imu_linear_acceleration_y == false)
  {
    rollangle_status->imu_acceleration_y = imu.linear_acceleration.y;
  }
  else if (rollangle_parameter.reverse_imu_linear_acceleration_y == true)
  {
    rollangle_status->imu_acceleration_y = -1* imu.linear_acceleration.y;
  }


// data buffer 
if(rollangle_status->imu_time_buffer.empty() && is_first_imu == true  && velocity_scale_factor.status.enabled_status == true)
  {
    rollangle_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    rollangle_status->yawrate_buffer.push_back(rollangle_status->yawrate);
    rollangle_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    rollangle_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    rollangle_status->acceleration_y_buffer.push_back(rollangle_status->imu_acceleration_y);
    rollangle_status->distance_buffer.push_back(distance.distance);
    is_first_imu = false;
  }
  else if (rollangle_status->imu_time_buffer.size() < rollangle_parameter.data_bufferNUM && velocity_scale_factor.status.enabled_status == true)
  {
    rollangle_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    rollangle_status->yawrate_buffer.push_back(rollangle_status->yawrate);
    rollangle_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    rollangle_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    rollangle_status->acceleration_y_buffer.push_back(rollangle_status->imu_acceleration_y);
    rollangle_status->distance_buffer.push_back(distance.distance);
  }
  else if (velocity_scale_factor.status.enabled_status == true)
  {
    rollangle_status->imu_time_buffer.erase(rollangle_status->imu_time_buffer.begin());
    rollangle_status->yawrate_buffer.erase(rollangle_status->yawrate_buffer.begin());
    rollangle_status->velocity_buffer.erase(rollangle_status->velocity_buffer.begin());
    rollangle_status->yawrate_offset_buffer.erase(rollangle_status->yawrate_offset_buffer.begin());
    rollangle_status->acceleration_y_buffer.erase(rollangle_status->acceleration_y_buffer.begin());
    rollangle_status->distance_buffer.erase(rollangle_status->distance_buffer.begin());
    
    rollangle_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    rollangle_status->yawrate_buffer.push_back(rollangle_status->yawrate);
    rollangle_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    rollangle_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    rollangle_status->acceleration_y_buffer.push_back(rollangle_status->imu_acceleration_y);
    rollangle_status->distance_buffer.push_back(distance.distance);
    data_buffer_status = true;
  }

// imu_lateral_accoffset
  if (data_buffer_status == true )
  {
    for ( i=0; i < rollangle_parameter.data_bufferNUM-1; i++ )
    {
      if(abs(rollangle_status->imu_time_buffer[i]-localization_pose.header.stamp.toSec()) < rollangle_parameter.link_Time_stamp_parameter)
      {
        if (abs(rollangle_status->distance_save - rollangle_status->distance_buffer[i]) >= rollangle_parameter.matching_update_distance )
        {
         accoffset_status = true;
         rollangle_status->distance_save = rollangle_status->distance_buffer[i];
        }
      }


      if (accoffset_status == true)
      {
        if (rollangle_status->velocity_buffer[i] > rollangle_parameter.stop_judgment_velocity_threshold)
        {
         q0 = localization_pose.pose.orientation.w;
         q1 = localization_pose.pose.orientation.x;
         q2 = localization_pose.pose.orientation.y;
         q3 = localization_pose.pose.orientation.z;

         ndt_roll_rad = atan((2*(q0*q1+q2*q3))/((q0*q0)-(q1*q1)-(q2*q2)+(q3*q3)));
         

         acc_offset = -1*(rollangle_status->velocity_buffer[i]*(rollangle_status->yawrate_buffer[i]+rollangle_status->yawrate_offset_buffer[i])-rollangle_status->acceleration_y_buffer[i]-g*sin(ndt_roll_rad));
         rollangle_status->acc_offset_sum = rollangle_status->acc_offset_sum + acc_offset;
         imu_lateral_accoffset->imu_lateral_accoffset = rollangle_status->acc_offset_sum/count; 
         count += 1;
         accoffset_status = false;
         imu_lateral_accoffset->status.enabled_status=true;
         imu_lateral_accoffset->status.estimate_status=true;
        }
        else
        {
          imu_lateral_accoffset->status.estimate_status=false;
        }
      }

    }
    
  }
  else
  {
    imu_lateral_accoffset->status.enabled_status=false;
  }


// rolling_angle_estimation
  if(imu_lateral_accoffset->status.estimate_status == true && velocity_scale_factor.status.enabled_status == true)
  {
    if (velocity_scale_factor.correction_velocity.linear.x > rollangle_parameter.stop_judgment_velocity_threshold)
    {
      rolling_est = asin((velocity_scale_factor.correction_velocity.linear.x*(rollangle_status->yawrate+yawrate_offset_2nd.yawrate_offset)/g)-(rollangle_status->imu_acceleration_y-imu_lateral_accoffset->imu_lateral_accoffset)/g);
    }
    else
    {
      rolling_est = asin((velocity_scale_factor.correction_velocity.linear.x*(rollangle_status->yawrate+yawrate_offset_stop.yawrate_offset)/g)-(rollangle_status->imu_acceleration_y-imu_lateral_accoffset->imu_lateral_accoffset)/g);
    }
  }

/// rollrad /// rolling_time_offset ///
  IMU_time_dt = imu.header.stamp.toSec() - rollangle_status->IMU_time_before ;
  rollangle_status->IMU_time_before =imu.header.stamp.toSec();
  if(velocity_scale_factor.correction_velocity.linear.x < rollangle_parameter.stop_judgment_velocity_threshold)
  {
    rollangle_status->rollrate_offset = rollangle_status->rollrate;
  }

  rollrad = (rollangle_status->rollrate - rollangle_status->rollrate_offset)*IMU_time_dt ;


///rollrad ///rolling_time_offset buffering///
  if(rollangle_status->rollrad_buffer.empty() && is_first_rollrate == true )
  {
    rollangle_status->rollrad_buffer.push_back(rollrad);
    is_first_rollrate = false;
  }
  else if (rollangle_status->rollrad_buffer.size() <rollangle_parameter.rollrad_bufferNUM)
  {
    rollangle_status->rollrad_buffer.push_back(rollrad);
  }
  else 
  {
    rollangle_status->rollrad_buffer.erase(rollangle_status->rollrad_buffer.begin());
   rollangle_status->rollrad_buffer.push_back(rollrad);
    rollrad_buffer_status = true;
  }


// roll_angle_estimation buffering
  if(rollangle_status->rolling_est_buffer.empty() && is_first_rollangle == true &&  velocity_scale_factor.status.enabled_status == true && imu_lateral_accoffset->status.estimate_status == true)
  {
    rollangle_status->rolling_est_buffer.push_back(rolling_est);
    is_first_rollangle = false;
  }
  else if (rollangle_status->rolling_est_buffer.size() < rollangle_parameter.rolling_bufferNUM && velocity_scale_factor.status.enabled_status == true && imu_lateral_accoffset->status.estimate_status == true)
  {
    rollangle_status->rolling_est_buffer.push_back(rolling_est);
  }
  else if (velocity_scale_factor.status.enabled_status == true && imu_lateral_accoffset->status.estimate_status == true)
  {
    rollangle_status->rolling_est_buffer.erase(rollangle_status->rolling_est_buffer.begin());
    rollangle_status->rolling_est_buffer.push_back(rolling_est);
    rolling_est_buffer_status = true;
  }else{
    rolling_est_buffer_status = false;
  }

/// rollrad ///
  if (rollrad_buffer_status == true)
  {
    //rolling_time_offset = 0.0;
    for( j=0; j <rollangle_parameter.rollrad_bufferNUM-1; j++)
    {
      rolling_time_offset += rollangle_status->rollrad_buffer[j];
    }
  }

// Moving average estimation of roll angle
  if (rolling_est_buffer_status == true )
  {
    for( k=0; k <rollangle_parameter.rolling_bufferNUM-1; k++ )
    {
      rolling_est_sum =rolling_est_sum + rollangle_status->rolling_est_buffer[k];
    }
    
  rolling_est_av = rolling_est_sum/rollangle_parameter.rolling_bufferNUM;
  rolling_angle->rolling_angle = rolling_est_av + rolling_time_offset;
  rolling_angle->status.enabled_status=true;
  rolling_angle->status.estimate_status=true;
  }
  else
  {
    rolling_angle->status.estimate_status=false;
  }

}