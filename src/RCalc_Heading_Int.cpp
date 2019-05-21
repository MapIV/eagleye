 /*
  * RCalc_Heading_Int.cpp
  * Heading estimate program
  * Author Sekino
  * Ver 1.00 2019/5/10 Supports extrapolation processing
  */

 #include "ros/ros.h"
 #include "geometry_msgs/Twist.h"
 #include "sensor_msgs/Imu.h"
 #include "imu_gnss_localizer/RTKLIB.h"
 #include "imu_gnss_localizer/Heading.h"
 #include "imu_gnss_localizer/Heading_raw.h"
 #include "imu_gnss_localizer/VelocitySF.h"
 #include "imu_gnss_localizer/YawrateOffset.h"
 #include <boost/circular_buffer.hpp>
 #include <math.h>
 #include <numeric>

 ros::Publisher pub;

 // default parameter
 bool reverse_imu = false;

 imu_gnss_localizer::Heading p_msg;

 bool flag_Heading_Est, flag_Heading_Start, flag_Est, flag_EstRaw;
 int i = 0;
 int count = 0;
 int Est_count = 0;
 int Est_index = 0;
 int BUFNUM = 0;
 double TH_VEL_STOP = 0.01;
 double Heading_time_stamp_Last = 0;
 double Heading_time_stamp = 0.0;
 double IMUTime;
 double IMUfrequency = 50;  // IMU Hz
 double IMUperiod = 1 / IMUfrequency;
 double ROSTime = 0.0;
 double Time = 0.0;
 double Time_Last = 0.0;
 double BUFNUM_MAX = 100;
 double YawrateOffset_Stop = 0.0;
 double YawrateOffset = 0.0;
 double Yawrate = 0.0;
 double Heading_EstRaw = 0.0;
 double Heading_Est = 0.0;
 double Heading = 0.0;
 double diff_Heading_Est = 0.0;
 double Heading_Last = 0.0;
 double Correction_Velocity = 0.0;

 ros::Time IMU_time_stamp;

 boost::circular_buffer<double> pHeading_Est(BUFNUM_MAX);
 boost::circular_buffer<double> pIMU_time_stamp(BUFNUM_MAX);

void receive_VelocitySF(const imu_gnss_localizer::VelocitySF::ConstPtr& msg)
{
  Correction_Velocity = msg->Correction_Velocity;
}

void receive_YawrateOffsetStop(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffset_Stop = msg->YawrateOffset;
}

void receive_YawrateOffset(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffset = msg->YawrateOffset;  // 1st is YawrateOffset_Stop here too
}

void receive_Heading(const imu_gnss_localizer::Heading_raw::ConstPtr& msg)
{
  Heading_EstRaw = msg->Heading_angle;
  Heading_time_stamp = msg->time_stamp;
}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg)
{
  ++count;
  IMUTime = IMUperiod * count;
  ROSTime = ros::Time::now().toSec();
  Time = ROSTime;  // IMUTime or ROSTime
  // ROS_INFO("Time = %lf" , Time);

  IMU_time_stamp = msg->header.stamp;

  if (reverse_imu == false)
  {
    Yawrate = msg->angular_velocity.z;
  }
  else if (reverse_imu == true)
  {
    Yawrate = -1 * msg->angular_velocity.z;
  }

  if (Correction_Velocity > TH_VEL_STOP)
  {
    Yawrate = Yawrate + YawrateOffset;
  }
  else
  {
    Yawrate = Yawrate + YawrateOffset_Stop;
  }

  if (BUFNUM < BUFNUM_MAX)
  {
    ++BUFNUM;
  }
  else
  {
    BUFNUM = BUFNUM_MAX;
  }

  if (Heading_time_stamp_Last == Heading_time_stamp)
  {
    flag_Heading_Est = false;
  }
  else
  {
    flag_Heading_Est = true;
    flag_Heading_Start = true;
    ++Est_count;
  }

  Heading_Est = Heading_Last + Yawrate * (Time - Time_Last);

  pHeading_Est.push_back(Heading_Est);
  pIMU_time_stamp.push_back(IMU_time_stamp.toSec());

  if (flag_Heading_Start == true)
  {
    if (flag_Heading_Est == true)
    {
      for (Est_index = BUFNUM; Est_index >= 0; Est_index--)
      {
        if (pIMU_time_stamp[Est_index - 1] == Heading_time_stamp)
        {
          break;
        }
      }
    }

    if (flag_Heading_Est == true && Est_index > 0 && BUFNUM >= Est_index && Est_count > 1)
    {
      diff_Heading_Est = (pHeading_Est[Est_index - 1] - Heading_EstRaw);
      for (i = Est_index; i <= BUFNUM; i++)
      {
        pHeading_Est[i - 1] = pHeading_Est[i - 1] - diff_Heading_Est;
      }
      Heading_Est = pHeading_Est[BUFNUM - 1];

      flag_Est = true;
      flag_EstRaw = true;
    }
    else if (Est_count == 1)
    {
      Heading_Est = Heading_EstRaw;
      flag_Est = true;
      flag_EstRaw = true;
    }
    else if (count > 1)
    {
      flag_EstRaw = false;
    }
  }

  // Angle reversal processing (-3.14~3.14)
  if (Heading_Est > M_PI)
  {
    Heading_Est = Heading_Est - 2.0 * M_PI;
  }
  else if (Heading_Est < -M_PI)
  {
    Heading_Est = Heading_Est + 2.0 * M_PI;
  }

  if (flag_Heading_Start == true)
  {
    Heading = Heading_Est;
  }
  else
  {
    Heading = 0.0;
    flag_Est = false;
    flag_EstRaw = false;
  }

  p_msg.Heading_angle = Heading;
  p_msg.flag_Est = flag_Est;
  p_msg.flag_EstRaw = flag_EstRaw;
  pub.publish(p_msg);

  Time_Last = Time;
  Heading_Last = Heading;
  Heading_time_stamp_Last = Heading_time_stamp;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RCalc_Heading_Int");
  ros::NodeHandle n("~");
  n.getParam("/imu_gnss_localizer/reverse_imu", reverse_imu);

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name_1 = "/subscribe_topic_name/invalid_1";
  std::string subscribe_topic_name_2 = "/subscribe_topic_name/invalid_2";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "/imu_gnss_localizer/Heading1st_Int";
      subscribe_topic_name_1 = "/imu_gnss_localizer/YawrateOffsetStop";
      subscribe_topic_name_2 = "/imu_gnss_localizer/Heading1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "/imu_gnss_localizer/Heading2nd_Int";
      subscribe_topic_name_1 = "/imu_gnss_localizer/YawrateOffset1st";
      subscribe_topic_name_2 = "/imu_gnss_localizer/Heading2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "/imu_gnss_localizer/Heading3rd_Int";
      subscribe_topic_name_1 = "/imu_gnss_localizer/YawrateOffset2nd";
      subscribe_topic_name_2 = "/imu_gnss_localizer/Heading3rd";
    }
    else
    {
      ROS_ERROR("Invalid argument");
      ros::shutdown();
    }
  }
  else
  {
    ROS_ERROR("No arguments");
    ros::shutdown();
  }

  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/YawrateOffsetStop", 1000, receive_YawrateOffsetStop);
  ros::Subscriber sub3 = n.subscribe(subscribe_topic_name_1, 1000, receive_YawrateOffset);
  ros::Subscriber sub4 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  ros::Subscriber sub5 = n.subscribe(subscribe_topic_name_2, 1000, receive_Heading);
  pub = n.advertise<imu_gnss_localizer::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
