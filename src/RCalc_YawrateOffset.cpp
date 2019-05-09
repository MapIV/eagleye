/*
 * RCalc_YawrateOffset.cpp
 * YawrateOffset estimate program
 * Author Sekino
 * Ver 2.00 2019/4/11 Integrate 1st, 2nd
 * Ver 1.00 2019/2/7
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/VelocitySF.h"
#include "imu_gnss_localizer/Heading.h"
#include "imu_gnss_localizer/YawrateOffset.h"
#include <math.h>

ros::Publisher pub;

// default parameter
bool reverse_imu = false;

bool flag_YawrateOffsetEstRaw, flag_YawrateOffsetEst, flag_Offset_Start, flag_GaDRaw, flag_Est;
int i = 0;
int I_flag = 0;
int count = 0;
int I_counter = 0;
int GPSTime_Last, GPSTime;
int ESTNUM_Offset = 0;
double IMUTime;
double IMUfrequency = 50;  // IMU Hz
double IMUperiod = 1.0 / IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
double sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;
double ESTNUM_MIN = 1500;
double ESTNUM_MAX;  // defined in main function
double ESTNUM_K = 1.0 / 50;
double TH_VEL_EST = 10 / 3.6;
double Heading = 0.0;
double YawrateOffset_Stop = 0.0;
double YawrateOffset_EstRaw = 0.0;
double YawrateOffset_Est = 0.0;
double YawrateOffset_Last = 0.0;
double Yawrate = 0.0;
double Correction_Velocity = 0.0;

double StartTime = 0.0;
double EndTime = 0.0;
double ProcessingTime = 0.0;

std::size_t length_index;
std::size_t length_pTime;
std::size_t length_index_inv_up;
std::size_t length_index_inv_down;

imu_gnss_localizer::YawrateOffset p_msg;

std::vector<double> pTime;
std::vector<double> pYawrate;
std::vector<double> pHeading;
std::vector<double> pVelocity;
std::vector<bool> pflag_GaDRaw;
std::vector<double> pYawrateOffset_Stop;

void receive_VelocitySF(const imu_gnss_localizer::VelocitySF::ConstPtr& msg)
{
  Correction_Velocity = msg->Correction_Velocity;
}

void receive_YawrateOffsetStop(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffset_Stop = msg->YawrateOffset;
}

void receive_Heading(const imu_gnss_localizer::Heading::ConstPtr& msg)
{
  Heading = msg->Heading_angle;
  flag_GaDRaw = msg->flag_EstRaw;
}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg)
{
  StartTime = ros::Time::now().toSec();

  ++count;
  IMUTime = IMUperiod * count;
  ROSTime = ros::Time::now().toSec();
  Time = ROSTime;  // IMUTime or ROSTime
  // ROS_INFO("Time = %lf" , Time);

  if (ESTNUM_Offset < ESTNUM_MAX)
  {
    ++ESTNUM_Offset;
  }
  else
  {
    ESTNUM_Offset = ESTNUM_MAX;
  }

  if (reverse_imu == false)
  {
    Yawrate = msg->angular_velocity.z;
  }
  else if (reverse_imu == true)
  {
    Yawrate = -1 * msg->angular_velocity.z;
  }

  // data buffer generate
  pTime.push_back(Time);
  pYawrate.push_back(Yawrate);
  pHeading.push_back(Heading);
  pVelocity.push_back(Correction_Velocity);
  pflag_GaDRaw.push_back(flag_GaDRaw);
  pYawrateOffset_Stop.push_back(YawrateOffset_Stop);

  length_pTime = std::distance(pTime.begin(), pTime.end());

  if (length_pTime > ESTNUM_MAX)
  {
    pTime.erase(pTime.begin());
    pYawrate.erase(pYawrate.begin());
    pHeading.erase(pHeading.begin());
    pVelocity.erase(pVelocity.begin());
    pflag_GaDRaw.erase(pflag_GaDRaw.begin());
    pYawrateOffset_Stop.erase(pYawrateOffset_Stop.begin());
  }

  if (I_flag == 0 && pflag_GaDRaw[ESTNUM_Offset - 1] == true)
  {
    I_flag = 1;
  }
  else if (I_flag == 1)
  {
    if (I_counter < ESTNUM_MIN)
    {
      ++I_counter;
    }
    else if (I_counter == ESTNUM_MIN)
    {
      I_flag = 2;
    }
  }

  if (I_flag == 2 && pVelocity[ESTNUM_Offset - 1] > TH_VEL_EST && pflag_GaDRaw[ESTNUM_Offset - 1] == true)
  {
    flag_Est = true;
  }
  else
  {
    flag_Est = false;
  }

  // dynamic array
  std::vector<int> pindex_vel;
  std::vector<int> pindex_GaD;
  std::vector<int> index;

  if (flag_Est == true)
  {
    for (i = 0; i < ESTNUM_Offset; i++)
    {
      if (pVelocity[i] > TH_VEL_EST)
      {
        pindex_vel.push_back(i);
      }
      if (pflag_GaDRaw[i] == true)
      {
        pindex_GaD.push_back(i);
      }
    }

    set_intersection(pindex_vel.begin(), pindex_vel.end(), pindex_GaD.begin(), pindex_GaD.end(),
                     inserter(index, index.end()));

    length_index = std::distance(index.begin(), index.end());

    if (length_index > ESTNUM_Offset * ESTNUM_K)
    {
      std::vector<double> ppHeading(ESTNUM_Offset, 0);

      for (i = 0; i < ESTNUM_Offset; i++)
      {
        if (i > 0)
        {
          ppHeading[i] = ppHeading[i - 1] + pYawrate[i] * (pTime[i] - pTime[i - 1]);
        }
      }

      std::vector<double> baseHeading;
      std::vector<double> pdiff;
      std::vector<double> ptime;
      std::vector<double> index_inv_up;
      std::vector<double> index_inv_down;

      for (i = 0; i < ESTNUM_Offset; i++)
      {
        baseHeading.push_back(pHeading[index[length_index - 1]] - ppHeading[index[length_index - 1]] + ppHeading[i]);
      }

      for (i = 0; i < length_index; i++)
      {
        pdiff.push_back(baseHeading[index[i]] - pHeading[index[i]]);
      }

      for (i = 0; i < length_index; i++)
      {
        if (pdiff[i] > M_PI / 2.0)
        {
          index_inv_up.push_back(index[i]);
        }
        if (pdiff[i] < -M_PI / 2.0)
        {
          index_inv_down.push_back(index[i]);
        }
      }

      length_index_inv_up = std::distance(index_inv_up.begin(), index_inv_up.end());
      length_index_inv_down = std::distance(index_inv_down.begin(), index_inv_down.end());

      if (length_index_inv_up != 0)
      {
        for (i = 0; i < length_index_inv_up; i++)
        {
          pHeading[index_inv_up[i]] = pHeading[index_inv_up[i]] + 2.0 * M_PI;
        }
      }

      if (length_index_inv_down != 0)
      {
        for (i = 0; i < length_index_inv_down; i++)
        {
          pHeading[index_inv_down[i]] = pHeading[index_inv_down[i]] - 2.0 * M_PI;
        }
      }

      length_index = std::distance(index.begin(), index.end());

      baseHeading.clear();
      for (i = 0; i < ESTNUM_Offset; i++)
      {
        baseHeading.push_back(pHeading[index[length_index - 1]] - ppHeading[index[length_index - 1]] + ppHeading[i]);
      }

      pdiff.clear();
      for (i = 0; i < length_index; i++)
      {
        // pdiff.push_back(baseHeading[index[i]] - pHeading[index[i]]);
        pdiff.push_back(pHeading[index[length_index - 1]] - ppHeading[index[length_index - 1]] + ppHeading[index[i]] -
                        pHeading[index[i]]);
      }

      for (i = 0; i < length_index; i++)
      {
        ptime.push_back(pTime[index[i]] - pTime[index[0]]);
      }

      // Least-square
      sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;
      for (i = 0; i < length_index; i++)
      {
        sum_xy += ptime[i] * pdiff[i];
        sum_x += ptime[i];
        sum_y += pdiff[i];
        sum_x2 += pow(ptime[i], 2);
      }

      YawrateOffset_EstRaw = -1 * (length_index * sum_xy - sum_x * sum_y) / (length_index * sum_x2 - pow(sum_x, 2));
      flag_YawrateOffsetEstRaw = true;
    }
    else
    {
      YawrateOffset_EstRaw = 0;
      flag_YawrateOffsetEstRaw = false;
    }
  }
  else
  {
    YawrateOffset_EstRaw = 0;
    flag_YawrateOffsetEstRaw = false;
  }

  if (flag_YawrateOffsetEstRaw == true)
  {
    YawrateOffset_Est = YawrateOffset_EstRaw;
    flag_Offset_Start = true;
  }
  else if (flag_YawrateOffsetEstRaw == false && flag_Offset_Start == true)
  {
    YawrateOffset_Est = YawrateOffset_Last;
  }
  else if (flag_YawrateOffsetEstRaw == false && flag_Offset_Start == false)
  {
    YawrateOffset_Est = YawrateOffset_Stop;
  }

  if (flag_Offset_Start == true)
  {
    flag_YawrateOffsetEst = true;
  }
  else
  {
    flag_YawrateOffsetEst = false;
  }

  p_msg.YawrateOffset = YawrateOffset_Est;
  p_msg.flag_Est = flag_YawrateOffsetEst;
  p_msg.flag_EstRaw = flag_YawrateOffsetEstRaw;
  pub.publish(p_msg);

  Time_Last = Time;
  YawrateOffset_Last = YawrateOffset_Est;

  EndTime = ros::Time::now().toSec();
  ProcessingTime = (EndTime - StartTime);
  if (ProcessingTime > IMUperiod)
  {
    ROS_ERROR("RCalc_YawrateOffset processing time %5.5lf [ms]", ProcessingTime * 1000);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RCalc_YawrateOffset");
  ros::NodeHandle n("~");
  n.getParam("/imu_gnss_localizer/reverse_imu", reverse_imu);

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "/imu_gnss_localizer/YawrateOffset1st";
      subscribe_topic_name = "/imu_gnss_localizer/Heading1st";
      ESTNUM_MAX = 14000;  // Parameters for 1st
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "/imu_gnss_localizer/YawrateOffset2nd";
      subscribe_topic_name = "/imu_gnss_localizer/Heading2nd";
      ESTNUM_MAX = 25000;  // Parameters for 2nd
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
  ros::Subscriber sub3 = n.subscribe(subscribe_topic_name, 1000, receive_Heading);
  ros::Subscriber sub4 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  pub = n.advertise<imu_gnss_localizer::YawrateOffset>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
