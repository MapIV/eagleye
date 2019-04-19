/*
 * RCalc_PositionDis.cpp
 * Vehicle position estimate program
 * Author Sekino
 * Ver 1.00 2019/02/26
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "imu_gnss_localizer/RTKLIB.h"
#include "imu_gnss_localizer/VelocitySF.h"
#include "imu_gnss_localizer/Heading.h"
#include "imu_gnss_localizer/Distance.h"
#include "imu_gnss_localizer/UsrVel_enu.h"
#include "imu_gnss_localizer/PositionDis_raw.h"
#include <boost/circular_buffer.hpp>
#include <math.h>
#include <numeric>

ros::Publisher pub1;

bool flag_GNSS, flag_Start, flag_Est_Raw_Heading;
int length_diff;
int i = 0;
int ESTNUM = 0;
int count = 0;
int GPS_count = 0;
int Distance_BUFNUM = 0;
int index_Dist = 0;
int GPSTime = 0;
int GPSTime_Last = 0;
int index_max = 0;
int UsrVel_index = 0;
double IMUTime;
double IMUfrequency = 50;  // IMU Hz
double IMUperiod = 1.0 / IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
double avg_x, avg_y, avg_z;
double Correction_Velocity = 0.0;
double Distance_BUFNUM_MAX = 100000;  //仮の値
double TH_VEL_EST = 10 / 3.6;
double ESTDIST = 500;
double TH_POSMAX = 3.0;
double TH_CALC_MINNUM = 1.0 / 20;
double TH_EST_GIVEUP_NUM = 1.0 / 100;
double UsrPos_enu_x = 0.0;
double UsrPos_enu_y = 0.0;
double UsrPos_enu_z = 0.0;
double UsrPos_EstRaw_enu_x = 0.0;
double UsrPos_EstRaw_enu_y = 0.0;
double UsrPos_EstRaw_enu_z = 0.0;
double UsrPos_Est_enu_x = 0.0;
double UsrPos_Est_enu_y = 0.0;
double UsrPos_Est_enu_z = 0.0;
double UsrVel_enu_E = 0.0;
double UsrVel_enu_N = 0.0;
double UsrVel_enu_U = 0.0;
double tUsrPos_enu_x = 0.0;
double tUsrPos_enu_y = 0.0;
double tUsrPos_enu_z = 0.0;
double UsrPos_Est_enu_x_Last = 0.0;
double UsrPos_Est_enu_y_Last = 0.0;
double UsrPos_Est_enu_z_Last = 0.0;
double Distance = 0.0;

double StartTime = 0.0;
double EndTime = 0.0;
double ProcessingTime = 0.0;

std::size_t length_index;
std::size_t length_pflag_GNSS;
std::size_t length_pVelocity;
std::size_t length_pUsrPos_enu_x;
std::size_t length_pUsrPos_enu_y;
std::size_t length_pUsrPos_enu_z;
std::size_t length_pUsrVel_enu_E;
std::size_t length_pUsrVel_enu_N;
std::size_t length_pUsrVel_enu_U;
std::size_t length_pTime;
std::size_t length_pindex_vel;
std::size_t length_pindex_Raw;
std::size_t length_pDistance;
std::size_t length_index_Raw;

boost::circular_buffer<double> pDistance(Distance_BUFNUM_MAX);
boost::circular_buffer<bool> pindex_Raw(Distance_BUFNUM_MAX);

std::vector<bool> pflag_GNSS;
std::vector<double> pUsrPos_enu_x;
std::vector<double> pUsrPos_enu_y;
std::vector<double> pUsrPos_enu_z;
std::vector<double> pUsrVel_enu_E;
std::vector<double> pUsrVel_enu_N;
std::vector<double> pUsrVel_enu_U;
std::vector<double> pVelocity;
std::vector<double> pTime;

std::vector<double> basepos_x;
std::vector<double> basepos_y;
std::vector<double> basepos_z;
std::vector<double> pdiff2_x;
std::vector<double> pdiff2_y;
std::vector<double> pdiff2_z;
std::vector<double> basepos2_x;
std::vector<double> basepos2_y;
std::vector<double> basepos2_z;
std::vector<double> pdiff_x;
std::vector<double> pdiff_y;
std::vector<double> pdiff_z;
std::vector<double> pdiff;

std::vector<double>::iterator max;

imu_gnss_localizer::PositionDis_raw p1_msg;

void receive_VelocitySF(const imu_gnss_localizer::VelocitySF::ConstPtr& msg)
{
  Correction_Velocity = msg->Correction_Velocity;
}

void receive_Distance(const imu_gnss_localizer::Distance::ConstPtr& msg)
{
  Distance = msg->Distance;
}

void receive_Heading3rd(const imu_gnss_localizer::Heading::ConstPtr& msg)
{
  flag_Est_Raw_Heading = msg->flag_EstRaw;
}

void receive_UsrPos_enu(const imu_gnss_localizer::RTKLIB::ConstPtr& msg)
{
  GPSTime = msg->GPSTime;
  UsrPos_enu_x = msg->enu_x;  // unit [m]
  UsrPos_enu_y = msg->enu_y;  // unit [m]
  UsrPos_enu_z = msg->enu_z;  // unit [m]
}

void receive_UsrVel_enu(const imu_gnss_localizer::UsrVel_enu::ConstPtr& msg)
{
  StartTime = ros::Time::now().toSec();

  ++count;

  IMUTime = IMUperiod * count;
  ROSTime = ros::Time::now().toSec();
  Time = ROSTime;  // IMUTime or ROSTime
  // ROS_INFO("Time = %lf" , Time);

  if (Distance_BUFNUM < Distance_BUFNUM_MAX)
  {
    ++Distance_BUFNUM;
  }
  else
  {
    Distance_BUFNUM = Distance_BUFNUM_MAX;
  }

  // GNSS receive flag
  if (GPSTime_Last == GPSTime)
  {
    flag_GNSS = false;
    UsrPos_enu_x = 0.0;
    UsrPos_enu_y = 0.0;
    UsrPos_enu_z = 0.0;
  }
  else
  {
    flag_GNSS = true;
    UsrPos_enu_x = UsrPos_enu_x;
    UsrPos_enu_y = UsrPos_enu_y;
    UsrPos_enu_z = UsrPos_enu_z;
    ++GPS_count;
  }

  UsrVel_enu_E = msg->VelE;
  UsrVel_enu_N = msg->VelN;
  UsrVel_enu_U = msg->VelU;
  UsrVel_index = msg->index;

  // data buffer generate
  pDistance.push_back(Distance);
  pflag_GNSS.push_back(flag_GNSS);
  pindex_Raw.push_back(flag_Est_Raw_Heading);
  pUsrPos_enu_x.push_back(UsrPos_enu_x);
  pUsrPos_enu_y.push_back(UsrPos_enu_y);
  pUsrPos_enu_z.push_back(UsrPos_enu_z);
  pUsrVel_enu_E.push_back(UsrVel_enu_E);
  pUsrVel_enu_N.push_back(UsrVel_enu_N);
  pUsrVel_enu_U.push_back(UsrVel_enu_U);
  pVelocity.push_back(Correction_Velocity);
  pTime.push_back(Time);

  std::vector<int> pindex_Est;
  std::vector<int> index_Raw;
  std::vector<int> pindex_GNSS;
  std::vector<int> pindex_vel;
  std::vector<int> index;

  length_pindex_Raw = std::distance(pindex_Raw.begin(), pindex_Raw.end());
  length_pDistance = std::distance(pDistance.begin(), pDistance.end());

  for (i = 0; i < length_pindex_Raw; i++)
  {
    if (pindex_Raw[i] == true)
    {
      index_Raw.push_back(i);
    }
  }

  length_index_Raw = std::distance(index_Raw.begin(), index_Raw.end());

  for (i = 0; i < Distance_BUFNUM; i++)
  {
    if (pDistance[i] > pDistance[Distance_BUFNUM - 1] - ESTDIST)
    {
      index_Dist = i;
      break;
    }
  }

  ESTNUM = Distance_BUFNUM - index_Dist;

  length_pTime = std::distance(pTime.begin(), pTime.end());
  length_diff = length_pTime - ESTNUM;

  if (length_diff > 0)
  {
    pTime.erase(pTime.begin(), pTime.begin() + abs(length_diff));
    pflag_GNSS.erase(pflag_GNSS.begin(), pflag_GNSS.begin() + abs(length_diff));
    pVelocity.erase(pVelocity.begin(), pVelocity.begin() + abs(length_diff));
    pUsrPos_enu_x.erase(pUsrPos_enu_x.begin(), pUsrPos_enu_x.begin() + abs(length_diff));
    pUsrPos_enu_y.erase(pUsrPos_enu_y.begin(), pUsrPos_enu_y.begin() + abs(length_diff));
    pUsrPos_enu_z.erase(pUsrPos_enu_z.begin(), pUsrPos_enu_z.begin() + abs(length_diff));
    pUsrVel_enu_E.erase(pUsrVel_enu_E.begin(), pUsrVel_enu_E.begin() + abs(length_diff));
    pUsrVel_enu_N.erase(pUsrVel_enu_N.begin(), pUsrVel_enu_N.begin() + abs(length_diff));
    pUsrVel_enu_U.erase(pUsrVel_enu_U.begin(), pUsrVel_enu_U.begin() + abs(length_diff));
  }
  else if (length_diff < 0)
  {
    for (i = 0; i < abs(length_diff); i++)
    {
      pTime.insert(pTime.begin(), 0);
      pflag_GNSS.insert(pflag_GNSS.begin(), false);
      pVelocity.insert(pVelocity.begin(), 0);
      pUsrPos_enu_x.insert(pUsrPos_enu_x.begin(), 0);
      pUsrPos_enu_y.insert(pUsrPos_enu_y.begin(), 0);
      pUsrPos_enu_z.insert(pUsrPos_enu_z.begin(), 0);
      pUsrVel_enu_E.insert(pUsrVel_enu_E.begin(), 0);
      pUsrVel_enu_N.insert(pUsrVel_enu_N.begin(), 0);
      pUsrVel_enu_U.insert(pUsrVel_enu_U.begin(), 0);
    }
  }

  for (i = 0; i < ESTNUM; i++)
  {
    if (pflag_GNSS[i] == true)
    {
      pindex_GNSS.push_back(i);
    }
    if (pVelocity[i] > TH_VEL_EST)
    {
      pindex_vel.push_back(i);
    }
  }

  length_pindex_vel = std::distance(pindex_vel.begin(), pindex_vel.end());

  set_intersection(pindex_GNSS.begin(), pindex_GNSS.end(), pindex_vel.begin(), pindex_vel.end(),
                   inserter(index, index.end()));

  length_index = std::distance(index.begin(), index.end());

  if (length_index_Raw > 0)
  {
    if (pDistance[Distance_BUFNUM - 1] > ESTDIST && flag_GNSS == true && Correction_Velocity > TH_VEL_EST &&
        index_Dist > index_Raw[0] && count > 1 && ESTNUM != Distance_BUFNUM_MAX && 0 == fmod(GPS_count, 10.0))
    {
      if (length_index > length_pindex_vel * TH_CALC_MINNUM)
      {
        std::vector<double> tTrajectory_x(ESTNUM, 0);
        std::vector<double> tTrajectory_y(ESTNUM, 0);
        std::vector<double> tTrajectory_z(ESTNUM, 0);

        for (i = 0; i < ESTNUM; i++)
        {
          if (i > 0)
          {
            tTrajectory_x[i] = tTrajectory_x[i - 1] + pUsrVel_enu_E[i] * (pTime[i] - pTime[i - 1]);
            tTrajectory_y[i] = tTrajectory_y[i - 1] + pUsrVel_enu_N[i] * (pTime[i] - pTime[i - 1]);
            tTrajectory_z[i] = tTrajectory_z[i - 1] + pUsrVel_enu_U[i] * (pTime[i] - pTime[i - 1]);
          }
        }

        while (1)
        {
          length_index = std::distance(index.begin(), index.end());

          basepos_x.clear();
          basepos_y.clear();
          basepos_z.clear();

          for (i = 0; i < ESTNUM; i++)
          {
            basepos_x.push_back(pUsrPos_enu_x[index[length_index - 1]] - tTrajectory_x[index[length_index - 1]] +
                                tTrajectory_x[i]);
            basepos_y.push_back(pUsrPos_enu_y[index[length_index - 1]] - tTrajectory_y[index[length_index - 1]] +
                                tTrajectory_y[i]);
            basepos_z.push_back(pUsrPos_enu_z[index[length_index - 1]] - tTrajectory_z[index[length_index - 1]] +
                                tTrajectory_z[i]);
          }

          pdiff2_x.clear();
          pdiff2_y.clear();
          pdiff2_z.clear();

          for (i = 0; i < length_index; i++)
          {
            pdiff2_x.push_back(basepos_x[index[i]] - pUsrPos_enu_x[index[i]]);
            pdiff2_y.push_back(basepos_y[index[i]] - pUsrPos_enu_y[index[i]]);
            pdiff2_z.push_back(basepos_z[index[i]] - pUsrPos_enu_z[index[i]]);
          }

          avg_x = std::accumulate(pdiff2_x.begin(), pdiff2_x.end(), 0.0) / length_index;
          avg_y = std::accumulate(pdiff2_y.begin(), pdiff2_y.end(), 0.0) / length_index;
          avg_z = std::accumulate(pdiff2_z.begin(), pdiff2_z.end(), 0.0) / length_index;

          tUsrPos_enu_x = pUsrPos_enu_x[index[length_index - 1]] - avg_x;
          tUsrPos_enu_y = pUsrPos_enu_y[index[length_index - 1]] - avg_y;
          tUsrPos_enu_z = pUsrPos_enu_z[index[length_index - 1]] - avg_z;

          basepos2_x.clear();
          basepos2_y.clear();
          basepos2_z.clear();

          for (i = 0; i < ESTNUM; i++)
          {
            basepos2_x.push_back(tUsrPos_enu_x - tTrajectory_x[index[length_index - 1]] + tTrajectory_x[i]);
            basepos2_y.push_back(tUsrPos_enu_y - tTrajectory_y[index[length_index - 1]] + tTrajectory_y[i]);
            basepos2_z.push_back(tUsrPos_enu_z - tTrajectory_z[index[length_index - 1]] + tTrajectory_z[i]);
          }

          pdiff_x.clear();
          pdiff_y.clear();
          pdiff_z.clear();

          for (i = 0; i < length_index; i++)
          {
            pdiff_x.push_back(basepos2_x[index[i]] - pUsrPos_enu_x[index[i]]);
            pdiff_y.push_back(basepos2_y[index[i]] - pUsrPos_enu_y[index[i]]);
            pdiff_z.push_back(basepos2_z[index[i]] - pUsrPos_enu_z[index[i]]);
          }

          pdiff.clear();
          for (i = 0; i < length_index; i++)
          {
            // pdiff.push_back(sqrt((pdiff_x[i] * pdiff_x[i]) + (pdiff_y[i] * pdiff_y[i]) + (pdiff_z[i] * pdiff_z[i])));
            pdiff.push_back(sqrt((pdiff_x[i] * pdiff_x[i]) + (pdiff_y[i] * pdiff_y[i])));
          }

          max = std::max_element(pdiff.begin(), pdiff.end());
          index_max = std::distance(pdiff.begin(), max);

          if (pdiff[index_max] > TH_POSMAX)
          {
            index.erase(index.begin() + index_max);
          }
          else
          {
            break;
          }

          length_index = std::distance(index.begin(), index.end());
          length_pindex_vel = std::distance(pindex_vel.begin(), pindex_vel.end());

          if (length_index < length_pindex_vel * TH_EST_GIVEUP_NUM)
          {
            break;
          }
        }

        length_index = std::distance(index.begin(), index.end());
        length_pindex_vel = std::distance(pindex_vel.begin(), pindex_vel.end());

        if (length_index >= length_pindex_vel * TH_EST_GIVEUP_NUM)
        {
          if (index[length_index - 1] == ESTNUM)
          {
            UsrPos_EstRaw_enu_x = tUsrPos_enu_x;
            UsrPos_EstRaw_enu_y = tUsrPos_enu_y;
            UsrPos_EstRaw_enu_z = tUsrPos_enu_z;
          }
          else
          {
            UsrPos_EstRaw_enu_x = tUsrPos_enu_x + (tTrajectory_x[ESTNUM - 1] - tTrajectory_x[index[length_index - 1]]);
            UsrPos_EstRaw_enu_y = tUsrPos_enu_y + (tTrajectory_y[ESTNUM - 1] - tTrajectory_y[index[length_index - 1]]);
            UsrPos_EstRaw_enu_z = tUsrPos_enu_z + (tTrajectory_z[ESTNUM - 1] - tTrajectory_z[index[length_index - 1]]);
          }

          p1_msg.enu_x = UsrPos_EstRaw_enu_x;
          p1_msg.enu_y = UsrPos_EstRaw_enu_y;
          p1_msg.enu_z = UsrPos_EstRaw_enu_z;
          p1_msg.index = UsrVel_index;
          pub1.publish(p1_msg);
        }
      }
    }
  }

  GPSTime_Last = GPSTime;
  Time_Last = Time;

  EndTime = ros::Time::now().toSec();
  ProcessingTime = (EndTime - StartTime);
  if (ProcessingTime > IMUperiod)
  {
    ROS_WARN("RCalc_PositionDis processing time %lf [ms]", ProcessingTime * 1000);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RCalc_PositionDis");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/UsrVel_enu", 1000, receive_UsrVel_enu);
  ros::Subscriber sub2 = n.subscribe("/RTKLIB", 1000, receive_UsrPos_enu);
  ros::Subscriber sub3 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub4 = n.subscribe("/imu_gnss_localizer/Distance", 1000, receive_Distance);
  ros::Subscriber sub5 = n.subscribe("/imu_gnss_localizer/Heading3rd", 1000, receive_Heading3rd);
  pub1 = n.advertise<imu_gnss_localizer::PositionDis_raw>("/imu_gnss_localizer/PositionDis", 1000);
  ros::spin();

  return 0;
}
