/*
 * RCalc_PositionDis.cpp
 * Vehicle position estimate program
 * Author Sekino
 * Ver 2.00 2019/05/24 Changed specifications to buffer based on time and distance based
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

bool flag_DATA, flag_GNSS, flag_Est_Raw_Heading;
int i = 0;
double ESTDIST = 300;
double SPLITDIST = 0.1;
int ESTNUM = 0;
int ESTNUM_MAX = ESTDIST / SPLITDIST;
int count = 0;
int index_Raw_count = 0;
int GPSTime = 0;
int GPSTime_Last = 0;
int index_max = 0; //pattern1
int index_max_x = 0; //pattern2
int index_max_y = 0; //pattern2
double UsrVel_time_stamp = 0.0;
double IMUTime;
double IMUfrequency = 50;  // IMU Hz
double IMUperiod = 1.0 / IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
double avg_x, avg_y, avg_z;
double Correction_Velocity = 0.0;
double TH_VEL_EST = 10 / 3.6;
double TH_POSMAX = 5.0;
double TH_CALC_MINNUM = 1.0 / 20;
double TH_EST_GIVEUP_NUM = 1.0 / 100;
double UsrPos_enu_x = 0.0;
double UsrPos_enu_y = 0.0;
double UsrPos_enu_z = 0.0;
double UsrPos_EstRaw_enu_x = 0.0;
double UsrPos_EstRaw_enu_y = 0.0;
double UsrPos_EstRaw_enu_z = 0.0;
double UsrVel_enu_E = 0.0;
double UsrVel_enu_N = 0.0;
double UsrVel_enu_U = 0.0;
double tUsrPos_enu_x = 0.0;
double tUsrPos_enu_y = 0.0;
double tUsrPos_enu_z = 0.0;
double Trajectory_x = 0.0;
double Trajectory_y = 0.0;
double Trajectory_z = 0.0;
double Trajectory_x_Last = 0.0;
double Trajectory_y_Last = 0.0;
double Trajectory_z_Last = 0.0;
double Distance = 0.0;
double Distance_Last = 0.0;

int log_1 = 0;
int log_2 = 0;
int log_3 = 0;
int log_4 = 0;

std::size_t length_index;
std::size_t length_pindex_vel;

boost::circular_buffer<double> pUsrPos_enu_x(ESTNUM_MAX);
boost::circular_buffer<double> pUsrPos_enu_y(ESTNUM_MAX);
boost::circular_buffer<double> pUsrPos_enu_z(ESTNUM_MAX);
boost::circular_buffer<double> tTrajectory_x(ESTNUM_MAX);
boost::circular_buffer<double> tTrajectory_y(ESTNUM_MAX);
boost::circular_buffer<double> tTrajectory_z(ESTNUM_MAX);
boost::circular_buffer<double> pVelocity(ESTNUM_MAX);
boost::circular_buffer<double> pDistance(ESTNUM_MAX);

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

std::vector<double>::iterator max; //pattern1
std::vector<double>::iterator max_x; //pattern2
std::vector<double>::iterator max_y; //pattern2

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

  ++count;

  IMUTime = IMUperiod * count;
  ROSTime = ros::Time::now().toSec();
  Time = ROSTime;  // IMUTime or ROSTime
  // ROS_INFO("Time = %lf" , Time);

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
  }

  if (flag_Est_Raw_Heading == true)
  {
    flag_Est_Raw_Heading = false; //In order to prevent being judged many times
    ++index_Raw_count;
  }

  UsrVel_enu_E = msg->VelE;
  UsrVel_enu_N = msg->VelN;
  UsrVel_enu_U = msg->VelU;
  UsrVel_time_stamp = msg->time_stamp;

  Trajectory_x = Trajectory_x_Last + UsrVel_enu_E * (Time - Time_Last);
  Trajectory_y = Trajectory_y_Last + UsrVel_enu_N * (Time - Time_Last);
  Trajectory_z = Trajectory_z_Last + UsrVel_enu_U * (Time - Time_Last);

  if (Distance-Distance_Last > SPLITDIST && flag_GNSS == true)
  {

    if (ESTNUM < ESTNUM_MAX)
    {
      ++ESTNUM;
    }
    else
    {
      ESTNUM = ESTNUM_MAX;
    }

    // data buffer generate
    pUsrPos_enu_x.push_back(UsrPos_enu_x);
    pUsrPos_enu_y.push_back(UsrPos_enu_y);
    //pUsrPos_enu_z.push_back(UsrPos_enu_z);
    pUsrPos_enu_z.push_back(0);
    pVelocity.push_back(Correction_Velocity);
    tTrajectory_x.push_back(Trajectory_x);
    tTrajectory_y.push_back(Trajectory_y);
    //tTrajectory_z.push_back(Trajectory_z);
    tTrajectory_z.push_back(0);
    pDistance.push_back(Distance);
    Distance_Last = Distance;

    flag_DATA = true; //Judgment that refreshed data
  }

  std::vector<int> pindex_distance;
  std::vector<int> pindex_vel;
  std::vector<int> index;

  for (i = 0; i < ESTNUM; i++)
  {
    if (pDistance[ESTNUM-1] - pDistance[i]  <= ESTDIST)
    {
      pindex_distance.push_back(i);
    }
    if (pVelocity[i] > TH_VEL_EST)
    {
      pindex_vel.push_back(i);
    }
  }

  length_pindex_vel = std::distance(pindex_vel.begin(), pindex_vel.end());

  set_intersection(pindex_distance.begin(), pindex_distance.end(), pindex_vel.begin(), pindex_vel.end(),
                   inserter(index, index.end()));

  length_index = std::distance(index.begin(), index.end());
  log_1 = length_index;

  if (flag_DATA == true)
  {
    if (Distance > ESTDIST && flag_GNSS == true && Correction_Velocity > TH_VEL_EST && index_Raw_count > 0 &&
        count > 1)
    {
      if (length_index > length_pindex_vel * TH_CALC_MINNUM)
      {
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
            pdiff_x.push_back(fabsf(basepos2_x[index[i]] - pUsrPos_enu_x[index[i]]));
            pdiff_y.push_back(fabsf(basepos2_y[index[i]] - pUsrPos_enu_y[index[i]]));
            pdiff_z.push_back(fabsf(basepos2_z[index[i]] - pUsrPos_enu_z[index[i]]));
          }


/*
          //pattern1
          pdiff.clear();
          for (i = 0; i < length_index; i++)
          {
            pdiff.push_back(sqrt((pdiff_x[i] * pdiff_x[i]) + (pdiff_y[i] * pdiff_y[i]) + (pdiff_z[i] * pdiff_z[i])));
          }

          max = std::max_element(pdiff.begin(), pdiff.end());

          index_max = std::distance(pdiff.begin(), max);

                if (pdiff[index_max] > TH_POSMAX)
                {
                  index.erase(index.begin() + index_max);
                }
                else
                {
                  //Trajectory_x_Last = 0.0;
                  //Trajectory_y_Last = 0.0;
                  //Trajectory_z_Last = 0.0;
                  log_2 = length_index;
                  break;
                }
*/
/*
          //pattern2
          max_x = std::max_element(pdiff_x.begin(), pdiff_x.end());
          max_y = std::max_element(pdiff_y.begin(), pdiff_y.end());

          index_max_x = std::distance(pdiff_x.begin(), max_x);
          index_max_y = std::distance(pdiff_y.begin(), max_y);

          if(pdiff_x[index_max_x] > pdiff_y[index_max_y])
          {
            if (pdiff_x[index_max_x] > TH_POSMAX)
            {
              index.erase(index.begin() + index_max_x);
            }
            else
            {
              //Trajectory_x_Last = 0.0;
              //Trajectory_y_Last = 0.0;
              //Trajectory_z_Last = 0.0;
              log_2 = length_index;
              break;
            }
          }
          else
          {
            if (pdiff_y[index_max_y] > TH_POSMAX)
            {
              index.erase(index.begin() + index_max_y);
            }
            else
            {
              //Trajectory_x_Last = 0.0;
              //Trajectory_y_Last = 0.0;
              //Trajectory_z_Last = 0.0;
              log_2 = length_index;
              break;
            }
          }

*/
	  //pattern3
          max_x = std::max_element(pdiff_x.begin(), pdiff_x.end());
          max_y = std::max_element(pdiff_y.begin(), pdiff_y.end());

          index_max_x = std::distance(pdiff_x.begin(), max_x);
          index_max_y = std::distance(pdiff_y.begin(), max_y);

          if(pdiff_x[index_max_x] < pdiff_y[index_max_y])
          {
            if (pdiff_x[index_max_x] > TH_POSMAX)
            {
              index.erase(index.begin() + index_max_x);
            }
            else
            {
              //Trajectory_x_Last = 0.0;
              //Trajectory_y_Last = 0.0;
              //Trajectory_z_Last = 0.0;
              log_2 = length_index;
              break;
            }
          }
          else
          {
            if (pdiff_y[index_max_y] > TH_POSMAX)
            {
              index.erase(index.begin() + index_max_y);
            }
            else
            {
              //Trajectory_x_Last = 0.0;
              //Trajectory_y_Last = 0.0;
              //Trajectory_z_Last = 0.0;
              log_2 = length_index;
              break;
            }
          }

//
          length_index = std::distance(index.begin(), index.end());
          length_pindex_vel = std::distance(pindex_vel.begin(), pindex_vel.end());

          if (length_index < length_pindex_vel * TH_EST_GIVEUP_NUM)
          {
            //Trajectory_x_Last = 0.0;
            //Trajectory_y_Last = 0.0;
            //Trajectory_z_Last = 0.0;
            log_3 = length_index;
            break;
          }

        }

        length_index = std::distance(index.begin(), index.end());
        length_pindex_vel = std::distance(pindex_vel.begin(), pindex_vel.end());

        if (length_index >= length_pindex_vel * TH_EST_GIVEUP_NUM)
        {
          if (index[length_index - 1] == ESTNUM-1)
          {
            UsrPos_EstRaw_enu_x = tUsrPos_enu_x;
            UsrPos_EstRaw_enu_y = tUsrPos_enu_y;
            UsrPos_EstRaw_enu_z = tUsrPos_enu_z;

            ++ log_4;

            p1_msg.enu_x = UsrPos_EstRaw_enu_x;
            p1_msg.enu_y = UsrPos_EstRaw_enu_y;
            p1_msg.enu_z = UsrPos_EstRaw_enu_z;
            p1_msg.time_stamp = UsrVel_time_stamp;
            p1_msg.log_1 = log_1;
            p1_msg.log_2 = log_2;
            p1_msg.log_3 = log_3;
            p1_msg.log_4 = log_4;
            pub1.publish(p1_msg);
          }
          else
          {
            UsrPos_EstRaw_enu_x = tUsrPos_enu_x + (tTrajectory_x[ESTNUM - 1] - tTrajectory_x[index[length_index - 1]]);
            UsrPos_EstRaw_enu_y = tUsrPos_enu_y + (tTrajectory_y[ESTNUM - 1] - tTrajectory_y[index[length_index - 1]]);
            UsrPos_EstRaw_enu_z = tUsrPos_enu_z + (tTrajectory_z[ESTNUM - 1] - tTrajectory_z[index[length_index - 1]]);
          }
        }
      }
    }
  }
  Trajectory_x_Last = Trajectory_x;
  Trajectory_y_Last = Trajectory_y;
  Trajectory_z_Last = Trajectory_z;
  GPSTime_Last = GPSTime;
  Time_Last = Time;
  flag_DATA = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RCalc_PositionDis");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/UsrVel_enu", 1000, receive_UsrVel_enu);
  ros::Subscriber sub2 = n.subscribe("/RTKLIB", 1000, receive_UsrPos_enu);
  ros::Subscriber sub3 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub4 = n.subscribe("/imu_gnss_localizer/Distance", 1000, receive_Distance);
  ros::Subscriber sub5 = n.subscribe("/imu_gnss_localizer/Heading3rd_Int", 1000, receive_Heading3rd);
  pub1 = n.advertise<imu_gnss_localizer::PositionDis_raw>("/imu_gnss_localizer/PositionDis", 1000);
  ros::spin();

  return 0;
}
