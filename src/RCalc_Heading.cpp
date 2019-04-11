/*
 * RCalc_Heading.cpp
 * Heading estimate program
 * Author Sekino
 * Ver 2.00 2019/4/11 Integrate 1st, 2nd, 3rd
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/RTKLIB.h"
#include "imu_gnss_localizer/Heading.h"
#include "imu_gnss_localizer/VelocitySF.h"
#include "imu_gnss_localizer/YawrateOffset.h"
#include <boost/circular_buffer.hpp>
#include <math.h>
#include <numeric>

ros::Publisher pub;

//default parameter
bool reverse_imu = false;

bool flag_GNSS, flag_EstRaw, flag_Start, flag_Est_Heading, flag_Est;
int i = 0;
int count = 0;
int GPSTime_Last, GPSTime;
int ESTNUM_Heading = 0;
int index_max = 0;
double IMUTime;
double IMUfrequency = 50; //IMU Hz
double IMUperiod = 1.0/IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
float avg = 0.0;
float ESTNUM_MIN = 500;
float ESTNUM_MAX = 1500;
float ESTNUM_GNSF = 1.0/10;
float ESTNUM_THSF = ESTNUM_GNSF*1.0/2.0;
float TH_HEADINGMAX = 3.0/180*M_PI;
float TH_VEL_EST = 10/3.6;
float TH_VEL_EST_Heading = TH_VEL_EST;
float TH_VEL_STOP = 0.01;
float TH_YAWRATE = 5.0/180*M_PI;
float Heading_Doppler = 0.0;
float Heading = 0.0;
float velN = 0.0;
float velE = 0.0;
float velU = 0.0;
float YawrateOffset_Stop = 0.0;
float YawrateOffset = 0.0;
float Yawrate = 0.0;
float Correction_Velocity = 0.0;
float tHeading = 0.0;
float Heading_EstRaw = 0.0;
float Heading_Est = 0.0;
float Heading_Last = 0.0;

std::size_t length_index;
std::size_t length_index_inv_up;
std::size_t length_index_inv_down;

std::vector<float>::iterator max;

imu_gnss_localizer::Heading p_msg;

boost::circular_buffer<double> pTime(ESTNUM_MAX);
boost::circular_buffer<float> pHeading(ESTNUM_MAX);
boost::circular_buffer<float> pYawrate(ESTNUM_MAX);
boost::circular_buffer<float> pVelocity(ESTNUM_MAX);
boost::circular_buffer<float> pYawrateOffset_Stop(ESTNUM_MAX);
boost::circular_buffer<float> pYawrateOffset(ESTNUM_MAX);
boost::circular_buffer<bool> pflag_GNSS(ESTNUM_MAX);

void receive_VelocitySF(const imu_gnss_localizer::VelocitySF::ConstPtr& msg){

  Correction_Velocity = msg->Correction_Velocity;

}

void receive_YawrateOffsetStop(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg){

  YawrateOffset_Stop = msg->YawrateOffset;

}

void receive_YawrateOffset(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg){

  YawrateOffset = msg->YawrateOffset; //1st is YawrateOffset_Stop here too

}

void receive_Gnss(const imu_gnss_localizer::RTKLIB::ConstPtr& msg){

  GPSTime = msg->GPSTime;
  velN = (float)msg->Vel_n; //unit [m/s]
  velE = (float)msg->Vel_e; //unit [m/s]
  velU = (float)msg->Vel_u; //unit [m/s]
  Heading_Doppler = atan2(velE , velN); //unit [rad]
  //ROS_INFO("Heading_Doppler = %f",Heading_Doppler);

}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg){

    ++count;
    IMUTime = IMUperiod * count;
    ROSTime = ros::Time::now().toSec();
    Time = IMUTime; //IMUTime or ROSTime
    //ROS_INFO("Time = %lf" , Time);

    if (ESTNUM_Heading < ESTNUM_MAX){
      ++ESTNUM_Heading;
    }
    else{
      ESTNUM_Heading = ESTNUM_MAX;
    }

    if (reverse_imu == false){
      Yawrate = msg->angular_velocity.z;
    }
    else if (reverse_imu == true){
      Yawrate = -1 * msg->angular_velocity.z;
    }

    //GNSS receive flag
    if (GPSTime_Last == GPSTime){
      flag_GNSS = false;
      Heading_Doppler = 0;
    }
    else{
      flag_GNSS = true;
      Heading_Doppler = Heading_Doppler;
    }

    //ROS_INFO("Yawrate %f ", Yawrate + YawrateOffset_Stop);

    //data buffer generate
    pTime.push_back(Time);
    pHeading.push_back(Heading_Doppler);
    pYawrate.push_back(Yawrate);
    pVelocity.push_back(Correction_Velocity);
    pYawrateOffset_Stop.push_back(YawrateOffset_Stop);
    pYawrateOffset.push_back(YawrateOffset);
    pflag_GNSS.push_back(flag_GNSS);

    //dynamic array
    std::vector<int> pindex_GNSS;
    std::vector<int> pindex_vel;
    std::vector<int> index;

    if (ESTNUM_Heading > ESTNUM_MIN && pflag_GNSS[ESTNUM_Heading-1] == true && pVelocity[ESTNUM_Heading-1] > TH_VEL_EST && fabsf(pYawrate[ESTNUM_Heading-1]) < TH_YAWRATE){
    flag_Est_Heading = true;
    }
    else{
    flag_Est_Heading = false;
    }

    if (flag_Est_Heading == true){

      //The role of "find function" in MATLAB !Suspect!
      for(i = 0; i < ESTNUM_Heading; i++){
        if(pflag_GNSS[i] == true){
            pindex_GNSS.push_back(i);
        }
        if(pVelocity[i] > TH_VEL_EST){
            pindex_vel.push_back(i);
        }
      }

      //The role of "intersect function" in MATLAB
        set_intersection(pindex_GNSS.begin(), pindex_GNSS.end()
                       , pindex_vel.begin(), pindex_vel.end()
                       , inserter(index, index.end()));

        length_index = std::distance(index.begin(), index.end());

        if(length_index > ESTNUM_Heading * ESTNUM_GNSF){

          //zeros array
          std::vector<float> ppHeading(ESTNUM_Heading,0);

          for(i = 0; i < ESTNUM_Heading; i++){
            if(i > 0){
              if(pVelocity[ESTNUM_Heading-1] > TH_VEL_STOP){
                ppHeading[i] = ppHeading[i-1] + (pYawrate[i] + pYawrateOffset[i]) * (pTime[i] - pTime[i-1]) ;
              }
              else{
                ppHeading[i] = ppHeading[i-1] + (pYawrate[i] + pYawrateOffset_Stop[i]) * (pTime[i] - pTime[i-1]);
              }
            }
          }

          //dynamic array
          std::vector<float> baseHeading;
          std::vector<float> baseHeading2;
          std::vector<float> pdiff;
          std::vector<float> index_inv_up;
          std::vector<float> index_inv_down;

          for(i = 0; i < ESTNUM_Heading; i++){
          baseHeading.push_back(pHeading[index[length_index-1]] - ppHeading[index[length_index-1]] + ppHeading[i]);
          }

          for(i = 0; i < length_index; i++){
          pdiff.push_back(baseHeading[index[i]] - pHeading[index[i]]);
          }

          //The role of "find function" in MATLAB !Suspect!
          for(i = 0; i < length_index; i++){
            if(pdiff[i] > M_PI/2.0){
                index_inv_up.push_back(index[i]);
            }
            if(pdiff[i] < -M_PI/2.0){
                index_inv_down.push_back(index[i]);
            }
          }

          length_index_inv_up = std::distance(index_inv_up.begin(), index_inv_up.end());
          length_index_inv_down = std::distance(index_inv_down.begin(), index_inv_down.end());

          if(length_index_inv_up != 0){
            for(i = 0; i < length_index_inv_up; i++){
              pHeading[index_inv_up[i]] = pHeading[index_inv_up[i]] + 2.0*M_PI;
              //ROS_INFO("UP r %f", pHeading[index_inv_up[i]]);
            }
          }

          if(length_index_inv_down != 0){
            for(i = 0; i < length_index_inv_down; i++){
              pHeading[index_inv_down[i]] = pHeading[index_inv_down[i]] - 2.0*M_PI;
              //ROS_INFO("DOWN r %f", pHeading[index_inv_down[i]]);
            }
          }

          std::size_t length_pHeading = std::distance(pHeading.begin(), pHeading.end());
          std::size_t length_ppHeading = std::distance(ppHeading.begin(), ppHeading.end());

          //ROS_INFO("index end %d pHeading %zu ppHeading %zu",index[length_index-1],length_pHeading,length_ppHeading);

            while(1){

              length_index = std::distance(index.begin(), index.end());

              baseHeading.clear();
              for(i = 0; i < ESTNUM_Heading; i++){
              baseHeading.push_back(pHeading[index[length_index-1]] - ppHeading[index[length_index-1]] + ppHeading[i]);
              }

              pdiff.clear();
              for(i = 0; i < length_index; i++){
              pdiff.push_back(baseHeading[index[i]] - pHeading[index[i]]);
              }

              avg = std::accumulate(pdiff.begin(), pdiff.end(), 0.0) / length_index;

              tHeading = pHeading[length_index-1] - avg;

              baseHeading2.clear();
              for(i = 0; i < ESTNUM_Heading; i++){
              baseHeading2.push_back(tHeading - ppHeading[index[length_index-1]] + ppHeading[i]);
              }

              pdiff.clear();
              for(i = 0; i < length_index; i++){
              pdiff.push_back(fabsf(baseHeading2[index[i]] - pHeading[index[i]]));
              }

              max = std::max_element(pdiff.begin(), pdiff.end());
              index_max = std::distance(pdiff.begin(), max);

              if(pdiff[index_max] > TH_HEADINGMAX){
                index.erase(index.begin() + index_max);
              }
              else{
                break;
              }

              length_index = std::distance(index.begin(), index.end());

              if(length_index < ESTNUM_Heading * ESTNUM_THSF){
                break;
              }

            }

        if(length_index == 0 || length_index  > ESTNUM_Heading * ESTNUM_THSF){
          if(index[length_index-1] == ESTNUM_Heading){
            Heading_EstRaw = tHeading;
          }
          else{
            Heading_EstRaw = tHeading + ( ppHeading[ESTNUM_Heading-1] - ppHeading[index[length_index-1]] );
          }

          if(Heading_EstRaw > M_PI){
            Heading_EstRaw = Heading_EstRaw - 2.0*M_PI;
          }
          else if(Heading_EstRaw < -M_PI){
            Heading_EstRaw = Heading_EstRaw + 2.0*M_PI;
          }

          flag_EstRaw = true;
          flag_Start = true;

        }
        else{
          Heading_EstRaw = 0.0;
          flag_EstRaw = false;
        }

      }

    }

    else{
        Heading_EstRaw = 0.0;
        flag_EstRaw = false;
        }


    if(pVelocity[ESTNUM_Heading-1] > TH_VEL_STOP){
      Yawrate = pYawrate[ESTNUM_Heading-1] + pYawrateOffset[ESTNUM_Heading-1];
    }
    else{
      Yawrate = pYawrate[ESTNUM_Heading-1] + pYawrateOffset_Stop[ESTNUM_Heading-1];
    }

    if(flag_EstRaw == true){
      Heading_Est = Heading_EstRaw;
    }
    else{
      Heading_Est = Heading_Last + Yawrate * ( pTime[ESTNUM_Heading-1] - Time_Last);
    }

    if(Heading_Est > M_PI){
      Heading_Est = Heading_Est - 2.0*M_PI;
    }
    else if(Heading_Est < -M_PI){
      Heading_Est = Heading_Est + 2.0*M_PI;
    }

    if(flag_Start == true){
      flag_Est = true;
      Heading = Heading_Est;
    }
    else{
      flag_Est = false;
      Heading = 0.0;
    }

  p_msg.Heading_angle = Heading;
  p_msg.flag_Est = flag_Est;
  p_msg.flag_EstRaw = flag_EstRaw;
  pub.publish(p_msg);

  GPSTime_Last = GPSTime;
  Time_Last = Time;
  Heading_Last = Heading_Est;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "RCalc_Heading");
  ros::NodeHandle n("~");
  n.getParam("/imu_gnss_localizer/reverse_imu",reverse_imu);

  char publish_topic_name[128];
  char subscribe_topic_name[128];

  if(argc == 2){
    if(strcmp(argv[1],"1st") == 0){
      char publish_topic_name[] = "/imu_gnss_localizer/Heading1st";
      char subscribe_topic_name[] = "/imu_gnss_localizer/YawrateOffsetStop";
    }
    else if(strcmp(argv[1],"2nd") == 0){
      char publish_topic_name[] = "/imu_gnss_localizer/Heading2nd";
      char subscribe_topic_name[] = "/imu_gnss_localizer/YawrateOffset1st";
    }
    else if(strcmp(argv[1],"3rd") == 0){
      char publish_topic_name[] = "/imu_gnss_localizer/Heading3rd";
      char subscribe_topic_name[] = "/imu_gnss_localizer/YawrateOffset2nd";
    }
    else{
      char publish_topic_name[] = "/publish_topic_name/invalid";
      char subscribe_topic_name[] = "/subscribe_topic_name/invalid";
      ROS_ERROR("Invalid argument");
      ros::shutdown();
    }
  }
  else{
    char publish_topic_name[] = "/publish_topic_name/invalid";
    char subscribe_topic_name[] = "/subscribe_topic_name/invalid";
    ROS_ERROR("No arguments");
    ros::shutdown();
  }

  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/YawrateOffsetStop", 1000, receive_YawrateOffsetStop);
  ros::Subscriber sub3 = n.subscribe(subscribe_topic_name, 1000, receive_YawrateOffset);
  ros::Subscriber sub4 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  ros::Subscriber sub5 = n.subscribe("/RTKLIB", 1000, receive_Gnss);
  pub = n.advertise<imu_gnss_localizer::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
