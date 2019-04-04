/*
 * RCalc_YawrateOffsetStop.cpp
 * Velocity SF estimate program
 * Author Sekino
 * Ver 1.00 2019/01/26
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "imu_gnss_localizer/RTKLIB.h"
#include "imu_gnss_localizer/VelocitySF.h"
#include <boost/circular_buffer.hpp>

ros::Publisher pub;

bool flag_GNSS, flag_SFRaw, flag_SF_Start, flag_SF;
int i = 0;
int count = 0;
int GPSTime_Last;
int GPSTime;
int ESTNUM_SF = 0;
float ESTNUM_MIN = 1000;
float ESTNUM_MAX = 20000;
float TH_VEL_EST = 10/3.6;
float ESTNUM_ESF = 0.05;
float Velocity_SFInit = 1.0;
float Velocity = 0.0;
float Velocity_Doppler = 0.0;
float velN = 0.0;
float velE = 0.0;
float velU = 0.0;
float Velocity_SFRaw = 0.0;
float Velocity_SF = 0.0;
float Correction_Velocity = 0.0;
float SF_Last = 0.0;

std::size_t length_index;

imu_gnss_localizer::VelocitySF p_msg;
boost::circular_buffer<bool> pflag_GNSS(ESTNUM_MAX);
boost::circular_buffer<float> pVelocity_Doppler(ESTNUM_MAX);
boost::circular_buffer<float> pVelocity(ESTNUM_MAX);

void receive_Gnss(const imu_gnss_localizer::RTKLIB::ConstPtr& msg){

  GPSTime = msg->GPSTime;
  velN = (float)msg->Vel_n; //unit [m/s]
  velE = (float)msg->Vel_e; //unit [m/s]
  velU = (float)msg->Vel_u; //unit [m/s]
  Velocity_Doppler = sqrt((velE * velE) + (velN * velN) + (velU * velU)); //unit [m/s]

  ROS_INFO("Velocity_Doppler = %f [m/s]" , Velocity_Doppler );

}

void receive_Velocity(const geometry_msgs::Twist::ConstPtr& msg){

  if (ESTNUM_SF < ESTNUM_MAX){
    ++ESTNUM_SF;
  }
  else{
    ESTNUM_SF = ESTNUM_MAX;
  }

  Velocity = msg->linear.x ;

    //GNSS receive flag
  if (GPSTime_Last == GPSTime){
    flag_GNSS = false;
  }
  else{
    flag_GNSS = true;
  }

    //data buffer generate
    pflag_GNSS.push_back(flag_GNSS);
    pVelocity_Doppler.push_back(Velocity_Doppler);
    pVelocity.push_back(Velocity);

    //dynamic array
    std::vector<int> pindex_GNSS;
    std::vector<int> pindex_vel;
    std::vector<int> index;
    std::vector<float> pSF;

  if (ESTNUM_SF > ESTNUM_MIN && pflag_GNSS[ESTNUM_SF-1] == true && pVelocity[ESTNUM_SF-1] > TH_VEL_EST){

    //The role of "find function" in MATLAB !Suspect!
    for(i = 0; i < ESTNUM_SF; i++){
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

    if(length_index > ESTNUM_SF * ESTNUM_ESF){

    //The role of " ./ " in MATLAB & Extract only necessary data (Processing different from MATLAB version)
          for(i = 0; i < length_index; i++){
              pSF.push_back(pVelocity_Doppler[index[i]] / pVelocity[index[i]]);
          }

        flag_SFRaw = true;
        flag_SF_Start = true;
      }
      else{
        flag_SFRaw = false;
      }
    }
    else{
      flag_SFRaw = false;
    }

    if(flag_SFRaw == true){

    //The role of "median function" in MATLAB
        size_t size = pSF.size();
        float *t = new float[size];
        std::copy(pSF.begin(), pSF.end(), t);
        std::sort(t, &t[size]);
        Velocity_SFRaw = size%2 ? t[size/2] : (t[(size/2)-1]+t[size/2])/2;
        delete[] t;

      //ROS_INFO("Velocity_SF = %f", Velocity_SFRaw );
      Velocity_SF = Velocity_SFRaw;
    }
    else if(flag_SFRaw == false){
      Velocity_SFRaw = 0;
      Velocity_SF = SF_Last;
    }

    if(flag_SF_Start == true){
      flag_SF = true;
      Correction_Velocity = Velocity * Velocity_SF;
    }
    else{
      flag_SF = false;
      Correction_Velocity = Velocity * Velocity_SFInit;
    }

    p_msg.Correction_Velocity = Correction_Velocity;
    p_msg.Scale_Factor = Velocity_SF;
    p_msg.flag_Est = flag_SF;
    p_msg.flag_EstRaw = flag_SFRaw;
    pub.publish(p_msg);

    SF_Last = Velocity_SF;
    GPSTime_Last = GPSTime;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "RCalc_VelocitySF");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/Vehicle/Velocity", 1000, receive_Velocity);
  ros::Subscriber sub2 = n.subscribe("/RTKLIB", 1000, receive_Gnss);
  pub = n.advertise<imu_gnss_localizer::VelocitySF>("/imu_gnss_localizer/VelocitySF", 1000);

  ros::spin();

  return 0;
}
