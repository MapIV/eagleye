/*
 * RCalc_PositionDis_Int.cpp
 * Vehicle position realtime estimate program
 * Author Sekino
 * Ver 1.00 2019/2/28
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "imu_gnss_localizer/RTKLIB.h"
#include "imu_gnss_localizer/UsrVel_enu.h"
#include "imu_gnss_localizer/PositionDis_raw.h"
#include "imu_gnss_localizer/PositionDis.h"
#include <boost/circular_buffer.hpp>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

bool flag_UsrPos_Est, flag_UsrPos_Start, flag_Est, flag_EstRaw;
int i = 0;
int count = 0;
int Est_count = 0;
int Est_index = 0;
int UsrPos_index = 0;
int UsrVel_index = 0;
int UsrPos_index_Last = 0;
int BUFNUM = 0;
double IMUTime;
double IMUfrequency = 50; //IMU Hz
double IMUperiod = 1/IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
double lon = 0.0;
double lat = 0.0;
double alt = 0.0;
float base_ECEF_x = 0.0;
float base_ECEF_y = 0.0;
float base_ECEF_z = 0.0;
float BUFNUM_MAX = 100;
float UsrVel_enu_E = 0.0;
float UsrVel_enu_N = 0.0;
float UsrVel_enu_U = 0.0;
float UsrPos_Est_enu_x = 0.0;
float UsrPos_Est_enu_y = 0.0;
float UsrPos_Est_enu_z = 0.0;
float UsrPos_EstRaw_enu_x = 0.0;
float UsrPos_EstRaw_enu_y = 0.0;
float UsrPos_EstRaw_enu_z = 0.0;
float diff_Est_enu_x = 0.0;
float diff_Est_enu_y = 0.0;
float diff_Est_enu_z = 0.0;
float UsrPos_Est_enu_x_Last = 0.0;
float UsrPos_Est_enu_y_Last = 0.0;
float UsrPos_Est_enu_z_Last = 0.0;

geometry_msgs::PoseStamped p1_msg;
imu_gnss_localizer::PositionDis p2_msg;
sensor_msgs::NavSatFix p3_msg;

boost::circular_buffer<float> pUsrPos_Est_enu_x(BUFNUM_MAX);
boost::circular_buffer<float> pUsrPos_Est_enu_y(BUFNUM_MAX);
boost::circular_buffer<float> pUsrPos_Est_enu_z(BUFNUM_MAX);

void enu2llh(double enu_x,double enu_y,double enu_z,double base_ECEF_x,double base_ECEF_y,double base_ECEF_z,double *lon,double *lat,double *alt){

  //ECEF基準点をllhに変換
  double x = base_ECEF_x;
  double y = base_ECEF_y;
  double z = base_ECEF_z;
  double x2 = x*x ;
  double y2 = y*y ;
  double z2 = z*z ;
  double a = 6378137.0000;
  double b = 6356752.3142;
  double e = sqrt(1-pow(b/a,2.0));
  double r = sqrt(x2+y2);
  double ep = e*(a/b);
  double b2 = b*b;
  double e2 = e*e;
  double r2 = r*r;
  double f = 54*b2*z2;
  double g = r2 + (1-e2)*z2 - e2*(a*a-b*b);
  double i = (e2*e2*f*r2)/(g*g*g);
  double o = pow(( 1 + i + sqrt(i*i + 2*i) ),1.0/3.0);
  double p = f / (3 * pow((o+1/o+1),2.0) * g*g);
  double q = sqrt(1+2*e2*e2*p);
  double s = -(p*e2*r)/(1+q) + sqrt((a*a/2)*(1+1/q) - (p*(1-e2)*z2)/(q*(1+q)) - p*r2/2);
  double tmp = pow((r - e2*s),2.0);
  double u = sqrt( tmp + z2 );
  double v = sqrt( tmp + (1-e2)*z2 );
  double w = (b2*z)/(a*v);
  double tmp_lon = atan(y/x);
  double base_lat = atan((z + ep*ep*w)/r);
  double base_lon = 0;

  if(x >= 0){
    base_lon = tmp_lon;
  }
  else{
    if(x < 0 && y >= 0){
      base_lon = M_PI + tmp_lon;
    }
    else{
      base_lon = tmp_lon - M_PI;
    }
  }

  double base_alt = u*( 1 - b2/(a*v) );

  //enuをECEFに変換
  double phi = base_lat;
  double lam = base_lon;

  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double sin_lam = sin(lam);
  double cos_lam = cos(lam);

  double ECEF_x = base_ECEF_x + ((-sin_lam * enu_x) + (-cos_lam * sin_phi * enu_y) + (cos_lam * cos_phi * enu_z));
  double ECEF_y = base_ECEF_y + ((cos_lam * enu_x) + (-sin_lam * sin_phi * enu_y) + (sin_lam * cos_phi * enu_z));
  double ECEF_z = base_ECEF_z + ((0 * enu_x) + (cos_phi * enu_y) + (sin_phi * enu_z));

  //ECEFをllhに変換
  x = ECEF_x;
  y = ECEF_y;
  z = ECEF_z;
  x2 = x*x ;
  y2 = y*y ;
  z2 = z*z ;
  a = 6378137.0000;
  b = 6356752.3142;
  e = sqrt(1-pow(b/a,2.0));
  r = sqrt(x2+y2);
  ep = e*(a/b);
  b2 = b*b;
  e2 = e*e;
  r2 = r*r;
  f = 54*b2*z2;
  g = r2 + (1-e2)*z2 - e2*(a*a-b*b);
  i = (e2*e2*f*r2)/(g*g*g);
  o = pow(( 1 + i + sqrt(i*i + 2*i) ),1.0/3.0);
  p = f / (3 * pow((o+1/o+1),2.0) * g*g);
  q = sqrt(1+2*e2*e2*p);
  s = -(p*e2*r)/(1+q) + sqrt((a*a/2)*(1+1/q) - (p*(1-e2)*z2)/(q*(1+q)) - p*r2/2);
  tmp = pow((r - e2*s),2.0);
  u = sqrt( tmp + z2 );
  v = sqrt( tmp + (1-e2)*z2 );
  w = (b2*z)/(a*v);
  tmp_lon = atan(y/x);
  *lat = (atan((z + ep*ep*w)/r))*180/M_PI;

  if(x >= 0){
    *lon = (tmp_lon)*180/M_PI;
  }
  else{
    if(x < 0 && y >= 0){
        *lon = (M_PI + tmp_lon)*180/M_PI;
    }
    else{
        *lon = (tmp_lon - M_PI)*180/M_PI;
    }
  }

  *alt = u*( 1 - b2/(a*v) );

}

void receive_Gnss(const imu_gnss_localizer::RTKLIB::ConstPtr& msg){

  base_ECEF_x = msg->ORG_x;
  base_ECEF_y = msg->ORG_y;
  base_ECEF_z = msg->ORG_z;

}

void receive_PositionDis(const imu_gnss_localizer::PositionDis_raw::ConstPtr& msg){

  UsrPos_EstRaw_enu_x = msg->enu_x;
  UsrPos_EstRaw_enu_y = msg->enu_y;
  UsrPos_EstRaw_enu_z = msg->enu_z;
  UsrPos_index = msg->index;

}

void receive_UsrVel_enu(const imu_gnss_localizer::UsrVel_enu::ConstPtr& msg){

    ++count;
    IMUTime = IMUperiod * count;
    ROSTime = ros::Time::now().toSec();
    Time = IMUTime; //IMUTime or ROSTime
    //ROS_INFO("Time = %lf" , Time);

    UsrVel_enu_E = msg->VelE;
    UsrVel_enu_N = msg->VelN;
    UsrVel_enu_U = msg->VelU;
    UsrVel_index = msg->index;

    if (BUFNUM < BUFNUM_MAX){
      ++BUFNUM;
    }
    else{
      BUFNUM = BUFNUM_MAX;
    }

    if (UsrPos_index_Last == UsrPos_index){
      flag_UsrPos_Est = false;
      UsrPos_index = 0;
      UsrVel_index = 0;
    }
    else{
      flag_UsrPos_Est = true;
      flag_UsrPos_Start = true;
      ++Est_count;
    }

    pUsrPos_Est_enu_x.push_back(UsrPos_Est_enu_x);
    pUsrPos_Est_enu_y.push_back(UsrPos_Est_enu_y);
    pUsrPos_Est_enu_z.push_back(UsrPos_Est_enu_z);

    if(flag_UsrPos_Start == true){

      Est_index = BUFNUM - (UsrVel_index - UsrPos_index);

      if(flag_UsrPos_Est == true && Est_index > 0 && Est_count > 1){
        diff_Est_enu_x = (pUsrPos_Est_enu_x[Est_index-1] - UsrPos_EstRaw_enu_x);
        diff_Est_enu_y = (pUsrPos_Est_enu_y[Est_index-1] - UsrPos_EstRaw_enu_y);
        diff_Est_enu_z = (pUsrPos_Est_enu_z[Est_index-1] - UsrPos_EstRaw_enu_z);
        for(i = Est_index; i <= BUFNUM; i++){
          pUsrPos_Est_enu_x[i-1] = pUsrPos_Est_enu_x[i-1] - diff_Est_enu_x;
          pUsrPos_Est_enu_y[i-1] = pUsrPos_Est_enu_y[i-1] - diff_Est_enu_y;
          pUsrPos_Est_enu_z[i-1] = pUsrPos_Est_enu_z[i-1] - diff_Est_enu_z;
        }
        UsrPos_Est_enu_x = pUsrPos_Est_enu_x[BUFNUM-1];
        UsrPos_Est_enu_y = pUsrPos_Est_enu_y[BUFNUM-1];
        UsrPos_Est_enu_z = pUsrPos_Est_enu_z[BUFNUM-1];

        flag_Est = true;
        flag_EstRaw = true;

        ROS_INFO("Est_index = %d" , Est_index);
      }

      else if(Est_count == 1){
        UsrPos_Est_enu_x = UsrPos_EstRaw_enu_x;
        UsrPos_Est_enu_y = UsrPos_EstRaw_enu_y;
        UsrPos_Est_enu_z = UsrPos_EstRaw_enu_z;
      }
      else if(count > 1){
        UsrPos_Est_enu_x = UsrPos_Est_enu_x_Last + UsrVel_enu_E * ( Time - Time_Last );
        UsrPos_Est_enu_y = UsrPos_Est_enu_y_Last + UsrVel_enu_N * ( Time - Time_Last );
        UsrPos_Est_enu_z = UsrPos_Est_enu_z_Last + UsrVel_enu_U * ( Time - Time_Last );

        flag_EstRaw = false;
      }

    }

    if(Est_count > 1){

    enu2llh(UsrPos_Est_enu_x,UsrPos_Est_enu_y,UsrPos_Est_enu_z,base_ECEF_x,base_ECEF_y,base_ECEF_z,&lon,&lat,&alt);
    p1_msg.pose.position.x = UsrPos_Est_enu_x;
    p1_msg.pose.position.y = UsrPos_Est_enu_y;
    p1_msg.pose.position.z = UsrPos_Est_enu_z;

    p2_msg.enu_x = UsrPos_Est_enu_x;
    p2_msg.enu_y = UsrPos_Est_enu_y;
    p2_msg.enu_z = UsrPos_Est_enu_z;
    p2_msg.flag_Est = flag_Est;
    p2_msg.flag_EstRaw = flag_EstRaw;

    p3_msg.header.stamp = ros::Time::now();
    p3_msg.header.frame_id = "map";
    p3_msg.latitude = lat;
    p3_msg.longitude = lon;
    p3_msg.altitude = alt;
    pub3.publish(p3_msg);
    }

    p1_msg.header.stamp = ros::Time::now();
    p1_msg.header.frame_id = "map";
    p1_msg.pose.orientation.w = 1;
    pub1.publish(p1_msg);
    pub2.publish(p2_msg);

    Time_Last = Time;
    UsrPos_Est_enu_x_Last = UsrPos_Est_enu_x;
    UsrPos_Est_enu_y_Last = UsrPos_Est_enu_y;
    UsrPos_Est_enu_z_Last = UsrPos_Est_enu_z;
    UsrPos_index_Last = UsrPos_index;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "PositionDis_Int");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/UsrVel_enu", 1000, receive_UsrVel_enu);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/PositionDis", 1000, receive_PositionDis);
  ros::Subscriber sub3 = n.subscribe("/RTKLIB", 1000, receive_Gnss);
  pub1 = n.advertise<geometry_msgs::PoseStamped>("/imu_gnss_pose", 1000);
  pub2 = n.advertise<imu_gnss_localizer::PositionDis>("/imu_gnss_localizer/PositionDis_Int", 1000);
  pub3 = n.advertise<sensor_msgs::NavSatFix>("imu_gnss_localizer/fix", 1000);

  ros::spin();

  return 0;
}
