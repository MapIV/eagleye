/*
 * rtklib_connecter.cpp
 * Program for connecting with RTKLIB
 * Author Sekino
 * Ver 1.00 2019/3/6
 */

#include "ros/ros.h"
#include "imu_gnss_localizer/RTKLIB.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>
#include "sensor_msgs/NavSatFix.h"//////////////////////////

int i ,counter;
int GPSTime = 0;
double ECEF_x = 0.0;
double ECEF_y = 0.0;
double ECEF_z = 0.0;
double base_ECEF_x = 0.0;
double base_ECEF_y = 0.0;
double base_ECEF_z = 0.0;
double enu_x = 0.0;
double enu_y = 0.0;
double enu_z = 0.0;
double Vel_x = 0.0;
double Vel_y = 0.0;
double Vel_z = 0.0;
double Vel_e = 0.0;
double Vel_n = 0.0;
double Vel_u = 0.0;

double lon = 0.0;
double lat = 0.0;
double alt = 0.0;

void xyz2enu(double ECEF_x,double ECEF_y,double ECEF_z,double base_ECEF_x,double base_ECEF_y,double base_ECEF_z,double *enu_x,double *enu_y,double *enu_z){

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

  //ECEFをenuに変換
  double phi = base_lat;
  double lam = base_lon;

  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double sin_lam = sin(lam);
  double cos_lam = cos(lam);

  *enu_x = ((-sin_lam * (ECEF_x - base_ECEF_x)) + (cos_lam * (ECEF_y - base_ECEF_y)) + (0 * (ECEF_z - base_ECEF_z)));
  *enu_y = ((-sin_phi * cos_lam * (ECEF_x - base_ECEF_x)) + (-sin_phi*sin_lam * (ECEF_y - base_ECEF_y)) + (cos_phi * (ECEF_z - base_ECEF_z)));
  *enu_z = ((cos_phi * cos_lam * (ECEF_x - base_ECEF_x)) + (cos_phi*sin_lam * (ECEF_y - base_ECEF_y)) + (sin_phi * (ECEF_z - base_ECEF_z)));

}

void xyz2enu_vel(double Vel_x,double Vel_y,double Vel_z,double base_ECEF_x,double base_ECEF_y,double base_ECEF_z,double *Vel_e,double *Vel_n,double *Vel_u){

  //ECEF基準点をllhに変換
    double x = base_ECEF_x;
    double y = base_ECEF_y;
    double z = base_ECEF_z;
    double x2 = x*x ;
    double y2 = y*y ;
    double z2 = z*z ;
    double a = 6378137.0000;
    double b = 6356752.3142;
    double e = sqrt(1-((b/a)*(b/a)));
    double r = sqrt(x2+y2);
    double ep = e*(a/b);
    double b2 = b*b;
    double e2 = e*e;
    double r2 = r*r;
    double f = 54*b2*z2;
    double g = r2 + (1-e2)*z2 - e2*(a*a-b*b);
    double i = (e2*e2*f*r2)/(g*g*g);
    double o = pow(( 1 + i + sqrt(i*i + 2*i) ),1.0/3.0);
    double p = f / (3 * (o+1/o+1) * (o+1/o+1) * g*g);
    double q = sqrt(1+2*e2*e2*p);
    double s = -(p*e2*r)/(1+q) + sqrt((a*a/2)*(1+1/q) - (p*(1-e2)*z2)/(q*(1+q)) - p*r2/2);
    double tmp = (r - e2*s)*(r - e2*s);
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

  //Vel_xyzをVel_enuに変換
    double phi = base_lat;
    double lam = base_lon;

    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    double sin_lam = sin(lam);
    double cos_lam = cos(lam);

    *Vel_e = (-Vel_x * sin_lam) + (Vel_y * cos_lam);
    *Vel_n = (-Vel_x * cos_lam * sin_phi) - (Vel_y * sin_lam * sin_phi) + (Vel_z * cos_phi);
    *Vel_u = (Vel_x * cos_lam * cos_phi) + (Vel_y * sin_lam * cos_phi) + (Vel_z * sin_phi);

}

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

int main(int argc, char **argv){

  ros::init(argc, argv, "rtklib_connecter");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<imu_gnss_localizer::RTKLIB>("/RTKLIB", 1);
  ros::Publisher pub3 = n.advertise<sensor_msgs::NavSatFix>("/fix", 1000);///////////////////////////////

  struct sockaddr_in server;

  int sock;
  char recv_buf[256];
  int recv_packet_size;

  sock = socket(AF_INET, SOCK_STREAM, 0);

  server.sin_family = AF_INET;
  server.sin_port = htons(61111);
  server.sin_addr.s_addr = inet_addr("127.0.0.1");

  connect(sock, (struct sockaddr *)&server, sizeof(server));

  char data_buf[256];
  memset(data_buf, 0, sizeof(data_buf));

  while(ros::ok()){

    ros::spinOnce();

    memset(recv_buf, 0, sizeof(recv_buf));
    recv_packet_size = recv(sock, recv_buf, sizeof(recv_buf), 0);

    //ROS_INFO("packet_size:%d\nRAWdata:%s",recv_packet_size,recv_buf);

    if(recv_packet_size > 0){

      std::vector<int> LF_index;

      ++counter;

      for(i = 0; i < recv_packet_size; i++){
        if(recv_buf[i] == 0x0a){  //0x0a = LF
          LF_index.push_back(i);
          //ROS_INFO("%d",i);
          }
      }

      memset(data_buf,0,sizeof(data_buf));
      memcpy(data_buf,&recv_buf[5], 11);
      GPSTime = atof(data_buf)*1000; //unit[mm sec]
      //ROS_INFO("GPSTime=%d",GPSTime);

      memset(data_buf,0,sizeof(data_buf));
      memcpy(data_buf,&recv_buf[LF_index[1]], LF_index[1]-LF_index[0]);
      //ROS_INFO("data_buf=%s",data_buf);
      ECEF_x = atof(data_buf);
      //ROS_INFO("ECEF_x=%10.10lf",ECEF_x);

      memset(data_buf,0,sizeof(data_buf));
      memcpy(data_buf,&recv_buf[LF_index[2]], LF_index[2]-LF_index[1]);
      //ROS_INFO("data_buf=%s",data_buf);
      ECEF_y = atof(data_buf);
      //ROS_INFO("ECEF_y=%10.10lf",ECEF_y);

      memset(data_buf,0,sizeof(data_buf));
      memcpy(data_buf,&recv_buf[LF_index[3]], LF_index[3]-LF_index[2]);
      //ROS_INFO("data_buf=%s",data_buf);
      ECEF_z = atof(data_buf);
      //ROS_INFO("ECEF_z=%10.10lf",ECEF_z);

      memset(data_buf,0,sizeof(data_buf));
      memcpy(data_buf,&recv_buf[LF_index[4]], LF_index[4]-LF_index[3]);
      //ROS_INFO("data_buf=%s",data_buf);
      Vel_x = atof(data_buf);
      //ROS_INFO("Vel_x=%10.10lf",Vel_x);

      memset(data_buf,0,sizeof(data_buf));
      memcpy(data_buf,&recv_buf[LF_index[5]], LF_index[5]-LF_index[4]);
      //ROS_INFO("data_buf=%s",data_buf);
      Vel_y = atof(data_buf);
      //ROS_INFO("Vel_y=%10.10lf",Vel_y);

      memset(data_buf,0,sizeof(data_buf));
      memcpy(data_buf,&recv_buf[LF_index[6]], LF_index[6]-LF_index[5]);
      //ROS_INFO("data_buf=%s",data_buf);
      Vel_z = atof(data_buf);
      //ROS_INFO("Vel_z=%10.10lf",Vel_z);

      if(counter == 1){
      //enu変換用基準値点

      base_ECEF_x = ECEF_x;
      base_ECEF_y = ECEF_y;
      base_ECEF_z = ECEF_z;

      //base_ECEF_x = -3784315.720;
      //base_ECEF_y = 3516603.177;
      //base_ECEF_z = 3728191.676;

      }

      xyz2enu(ECEF_x,ECEF_y,ECEF_z,base_ECEF_x,base_ECEF_y,base_ECEF_z,&enu_x,&enu_y,&enu_z);
      xyz2enu_vel(Vel_x,Vel_y,Vel_z,base_ECEF_x,base_ECEF_y,base_ECEF_z,&Vel_e,&Vel_n,&Vel_u);

      imu_gnss_localizer::RTKLIB p_msg;
      p_msg.header.stamp = ros::Time::now();
      p_msg.header.frame_id = "map";
      p_msg.GPSTime = GPSTime;
      p_msg.enu_x = enu_x;
      p_msg.enu_y = enu_y;
      p_msg.enu_z = enu_z;
      p_msg.Vel_e = Vel_e;
      p_msg.Vel_n = Vel_n;
      p_msg.Vel_u = Vel_u;
      p_msg.ORG_x = base_ECEF_x;
      p_msg.ORG_y = base_ECEF_y;
      p_msg.ORG_z = base_ECEF_z;

      pub.publish(p_msg);

      enu2llh(enu_x,enu_y,enu_z,base_ECEF_x,base_ECEF_y,base_ECEF_z,&lon,&lat,&alt);

      sensor_msgs::NavSatFix p3_msg;

      p3_msg.header.stamp = ros::Time::now();
      p3_msg.header.frame_id = "map";
      p3_msg.latitude = lat;
      p3_msg.longitude = lon;
      p3_msg.altitude = alt;
      pub3.publish(p3_msg);

      ROS_INFO("%lf %lf",lat,lon);

    }
    else if(recv_packet_size == 0){
      ROS_WARN("RTKLIB has been disconnected");
      break;
    }
    else{
      ROS_WARN("RTKLIB is not started");
      break;
    }

  }

   close(sock);
   n.shutdown();

   return 0;
}
