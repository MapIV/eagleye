#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static ros::Publisher pub1,pub2;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::Distance distance;
static sensor_msgs::Imu imu;
static geometry_msgs::PoseStamped localization_pose;

static eagleye_msgs::Rolling rolling_angle;
static eagleye_msgs::AccYOffset imu_lateral_acc_offset;

struct RollangleParameterUsingInputPose rollangle_parameter_using_input_pose;
struct RollangleStatusUsingInputPose roll_angle_status;

void  velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr &msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
}

void  yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr &msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void  yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr &msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

void  localization_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  localization_pose.header =  msg->header;
  localization_pose.pose.position =  msg->pose.position;
  localization_pose.pose.orientation = msg->pose.orientation;
}

void  imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  imu_lateral_acc_offset.header = msg->header;
  imu_lateral_acc_offset.header.frame_id = "imu";
  rolling_angle.header = msg->header;
  rolling_angle.header.frame_id = "imu";
  rolling_estimate_using_input_pose(velocity_scale_factor,yawrate_offset_2nd,yawrate_offset_stop,distance,imu,localization_pose,rollangle_parameter_using_input_pose,&roll_angle_status,&rolling_angle,&imu_lateral_acc_offset);
  pub1.publish(imu_lateral_acc_offset);
  pub2.publish(rolling_angle);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roll_angle_using_input_pose");
  ros::NodeHandle n;

  std::string subscribe_imu_topic_name = "/sensing/imu/imu_data";
  std::string subscribe_localization_pose_topic_name = "/localization/pose_estimator/pose";

  n.getParam("localization_pose_topic",subscribe_localization_pose_topic_name);
  n.getParam("imu_topic",subscribe_imu_topic_name);
  n.getParam("reverse_imu",rollangle_parameter_using_input_pose.reverse_imu);
  n.getParam("reverse_imu_angular_velocity_x",rollangle_parameter_using_input_pose.reverse_imu_angular_velocity_x);
  n.getParam("reverse_imu_linear_acceleration_y",rollangle_parameter_using_input_pose.reverse_imu_linear_acceleration_y);
  n.getParam("rollangle/link_Time_stamp_parameter",rollangle_parameter_using_input_pose.link_Time_stamp_parameter);
  n.getParam("rollangle/matching_update_distance",rollangle_parameter_using_input_pose.matching_update_distance);
  n.getParam("rollangle/imu_buffer_num",rollangle_parameter_using_input_pose.imu_buffer_num);
  n.getParam("rollangle/rolling_buffer_num",rollangle_parameter_using_input_pose.rolling_buffer_num);
  n.getParam("rollangle/stop_judgment_velocity_threshold",rollangle_parameter_using_input_pose.stop_judgment_velocity_threshold);

  std::cout<< "subscribe_localization_pose_topic_name "<<subscribe_localization_pose_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "reverse_imu "<<rollangle_parameter_using_input_pose.reverse_imu<<std::endl;
  std::cout<< "matching_update_distance" <<rollangle_parameter_using_input_pose.matching_update_distance<<std::endl;
  std::cout<< "imu_buffer_num" <<rollangle_parameter_using_input_pose.imu_buffer_num<<std::endl;
  std::cout<< "rolling_buffer_num" <<rollangle_parameter_using_input_pose.rolling_buffer_num<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold" <<rollangle_parameter_using_input_pose.stop_judgment_velocity_threshold<<std::endl;

  ros::Subscriber sub1 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("distance", 1000, distance_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe(subscribe_localization_pose_topic_name, 1000, localization_pose_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback , ros::TransportHints().tcpNoDelay());

  pub1 = n.advertise<eagleye_msgs::AccYOffset>("imu_lateral_acc_offset_using_input_pose", 1000);
  pub2 = n.advertise<eagleye_msgs::Rolling>("rolling_angle_using_input_pose", 1000);
 
  ros::spin();

  return 0;
}
