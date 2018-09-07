#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geodetic_utils/geodetic_conv.hpp>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <chrono>

double H_latitude, H_longitude, H_altitude;
std::vector<bool> g_got_imu(3, false);
std::vector<sensor_msgs::Imu> g_latest_imu_msg(3);
std::vector<sensor_msgs::Range> height(3);
geodetic_converter::GeodeticConverter g_geodetic_converter;
std::vector<ros::Publisher> g_gps_position_pub;

void imu_callback(const sensor_msgs::ImuConstPtr& msg, int num)
{
  g_latest_imu_msg[num] = *msg;
  g_got_imu[num] = true;
}

void imu_callback_1(const sensor_msgs::ImuConstPtr& msg)
{
  imu_callback(msg, 0);
}

void imu_callback_2(const sensor_msgs::ImuConstPtr& msg)
{
  imu_callback(msg, 1);
}

void imu_callback_3(const sensor_msgs::ImuConstPtr& msg)
{
  imu_callback(msg, 2);
}

void dist_callback(const sensor_msgs::Range &msg, int num){
  height[num] = msg;
}

void dist_callback_1(const sensor_msgs::Range &msg){
  dist_callback(msg,0);
}

void dist_callback_2(const sensor_msgs::Range &msg){
  dist_callback(msg,1);
}

void dist_callback_3(const sensor_msgs::Range &msg){
  dist_callback(msg,2);
}

void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg, int num)
{
  //ROS_INFO_THROTTLE(1, "GOT GPS DATA");
  if (!g_got_imu[num]) {
    ROS_WARN_STREAM_THROTTLE(1, "No IMU data yet");
    //return;
  }

  if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    //return;
  }

  if (!g_geodetic_converter.isInitialised()) {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
    return;
  }

  double x, y, z;
  g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);

  geometry_msgs::PoseStamped position_msg;
  position_msg.header = msg->header;
  position_msg.header.frame_id = "world";
  position_msg.pose.position.x = x;
  position_msg.pose.position.y = y;
  position_msg.pose.position.z = height[num].range;
  position_msg.pose.orientation = g_latest_imu_msg[num].orientation;

  g_gps_position_pub[num].publish(position_msg); //local_position publisher

}

void gps_callback_1(const sensor_msgs::NavSatFixConstPtr& msg){
  gps_callback(msg,0);
}

void gps_callback_2(const sensor_msgs::NavSatFixConstPtr& msg){
  gps_callback(msg,1);
}

void gps_callback_3(const sensor_msgs::NavSatFixConstPtr& msg){
  gps_callback(msg,2);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "local_position_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //  double latitude, longitude, altitude;
  do {
    ROS_INFO("Waiting for GPS reference parameters...");
    if (nh.getParam("/gps_ref_latitude", H_latitude) &&
        nh.getParam("/gps_ref_longitude", H_longitude) &&
        nh.getParam("/gps_ref_altitude", H_altitude)) {
      g_geodetic_converter.initialiseReference(H_latitude, H_longitude, H_altitude);
    } else {
      ROS_INFO(
          "GPS reference not ready yet, use set_gps_reference_node to set it");
      ros::Duration(2.0).sleep(); // sleep for half a second
    }
  } while (!g_geodetic_converter.isInitialised() && ros::ok());

  std::vector<ros::Subscriber> imu_sub;
  std::vector<ros::Subscriber> gps_sub;
  std::vector<ros::Subscriber> dist_sub;

  g_gps_position_pub.push_back(nh.advertise<geometry_msgs::PoseStamped>("/uav1/local_position", 1));
  g_gps_position_pub.push_back(nh.advertise<geometry_msgs::PoseStamped>("/uav2/local_position", 1));
  g_gps_position_pub.push_back(nh.advertise<geometry_msgs::PoseStamped>("/uav3/local_position", 1));
  
  imu_sub.push_back(nh.subscribe("/uav1/mavros/imu/data", 1, &imu_callback_1));
  imu_sub.push_back(nh.subscribe("/uav2/mavros/imu/data", 1, &imu_callback_2));
  imu_sub.push_back(nh.subscribe("/uav3/mavros/imu/data", 1, &imu_callback_3));

  gps_sub.push_back(nh.subscribe("/uav1/mavros/global_position/global", 1, &gps_callback_1));
  gps_sub.push_back(nh.subscribe("/uav2/mavros/global_position/global", 1, &gps_callback_2));
  gps_sub.push_back(nh.subscribe("/uav3/mavros/global_position/global", 1, &gps_callback_3));

  dist_sub.push_back(nh.subscribe("/uav1/mavros/distance_sensor/lidarlite_pub", 1, &dist_callback_1));
  dist_sub.push_back(nh.subscribe("/uav2/mavros/distance_sensor/lidarlite_pub", 1, &dist_callback_2));
  dist_sub.push_back(nh.subscribe("/uav3/mavros/distance_sensor/lidarlite_pub", 1, &dist_callback_3));
    
  // this continually pulls new home params if there is a new is set
  while (ros::ok()){
    double latitude, longitude, altitude;
    nh.getParam("/gps_ref_latitude", latitude);
    nh.getParam("/gps_ref_longitude", longitude);
    
    if (latitude != H_latitude || longitude != H_longitude){
      g_geodetic_converter.initialiseReference(latitude, longitude, H_altitude);
      ROS_WARN("[geo] got home data, reseting target");
      H_latitude = latitude;
      H_longitude = longitude;
    }
    ros::spinOnce();
  }

}
