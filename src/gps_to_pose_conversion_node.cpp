/*
  Use altitude from 'external_altitude' topic if messages are received
  (To enable the messages arriving, publish to the topic by remapping in the launch file
  Otherwise, altitude from GPS is taken
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geodetic_utils/geodetic_conv.hpp>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <chrono>



bool g_is_sim;
bool g_publish_pose;
sensor_msgs::Range height;
bool newHeightData = false;

geodetic_converter::GeodeticConverter g_geodetic_converter;
sensor_msgs::Imu g_latest_imu_msg;
std_msgs::Float64 g_latest_altitude_msg;
bool g_got_imu;
bool g_got_altitude;
// std::mt19937 generator;
// std::normal_distribution<double> Gsampler;

ros::Publisher g_gps_pose_pub;
ros::Publisher g_gps_transform_pub;
ros::Publisher g_gps_position_pub;
;

bool g_trust_gps;
// bool add_noise;
// double noiseCov;
// double noiseX;
// double noiseY;
// double noiseZ;

double H_latitude, H_longitude, H_altitude;

double g_covariance_position_x;
double g_covariance_position_y;
double g_covariance_position_z;
double g_covariance_orientation_x;
double g_covariance_orientation_y;
double g_covariance_orientation_z;
std::string g_frame_id;
std::string g_tf_child_frame_id;

std::shared_ptr<tf::TransformBroadcaster> p_tf_broadcaster;

// enif_iuc::AgentHome homeRef;

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  g_latest_imu_msg = *msg;
  g_got_imu = true;
}

void altitude_callback(const std_msgs::Float64ConstPtr& msg)
{
  // Only the z value in the PointStamped message is used
  g_latest_altitude_msg = *msg;
  g_got_altitude = true;
}

// void home_callback(const enif_iuc::AgentHome &msg){
  // homeRef = msg;  
// }

void dist_callback(const sensor_msgs::Range &msg){
  height = msg;
  newHeightData = true;
  ROS_INFO_ONCE("using external height in geodetic node!");
}

void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  
  if (!g_got_imu) {
    ROS_WARN_STREAM_THROTTLE(1, "No IMU data yet");
    return;
  }

  if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
    return;
  }

  if (!g_geodetic_converter.isInitialised()) {
    ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
    return;
  }



  double x, y, z;
  g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);

  // (NWU -> ENU) for simulation
  if (g_is_sim) {
    double aux = x;
    x = y;
    y = -aux;
    //z = z;
  }

  // if (add_noise)
  //   {
  //   noiseX = Gsampler(generator);
  //   noiseY = Gsampler(generator);
  //   noiseZ = Gsampler(generator);
  //   }
  // else
  //   {
  //     noiseX = 0;
  //     noiseY = 0;
  //     noiseZ = 0;
  //   }
  
  // Fill up pose message
  geometry_msgs::PoseWithCovarianceStampedPtr pose_msg(
      new geometry_msgs::PoseWithCovarianceStamped);
  pose_msg->header = msg->header;
  pose_msg->header.frame_id = g_frame_id;
  pose_msg->pose.pose.position.x = x;
  pose_msg->pose.pose.position.y = y;
  pose_msg->pose.pose.position.z = z;
  pose_msg->pose.pose.orientation = g_latest_imu_msg.orientation;

  // Fill up position message
  geometry_msgs::PointStampedPtr position_msg(
    new geometry_msgs::PointStamped);
  position_msg->header = pose_msg->header;
  position_msg->header.frame_id = g_frame_id;
  position_msg->point = pose_msg->pose.pose.position;

  // If external altitude messages received, include in pose and position messages
  if (g_got_altitude) {
    pose_msg->pose.pose.position.z = g_latest_altitude_msg.data;
    position_msg->point.z = g_latest_altitude_msg.data;
  }
  else if (newHeightData){
    pose_msg->pose.pose.position.z = height.range;
    position_msg->point.z = height.range;
  }

  pose_msg->pose.covariance.assign(0);

  // Set default covariances
  pose_msg->pose.covariance[6 * 0 + 0] = g_covariance_position_x;
  pose_msg->pose.covariance[6 * 1 + 1] = g_covariance_position_y;
  pose_msg->pose.covariance[6 * 2 + 2] = g_covariance_position_z;
  pose_msg->pose.covariance[6 * 3 + 3] = g_covariance_orientation_x;
  pose_msg->pose.covariance[6 * 4 + 4] = g_covariance_orientation_y;
  pose_msg->pose.covariance[6 * 5 + 5] = g_covariance_orientation_z;

  // Take covariances from GPS
  if (g_trust_gps) {
    if (msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN
        || msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {
      // Fill in completely
      for (int i = 0; i <= 2; i++) {
        for (int j = 0; j <= 2; j++) {
          pose_msg->pose.covariance[6 * i + j] = msg->position_covariance[3 * i + j];
        }
      }
    } else if (msg->position_covariance_type
        == sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
      // Only fill in diagonal
      for (int i = 0; i <= 2; i++) {
        pose_msg->pose.covariance[6 * i + i] = msg->position_covariance[3 * i + i];
      }
    }
  }


  
  if (g_publish_pose) {
    g_gps_pose_pub.publish(pose_msg);
    
  }
  g_gps_position_pub.publish(position_msg);

  // Fill up transform message
  geometry_msgs::TransformStampedPtr transform_msg(
      new geometry_msgs::TransformStamped);
  transform_msg->header = msg->header;
  transform_msg->header.frame_id = g_frame_id;
  transform_msg->transform.translation.x = x;
  transform_msg->transform.translation.y = y;
  transform_msg->transform.translation.z = z;
  transform_msg->transform.rotation = g_latest_imu_msg.orientation;

  if (g_got_altitude) {
    transform_msg->transform.translation.z = g_latest_altitude_msg.data;
  }

  g_gps_transform_pub.publish(transform_msg);

  // Fill up TF broadcaster
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, z));
  transform.setRotation(tf::Quaternion(g_latest_imu_msg.orientation.x,
                                       g_latest_imu_msg.orientation.y,
                                       g_latest_imu_msg.orientation.z,
                                       g_latest_imu_msg.orientation.w));
  p_tf_broadcaster->sendTransform(tf::StampedTransform(transform,
                                                       ros::Time::now(),
                                                       g_frame_id,
                                                       g_tf_child_frame_id));
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "gps_to_pose_conversion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  // ros::param::param("~add_noise", add_noise, true);
  // ros::param::param("~noiseCov", noiseCov, 1.0);
  
  // Gsampler = std::normal_distribution<double> (0.0,noiseCov);
  // unsigned seed_time = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  // generator.seed(seed_time);
    
  g_got_imu = false;
  g_got_altitude = false;
  p_tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();

  // Use different coordinate transform if using simulator
  if (!pnh.getParam("is_sim", g_is_sim)) {
    ROS_WARN("Could not fetch 'sim' param, defaulting to 'false'");
    g_is_sim = false;
  }

  // FIXME: if parameters not found and using defaults, throw a ROS_WARN

  // Specify whether covariances should be set manually or from GPS
  ros::param::param("~trust_gps", g_trust_gps, false);

  // Get manual parameters
  ros::param::param("~fixed_covariance/position/x", g_covariance_position_x,
                    4.0);
  ros::param::param("~fixed_covariance/position/y", g_covariance_position_y,
                    4.0);
  ros::param::param("~fixed_covariance/position/z", g_covariance_position_z,
                    100.0);
  ros::param::param("~fixed_covariance/orientation/x",
                    g_covariance_orientation_x, 0.02);
  ros::param::param("~fixed_covariance/orientation/y",
                    g_covariance_orientation_y, 0.02);
  ros::param::param("~fixed_covariance/orientation/z",
                    g_covariance_orientation_z, 0.11);
  ros::param::param<std::string>("~frame_id",
                                 g_frame_id, "world");
  ros::param::param<std::string>("~tf_child_frame_id",
                                 g_tf_child_frame_id, "gps_receiver");

  // Specify whether to publish pose or not
  ros::param::param("~publish_pose", g_publish_pose, true);

  std::cout<<"publish pose: "<<g_publish_pose<<std::endl;    

  // Wait until GPS reference parameters are initialized.
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

  // Show reference point
  double initial_latitude, initial_longitude, initial_altitude;
  g_geodetic_converter.getReference(&initial_latitude, &initial_longitude,
                                    &initial_altitude);
  ROS_INFO("GPS reference initialized correctly %f, %f, %f", initial_latitude,
           initial_longitude, initial_altitude);

  // Initialize publishers
  g_gps_pose_pub =
    nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps_pose", 1);
  g_gps_transform_pub =
    nh.advertise<geometry_msgs::TransformStamped>("gps_transform", 1);
  g_gps_position_pub =
    nh.advertise<geometry_msgs::PointStamped>("gps_position", 1);

  std::string agentName = ros::this_node::getNamespace();
  agentName.erase(0,1);

  // Subscribe to IMU and GPS fixes, and convert in GPS callback
  ros::Subscriber imu_sub = nh.subscribe(agentName+"/mavros/imu/data", 1, &imu_callback);
  ros::Subscriber gps_sub = nh.subscribe(agentName+"/mavros/global_position/global", 1, &gps_callback);
  ros::Subscriber dist_sub = nh.subscribe(agentName+"/mavros/distance_sensor/lidarlite_pub", 1, &dist_callback);
  // ros::Subscriber home_sub = nh.subscribe("agent_home_data", 1, &home_callback);
  ros::Subscriber altitude_sub =
    nh.subscribe("external_altitude", 1, &altitude_callback);


  // this continually pulls new home params if there is a new is set
  while (ros::ok()){
    double latitude, longitude, altitude;
    nh.getParam("/gps_ref_latitude", latitude);
    nh.getParam("/gps_ref_longitude", longitude);
    nh.getParam("/gps_ref_altitude", altitude);

    //std::cout<<"lat: "<<latitude<<", "<<H_latitude<<", long: "<<longitude<<", "<<H_longitude<<std::endl;
    
    if (latitude != H_latitude || longitude != H_longitude){
      g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
      ROS_WARN("[geo] got home data, reseting target");
      H_latitude = latitude;
      H_longitude = longitude;
    }
    ros::spinOnce();
  }

}
