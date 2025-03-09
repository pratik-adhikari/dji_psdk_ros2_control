/**
 * @file gimbal_transform_node.cpp
 * @brief Example of a ROS2 node that uses TF2, parameters.yaml, and transform_config.yaml
 *        to publish a gimbal center position and broadcast dynamic TF frames.
 */

 #include "geogimbal/gimbal_transform_node.hpp"
 #include <ament_index_cpp/get_package_share_directory.hpp>
 
 // TF2
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2/LinearMath/Matrix3x3.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 #include <tf2_ros/transform_broadcaster.h>
 
 // YAML & Eigen
 #include <yaml-cpp/yaml.h>
 #include <Eigen/Dense>
 
 #include <stdexcept>
 #include <cmath>
 #include <fstream>
 #include <sstream>
 
 /* -------------------------------------------------------------------------
  * Constructor
  * ------------------------------------------------------------------------- */
 GimbalTransformNode::GimbalTransformNode(const rclcpp::NodeOptions &options)
   : Node("gimbal_transform_node", options)
 {
   RCLCPP_INFO(get_logger(), "GimbalTransformNode constructor called.");
 
   // 1) Load parameters from parameters.yaml
   loadParameters();
 
   // 2) If apply_gimbal_transform_==true, parse transform_config.yaml for DH
   if (apply_gimbal_transform_) {
     loadDhParameters();
   }
 
   // 3) Setup coordinate transform logic
   setupTransformer();
 
   // 4) Initialize angles
   imu_roll_ = imu_pitch_ = imu_yaw_ = 0.0;
   yaw_angle_ = roll_angle_ = pitch_angle_ = 0.0;  // If you have gimbal angles
 
   // 5) Initialize UAV position
   uav_x_ = uav_y_ = uav_z_ = 0.0;
 
   // 6) Create TF broadcaster
   tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
 
   // 7) Setup subscribers & publisher
   setupSubscriptionsAndPublisher();
 
   RCLCPP_INFO(get_logger(), "GimbalTransformNode fully initialized.");
 }
 
 /* -------------------------------------------------------------------------
  * loadParameters()
  * ------------------------------------------------------------------------- */
 void GimbalTransformNode::loadParameters()
 {
   // Reading from "gimbal_transform_node.ros__parameters" scope in parameters.yaml
   gps_position_topic_  = declare_parameter<std::string>("gps_position_topic", "/gps");
   imu_topic_           = declare_parameter<std::string>("imu_topic", "/imu");
   gimbal_center_topic_ = declare_parameter<std::string>("gimbal_center_topic", "/gimbal_center_position");
 
   // Use "UTM" or "LOCAL"
   coordinate_frame_    = declare_parameter<std::string>("coordinate_frame", "UTM");
   utm_zone_            = declare_parameter<int>("utm_zone", 32);
   northern_hemisphere_ = declare_parameter<bool>("northern_hemisphere", true);
 
   base_lat_ = declare_parameter<double>("base_lat", 0.0);
   base_lon_ = declare_parameter<double>("base_lon", 0.0);
   base_alt_ = declare_parameter<double>("base_alt", 0.0);
 
   // Whether to apply DH chain
   apply_gimbal_transform_ = declare_parameter<bool>("apply_gimbal_transform", true);
 
   // TF frames
   map_frame_id_      = declare_parameter<std::string>("map_frame_id", "map");
   uav_base_frame_id_ = declare_parameter<std::string>("uav_base_frame_id", "uav_base_link");
   imu_frame_id_      = declare_parameter<std::string>("imu_frame_id", "imu_link");
   gimbal_frame_id_   = declare_parameter<std::string>("gimbal_frame_id", "gimbal_link");
 
   RCLCPP_INFO(get_logger(), "Parameters loaded successfully.");
 }
 
 /* -------------------------------------------------------------------------
  * loadDhParameters()
  * ------------------------------------------------------------------------- */
 void GimbalTransformNode::loadDhParameters()
 {
   // 1) Retrieve package share directory (assuming "geogimbal" package)
   std::string share_dir = ament_index_cpp::get_package_share_directory("geogimbal");
   std::string yaml_path = share_dir + "/config/transform_config.yaml";
 
   // 2) Parse via YAML-CPP
   YAML::Node config = YAML::LoadFile(yaml_path);
 
   if (!config["dh_parameters"]) {
     RCLCPP_FATAL(get_logger(), "No 'dh_parameters' section in transform_config.yaml!");
     throw std::runtime_error("Missing 'dh_parameters' in transform_config.yaml");
   }
 
   // 3) Store them
   for (auto node : config["dh_parameters"])
   {
     DHLink link;
     link.joint_name = node["joint_name"].as<std::string>();
     link.a          = node["a"].as<double>();
     link.alpha      = node["alpha"].as<double>();
     link.d          = node["d"].as<double>();
     link.theta      = node["theta"].as<double>(); // default angle
     dh_links_.push_back(link);
   }
 
   RCLCPP_INFO(get_logger(), "Loaded %zu DH links from transform_config.yaml", dh_links_.size());
 }
 
 /* -------------------------------------------------------------------------
  * setupTransformer()
  * ------------------------------------------------------------------------- */
 void GimbalTransformNode::setupTransformer()
 {
   // For demonstration, we just log. 
   // A real system might create a PROJ-based object if you needed accurate transforms.
   if (coordinate_frame_ == "UTM") {
     RCLCPP_INFO(get_logger(), "Using UTM mode (zone=%d, northern=%s).", 
                 utm_zone_, (northern_hemisphere_ ? "true" : "false"));
   } else if (coordinate_frame_ == "LOCAL") {
     RCLCPP_INFO(get_logger(), "Using LOCAL mode with base lat=%.5f, lon=%.5f, alt=%.2f",
                 base_lat_, base_lon_, base_alt_);
   } else {
     RCLCPP_FATAL(get_logger(), "coordinate_frame must be 'UTM' or 'LOCAL'");
     throw std::runtime_error("Invalid coordinate_frame param");
   }
 }
 
 /* -------------------------------------------------------------------------
  * setupSubscriptionsAndPublisher()
  * ------------------------------------------------------------------------- */
 void GimbalTransformNode::setupSubscriptionsAndPublisher()
 {
   // GPS subscription
   gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
     gps_position_topic_, 
     10,
     std::bind(&GimbalTransformNode::gpsCallback, this, std::placeholders::_1)
   );
 
   // IMU subscription
   imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
     imu_topic_, 
     10,
     std::bind(&GimbalTransformNode::imuCallback, this, std::placeholders::_1)
   );
 
   // Publisher for gimbal center
   gimbal_pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
     gimbal_center_topic_, 10
   );
 
   RCLCPP_INFO(get_logger(), 
     "Subscribed to GPS:[%s], IMU:[%s]. Publishing gimbal center:[%s]", 
     gps_position_topic_.c_str(),
     imu_topic_.c_str(),
     gimbal_center_topic_.c_str());
 }
 
 /* -------------------------------------------------------------------------
  * imuCallback()
  * ------------------------------------------------------------------------- */
 void GimbalTransformNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
 {
   // Convert IMU quaternion -> RPY (radians)
   tf2::Quaternion q(
     msg->orientation.x,
     msg->orientation.y,
     msg->orientation.z,
     msg->orientation.w
   );
   tf2::Matrix3x3 m(q);
   double r, p, y;
   m.getRPY(r, p, y);
 
   // Convert to degrees for your DH usage
   imu_roll_  = r * 180.0 / M_PI;
   imu_pitch_ = p * 180.0 / M_PI;
   imu_yaw_   = y * 180.0 / M_PI;
 
   // ---------------------------------------------------
   // 1) Broadcast TF: uav_base_link -> imu_link
   // ---------------------------------------------------
   {
     geometry_msgs::msg::TransformStamped ts;
     ts.header.stamp = now();
     ts.header.frame_id = uav_base_frame_id_; // "uav_base_link"
     ts.child_frame_id = imu_frame_id_;       // "imu_link"
 
     // If your IMU is physically offset, put values here
     ts.transform.translation.x = 0.0;
     ts.transform.translation.y = 0.0;
     ts.transform.translation.z = 0.0;
 
     // Recreate quaternion from the RPY in radians
     tf2::Quaternion imu_q;
     imu_q.setRPY(r, p, y);
     ts.transform.rotation = tf2::toMsg(imu_q);
 
     tf_broadcaster_->sendTransform(ts);
   }
 
   // ---------------------------------------------------
   // 2) Broadcast TF: imu_link -> gimbal_link
   // ---------------------------------------------------
   {
     geometry_msgs::msg::TransformStamped ts;
     ts.header.stamp = now();
     ts.header.frame_id = imu_frame_id_;     // "imu_link"
     ts.child_frame_id = gimbal_frame_id_;   // "gimbal_link"
 
     ts.transform.translation.x = 0.0;
     ts.transform.translation.y = 0.0;
     ts.transform.translation.z = 0.0;
 
     // If you want to incorporate roll_angle_, pitch_angle_, yaw_angle_ from your gimbal,
     // set them here. For example:
     tf2::Quaternion gimbal_q;
     // gimbal_q.setRPY(
     //   roll_angle_  * M_PI / 180.0,
     //   pitch_angle_ * M_PI / 180.0,
     //   yaw_angle_   * M_PI / 180.0
     // );
     // For now, we'll just keep it neutral:
     gimbal_q.setRPY(0, 0, 0);
 
     ts.transform.rotation = tf2::toMsg(gimbal_q);
     tf_broadcaster_->sendTransform(ts);
   }
 }
 
 /* -------------------------------------------------------------------------
  * gpsCallback()
  * ------------------------------------------------------------------------- */
 void GimbalTransformNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
 {
   // 1) Validate
   if (!std::isfinite(msg->latitude) || 
       !std::isfinite(msg->longitude) || 
       !std::isfinite(msg->altitude))
   {
     RCLCPP_WARN(get_logger(), "GPS data invalid; skipping");
     return;
   }
 
   // 2) Convert WGS84 -> local or UTM
   Coordinate c = wgs84ToFrame(msg->latitude, msg->longitude, msg->altitude);
   uav_x_ = c.x; 
   uav_y_ = c.y; 
   uav_z_ = c.z;
 
   // ---------------------------------------------------
   // 3) Broadcast TF: map -> uav_base_link
   // ---------------------------------------------------
   {
     geometry_msgs::msg::TransformStamped ts;
     ts.header.stamp = now();
     ts.header.frame_id = map_frame_id_;       // "map"
     ts.child_frame_id  = uav_base_frame_id_;  // "uav_base_link"
 
     ts.transform.translation.x = uav_x_;
     ts.transform.translation.y = uav_y_;
     ts.transform.translation.z = uav_z_;
 
     // If you have no heading in GPS, set orientation to identity
     tf2::Quaternion q_ident(0, 0, 0, 1);
     ts.transform.rotation = tf2::toMsg(q_ident);
 
     tf_broadcaster_->sendTransform(ts);
   }
 
   // ---------------------------------------------------
   // 4) Publish final "gimbal center" as a PointStamped
   // ---------------------------------------------------
   // (If apply_gimbal_transform_ is false, we publish raw coords.)
   if (!apply_gimbal_transform_)
   {
     geometry_msgs::msg::PointStamped ps;
     ps.header.stamp = now();
     ps.header.frame_id = map_frame_id_;
     ps.point.x = c.x;
     ps.point.y = c.y;
     ps.point.z = c.z;
 
     gimbal_pos_pub_->publish(ps);
     return;
   }
 
   // ---------------------------------------------------
   // 5) Otherwise, apply the DH chain
   // ---------------------------------------------------
   Eigen::Matrix4d chain = computeDhChain();
   Eigen::Vector4d in_pt(c.x, c.y, c.z, 1.0);
   Eigen::Vector4d out_pt = chain * in_pt;
 
   // 6) Publish the final point
   geometry_msgs::msg::PointStamped out_msg;
   out_msg.header.stamp = now();
   out_msg.header.frame_id = map_frame_id_;
   out_msg.point.x = out_pt(0);
   out_msg.point.y = out_pt(1);
   out_msg.point.z = out_pt(2);
 
   gimbal_pos_pub_->publish(out_msg);
 }
 
 /* -------------------------------------------------------------------------
  * wgs84ToFrame()
  * ------------------------------------------------------------------------- */
 Coordinate GimbalTransformNode::wgs84ToFrame(double lat, double lon, double alt)
 {
   // Naive example: 1 degree ~ 111319.9 m at Equator.
   // Replace with real code if needed (e.g. PROJ for UTM).
   double scale = 111319.9;
   Coordinate c;
 
   if (coordinate_frame_ == "UTM")
   {
     // Very rough: (lon -> x, lat -> y)
     c.x = lon * scale;
     c.y = lat * scale;
     c.z = alt;
   }
   else
   {
     // LOCAL => offset from base_lat_, base_lon_
     c.x = (lon - base_lon_) * scale;
     c.y = (lat - base_lat_) * scale;
     c.z = alt - base_alt_;
   }
 
   return c;
 }
 
 /* -------------------------------------------------------------------------
  * computeDhChain()
  * ------------------------------------------------------------------------- */
 Eigen::Matrix4d GimbalTransformNode::computeDhChain()
 {
   Eigen::Matrix4d full = Eigen::Matrix4d::Identity();
 
   for (auto &link : dh_links_)
   {
     // Optionally update link.theta based on your naming scheme:
     if (link.joint_name == "UAV_Center_to_IMU_Z") {
       link.theta = imu_yaw_;   // example
     } else if (link.joint_name == "IMU_Z_to_IMU_Y") {
       link.theta = imu_pitch_;
     } else if (link.joint_name == "IMU_Y_to_IMU_X") {
       link.theta = imu_roll_;
     }
     // etc. for gimbal angles if you have them:
     // else if (link.joint_name == "Gimbal_offset_to_Gimbal_Yaw") {
     //   link.theta = yaw_angle_;
     // }
 
     full = full * dhToMatrix(link.a, link.alpha, link.d, link.theta);
   }
 
   return full;
 }
 
 /* -------------------------------------------------------------------------
  * dhToMatrix()
  * ------------------------------------------------------------------------- */
 Eigen::Matrix4d GimbalTransformNode::dhToMatrix(double a, double alpha, double d, double theta)
 {
   // Convert degrees -> radians
   double alpha_r = alpha * M_PI / 180.0;
   double theta_r = theta * M_PI / 180.0;
 
   double ca = cos(alpha_r);
   double sa = sin(alpha_r);
   double ct = cos(theta_r);
   double st = sin(theta_r);
 
   Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
 
   // Standard or Modified DH
   T(0,0) = ct;        T(0,1) = -st * ca;  T(0,2) = st * sa;   T(0,3) = a * ct;
   T(1,0) = st;        T(1,1) = ct * ca;   T(1,2) = -ct * sa;  T(1,3) = a * st;
   T(2,1) = sa;        T(2,2) = ca;        T(2,3) = d;
 
   return T;
 }
 