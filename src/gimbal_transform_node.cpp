/**
 * @file gimbal_transform_node.cpp
 * @brief Implementation of GimbalTransformNode, a ROS2 node that transforms UAV GPS (WGS84) to UTM (or LOCAL),
 *        then optionally applies a Denavit–Hartenberg chain to produce the gimbal center position.
 */

 #include "geogimbal/gimbal_transform_node.hpp"
 #include <ament_index_cpp/get_package_share_directory.hpp>
 // ROS2 / TF includes
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2/LinearMath/Matrix3x3.h>
 
 // YAML and Eigen
 #include <yaml-cpp/yaml.h>
 #include <Eigen/Dense>
 
 #include <stdexcept>
 #include <cmath>
 #include <fstream>
 #include <sstream>
 #include <limits>
 
 /* ---------------------------------------------------------------------
  * GimbalTransformNode CONSTRUCTOR
  * --------------------------------------------------------------------- */
 GimbalTransformNode::GimbalTransformNode(const rclcpp::NodeOptions &options)
     : Node("gimbal_transform_node", options)
 {
     RCLCPP_INFO(get_logger(), "GimbalTransformNode constructor called.");
 
     // Load required parameters from parameters.yaml
     loadParameters();
 
     // Load DH chain from transform_config.yaml (if we want to transform)
     loadDhParameters();
 
     // Initialize coordinate transform logic
     setupTransformer();
 
     // Setup GPS/IMU subscriptions & gimbal center publisher
     setupSubscriptionsAndPublisher();
 
     RCLCPP_INFO(get_logger(), "GimbalTransformNode fully initialized.");
 }
 
 /* ---------------------------------------------------------------------
  * loadParameters()
  * --------------------------------------------------------------------- */
 void GimbalTransformNode::loadParameters()
 {
     gps_position_topic_ = declare_parameter<std::string>("gps_position_topic", "");
     imu_topic_          = declare_parameter<std::string>("imu_topic", "");
     gimbal_center_topic_= declare_parameter<std::string>("gimbal_center_topic", "");
 
     if (gps_position_topic_.empty() || imu_topic_.empty() || gimbal_center_topic_.empty())
     {
         RCLCPP_FATAL(get_logger(),
             "Missing mandatory topic parameters in parameters.yaml!");
         throw std::runtime_error("Missing required parameters for topics.");
     }
 
     // Coordinate frame config
     coordinate_frame_      = declare_parameter<std::string>("coordinate_frame", "UTM");
     utm_zone_              = declare_parameter<int>("utm_zone", 32);
     northern_hemisphere_   = declare_parameter<bool>("northern_hemisphere", true);
 
     base_lat_ = declare_parameter<double>("base_lat", 0.0);
     base_lon_ = declare_parameter<double>("base_lon", 0.0);
     base_alt_ = declare_parameter<double>("base_alt", 0.0);
 
     // Whether to apply the DH transform or just publish raw local coords
     apply_gimbal_transform_ = declare_parameter<bool>("apply_gimbal_transform", true);
 
     RCLCPP_INFO(get_logger(),
         "apply_gimbal_transform = %s",
         (apply_gimbal_transform_ ? "TRUE" : "FALSE"));
 
     // Initialize angles to zero
     imu_roll_ = imu_pitch_ = imu_yaw_ = 0.0;
     yaw_angle_ = roll_angle_ = pitch_angle_ = 0.0;
 }
 
 /* ---------------------------------------------------------------------
  * loadDhParameters()
  * --------------------------------------------------------------------- */
void GimbalTransformNode::loadDhParameters()
{
    RCLCPP_INFO(get_logger(), "Loading DH parameters from transform_config.yaml...");

    // 1) Find our package's share directory
    std::string share_dir = ament_index_cpp::get_package_share_directory("geogimbal");

    // 2) Construct the absolute path
    std::string yaml_path = share_dir + "/config/transform_config.yaml";

    // 3) Load it via YAML-CPP
    YAML::Node config = YAML::LoadFile(yaml_path);

    if (!config["dh_parameters"])
    {
        RCLCPP_FATAL(get_logger(),
            "No 'dh_parameters' in %s!", yaml_path.c_str());
        throw std::runtime_error("Missing 'dh_parameters' in transform_config.yaml.");
    }

    // 4) Parse
    for (auto node : config["dh_parameters"])
    {
        DHLink link;
        link.joint_name = node["joint_name"].as<std::string>();
        link.a          = node["a"].as<double>();
        link.alpha      = node["alpha"].as<double>();
        link.d          = node["d"].as<double>();
        link.theta      = node["theta"].as<double>(); 
        dh_links_.push_back(link);

        RCLCPP_DEBUG(get_logger(),
            "Loaded link: %s, a=%.2f, alpha=%.2f, d=%.2f, theta=%.2f",
            link.joint_name.c_str(), link.a, link.alpha, link.d, link.theta);
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu DH links from %s.", dh_links_.size(), yaml_path.c_str());
} 
 
 /* ---------------------------------------------------------------------
  * setupTransformer()
  * --------------------------------------------------------------------- */
 void GimbalTransformNode::setupTransformer()
 {
     if (coordinate_frame_ == "UTM")
     {
         RCLCPP_INFO(get_logger(),
             "Using UTM transform (zone=%d, north=%s).",
             utm_zone_,
             (northern_hemisphere_ ? "true" : "false"));
     }
     else if (coordinate_frame_ == "LOCAL")
     {
         RCLCPP_INFO(get_logger(),
             "Using LOCAL transform with base lat=%.4f, lon=%.4f, alt=%.1f.",
             base_lat_, base_lon_, base_alt_);
     }
     else
     {
         RCLCPP_FATAL(get_logger(), "coordinate_frame must be 'UTM' or 'LOCAL', got: %s",
                      coordinate_frame_.c_str());
         throw std::runtime_error("Invalid coordinate_frame param");
     }
 }
 
 /* ---------------------------------------------------------------------
  * setupSubscriptionsAndPublisher()
  * --------------------------------------------------------------------- */
 void GimbalTransformNode::setupSubscriptionsAndPublisher()
 {
     // Subscribe to GPS
     gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
         gps_position_topic_, 10,
         std::bind(&GimbalTransformNode::gpsCallback, this, std::placeholders::_1)
     );
 
     // Subscribe to IMU
     imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
         imu_topic_, 10,
         std::bind(&GimbalTransformNode::imuCallback, this, std::placeholders::_1)
     );
 
     // Publisher for the final position
     gimbal_pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
         gimbal_center_topic_, 10
     );
 
     RCLCPP_INFO(get_logger(),
         "Subscribed to [%s, %s], publishing gimbal center to [%s].",
         gps_position_topic_.c_str(), imu_topic_.c_str(), gimbal_center_topic_.c_str());
 }
 
 /* ---------------------------------------------------------------------
  * imuCallback()
  * --------------------------------------------------------------------- */
 void GimbalTransformNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
 {
     // Convert quaternion to RPY
     tf2::Quaternion q(
         msg->orientation.x,
         msg->orientation.y,
         msg->orientation.z,
         msg->orientation.w
     );
     tf2::Matrix3x3 m(q);
 
     double roll, pitch, yaw; 
     m.getRPY(roll, pitch, yaw); // in radians
 
     // Convert to degrees for the DH chain
     imu_roll_  = roll  * 180.0 / M_PI;
     imu_pitch_ = pitch * 180.0 / M_PI;
     imu_yaw_   = yaw   * 180.0 / M_PI;
 
     RCLCPP_DEBUG(get_logger(),
         "IMU => roll=%.2f, pitch=%.2f, yaw=%.2f (deg)",
         imu_roll_, imu_pitch_, imu_yaw_);
 }
 
 /* ---------------------------------------------------------------------
  * gpsCallback()
  * --------------------------------------------------------------------- */
 void GimbalTransformNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
 {
     // 1) Validate the GPS data
     if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude))
     {
         RCLCPP_WARN(get_logger(), "Ignoring invalid GPS (NaN in lat/lon/alt).");
         return;
     }
 
     // 2) Convert from WGS84 -> local XY(Z)
     Coordinate c = wgs84ToFrame(msg->latitude, msg->longitude, msg->altitude);
 
     // 3) If user disabled gimbal transform, just publish the raw XY(Z)
     if (!apply_gimbal_transform_)
     {
         geometry_msgs::msg::PointStamped point_msg;
         point_msg.header.stamp = now();
         point_msg.header.frame_id = coordinate_frame_;
         point_msg.point.x = c.x;
         point_msg.point.y = c.y;
         point_msg.point.z = c.z;
 
         gimbal_pos_pub_->publish(point_msg);
 
         RCLCPP_DEBUG(get_logger(),
             "apply_gimbal_transform=false -> Published coords (%.3f, %.3f, %.3f)",
             c.x, c.y, c.z);
         return;
     }
 
     // 4) Otherwise, apply the full DH chain
     Eigen::Matrix4d chain_matrix = computeDhChain();
 
     // Multiply the coordinate as a 4D vector
     Eigen::Vector4d inPoint(c.x, c.y, c.z, 1.0);
     Eigen::Vector4d outPoint = chain_matrix * inPoint;
 
     // 5) Publish the final gimbal center
     geometry_msgs::msg::PointStamped out_msg;
     out_msg.header.stamp = now();
     out_msg.header.frame_id = coordinate_frame_;
     out_msg.point.x = outPoint(0);
     out_msg.point.y = outPoint(1);
     out_msg.point.z = outPoint(2);
 
     gimbal_pos_pub_->publish(out_msg);
 
     RCLCPP_DEBUG(get_logger(),
         "apply_gimbal_transform=true -> Gimbal center= (%.3f, %.3f, %.3f)",
         outPoint(0), outPoint(1), outPoint(2));
 }
 
 /* ---------------------------------------------------------------------
  * wgs84ToFrame()
  * --------------------------------------------------------------------- */
 Coordinate GimbalTransformNode::wgs84ToFrame(double lat, double lon, double alt)
 {
     // For demonstration only: naive degrees->meters
     double scale = 111319.9; // approx. meters per degree at Equator
     Coordinate c;
     c.x = lon * scale;
     c.y = lat * scale;
     c.z = alt;
     return c;
 }
 
 /* ---------------------------------------------------------------------
  * computeDhChain()
  * --------------------------------------------------------------------- */
 Eigen::Matrix4d GimbalTransformNode::computeDhChain()
 {
     // Start with identity
     Eigen::Matrix4d full = Eigen::Matrix4d::Identity();
 
     for (auto &link : dh_links_)
     {
         // Update link's theta if it references IMU or gimbal angles
         if (link.joint_name == "UAV_Center_to_IMU_Z")
             link.theta = imu_yaw_;
         else if (link.joint_name == "IMU_Z_to_IMU_Y")
             link.theta = imu_pitch_ + 90.0;
         else if (link.joint_name == "IMU_Y_to_IMU_X")
             link.theta = imu_roll_ + 90.0;
         else if (link.joint_name == "IMU_X_to_Gimbal_Yaw")
             link.theta = yaw_angle_ + 30.3 - 90.0;
         else if (link.joint_name == "Gimbal_Yaw_to_Gimbal_Roll")
             link.theta = roll_angle_ + 120.3;
         else if (link.joint_name == "Gimbal_Roll_to_Gimbal_Pitch")
             link.theta = pitch_angle_ - 90.0;
 
         // Multiply link transform
         full = full * dhToMatrix(link.a, link.alpha, link.d, link.theta);
     }
 
     return full;
 }
 
 /* ---------------------------------------------------------------------
  * dhToMatrix()
  * --------------------------------------------------------------------- */
 Eigen::Matrix4d GimbalTransformNode::dhToMatrix(double a, double alpha, double d, double theta)
 {
     // Creates a 4x4 matrix from the given DH parameters (in degrees)
     Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
     double rad = M_PI / 180.0;
 
     double ca = std::cos(alpha * rad);
     double sa = std::sin(alpha * rad);
     double ct = std::cos(theta * rad);
     double st = std::sin(theta * rad);
 
     // Example standard/modified DH formula:
     T(0,0) = ct;       T(0,1) = -st * ca;  T(0,2) =  st * sa;   T(0,3) = a * ct;
     T(1,0) = st;       T(1,1) =  ct * ca;  T(1,2) = -ct * sa;   T(1,3) = a * st;
     T(2,1) = sa;       T(2,2) =  ca;       T(2,3) = d;
     return T;
 }
 