/**
 * @file gimbal_transform_node.hpp
 * @brief Declarations for GimbalTransformNode, a ROS2 node that:
 *        1) Converts UAV GPS (WGS84) to UTM or LOCAL,
 *        2) Optionally applies a Denavit–Hartenberg chain,
 *        3) Publishes the resulting point as a PointStamped,
 *        4) Broadcasts TF frames (map->uav_base_link->imu_link->gimbal_link).
 */

 #ifndef GEOGIMBAL__GIMBAL_TRANSFORM_NODE_HPP_
 #define GEOGIMBAL__GIMBAL_TRANSFORM_NODE_HPP_
 
 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/nav_sat_fix.hpp>
 #include <sensor_msgs/msg/imu.hpp>
 #include <geometry_msgs/msg/point_stamped.hpp>
 #include <tf2_ros/transform_broadcaster.h>
 
 #include <Eigen/Dense>
 #include <string>
 #include <vector>
 
 /**
  * @struct DHLink
  * @brief Encapsulates one link in a Denavit–Hartenberg chain, storing standard DH parameters.
  */
 struct DHLink
 {
   std::string joint_name;  ///< Identifier/name of the link or joint
   double a;                ///< Link length (x-axis distance)
   double alpha;            ///< Link twist (rotation around x-axis, degrees)
   double d;                ///< Link offset (translation along z-axis)
   double theta;            ///< Joint angle (rotation around z-axis, degrees)
 };
 
 /**
  * @struct Coordinate
  * @brief A simple struct holding x, y, z coordinates, used for local transformations.
  */
 struct Coordinate
 {
   double x;
   double y;
   double z;
 };
 
 /**
  * @class GimbalTransformNode
  * @brief A ROS2 node that:
  *   - Subscribes to GPS (NavSatFix) and IMU (sensor_msgs::Imu).
  *   - Converts GPS data from WGS84 -> UTM or LOCAL coords.
  *   - Optionally applies a Denavit–Hartenberg chain to get a final "gimbal center" position.
  *   - Publishes that position as geometry_msgs::msg::PointStamped.
  *   - **Broadcasts TF** frames:
  *       - map -> uav_base_link (from GPS)
  *       - uav_base_link -> imu_link (from IMU)
  *       - imu_link -> gimbal_link (could also incorporate gimbal angles).
  */
 class GimbalTransformNode : public rclcpp::Node
 {
 public:
   /**
    * @brief Constructor for GimbalTransformNode
    * @param options NodeOptions for parameter loading, etc.
    *
    * This constructor:
    *  - Declares/loads mandatory topics & config from `parameters.yaml`.
    *  - Loads the DH chain from `transform_config.yaml` if needed.
    *  - Sets up subscriptions to GPS (NavSatFix) and IMU.
    *  - Creates a publisher for the gimbal center position.
    *  - Initializes a tf2_ros::TransformBroadcaster for dynamic TF.
    */
   explicit GimbalTransformNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
 
 private:
   // ------------------------------------------------------
   // Parameters & Config
   // ------------------------------------------------------
   std::string gps_position_topic_;    ///< GPS subscription topic
   std::string imu_topic_;            ///< IMU subscription topic
   std::string gimbal_center_topic_;  ///< Gimbal position publisher topic
 
   bool apply_gimbal_transform_;      ///< If false, skip the DH chain step
 
   // "UTM" or "LOCAL"
   std::string coordinate_frame_;
   int utm_zone_;                     ///< UTM zone, if coordinate_frame_ == "UTM"
   bool northern_hemisphere_;         ///< True if in northern hemisphere
   double base_lat_, base_lon_, base_alt_; ///< Base lat/lon/alt if coordinate_frame_ == "LOCAL"
 
   // Frame IDs for TF
   std::string map_frame_id_;         ///< Parent (e.g. "map", "world")
   std::string uav_base_frame_id_;    ///< e.g. "uav_base_link"
   std::string imu_frame_id_;         ///< e.g. "imu_link"
   std::string gimbal_frame_id_;      ///< e.g. "gimbal_link"
 
   // ------------------------------------------------------
   // DH Chain Storage
   // ------------------------------------------------------
   std::vector<DHLink> dh_links_; ///< All links read from transform_config.yaml
 
   // ------------------------------------------------------
   // UAV & Gimbal Angles
   // ------------------------------------------------------
   // IMU angles (in degrees), updated each IMU callback
   double imu_roll_;
   double imu_pitch_;
   double imu_yaw_;
 
   // Gimbal angles (if you want to incorporate them in your chain)
   double yaw_angle_;
   double roll_angle_;
   double pitch_angle_;
 
   // Current UAV position (x, y, z) in the local or UTM frame
   // (Updated when GPS arrives; used for broadcasting TF "map->uav_base_link".)
   double uav_x_;
   double uav_y_;
   double uav_z_;
 
   // ------------------------------------------------------
   // ROS Communication
   // ------------------------------------------------------
   rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_; ///< Subscription to GPS
   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;       ///< Subscription to IMU
   rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gimbal_pos_pub_; ///< Publisher
 
   /// TF Broadcaster for dynamic transforms
   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
 
   // ------------------------------------------------------
   // Private Methods
   // ------------------------------------------------------
 
   /**
    * @brief Load essential parameters from `parameters.yaml`.
    * @throw std::runtime_error if any mandatory parameter is missing.
    */
   void loadParameters();
 
   /**
    * @brief Load the DH chain from `transform_config.yaml`.
    * @throw std::runtime_error if the file or 'dh_parameters' node is missing.
    */
   void loadDhParameters();
 
   /**
    * @brief Initialize coordinate transformation logic (WGS84 -> UTM or WGS84 -> LOCAL).
    */
   void setupTransformer();
 
   /**
    * @brief Create the subscriptions (GPS, IMU) and the gimbal position publisher.
    */
   void setupSubscriptionsAndPublisher();
 
   /**
    * @brief IMU callback. Converts quaternion -> roll/pitch/yaw in degrees for the DH chain.
    * @param msg The incoming IMU data.
    *
    * Also broadcasts:
    *   uav_base_link -> imu_link
    *   imu_link      -> gimbal_link
    * using the orientation and optional gimbal angles.
    */
   void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
 
   /**
    * @brief GPS callback. Converts WGS84 -> local coords (or UTM), optionally applies DH transform,
    *        and publishes the final "gimbal center" position as a PointStamped.
    * @param msg The incoming GPS (NavSatFix) data.
    *
    * Also broadcasts:
    *   map -> uav_base_link
    * using the computed (x, y, z) from GPS.
    */
   void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
 
   /**
    * @brief Converts latitude, longitude, altitude from WGS84 to local XY(Z).
    * @param lat Latitude in degrees
    * @param lon Longitude in degrees
    * @param alt Altitude in meters
    * @return A Coordinate struct containing x,y,z in the local or UTM frame.
    */
   Coordinate wgs84ToFrame(double lat, double lon, double alt);
 
   /**
    * @brief Build the final 4x4 transformation matrix from the loaded DH chain.
    *        Updates any link angles that reference UAV IMU or gimbal angles.
    * @return The chain matrix T from (some reference) -> gimbal center.
    */
   Eigen::Matrix4d computeDhChain();
 
   /**
    * @brief Convert a single link's DH parameters (in degrees) into a 4x4 transformation matrix.
    * @param a The link length
    * @param alpha The link twist (deg)
    * @param d The link offset
    * @param theta The joint angle (deg)
    * @return A 4x4 matrix representing this link's transform.
    */
   Eigen::Matrix4d dhToMatrix(double a, double alpha, double d, double theta);
 };
 
 #endif // GEOGIMBAL__GIMBAL_TRANSFORM_NODE_HPP_
 