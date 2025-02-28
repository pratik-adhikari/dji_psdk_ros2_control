/**
 * @file gimbal_transform_node.hpp
 * @brief Declarations for GimbalTransformNode, a ROS2 node that transforms UAV GPS (WGS84) to UTM or LOCAL,
 *        and optionally applies a Denavit–Hartenberg chain to produce the gimbal center position.
 */

 #ifndef GEOGIMBAL__GIMBAL_TRANSFORM_NODE_HPP_
 #define GEOGIMBAL__GIMBAL_TRANSFORM_NODE_HPP_
 
 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/nav_sat_fix.hpp>
 #include <sensor_msgs/msg/imu.hpp>
 #include <geometry_msgs/msg/point_stamped.hpp>
 #include <Eigen/Dense>
 #include <vector>
 #include <string>
 
 /**
  * @struct DHLink
  * @brief Encapsulates a single link in the Denavit–Hartenberg chain, storing standard DH parameters.
  */
 struct DHLink
 {
     std::string joint_name; ///< Identifier/name of the link or joint
     double a;               ///< Link length (x-axis distance)
     double alpha;           ///< Link twist (rotation around x-axis)
     double d;               ///< Link offset (translation along z-axis)
     double theta;           ///< Joint angle (rotation around z-axis)
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
  *   1) Loads topics & config from parameters.yaml,
  *   2) Receives UAV GPS data (NavSatFix) & IMU data,
  *   3) Optionally applies a Denavit–Hartenberg chain for final gimbal center,
  *   4) Publishes the resulting position as a PointStamped.
  */
 class GimbalTransformNode : public rclcpp::Node
 {
 public:
     /**
      * @brief Constructor for GimbalTransformNode.
      * @param options NodeOptions for parameter loading, etc.
      *
      * This constructor:
      *  - Initializes the node name ("gimbal_transform_node")
      *  - Loads and validates parameters from the param server (parameters.yaml)
      *  - Loads the DH chain from transform_config.yaml if needed
      *  - Sets up subscriptions to GPS (NavSatFix) and IMU data
      *  - Creates a publisher for the gimbal center position
      */
     explicit GimbalTransformNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
 
 private:
     // ------------------ ROS Parameters & Config ------------------
     std::string gps_position_topic_;    ///< GPS subscription topic
     std::string imu_topic_;            ///< IMU subscription topic
     std::string gimbal_center_topic_;  ///< Gimbal position publisher topic
 
     bool apply_gimbal_transform_;      ///< Flag: If false, skip DH chain
 
     std::string coordinate_frame_;     ///< "UTM" or "LOCAL"
     int utm_zone_;                     ///< UTM zone if coordinate_frame_ == "UTM"
     bool northern_hemisphere_;         ///< True if in northern hemisphere
     double base_lat_, base_lon_, base_alt_; ///< Base lat/lon/alt if coordinate_frame_ == "LOCAL"
 
     // ------------------ DH Chain Storage ------------------
     std::vector<DHLink> dh_links_; ///< All links read from transform_config.yaml
 
     // ------------------ UAV & Gimbal Angles ------------------
     // IMU angles (in degrees), updated each IMU callback
     double imu_roll_;
     double imu_pitch_;
     double imu_yaw_;
 
     // Gimbal angles (placeholders if you read them from somewhere else)
     double yaw_angle_;
     double roll_angle_;
     double pitch_angle_;
 
     // ------------------ ROS Communication ------------------
     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_; ///< Subscription to GPS
     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;       ///< Subscription to IMU
     rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gimbal_pos_pub_; ///< Publisher
 
     // ------------------ Private Methods ------------------
 
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
      */
     void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
 
     /**
      * @brief GPS callback. Converts WGS84 -> local coords, optionally applies DH transform,
      *        and publishes the final position.
      * @param msg The incoming GPS (NavSatFix) data.
      */
     void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
 
     /**
      * @brief Converts latitude, longitude, altitude from WGS84 to local XY(Z).
      * @param lat Latitude in degrees
      * @param lon Longitude in degrees
      * @param alt Altitude in meters
      * @return A Coordinate struct containing x,y,z in the local frame.
      *
      * @note Currently a naive degrees->meters approach. Replace with a proper transformation
      *       if you require real UTM or local tangent-plane accuracy.
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
      * @param alpha The link twist
      * @param d The link offset
      * @param theta The joint angle
      * @return A 4x4 matrix representing this link's transform.
      */
     Eigen::Matrix4d dhToMatrix(double a, double alpha, double d, double theta);
 };
 
 #endif // GEOGIMBAL__GIMBAL_TRANSFORM_NODE_HPP_
 