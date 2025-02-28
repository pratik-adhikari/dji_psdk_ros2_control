#ifndef GEO_GIMBAL_CONTROL_HPP
#define GEO_GIMBAL_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <psdk_interfaces/msg/gimbal_rotation.hpp>
#include <string>
#include <tuple>

/**
 * @file geo_gimbal_control.hpp
 * @brief A node that subscribes to a gimbal center topic, stores a target in the same frame,
 *        and publishes pitch/yaw commands to a gimbal.
 *
 * This code uses TargetLocation to represent the stored target (x, y, z).
 */

/**
 * @struct TargetLocation
 * @brief Represents a coordinate in the same frame as the incoming gimbal center (e.g. UTM or local).
 */
struct TargetLocation {
    double x;
    double y;
    double z;
};

/**
 * @class GeoGimbalControl
 * @brief Subscribes to "gimbal_center_topic" (PointStamped), has set_target(...) to store target,
 *        and publishes commands to "gimbal_command_topic" (GimbalRotation).
 */
class GeoGimbalControl : public rclcpp::Node
{
public:
    /**
     * @brief Default constructor: declares parameters, sets up subscription and publisher.
     */
    GeoGimbalControl();

    /// Default destructor
    ~GeoGimbalControl() override = default;

    /**
     * @brief Sets a new target coordinate (x, y, z).
     *        In normal usage, these are the same units/frame as the gimbal center input.
     */
    void set_target(const TargetLocation& target);

private:
    /**
     * @brief Loads parameters (topics, etc.) and sets up subscription/publisher.
     */
    void setup_node();

    /**
     * @brief Callback for the gimbal center position (PointStamped).
     *
     * Computes the pitch and yaw angles from the gimbal center to the target,
     * then publishes a command.
     *
     * @param msg The received gimbal center position.
     */
    void gimbal_center_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /**
     * @brief Compute angles from current UAV/gimbal center to the stored target.
     *
     * @param cx Gimbal center X coordinate.
     * @param cy Gimbal center Y coordinate.
     * @param cz Gimbal center Z coordinate.
     * @param tx Target X coordinate.
     * @param ty Target Y coordinate.
     * @param tz Target Z coordinate.
     * @return A tuple containing (pitch, yaw) angles in radians.
     */
    std::tuple<double, double> compute_angles(double cx, double cy, double cz,
                                              double tx, double ty, double tz) const;

    /**
     * @brief Publish the final gimbal pitch/yaw command.
     *
     * @param pitch The computed pitch angle in radians.
     * @param yaw The computed yaw angle in radians.
     */
    void publish_gimbal_command(double pitch, double yaw);

private:
    // Topic names loaded from parameters
    std::string gimbal_center_topic_;
    std::string gimbal_command_topic_;

    // Subscription & Publisher
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gimbal_center_sub_;
    rclcpp::Publisher<psdk_interfaces::msg::GimbalRotation>::SharedPtr gimbal_cmd_pub_;

    // Whether we have a valid target set
    bool target_set_;
    // The current target location in the same frame as gimbal center
    TargetLocation target_;
};

#endif // GEO_GIMBAL_CONTROL_HPP
