#include "geogimbal/geo_gimbal_control.hpp"
#include <cmath>
#include <tuple>

/**
 * @file geo_gimbal_control.cpp
 * @brief Implementation of GeoGimbalControl, storing a target and publishing pitch/yaw commands.
 */

/**
 * @brief Constructor for GeoGimbalControl.
 *
 * Initializes the ROS2 node, sets up subscriptions and publishers, and initializes target values.
 */
GeoGimbalControl::GeoGimbalControl()
  : Node("geo_gimbal_control"),
    target_set_(false),
    target_{0.0, 0.0, 0.0}
{
    setup_node();
    RCLCPP_INFO(this->get_logger(), "GeoGimbalControl node initialized.");
}

/**
 * @brief Sets up ROS parameters, subscribers, and publishers.
 *
 * Reads topic names from the parameter server, subscribes to the gimbal center topic,
 * and sets up a publisher for sending gimbal rotation commands.
 */
void GeoGimbalControl::setup_node()
{
    gimbal_center_topic_ = this->declare_parameter<std::string>(
        "gimbal_center_topic", "/uav/gimbal_center_position"
    );
    gimbal_command_topic_ = this->declare_parameter<std::string>(
        "gimbal_command_topic", "/wrapper/psdk_ros2/gimbal_rotation"
    );

    gimbal_center_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        gimbal_center_topic_, 10,
        std::bind(&GeoGimbalControl::gimbal_center_callback, this, std::placeholders::_1)
    );

    gimbal_cmd_pub_ = this->create_publisher<psdk_interfaces::msg::GimbalRotation>(
        gimbal_command_topic_, 10
    );
}

/**
 * @brief Sets the target location for the gimbal to track.
 *
 * @param target The target location in the global coordinate frame.
 */
void GeoGimbalControl::set_target(const TargetLocation& target)
{
    target_ = target;
    target_set_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "New target set: (%.2f, %.2f, %.2f)",
        target_.x, target_.y, target_.z
    );
}

/**
 * @brief Callback function triggered when the gimbal center position is received.
 *
 * Computes the pitch and yaw angles required to align the gimbal with the target location.
 * If no target is set, the function exits early.
 *
 * @param msg The received PointStamped message containing the gimbal's center position.
 */
void GeoGimbalControl::gimbal_center_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (!target_set_) {
        RCLCPP_DEBUG(this->get_logger(), "No target set. Skipping angle computation.");
        return;
    }

    double cx = msg->point.x;
    double cy = msg->point.y;
    double cz = msg->point.z;

    auto [pitch, yaw] = compute_angles(cx, cy, cz, target_.x, target_.y, target_.z);
    publish_gimbal_command(pitch, yaw);
}

/**
 * @brief Computes pitch and yaw angles required to aim at the target.
 *
 * Uses trigonometry to compute the angles based on the difference between the target and gimbal center.
 *
 * @param cx Gimbal center X-coordinate.
 * @param cy Gimbal center Y-coordinate.
 * @param cz Gimbal center Z-coordinate.
 * @param tx Target X-coordinate.
 * @param ty Target Y-coordinate.
 * @param tz Target Z-coordinate.
 * @return A tuple representing pitch and yaw angles in radians.
 */
std::tuple<double, double> GeoGimbalControl::compute_angles(double cx, double cy, double cz,
                                                            double tx, double ty, double tz) const
{
    double dx = tx - cx;
    double dy = ty - cy;
    double dz = tz - cz;

    double yaw   = std::atan2(dy, dx);
    double range = std::hypot(dx, dy);
    double pitch = std::atan2(-dz, range);

    return {pitch, yaw};
}

/**
 * @brief Publishes a gimbal rotation command.
 *
 * Constructs and sends a message to control the gimbal's pitch and yaw angles.
 * The command is set in absolute mode with a predefined execution time.
 *
 * @param pitch The pitch angle in radians.
 * @param yaw The yaw angle in radians.
 */
void GeoGimbalControl::publish_gimbal_command(double pitch, double yaw)
{
    psdk_interfaces::msg::GimbalRotation cmd;
    cmd.payload_index = 1;
    cmd.rotation_mode = 1;  // absolute mode (example)
    cmd.pitch = pitch;
    cmd.yaw   = yaw;
    cmd.roll  = 0.0;
    cmd.time  = 0.5;

    gimbal_cmd_pub_->publish(cmd);

    RCLCPP_INFO(
        this->get_logger(),
        "Published gimbal cmd => pitch=%.1f deg, yaw=%.1f deg",
        pitch * 180.0 / M_PI,
        yaw   * 180.0 / M_PI
    );
}
