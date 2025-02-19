#ifndef GEO_GIMBAL_CONTROL_HPP
#define GEO_GIMBAL_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <psdk_interfaces/msg/gimbal_rotation.hpp>
#include <proj.h>
#include <vector>
#include <string>

struct TargetLocation {
    double latitude;
    double longitude;
    double altitude;
};

class GeoGimbalControl : public rclcpp::Node {
public:
    GeoGimbalControl();
    virtual ~GeoGimbalControl();  // Declare the destructor

private:
    void declare_parameters();
    void get_parameters();
    void initialize_coordinate_systems();
    std::vector<TargetLocation> load_target_locations(const std::string& file_path);

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void user_signal_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Declare transform_to_wgs here
    void transform_to_wgs(double easting, double northing, double& latitude, double& longitude);
    void transform_to_local(const TargetLocation& target, const TargetLocation& uav, double& dx, double& dy, double& dz);
    void calculate_gimbal_angles(double dx, double dy, double dz, double& yaw, double& pitch);
    void publish_gimbal_command(double pitch, double yaw);

    std::string target_file_path_;
    std::string target_coordinate_format_;
    std::vector<TargetLocation> target_list_;
    size_t target_index_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr user_signal_subscription_;
    rclcpp::Publisher<psdk_interfaces::msg::GimbalRotation>::SharedPtr gimbal_command_publisher_;

    PJ_CONTEXT* proj_context_;
    PJ* proj_transformer_;
};

#endif // GEO_GIMBAL_CONTROL_HPP
