#ifndef GEOGIMBAL_CONTROLLER_HPP
#define GEOGIMBAL_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "geogimbal/ProcessedGimbalCommand.msg"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose.hpp>


class GimbalController : public rclcpp::Node
{
public:
    GimbalController();
    void processGnssData(const sensor_msgs::msg::NavSatFix &gnss_data);
    void calculateGimbalCommand();

private:
    double target_latitude_, target_longitude_, target_altitude_;
    rclcpp::Publisher<geogimbal::msg::ProcessedGimbalCommand>::SharedPtr command_publisher_;
};

#endif // GEOGIMBAL_CONTROLLER_HPP
