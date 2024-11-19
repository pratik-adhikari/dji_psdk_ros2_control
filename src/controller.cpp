#include "geogimbal/controller.hpp"

GimbalController::GimbalController()
    : Node("gimbal_controller"), target_latitude_(0.0), target_longitude_(0.0), target_altitude_(0.0)
{
    command_publisher_ = this->create_publisher<geogimbal::msg::ProcessedGimbalCommand>(
        "/geogimbal/processed_command", 10);
}

void GimbalController::processGnssData(const sensor_msgs::msg::NavSatFix &gnss_data)
{
    target_latitude_ = gnss_data.latitude;
    target_longitude_ = gnss_data.longitude;
    target_altitude_ = gnss_data.altitude;
    calculateGimbalCommand();
}

void GimbalController::calculateGimbalCommand()
{
    // Placeholder logic for calculating gimbal angles
    auto command = geogimbal::msg::ProcessedGimbalCommand();
    command.pitch = 0.0; // Calculate pitch
    command.yaw = 0.0;   // Calculate yaw
    command.time = 1.0;

    RCLCPP_INFO(this->get_logger(), "Publishing processed command: pitch=%.2f, yaw=%.2f", command.pitch, command.yaw);
    command_publisher_->publish(command);
}
