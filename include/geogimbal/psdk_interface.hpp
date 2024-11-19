#ifndef GEOGIMBAL_PSDK_INTERFACE_HPP
#define GEOGIMBAL_PSDK_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <psdk_interfaces/msg/gimbal_rotation.hpp>
#include "geogimbal/ProcessedGimbalCommand.msg"

class PSDKInterface : public rclcpp::Node
{
public:
    PSDKInterface();

private:
    void commandCallback(const geogimbal::msg::ProcessedGimbalCommand::SharedPtr msg);
    rclcpp::Subscription<geogimbal::msg::ProcessedGimbalCommand>::SharedPtr command_subscription_;
    rclcpp::Publisher<psdk_interfaces::msg::GimbalRotation>::SharedPtr gimbal_publisher_;
};

#endif // GEOGIMBAL_PSDK_INTERFACE_HPP
