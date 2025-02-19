#include "geogimbal/geo_gimbal_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeoGimbalControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
