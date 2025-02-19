#include "geogimbal/geo_gimbal_control.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

GeoGimbalControl::GeoGimbalControl()
    : Node("geo_gimbal_control"), target_index_(0), proj_context_(nullptr), proj_transformer_(nullptr) {
    declare_parameters();
    get_parameters();
    initialize_coordinate_systems();

    target_list_ = load_target_locations(target_file_path_);
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/wrapper/psdk_ros2/gps_position_fused", 10,
        std::bind(&GeoGimbalControl::gps_callback, this, std::placeholders::_1));

    user_signal_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/next_target_signal", 10,
        std::bind(&GeoGimbalControl::user_signal_callback, this, std::placeholders::_1));

    gimbal_command_publisher_ = this->create_publisher<psdk_interfaces::msg::GimbalRotation>(
        "/wrapper/psdk_ros2/gimbal_rotation", 10);

    RCLCPP_INFO(this->get_logger(), "GeoGimbalControl node initialized.");
}

GeoGimbalControl::~GeoGimbalControl() {
    if (proj_transformer_) {
        proj_destroy(proj_transformer_);
    }
    if (proj_context_) {
        proj_context_destroy(proj_context_);
    }
    RCLCPP_INFO(this->get_logger(), "GeoGimbalControl resources cleaned up.");
}

void GeoGimbalControl::declare_parameters() {
    this->declare_parameter<std::string>("target_file_path", "data/default_target_locations.txt");
    this->declare_parameter<std::string>("target_coordinate_format", "WGS84");
}

void GeoGimbalControl::get_parameters() {
    this->get_parameter("target_file_path", target_file_path_);
    this->get_parameter("target_coordinate_format", target_coordinate_format_);

    if (target_coordinate_format_ != "UTM" && target_coordinate_format_ != "WGS84") {
        RCLCPP_ERROR(this->get_logger(), "Invalid target_coordinate_format. Supported formats are 'UTM' and 'WGS84'.");
        throw std::runtime_error("Invalid target_coordinate_format.");
    }
}

void GeoGimbalControl::initialize_coordinate_systems() {
    proj_context_ = proj_context_create();
    proj_transformer_ = proj_create_crs_to_crs(proj_context_,
                                               "+proj=utm +zone=32 +datum=WGS84",
                                               "+proj=longlat +datum=WGS84",
                                               nullptr);
    if (!proj_transformer_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize coordinate transformation.");
        throw std::runtime_error("Projection initialization failed.");
    }
}

std::vector<TargetLocation> GeoGimbalControl::load_target_locations(const std::string& file_path) {
    std::vector<TargetLocation> targets;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open target file: %s", file_path.c_str());
        throw std::runtime_error("File open error");
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        TargetLocation target;
        if (target_coordinate_format_ == "WGS84") {
            ss >> target.latitude >> target.longitude >> target.altitude;
        } else if (target_coordinate_format_ == "UTM") {
            double easting, northing;
            ss >> easting >> northing >> target.altitude;
            transform_to_wgs(easting, northing, target.latitude, target.longitude);
        }
        targets.push_back(target);
    }

    return targets;
}

void GeoGimbalControl::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (target_index_ >= target_list_.size()) {
        RCLCPP_INFO(this->get_logger(), "All targets processed.");
        return;
    }

    const auto& uav_position = TargetLocation{msg->latitude, msg->longitude, msg->altitude};
    const auto& target_position = target_list_[target_index_];

    double dx, dy, dz;
    transform_to_local(target_position, uav_position, dx, dy, dz);

    double yaw, pitch;
    calculate_gimbal_angles(dx, dy, dz, yaw, pitch);
    publish_gimbal_command(pitch, yaw);
}

void GeoGimbalControl::user_signal_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && target_index_ < target_list_.size() - 1) {
        ++target_index_;
        RCLCPP_INFO(this->get_logger(), "Moving to the next target (Index: %lu).", target_index_);
    }
}

void GeoGimbalControl::transform_to_wgs(double easting, double northing, double& latitude, double& longitude) {
    PJ_COORD utm = proj_coord(easting, northing, 0, 0);
    PJ_COORD wgs = proj_trans(proj_transformer_, PJ_INV, utm);

    longitude = wgs.lp.lam * 180.0 / M_PI;
    latitude = wgs.lp.phi * 180.0 / M_PI;
}

void GeoGimbalControl::transform_to_local(const TargetLocation& target, const TargetLocation& uav,
                                          double& dx, double& dy, double& dz) {
    dx = target.longitude - uav.longitude;
    dy = target.latitude - uav.latitude;
    dz = target.altitude - uav.altitude;
}

void GeoGimbalControl::calculate_gimbal_angles(double dx, double dy, double dz, double& yaw, double& pitch) {
    yaw = std::atan2(dy, dx);
    pitch = std::atan2(-dz, std::hypot(dx, dy));
}

void GeoGimbalControl::publish_gimbal_command(double pitch, double yaw) {
    auto command = psdk_interfaces::msg::GimbalRotation();
    command.payload_index = 1;
    command.rotation_mode = 1;
    command.pitch = pitch;
    command.yaw = yaw;
    command.roll = 0.0;
    command.time = 0.5;

    gimbal_command_publisher_->publish(command);
    RCLCPP_INFO(this->get_logger(), "Published gimbal command: Yaw: %.2f°, Pitch: %.2f°", yaw * 180.0 / M_PI, pitch * 180.0 / M_PI);
}
