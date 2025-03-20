#include "geogimbal/geo_gimbal_control.hpp"

GeoGimbalControl::GeoGimbalControl()
: target_set_(false),
  target_{0.0, 0.0, 0.0},
  // We'll create a logger here that uses the name "GeoGimbalControl"
  logger_(rclcpp::get_logger("GeoGimbalControl"))
{
  RCLCPP_INFO(logger_, "Constructed GeoGimbalControl (logic-only).");
}

GeoGimbalControl::~GeoGimbalControl()
{
  RCLCPP_DEBUG(logger_, "Destroying GeoGimbalControl...");
}

void GeoGimbalControl::set_target(const TargetLocation &t)
{
  target_ = t;
  target_set_ = true;

  RCLCPP_INFO(logger_, 
    "New gimbal target set => (x=%.6f, y=%.6f, z=%.2f)", 
    t.x, t.y, t.z);
}

psdk_interfaces::msg::GimbalRotation 
GeoGimbalControl::computeCommandFromCenter(double cx, double cy, double cz)
{
  if (!target_set_) {
    // Log a warning, then throw
    RCLCPP_WARN(logger_, "No target set, cannot compute angles!");
    throw std::runtime_error("[GeoGimbalControl] No target set!");
  }

  double dx = target_.x - cx;
  double dy = target_.y - cy;
  double dz = target_.z - cz;

  double yaw = std::atan2(dy, dx);
  double dist_xy = std::hypot(dx, dy);
  double pitch = std::atan2(-dz, dist_xy);

  // If you want to log inside this class, do so:
  RCLCPP_DEBUG(logger_,
    "computeCommandFromCenter: center=(%.6f,%.6f,%.2f), target=(%.6f,%.6f,%.2f) => pitch=%.3f rad, yaw=%.3f rad",
    cx, cy, cz, target_.x, target_.y, target_.z, pitch, yaw
  );

  psdk_interfaces::msg::GimbalRotation cmd;
  cmd.payload_index = 1;
  cmd.rotation_mode = 1; // absolute mode
  cmd.roll  = 0.0;
  cmd.pitch = pitch;
  cmd.yaw   = yaw;
  cmd.time  = 0.5;

  return cmd;
}
