#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <psdk_interfaces/msg/gimbal_rotation.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include "geogimbal/geo_gimbal_control.hpp"

/**
 * @class GimbalOrchestrator
 * @brief
 *   - Subscribes to /gps/aruco => sets target in GeoGimbalControl
 *   - Subscribes to /gps/camera => computes angles => publishes /geogimbal/rotation_cmd
 *   - Logs everything, including when no data arrives, via a watchdog timer
 */
class GimbalOrchestrator : public rclcpp::Node
{
public:
  GimbalOrchestrator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("gimbal_orchestrator", options),
    aruco_count_(0),
    camera_count_(0)
  {
    gimbal_control_ = std::make_shared<GeoGimbalControl>();

    cmd_pub_ = create_publisher<psdk_interfaces::msg::GimbalRotation>(
      "/geogimbal/rotation_cmd", 10);

    // Subscribe to /gps/aruco
    aruco_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/aruco",
      10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
      {
        aruco_count_++;
        last_aruco_time_ = now();
        RCLCPP_INFO(this->get_logger(),
          "[ArucoCB] Received #%ld => lat=%.6f, lon=%.6f, alt=%.2f",
          aruco_count_, msg->latitude, msg->longitude, msg->altitude);

        if (!std::isfinite(msg->latitude) ||
            !std::isfinite(msg->longitude) ||
            !std::isfinite(msg->altitude))
        {
          RCLCPP_WARN(this->get_logger(), "[ArucoCB] Invalid data, skipping set_target.");
          return;
        }

        TargetLocation t;
        t.x = msg->longitude;  // or transform
        t.y = msg->latitude;
        t.z = msg->altitude;

        gimbal_control_->set_target(t);
      }
    );

    // Subscribe to /gps/camera
    camera_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/camera",
      10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
      {
        camera_count_++;
        last_camera_time_ = now();
        RCLCPP_INFO(this->get_logger(),
          "[CameraCB] Received #%ld => lat=%.6f, lon=%.6f, alt=%.2f",
          camera_count_, msg->latitude, msg->longitude, msg->altitude);

        if (!std::isfinite(msg->latitude) ||
            !std::isfinite(msg->longitude) ||
            !std::isfinite(msg->altitude))
        {
          RCLCPP_WARN(this->get_logger(), "[CameraCB] Invalid data, skipping angle compute.");
          return;
        }

        double cx = msg->longitude;
        double cy = msg->latitude;
        double cz = msg->altitude;

        // Try to compute angles
        try {
          auto cmd = gimbal_control_->computeCommandFromCenter(cx, cy, cz);

          double pitch_deg = cmd.pitch * 180.0 / M_PI;
          double yaw_deg   = cmd.yaw   * 180.0 / M_PI;

          RCLCPP_INFO(this->get_logger(),
            "[CameraCB] GimbalRotation => pitch=%.1f deg, yaw=%.1f deg. Publishing => /geogimbal/rotation_cmd",
            pitch_deg, yaw_deg
          );
          cmd_pub_->publish(cmd);
        }
        catch(const std::exception &e) {
          // e.g. no target set
          RCLCPP_ERROR(this->get_logger(),
            "[CameraCB] Error computing gimbal command: %s", e.what());
        }
      }
    );

    // Watchdog + Heartbeat: checks every 3s if we have data or if it's stale
    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&GimbalOrchestrator::watchdogCallback, this)
    );
  }

private:
  void watchdogCallback()
  {
    // 1) Heartbeat
    RCLCPP_INFO(this->get_logger(),
      "Orchestrator Watchdog: so far => /gps/aruco msgs=%ld, /gps/camera msgs=%ld",
      aruco_count_, camera_count_
    );

    // Current time
    auto now_t = now();

    // 2) Check time since last aruco
    if (aruco_count_ == 0) {
      RCLCPP_WARN(this->get_logger(),
        "No /gps/aruco messages have EVER arrived. Possibly no ArUco tracking data is published?");
    } else {
      rclcpp::Duration delta = now_t - last_aruco_time_;
      // Convert nanoseconds -> milliseconds
      double ms = static_cast<double>(delta.nanoseconds()) / 1.0e6; // 1e6 => ms
      if (ms > 5000.0) { // 5000 ms => 5 seconds
        RCLCPP_WARN(this->get_logger(),
          "/gps/aruco is stale, last message was %.2f seconds ago.",
          ms / 1000.0
        );
      }
    }

    // 3) Check time since last camera
    if (camera_count_ == 0) {
      RCLCPP_WARN(this->get_logger(),
        "No /gps/camera messages have EVER arrived. Possibly no camera GPS is published?");
    } else {
      rclcpp::Duration delta = now_t - last_camera_time_;
      double ms = static_cast<double>(delta.nanoseconds()) / 1.0e6;
      if (ms > 5000.0) {
        RCLCPP_WARN(this->get_logger(),
          "/gps/camera is stale, last message was %.2f seconds ago.",
          ms / 1000.0
        );
      }
    }
  }


  // Logic-only instance
  std::shared_ptr<GeoGimbalControl> gimbal_control_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr aruco_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr camera_sub_;

  // Publisher
  rclcpp::Publisher<psdk_interfaces::msg::GimbalRotation>::SharedPtr cmd_pub_;

  // Watchdog
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Counters & Timestamps
  size_t aruco_count_;
  size_t camera_count_;
  rclcpp::Time last_aruco_time_;
  rclcpp::Time last_camera_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GimbalOrchestrator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
