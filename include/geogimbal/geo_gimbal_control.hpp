#ifndef GEO_GIMBAL_CONTROL_HPP
#define GEO_GIMBAL_CONTROL_HPP

#include <psdk_interfaces/msg/gimbal_rotation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <cmath>

/**
 * @brief A simple struct to store a target location (x, y, z).
 */
struct TargetLocation
{
  double x;
  double y;
  double z;
};

/**
 * @class GeoGimbalControl
 * @brief A logic-only class to compute a gimbal command from (cx, cy, cz)
 *        to the stored target (x, y, z).
 *
 *        It logs whenever a new target is set. The actual angle computation
 *        can also log if you desire, or you can log in the node that calls it.
 */
class GeoGimbalControl
{
public:
  /// Constructor & Destructor
  GeoGimbalControl();
  ~GeoGimbalControl();

  /**
   * @brief Store or update a target location. Logs new target automatically.
   */
  void set_target(const TargetLocation &t);

  /**
   * @brief Compute a psdk_interfaces::msg::GimbalRotation from the given camera center (cx,cy,cz)
   *        to the previously set target. Throws if no target set yet.
   */
  psdk_interfaces::msg::GimbalRotation computeCommandFromCenter(double cx, double cy, double cz);

private:
  bool target_set_;
  TargetLocation target_;
  rclcpp::Logger logger_;
};

#endif // GEO_GIMBAL_CONTROL_HPP
