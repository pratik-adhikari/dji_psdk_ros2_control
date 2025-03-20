#include <rclcpp/rclcpp.hpp>
#include <psdk_interfaces/msg/gimbal_rotation.hpp>
#include <std_msgs/msg/float64.hpp>

/**
 * @class GimbalSimBridge
 * @brief Subscribes to "/geogimbal/rotation_cmd" and repubs them
 *        as Float64 on Gazebo topics:
 *          /model/x500_gimbal/command/gimbal_pitch
 *          /model/x500_gimbal/command/gimbal_roll
 *          /model/x500_gimbal/command/gimbal_yaw
 */
class GimbalSimBridge : public rclcpp::Node
{
public:
  GimbalSimBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("gimbal_sim_bridge", options)
  {
    // Declare param or use default
    input_topic_  = declare_parameter<std::string>(
      "input_topic", "/geogimbal/rotation_cmd");
    pitch_topic_  = declare_parameter<std::string>(
      "pitch_topic", "/model/x500_gimbal/command/gimbal_pitch");
    roll_topic_   = declare_parameter<std::string>(
      "roll_topic",  "/model/x500_gimbal/command/gimbal_roll");
    yaw_topic_    = declare_parameter<std::string>(
      "yaw_topic",   "/model/x500_gimbal/command/gimbal_yaw");

    RCLCPP_INFO(get_logger(),
      "GimbalSimBridge => Subscribing to '%s'; publishing pitch='%s', roll='%s', yaw='%s'.",
      input_topic_.c_str(), pitch_topic_.c_str(), roll_topic_.c_str(), yaw_topic_.c_str()
    );

    // Publishers for the 3 Float64 commands
    pitch_pub_ = create_publisher<std_msgs::msg::Float64>(pitch_topic_, 10);
    roll_pub_  = create_publisher<std_msgs::msg::Float64>(roll_topic_, 10);
    yaw_pub_   = create_publisher<std_msgs::msg::Float64>(yaw_topic_, 10);

    // Subscription to the GimbalRotation
    cmd_sub_ = create_subscription<psdk_interfaces::msg::GimbalRotation>(
      input_topic_,
      10,
      [this](psdk_interfaces::msg::GimbalRotation::SharedPtr msg)
      {
        // Convert to Float64
        std_msgs::msg::Float64 pitch_msg, roll_msg, yaw_msg;
        pitch_msg.data = msg->pitch;
        roll_msg.data  = msg->roll;
        yaw_msg.data   = msg->yaw;

        pitch_pub_->publish(pitch_msg);
        roll_pub_->publish(roll_msg);
        yaw_pub_->publish(yaw_msg);

        // Log at info or debug
        RCLCPP_INFO(this->get_logger(),
          "GimbalSimBridge => bridging pitch=%.3f rad, roll=%.3f rad, yaw=%.3f rad",
          msg->pitch, msg->roll, msg->yaw
        );
      }
    );
  }

private:
  std::string input_topic_;
  std::string pitch_topic_;
  std::string roll_topic_;
  std::string yaw_topic_;

  rclcpp::Subscription<psdk_interfaces::msg::GimbalRotation>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
};

// Optional main if you want to launch just the bridging node by itself
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GimbalSimBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
