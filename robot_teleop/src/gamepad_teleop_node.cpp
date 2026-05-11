#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <iostream>

class GamepadTeleop : public rclcpp::Node
{
public:
  GamepadTeleop() : Node("gamepad_teleop_node")
  {
    this->declare_parameter("mapping.axis_linear_x_fwd", 5);  // RT
    this->declare_parameter("mapping.axis_linear_x_bwd", 2);  // LT
    this->declare_parameter("mapping.axis_linear_y", 3);      // Right Stick Horizontal
    this->declare_parameter("mapping.axis_angular_z", 0);     // Left Stick Horizontal
    this->declare_parameter("mapping.enable_button", -1);     // -1 to disable
    this->declare_parameter("mapping.estop_button", 1);       // Button B
    this->declare_parameter("mapping.turbo_button", 5);       // RB
    this->declare_parameter("mapping.precision_button", 4);   // LB

    this->declare_parameter("settings.scale_linear_x", 0.5);
    this->declare_parameter("settings.scale_linear_y", 0.5);
    this->declare_parameter("settings.scale_angular_z", 0.5);
    this->declare_parameter("settings.turbo_scale", 2.0);
    this->declare_parameter("settings.precision_scale", 0.5);
    this->declare_parameter("settings.deadzone", 0.1);

    // Get parameters
    axis_linear_x_fwd_ = this->get_parameter("mapping.axis_linear_x_fwd").as_int();
    axis_linear_x_bwd_ = this->get_parameter("mapping.axis_linear_x_bwd").as_int();
    axis_linear_y_ = this->get_parameter("mapping.axis_linear_y").as_int();
    axis_angular_z_ = this->get_parameter("mapping.axis_angular_z").as_int();
    enable_button_ = this->get_parameter("mapping.enable_button").as_int();
    estop_button_ = this->get_parameter("mapping.estop_button").as_int();
    turbo_button_ = this->get_parameter("mapping.turbo_button").as_int();
    precision_button_ = this->get_parameter("mapping.precision_button").as_int();

    scale_linear_x_ = this->get_parameter("settings.scale_linear_x").as_double();
    scale_linear_y_ = this->get_parameter("settings.scale_linear_y").as_double();
    scale_angular_z_ = this->get_parameter("settings.scale_angular_z").as_double();
    turbo_scale_ = this->get_parameter("settings.turbo_scale").as_double();
    precision_scale_ = this->get_parameter("settings.precision_scale").as_double();
    deadzone_ = this->get_parameter("settings.deadzone").as_double();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&GamepadTeleop::joy_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Gamepad Teleop Node Started with Grouped Parameters");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto twist = geometry_msgs::msg::Twist();

    // Check E-Stop button
    if (estop_button_ >= 0 && (size_t)estop_button_ < msg->buttons.size()) {
      if (msg->buttons[estop_button_]) {
        publisher_->publish(geometry_msgs::msg::Twist());
        return;
      }
    }

    // Check enable button
    if (enable_button_ >= 0 && (size_t)enable_button_ < msg->buttons.size()) {
      if (!msg->buttons[enable_button_]) {
        publisher_->publish(twist);
        return;
      }
    }

    // Determine Speed Multiplier (Turbo / Precision)
    double speed_multiplier = 1.0;
    if (turbo_button_ >= 0 && (size_t)turbo_button_ < msg->buttons.size()) {
      if (msg->buttons[turbo_button_]) {
        speed_multiplier *= turbo_scale_;
      }
    }
    if (precision_button_ >= 0 && (size_t)precision_button_ < msg->buttons.size()) {
      if (msg->buttons[precision_button_]) {
        speed_multiplier *= precision_scale_;
      }
    }

    // Linear X (Forward/Backward with Triggers)
    double val_fwd = (axis_linear_x_fwd_ < (int)msg->axes.size()) ? msg->axes[axis_linear_x_fwd_] : 1.0;
    double val_bwd = (axis_linear_x_bwd_ < (int)msg->axes.size()) ? msg->axes[axis_linear_x_bwd_] : 1.0;

    double norm_fwd = (1.0 - val_fwd) / 2.0;
    double norm_bwd = (1.0 - val_bwd) / 2.0;

    double net_linear_x = norm_fwd - norm_bwd;
    
    twist.linear.x = (net_linear_x * net_linear_x * net_linear_x) * scale_linear_x_ * speed_multiplier;

    // Linear Y (Strafe)
    if (axis_linear_y_ < (int)msg->axes.size()) {
      double val_lat = msg->axes[axis_linear_y_];
      if (std::abs(val_lat) > deadzone_) {
        twist.linear.y = (val_lat * val_lat * val_lat) * scale_linear_y_ * speed_multiplier;
      }
    }

    // Angular Z (Turn)
    if (axis_angular_z_ < (int)msg->axes.size()) {
      double val_ang = msg->axes[axis_angular_z_];
      if (std::abs(val_ang) > deadzone_) {
        twist.angular.z = (val_ang * val_ang * val_ang) * scale_angular_z_ * speed_multiplier;
      }
    }

    publisher_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

  int axis_linear_x_fwd_;
  int axis_linear_x_bwd_;
  int axis_linear_y_;
  int axis_angular_z_;
  int enable_button_;
  int estop_button_;
  int turbo_button_;
  int precision_button_;
  
  double scale_linear_x_;
  double scale_linear_y_;
  double scale_angular_z_;
  double turbo_scale_;
  double precision_scale_;
  double deadzone_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamepadTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
