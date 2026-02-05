#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class MecanumController : public rclcpp::Node
{
public:
  MecanumController() : Node("mecanum_controller")
  {
    // Robot parametreleri
    wheelbase_ = 0.128;           // metre (ön-arka tekerlek arası)
    wheel_separation_ = 0.310;    // metre (sağ-sol tekerlek arası)
    wheel_radius_ = 0.035;        // metre
    
    lx_ = wheelbase_ / 2.0;       // 0.064
    ly_ = wheel_separation_ / 2.0; // 0.155
    
    // Subscriber: cmd_vel (Twist mesajları)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MecanumController::cmdVelCallback, this, std::placeholders::_1));
    
    // Publisher: wheel velocities (ROS Control'e komut)
    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mecanum_velocity_controller/commands", 10);
    
    RCLCPP_INFO(this->get_logger(), "Mecanum Controller initialized");
    RCLCPP_INFO(this->get_logger(), "  Wheelbase: %.3f m", wheelbase_);
    RCLCPP_INFO(this->get_logger(), "  Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "  Wheel radius: %.3f m", wheel_radius_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Twist mesajından hız bileşenlerini al
    double vx = msg->linear.x;      // İleri/geri hız (m/s)
    double vy = msg->linear.y;      // Yanal hız (m/s)
    double omega = msg->angular.z;  // Açısal hız (rad/s)
    
    // Mecanum kinematik formülleri
    // FL (Front Left), FR (Front Right), RL (Rear Left), RR (Rear Right)
    // 
    // Mecanum wheel kinematics:
    // wheel_fl = (vx - vy - (lx + ly) * ω) / r
    // wheel_fr = (vx + vy + (lx + ly) * ω) / r
    // wheel_rl = (vx + vy - (lx + ly) * ω) / r
    // wheel_rr = (vx - vy + (lx + ly) * ω) / r
    
    double wheel_fl = (vx - vy - (lx_ + ly_) * omega) / wheel_radius_;
    double wheel_fr = (vx + vy + (lx_ + ly_) * omega) / wheel_radius_;
    double wheel_rl = (vx + vy - (lx_ + ly_) * omega) / wheel_radius_;
    double wheel_rr = (vx - vy + (lx_ + ly_) * omega) / wheel_radius_;
    
    // Float64MultiArray mesajı oluştur
    // Sıralama: FL, FR, RL, RR (controller config ile eşleşmeli)
    std_msgs::msg::Float64MultiArray wheel_cmd;
    wheel_cmd.data = {wheel_fl, wheel_fr, wheel_rl, wheel_rr};
    
    // Yayınla
    wheel_cmd_pub_->publish(wheel_cmd);
    
    // Debug log
    RCLCPP_DEBUG(this->get_logger(), 
      "cmd_vel: vx=%.2f vy=%.2f ω=%.2f -> wheels: FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
      vx, vy, omega, wheel_fl, wheel_fr, wheel_rl, wheel_rr);
  }
  
  // Robot parametreleri
  double wheelbase_;
  double wheel_separation_;
  double wheel_radius_;
  double lx_, ly_;
  
  // ROS 2 interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MecanumController>();
  
  RCLCPP_INFO(node->get_logger(), "Spinning mecanum controller node...");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
