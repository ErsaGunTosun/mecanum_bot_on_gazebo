/**
 * Mecanum Kinematics Node
 * 
 * Subscribes to /cmd_vel_mecanum (desired mecanum motion)
 * Publishes to /cmd_vel (for diff_drive plugin)
 * 
 * Converts mecanum kinematics to differential drive approximation
 * This allows testing mecanum control logic while using stable diff_drive in simulation
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MecanumKinematicsNode : public rclcpp::Node
{
public:
    MecanumKinematicsNode() : Node("mecanum_kinematics_node")
    {
        // Subscribe to mecanum command
        mecanum_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_mecanum", 10,
            std::bind(&MecanumKinematicsNode::mecanumCmdCallback, this, std::placeholders::_1));

        // Publish to diff drive
        diff_drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Mecanum Kinematics Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribe: /cmd_vel_mecanum");
        RCLCPP_INFO(this->get_logger(), "Publish: /cmd_vel (for diff_drive)");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Mecanum motion mapping:");
        RCLCPP_INFO(this->get_logger(), "  - Forward/Backward: Direct mapping");
        RCLCPP_INFO(this->get_logger(), "  - Strafe Left/Right: Simulated (robot will rotate slightly)");
        RCLCPP_INFO(this->get_logger(), "  - Rotation: Direct mapping");
    }

private:
    void mecanumCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto diff_cmd = geometry_msgs::msg::Twist();

        // Extract mecanum velocities
        double vx = msg->linear.x;   // Forward/backward
        double vy = msg->linear.y;   // Strafe left/right (mecanum special!)
        double wz = msg->angular.z;  // Rotation

        // Diff drive can only do vx and wz
        // For vy (strafe), we approximate by combining rotation and forward motion
        
        if (std::abs(vy) > 0.01) {
            // Strafe motion requested
            // Strategy: Convert strafe to diagonal motion
            // This is an approximation - real mecanum would move purely sideways
            
            // Combine vx and vy into diagonal motion
            double diagonal_speed = std::sqrt(vx * vx + vy * vy);
            double angle = std::atan2(vy, vx);
            
            // For pure strafe (vx=0, vy!=0), rotate robot to face strafe direction
            // then move forward
            if (std::abs(vx) < 0.01) {
                // Pure strafe - convert to rotation + forward motion sequence
                // This is a simplified approximation
                diff_cmd.linear.x = 0.0;
                diff_cmd.angular.z = vy * 2.0;  // Rotate to strafe
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Strafe motion: vy=%.2f -> approximated with rotation", vy);
            } else {
                // Combined motion - use diagonal
                diff_cmd.linear.x = diagonal_speed * std::cos(angle);
                diff_cmd.angular.z = wz + vy * 0.5;  // Add some rotation for strafe effect
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Diagonal motion: vx=%.2f, vy=%.2f -> vx=%.2f, wz=%.2f", 
                    vx, vy, diff_cmd.linear.x, diff_cmd.angular.z);
            }
        } else {
            // No strafe - direct mapping
            diff_cmd.linear.x = vx;
            diff_cmd.angular.z = wz;
            
            if (std::abs(vx) > 0.01 || std::abs(wz) > 0.01) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Direct motion: vx=%.2f, wz=%.2f", vx, wz);
            }
        }

        // Publish to diff drive
        diff_drive_pub_->publish(diff_cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mecanum_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_drive_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
