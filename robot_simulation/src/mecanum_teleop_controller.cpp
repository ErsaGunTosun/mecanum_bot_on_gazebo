/**
 * Mecanum Drive Teleop Controller with Force Control
 * 
 * Subscribes to /cmd_vel (Twist) and /joint_states
 * Publishes force commands to individual wheels using PID velocity control
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <map>
#include <string>

class MecanumTeleopController : public rclcpp::Node
{
public:
    MecanumTeleopController() : Node("mecanum_teleop_controller")
    {
        // Subscribe to cmd_vel and joint_states
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MecanumTeleopController::cmdVelCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MecanumTeleopController::jointStateCallback, this, std::placeholders::_1));

        // Publishers for force commands
        fl_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/robot/joint/wheel_front_left_joint/force", 10);
        fr_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/robot/joint/wheel_front_right_joint/force", 10);
        rl_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/robot/joint/wheel_rear_left_joint/force", 10);
        rr_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/robot/joint/wheel_rear_right_joint/force", 10);

        // Robot parameters
        wheel_radius_ = 0.035;  // 35mm from URDF
        wheel_base_ = 0.266;    // Distance between front and rear wheels
        track_width_ = 0.310;   // Distance between left and right wheels

        // PID gains for velocity control
        kp_ = 50000.0;   // Extreme gain - last attempt
        ki_ = 0.0;       // Integral gain (disabled for now)
        kd_ = 100.0;     // Derivative gain

        // Initialize desired velocities to zero
        desired_vel_["wheel_front_left_joint"] = 0.0;
        desired_vel_["wheel_front_right_joint"] = 0.0;
        desired_vel_["wheel_rear_left_joint"] = 0.0;
        desired_vel_["wheel_rear_right_joint"] = 0.0;

        // Initialize actual velocities to zero
        actual_vel_["wheel_front_left_joint"] = 0.0;
        actual_vel_["wheel_front_right_joint"] = 0.0;
        actual_vel_["wheel_rear_left_joint"] = 0.0;
        actual_vel_["wheel_rear_right_joint"] = 0.0;

        // Control loop timer (100Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MecanumTeleopController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Mecanum Teleop Controller (Force Control) started");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update actual wheel velocities from joint states
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (actual_vel_.find(msg->name[i]) != actual_vel_.end()) {
                if (i < msg->velocity.size()) {
                    actual_vel_[msg->name[i]] = msg->velocity[i];
                }
            }
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract velocities from Twist message
        double vx = msg->linear.x;   // Forward/backward velocity
        double vy = msg->linear.y;   // Strafe left/right velocity
        double wz = msg->angular.z;  // Rotation velocity

        // Mecanum wheel kinematics
        double lx = (wheel_base_ + track_width_) / 2.0;
        
        // Calculate desired wheel velocities (rad/s)
        desired_vel_["wheel_front_left_joint"] = (vx - vy - wz * lx) / wheel_radius_;
        desired_vel_["wheel_front_right_joint"] = (vx + vy + wz * lx) / wheel_radius_;
        desired_vel_["wheel_rear_left_joint"] = (vx + vy - wz * lx) / wheel_radius_;
        desired_vel_["wheel_rear_right_joint"] = (vx - vy + wz * lx) / wheel_radius_;
    }

    void controlLoop()
    {
        // PID control for each wheel
        auto fl_force = computeForce("wheel_front_left_joint");
        auto fr_force = computeForce("wheel_front_right_joint");
        auto rl_force = computeForce("wheel_rear_left_joint");
        auto rr_force = computeForce("wheel_rear_right_joint");

        // Debug logging (only when there's a command)
        if (desired_vel_["wheel_front_left_joint"] != 0.0 || 
            desired_vel_["wheel_front_right_joint"] != 0.0 ||
            desired_vel_["wheel_rear_left_joint"] != 0.0 ||
            desired_vel_["wheel_rear_right_joint"] != 0.0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Forces: FL=%.2f FR=%.2f RL=%.2f RR=%.2f | Desired: FL=%.2f FR=%.2f RL=%.2f RR=%.2f | Actual: FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
                fl_force, fr_force, rl_force, rr_force,
                desired_vel_["wheel_front_left_joint"], desired_vel_["wheel_front_right_joint"],
                desired_vel_["wheel_rear_left_joint"], desired_vel_["wheel_rear_right_joint"],
                actual_vel_["wheel_front_left_joint"], actual_vel_["wheel_front_right_joint"],
                actual_vel_["wheel_rear_left_joint"], actual_vel_["wheel_rear_right_joint"]);
        }

        // Publish forces
        auto fl_msg = std_msgs::msg::Float64();
        auto fr_msg = std_msgs::msg::Float64();
        auto rl_msg = std_msgs::msg::Float64();
        auto rr_msg = std_msgs::msg::Float64();

        fl_msg.data = fl_force;
        fr_msg.data = fr_force;
        rl_msg.data = rl_force;
        rr_msg.data = rr_force;

        fl_pub_->publish(fl_msg);
        fr_pub_->publish(fr_msg);
        rl_pub_->publish(rl_msg);
        rr_pub_->publish(rr_msg);
    }

    double computeForce(const std::string& joint_name)
    {
        // PID control: F = Kp * error + Kd * d(error)/dt
        double desired = desired_vel_[joint_name];
        double actual = actual_vel_[joint_name];
        double error = desired - actual;

        // Simple PD control (no integral for stability)
        double force = kp_ * error;

        // No force limit - let's see what happens
        return force;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fr_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rr_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    double wheel_radius_;
    double wheel_base_;
    double track_width_;
    double kp_, ki_, kd_;

    std::map<std::string, double> desired_vel_;
    std::map<std::string, double> actual_vel_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumTeleopController>());
    rclcpp::shutdown();
    return 0;
}
