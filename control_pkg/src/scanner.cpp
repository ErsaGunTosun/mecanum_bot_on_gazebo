#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class Scanner: public rclcpp::Node
{
public:
    Scanner(): Node("scanner")
    {
        subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/obstacle_data", 10,
            std::bind(&Scanner::obstacle_callback, this, std::placeholders::_1)
        );

        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom",10,
            std::bind(&Scanner::odom_callback,this,std::placeholders::_1)
        );
        
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Scanner::control_loop, this)
        );
        
        current_state = State::GO;
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    enum class State {
        GO,
        STOP,
        TURN_RIGHT,
        TURN_LEFT,
        LEFT,
        RIGHT
    };	

	State current_state;

    // control
    float front, right, left, back;
    bool is_start = false;
    float min_front_distance = 0.45;
    float min_side_difference = 0.30;
    double desired_yaw;
    

    // odom 
    double start_x,start_y,start_yaw;
	double current_x,current_y,current_yaw;


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
	{
			current_x = odom->pose.pose.position.x;
			current_y = odom->pose.pose.position.y;
			
			tf2::Quaternion q;
			tf2::fromMsg(odom->pose.pose.orientation,q);
			tf2::Matrix3x3 m(q);
			double roll,pitch,yaw;
			m.getRPY(roll,pitch,yaw);

			current_yaw = yaw;
			if(!is_start){
				start_x = current_x;
				start_y = current_y;
				start_yaw = current_yaw;
				is_start = true;
			}
	}

    void obstacle_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
            front = msg->data[0];
            right = msg->data[1];
            left  = msg->data[2];
            back  = msg->data[3];

            if(!is_start)
                is_start = true;
    }

    void correct_heading(geometry_msgs::msg::Twist& velocity) {
        double current_deg = current_yaw * 180.0 / M_PI;
        double nearest_90 = round(current_deg / 90.0) * 90.0;
        
        double error_deg = nearest_90 - current_deg;
        
        if (fabs(error_deg) > 2.0) {
            double error_rad = error_deg * M_PI / 180.0;
        
            double correction = std::max(-0.15, std::min(0.15, error_rad * 0.3));
            velocity.angular.z = correction;
        }
    }
   

    void control_loop(){
        
        std::string state_str = (current_state == State::GO) ? "GO" : 
                               (current_state == State::TURN_RIGHT) ? "TURN_RIGHT" : 
                               (current_state == State::TURN_LEFT) ? "TURN_LEFT" : "STOP";

        auto velocity = geometry_msgs::msg::Twist();

        if(current_state == State::GO){
            double distance_traveled = sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
            
           if(right > min_side_difference){
                current_state = State::RIGHT;
           }
        }
        else if (current_state == State::TURN_LEFT){

        }
        else if (current_state == State::TURN_RIGHT){
        }
        else if (current_state == State::RIGHT){
            if (right <= min_side_difference)
                current_state = State::GO;
            
            velocity.linear.x = 0.0;
            velocity.linear.y = -0.2;
            velocity.linear.z = 0;
            RCLCPP_INFO(get_logger(),"Sağa yaklaşılıyor %f",right);
        }

        publisher_->publish(velocity);

    }

 

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Scanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}