#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MyIsaacDiffDrive : public rclcpp::Node
{
public:
    MyIsaacDiffDrive()
    : Node("isaac_diff_drive_controller")  // used to be just joint_publisher
    {
        // use namespace to command the right robot
        isaac_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "isaac_joint_commands", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MyIsaacDiffDrive::cmd_vel_callback, this, std::placeholders::_1));

        auto isaac_vel_message = sensor_msgs::msg::JointState();

        isaac_vel_message.name.resize(2);
        isaac_vel_message.position.resize(2);

        isaac_vel_message.name[0] = "right_wheel_joint";
        isaac_vel_message.position[0] = 0.0;

        isaac_vel_message.name[1] = "left_wheel_joint";
        isaac_vel_message.position[1] = 0.0;
        
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto isaac_vel_message = sensor_msgs::msg::JointState();

        isaac_vel_message.name.resize(2);
        isaac_vel_message.velocity.resize(2);

        isaac_vel_message.name[0] = "right_wheel_joint";
        isaac_vel_message.velocity[0] = (msg->linear.x + 0.77 * msg->angular.z / 2) / 0.16;

        isaac_vel_message.name[1] = "left_wheel_joint";
        isaac_vel_message.velocity[1] = (msg->linear.x - 0.77 * msg->angular.z / 2) / 0.16;

        isaac_publisher_->publish(isaac_vel_message);

        RCLCPP_INFO(this->get_logger(), "Publishing velocities - Right: '%f' , Left: '%f'", isaac_vel_message.velocity[0], isaac_vel_message.velocity[1]);
    
    }

    

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr isaac_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyIsaacDiffDrive>());
    rclcpp::shutdown();
    return 0;
}
