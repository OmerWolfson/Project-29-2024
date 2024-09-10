#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "sensor_msgs/msg/joint_state.hpp"

class MyDiffDrive : public rclcpp::Node
{
public:
    MyDiffDrive()
    : Node("my_diff_drive_controller")  // used to be just joint_publisher
    {
        gazebo_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_velocity_controller/commands", 10);
        // If using Isaac Sim switch to using this publisher:
        // isaac_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        //     "/isaac_joint_commands", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MyDiffDrive::cmd_vel_callback, this, std::placeholders::_1));
        
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto vel_message = std_msgs::msg::Float64MultiArray();
        

        vel_message.data.resize(2);

        vel_message.data[0] = (msg->linear.x + 0.77 * msg->angular.z / 2) / 0.16;
        vel_message.data[1] = (msg->linear.x - 0.77 * msg->angular.z / 2) / 0.16;

        // auto isaac_vel_message = sensor_msgs::msg::JointState();
        // //isaac_vel_message.header = // time stamp

        // isaac_vel_message.name[0] = "right_wheel_joint";
        // isaac_vel_message.velocity[0] = (msg->linear.x + 0.77 * msg->angular.z / 2) / 0.16;

        // isaac_vel_message.name[1] = "left_wheel_joint";
        // isaac_vel_message.velocity[1] = (msg->linear.x - 0.77 * msg->angular.z / 2) / 0.16;

        gazebo_publisher_->publish(vel_message);
        // isaac_publisher_->publish(isaac_vel_message);

        RCLCPP_INFO(this->get_logger(), "Publishing velocities - Right: '%f' , Left: '%f'", vel_message.data[0], vel_message.data[1]);
    
    }

    

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gazebo_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyDiffDrive>());
    rclcpp::shutdown();
    return 0;
}
