#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"     
#include "geometry_msgs/msg/twist.hpp"

class MyJoy : public rclcpp::Node
{
public:
    MyJoy()
    : Node("teleop_joy_node") 
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&MyJoy::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist_message = geometry_msgs::msg::Twist();
        // Rotation speed
        twist_message.angular.z = msg->axes[0];

        // Linear speed - forward
        if(msg->axes[5] >= 0){
            twist_message.linear.x = -(msg->axes[5]) / 2.0 + 0.5;
        }
        else{
            twist_message.linear.x = (-msg->axes[5] + 1.0) / 2.0;
        }

        // Boost/HighSpeed button (ONLY forward if comes before 'Reverse' logic)           - comment one of us
        // if(msg->buttons[1])
        //     twist_message.linear.x = twist_message.linear.x * 2.0;

        // Reverse (only works if we're not going forward)
        if(twist_message.linear.x <= 0){
            if(msg->axes[2] >= 0){
            twist_message.linear.x = (msg->axes[2]) / 2.0 - 0.5;
            }
            else{
                twist_message.linear.x = (msg->axes[2] - 1.0) / 2.0;
            }
        }

        // Boost/HighSpeed button (ALSO for reverse if comes after 'Reverse' logic)        - comment one of us
        if(msg->buttons[1])
            twist_message.linear.x = twist_message.linear.x * 2.0;

        // Drift (high rotation speed)
        if(msg->buttons[4])
            twist_message.angular.z = twist_message.angular.z * 2.0;

        

        publisher_->publish(twist_message);
        

        RCLCPP_INFO(this->get_logger(), "Publishing velocities - Linear: '%f' , Angular: '%f'", twist_message.linear.x, twist_message.angular.z);
    
    }

    

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyJoy>());
    rclcpp::shutdown();
    return 0;
}
