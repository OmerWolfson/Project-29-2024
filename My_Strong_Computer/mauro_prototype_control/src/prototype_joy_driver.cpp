#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"     
#include "mauro_prototype_control/prototype_joy_controller_class.hpp"

#include "project29_interfaces/msg/ser_com_struct.hpp"

class PrototypeJoy : public rclcpp::Node
{
public:
    PrototypeJoy()
    : Node("prototype_joy_driver")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&PrototypeJoy::callback_func, this, std::placeholders::_1));

        // publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        //     "/joint_velocity_controller/commands", 10);
        publisher_ = this->create_publisher<project29_interfaces::msg::SerComStruct>(
            "motor_throttle_control", 10);
    }

private:
    void callback_func(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Comments reffer to Sony's PlayStation controller 
        if(msg->buttons[0]) // pressing 'X' shifts to high gear
            is_high_gear_ = true;
        else if(msg->buttons[3]) // pressing 'square' shifts to low gear
            is_high_gear_ = false;

        // Forward
        if(msg->axes[5] >= 0){
            throttle_ = -(msg->axes[5]) / 2.0 + 0.5;
        }
        else{
            throttle_ = (-msg->axes[5] + 1.0) / 2.0;
        }

        // Reverse (only works if we're not going forward)
        if(throttle_ <= 0){
            if(msg->axes[2] >= 0){
            throttle_ = (msg->axes[2]) / 2.0 - 0.5;
            }
            else{
                throttle_ = (msg->axes[2] - 1.0) / 2.0;
            }
        }
        
        PrototypeControl control;
        SerialCom ser_com = control.compute_control(throttle_*100.0, msg->axes[0]*100.0, is_high_gear_);

        auto pwm_struct = project29_interfaces::msg::SerComStruct();
        pwm_struct.pwm_l = ser_com.pwm_L * 0.594177; // if using % output like RPi
        pwm_struct.pwm_r = ser_com.pwm_R * 0.594177;
        pwm_struct.is_reverse_dir_l = ser_com.is_reverse_dir_L;
        pwm_struct.is_reverse_dir_r = ser_com.is_reverse_dir_R;
        pwm_struct.gear = ser_com.G;
        publisher_->publish(pwm_struct);

        // RCLCPP_INFO(this->get_logger(), "Right wheel speed : '%f'", vel_message.data[0]);
        // RCLCPP_INFO(this->get_logger(), "Left wheel speed : '%f'", vel_message.data[1]);
        // RCLCPP_INFO(this->get_logger(), "Right wheel PWM : '%d'", phi_right_);
        // RCLCPP_INFO(this->get_logger(), "Left wheel PWM : '%d'", phi_left_);
        // RCLCPP_INFO(this->get_logger(), "High gear : '%s'", ser_com.G? "true" : "false");

    }

    rclcpp::Publisher<project29_interfaces::msg::SerComStruct>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    bool is_high_gear_ = false; // Default low gear configuration
    double throttle_;
    int phi_right_;
    int phi_left_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrototypeJoy>());
    rclcpp::shutdown();
    return 0;
}