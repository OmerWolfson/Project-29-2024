#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class Ur10eControllerNode : public rclcpp::Node
{
public:
    Ur10eControllerNode()
    : Node("ur10e_position_controller"), angle_(0.0)  // used to be just joint_publisher
    {
        // If using Isaac Sim switch to using this publisher: (when running use namespace ur10e${id number})
        isaac_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "isaac_joint_commands", 10);


        auto position = sensor_msgs::msg::JointState();
        position.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        position.position = {2.3, -1.5708, -1.5708, 0.0, 0.0, 0.0};

        //position.position = {0.0, 0.0, -1.5708, 0.0};
        // position.position = {1.2708, -1.5708, -2.9, 1.5, 0.0, 0.0}; final position
        // shoulder up/back - negative
        // eblow up/back - negative

        isaac_publisher_->publish(position);

        step_1_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1400),
            std::bind(&Ur10eControllerNode::CallbackStep1, this));
        step_2_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3200),
            std::bind(&Ur10eControllerNode::CallbackStep2, this));
    }

private:
    void CallbackStep1()
    {
        step_1_timer_->cancel();

        auto position = sensor_msgs::msg::JointState();
        
        position.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        position.position = {3.14, -1.5708, 0.0, 0.0, 0.0, 0.0}; // -2.5 as for mid point to reach 2.9

        isaac_publisher_->publish(position);

        // RCLCPP_INFO(this->get_logger(), "Publishing velocities - Right: '%f' , Left: '%f'", vel_message.data[0], vel_message.data[1]);
    
    }

    void CallbackStep2()
    {
        // step_2_timer_->cancel();

        auto position = sensor_msgs::msg::JointState();
        
        position.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        

        position.position = {3.14, -1.5708, 0.0, 0.0, 0.0, 0.0};

        isaac_publisher_->publish(position);

        loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3200),
            std::bind(&Ur10eControllerNode::spin_loop, this));
    
    }

    void spin_loop()
    {
        angle_ += 0.2;
        auto position = sensor_msgs::msg::JointState();
        
        position.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        position.position = {angle_, -1.5708, 0.0, 0.0, 0.0, 0.0};

        isaac_publisher_->publish(position);

    }

    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr isaac_publisher_;
    rclcpp::TimerBase::SharedPtr step_1_timer_;
    rclcpp::TimerBase::SharedPtr step_2_timer_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    float angle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ur10eControllerNode>());
    rclcpp::shutdown();
    return 0;
}
