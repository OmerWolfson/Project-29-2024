#include <cmath>
#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class MPU6050DriverNode : public rclcpp::Node {
public:
    MPU6050DriverNode() : Node("mpu6050_driver_node"), bus_number_(3), address_(0x68), timer_period_(10) {
        this->declare_parameter("bus_number", bus_number_);
        this->declare_parameter("address", address_);
        std::string topic_name = "mpu6050";
        this->declare_parameter("topic_name", topic_name);
        this->declare_parameter("timer_period", timer_period_);

        // Parameters for LPF cutoff frequencies
        enum class accel_dlpf_frequency_type
        {
            F_218HZ = 0x00,
            F_99HZ = 0x02,
            F_44HZ = 0x03,
            F_21HZ = 0x04,
            F_10HZ = 0x05,
            F_5HZ = 0x06
        };

        enum class gyro_dlpf_frequency_type
        {
            F_250HZ = 0x00,
            F_184HZ = 0x01,
            F_92HZ = 0x02,
            F_41HZ = 0x03,
            F_20Hz = 0x04,
            F_10Hz = 0x05,
            F_5HZ = 0x06
        };

        int gyro_dlpf_frequency, accel_dlpf_frequency;
        this->declare_parameter("gyro_dlpf_frequency", gyro_dlpf_frequency);
        this->declare_parameter("accel_dlpf_frequency", accel_dlpf_frequency);
        
        // Set the LPF frequencies using the enum values
        gyro_dlpf_frequency = static_cast<int>(gyro_dlpf_frequency_type::F_250HZ);
        accel_dlpf_frequency = static_cast<int>(accel_dlpf_frequency_type::F_218HZ);

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name, 10);

        // Open the I2C bus
        char bus_path[20];
        sprintf(bus_path, "/dev/i2c-%d", bus_number_);
        if ((file_ = open(bus_path, O_RDWR)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the i2c bus");
            return;
        }
        if (ioctl(file_, I2C_SLAVE, address_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to access the i2c slave at address 0x%X", address_);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Serial connection successfully connected.");

        // Initialize MPU-6050
        initialize_mpu6050(gyro_dlpf_frequency, accel_dlpf_frequency);

        // Timer setup
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_),
            std::bind(&MPU6050DriverNode::publish_imu_data, this));
    }

    ~MPU6050DriverNode() {
        close(file_);
    }

private:

    void initialize_mpu6050(int gyro_dlpf_frequency, int accel_dlpf_frequency) {
        write_register(PWR_MGMT_1, 0x00);
        write_register(CONFIG, static_cast<uint8_t>(gyro_dlpf_frequency)); // gyro LPF
        write_register(ACCEL_CONFIG_2, static_cast<uint8_t>(accel_dlpf_frequency)); // accel LPF
        write_register(SAMPLE_RATE_DIVIDER, 0x09); // rate
        RCLCPP_INFO(this->get_logger(), "MPU6050 successfully initialized.");
    }

    void write_register(uint8_t reg, uint8_t value) {
        uint8_t buffer[2];
        buffer[0] = reg;
        buffer[1] = value;
        if (write(file_, buffer, 2) != 2) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to the i2c bus");
        }
    }

    int read_registers(uint8_t start_reg, uint8_t* buffer, size_t length) {
        if (write(file_, &start_reg, 1) != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set start register");
            return -1;
        }

        if (read(file_, buffer, length) != length) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read from the i2c bus");
            return -1;
        }

        return 0;
    }
                                              
    void publish_imu_data() {
        uint8_t buffer[14];
        read_registers(ACCEL_XOUT_H, buffer, 14);

        // Process raw data
        int16_t ax = (buffer[0] << 8) | buffer[1];
        int16_t ay = (buffer[2] << 8) | buffer[3];
        int16_t az = (buffer[4] << 8) | buffer[5];
        int16_t gx = (buffer[8] << 8) | buffer[9];
        int16_t gy = (buffer[10] << 8) | buffer[11];
        int16_t gz = (buffer[12] << 8) | buffer[13];

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.linear_acceleration.x = convert_accel(ax);
        imu_msg.linear_acceleration.y = convert_accel(ay);
        imu_msg.linear_acceleration.z = convert_accel(az);
        imu_msg.angular_velocity.x = convert_gyro(gx);
        imu_msg.angular_velocity.y = convert_gyro(gy);
        imu_msg.angular_velocity.z = convert_gyro(gz);

        imu_pub_->publish(imu_msg);
    }

    float convert_accel(int16_t raw) {
        return static_cast<float>(raw) / 16384.0f * 9.81f; // 16384 LSB/g for 2g, adjust as needed
    }

    float convert_gyro(int16_t raw) {
        return static_cast<float>(raw) / 131.0f * (M_PI / 180.0f); // 131 LSB/(º/s), adjust as needed
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int file_;
    int bus_number_;
    int address_;
    int timer_period_;

    // MPU-6050 register addresses
    const uint8_t PWR_MGMT_1 = 0x6B;
    const uint8_t CONFIG = 0x1A;
    const uint8_t ACCEL_CONFIG_2 = 0x1D;
    const uint8_t SAMPLE_RATE_DIVIDER = 0x19;
    const uint8_t ACCEL_XOUT_H = 0x3B;
    const uint8_t GYRO_XOUT_H = 0x44;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050DriverNode>());
    rclcpp::shutdown();
    return 0;
}
