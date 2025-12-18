#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

class SerialReaderNode : public rclcpp::Node
{
public:
    SerialReaderNode() : Node("serial_reader_node"), serial_fd_(-1)
    {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);

        // Get parameters
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

        // create subscriber
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&SerialReaderNode::cmdVelCallback, this, std::placeholders::_1));

        // Open serial port
        if (!openSerial(port, baudrate))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port opened: %s at %d baud", port.c_str(), baudrate);
        sleep(5);
        sendCommand("kl", {"30"});
        // Create timer to read serial data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SerialReaderNode::readSerial, this));
    }

    ~SerialReaderNode()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
        }
    }

private:
    bool openSerial(const std::string &port, int baudrate)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0)
        {
            return false;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);

        // Set baud rate
        speed_t speed = B115200;
        switch (baudrate)
        {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            speed = B115200;
            break;
        }

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        // 8N1
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // No hardware flow control
        options.c_cflag &= ~CRTSCTS;

        // Enable receiver
        options.c_cflag |= CREAD | CLOCAL;

        // Raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // Raw output
        options.c_oflag &= ~OPOST;

        // Apply settings
        tcsetattr(serial_fd_, TCSANOW, &options);

        return true;
    }

    void readSerial()
    {
        if (serial_fd_ < 0)
        {
            return;
        }

        char buffer[256];
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0)
        {
            buffer[bytes_read] = '\0';

            std::string data(buffer, bytes_read);
            std::string command;
            std::vector<std::string> values;

            // Parse the serial data
            if (parseSerialData(data, command, values))
            {
                // Handle IMU command
                if (command == "imu" && values.size() >= 6)
                {
                    publishImuData(values);
                }
            }

            // Still publish raw data to serial_data topic
            auto message = std_msgs::msg::String();
            message.data = data;
            publisher_->publish(message);
            RCLCPP_DEBUG(this->get_logger(), "Received: %s", buffer);
        }
    }

    bool writeSerial(const std::string &data)
    {
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            return false;
        }

        int bytes_written = write(serial_fd_, data.c_str(), data.length());

        if (bytes_written < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
            return false;
        }

        if (bytes_written != static_cast<int>(data.length()))
        {
            RCLCPP_WARN(this->get_logger(), "Partial write: %d of %zu bytes", bytes_written, data.length());
        }

        return true;
    }

    // Parse incoming serial data: @command:val1;val2;valx;;\r\n
    bool parseSerialData(const std::string &data, std::string &command, std::vector<std::string> &values)
    {
        // Check if data starts with '@'
        if (data.empty() || data[0] != '@')
        {
            return false;
        }

        // Find the colon separator
        size_t colon_pos = data.find(':');
        if (colon_pos == std::string::npos)
        {
            return false;
        }

        // Extract command (between @ and :)
        command = data.substr(1, colon_pos - 1);

        // Find end of data (remove \r\n)
        size_t end_pos = data.find("\r\n");
        if (end_pos == std::string::npos)
        {
            end_pos = data.length();
        }

        // Extract values section
        std::string values_str = data.substr(colon_pos + 1, end_pos - colon_pos - 1);

        // Remove trailing ";;"
        if (values_str.length() >= 2 && values_str.substr(values_str.length() - 2) == ";;")
        {
            values_str = values_str.substr(0, values_str.length() - 2);
        }

        // Split by semicolon
        values.clear();
        size_t start = 0;
        size_t pos = 0;
        while ((pos = values_str.find(';', start)) != std::string::npos)
        {
            values.push_back(values_str.substr(start, pos - start));
            start = pos + 1;
        }
        // Add last value
        if (start < values_str.length())
        {
            values.push_back(values_str.substr(start));
        }

        return true;
    }

    // Send serial data: #command:val1;val2;valx;;\r\n
    bool sendCommand(const std::string &command, const std::vector<std::string> &values)
    {
        std::string message = "#" + command + ":";

        for (size_t i = 0; i < values.size(); ++i)
        {
            message += values[i];
            if (i < values.size() - 1)
            {
                message += ";";
            }
        }

        message += ";;\r\n";
        RCLCPP_INFO(this->get_logger(), "message %s", message.c_str());
        return writeSerial(message);
    }

    // Convert roll, pitch, yaw (degrees) to quaternion
    void eulerToQuaternion(double roll, double pitch, double yaw,
                           double &qx, double &qy, double &qz, double &qw)
    {
        // Convert degrees to radians
        roll = roll * M_PI / 180.0;
        pitch = pitch * M_PI / 180.0;
        yaw = yaw * M_PI / 180.0;

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
    }

    // Publish IMU data from parsed values
    void publishImuData(const std::vector<std::string> &values)
    {
        try
        {
            double roll = std::stod(values[0]);
            double pitch = std::stod(values[1]);
            double yaw = std::stod(values[2]);
            double accel_x = std::stod(values[3]);
            double accel_y = std::stod(values[4]);
            double accel_z = std::stod(values[5]);

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";

            // Convert Euler angles to quaternion
            eulerToQuaternion(roll, pitch, yaw,
                              imu_msg.orientation.x,
                              imu_msg.orientation.y,
                              imu_msg.orientation.z,
                              imu_msg.orientation.w);

            // Set linear acceleration
            imu_msg.linear_acceleration.x = accel_x;
            imu_msg.linear_acceleration.y = accel_y;
            imu_msg.linear_acceleration.z = accel_z;

            // Set covariance (unknown, set to -1)
            imu_msg.orientation_covariance[0] = -1;
            imu_msg.angular_velocity_covariance[0] = -1;
            imu_msg.linear_acceleration_covariance[0] = -1;

            imu_publisher_->publish(imu_msg);

            RCLCPP_DEBUG(this->get_logger(),
                         "IMU: roll=%.2f, pitch=%.2f, yaw=%.2f, ax=%.2f, ay=%.2f, az=%.2f",
                         roll, pitch, yaw, accel_x, accel_y, accel_z);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU data: %s", e.what());
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float speed = 550 * msg->linear.x;
        float steer = -230 * msg->angular.z;
        sendCommand("speed", {std::to_string(speed)});
        sendCommand("steer", {std::to_string(steer)});
    }

    int serial_fd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReaderNode>());
    rclcpp::shutdown();
    return 0;
}