#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

class AckermannToTwistNode : public rclcpp::Node
{
public:
    AckermannToTwistNode() : Node("ackermann_to_twist")
    {
        wheelbase_ = this->declare_parameter<double>("wheelbase", 0.255);

        // Topics
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/drive_cmd");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel");

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(10));

        sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            input_topic_, rclcpp::QoS(10),
            std::bind(&AckermannToTwistNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "ackermann_to_twist: %s (AckermannDriveStamped) -> %s (Twist), wheelbase=%.3f m",
                    input_topic_.c_str(), output_topic_.c_str(), wheelbase_);
    }

private:
    void callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        const double v = msg->drive.speed;              // m/s
        const double delta = msg->drive.steering_angle; // rad

        geometry_msgs::msg::Twist out;
        out.linear.x = v;

        // Ackermann kinematics: yaw_rate = v * tan(delta) / L
        out.angular.z = (std::abs(v) < 1e-6) ? 0.0 : (v * std::tan(delta) / wheelbase_);

        pub_->publish(out);
    }

    double wheelbase_;
    std::string input_topic_;
    std::string output_topic_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToTwistNode>());
    rclcpp::shutdown();
    return 0;
}
