#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class AckermannToTwistStampedNode : public rclcpp::Node
{
public:
    AckermannToTwistStampedNode() : Node("ackermann_to_twist_stamped")
    {
        wheelbase_ = this->declare_parameter<double>("wheelbase", 0.255);

        // Topics
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/drive_cmd");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/ackermann_controller/reference");

        frame_id_ = this->declare_parameter<std::string>("frame_id", "base_link");

        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, rclcpp::QoS(10));

        sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            input_topic_, rclcpp::QoS(10),
            std::bind(&AckermannToTwistStampedNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "ackermann_to_twist_stamped: %s (AckermannDriveStamped) -> %s (TwistStamped), wheelbase=%.3f m",
            input_topic_.c_str(), output_topic_.c_str(), wheelbase_);
    }

private:
    void callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        const double v = msg->drive.speed;              // m/s
        const double delta = msg->drive.steering_angle; // rad

        geometry_msgs::msg::TwistStamped out;
        out.header.stamp = this->now();
        out.header.frame_id = frame_id_;

        out.twist.linear.x = v;

        // Ackermann kinematics: yaw_rate = v * tan(delta) / L
        out.twist.angular.z = (std::abs(v) < 1e-6) ? 0.0 : (v * std::tan(delta) / wheelbase_);

        pub_->publish(out);
    }

    double wheelbase_;
    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToTwistStampedNode>());
    rclcpp::shutdown();
    return 0;
}
