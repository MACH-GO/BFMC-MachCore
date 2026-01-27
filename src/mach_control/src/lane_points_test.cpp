#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class DummyLanePointsPublisher : public rclcpp::Node
{
public:
    DummyLanePointsPublisher() : Node("dummy_lane_points_pub")
    {
        topic_ = this->declare_parameter<std::string>("topic", "/lane_center");
        rate_hz_ = this->declare_parameter<double>("rate_hz", 30.0);
        amplitude_ = this->declare_parameter<double>("amplitude", 0.6); // max offset (normalized)
        frequency_ = this->declare_parameter<double>("frequency", 0.1); // Hz (slow drift)

        pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_, 10);

        start_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz_),
            std::bind(&DummyLanePointsPublisher::tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "Publishing dummy data lane offset on %s at %.1f Hz",
                    topic_.c_str(), rate_hz_);
    }

private:
    void tick()
    {
        double t = (this->now() - start_time_).seconds();

        // Smooth oscillating lane drift [-amplitude, +amplitude]
        float offset = static_cast<float>(
            amplitude_ * std::sin(2.0 * M_PI * frequency_ * t));

        std_msgs::msg::Float32 msg;
        msg.data = offset;

        pub_->publish(msg);
    }

private:
    std::string topic_;
    double rate_hz_;
    double amplitude_;
    double frequency_;

    rclcpp::Time start_time_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyLanePointsPublisher>());
    rclcpp::shutdown();
    return 0;
}
