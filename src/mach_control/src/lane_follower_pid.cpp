#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class LaneFollowerPID : public rclcpp::Node
{
public:
    LaneFollowerPID() : Node("lane_follower_pid")
    {
        // Topics
        lane_topic_ = this->declare_parameter<std::string>("lane_topic", "/lane_center");
        cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/drive_cmd");

        // PID gains
        kp_ = this->declare_parameter<double>("kp", 1.0);
        ki_ = this->declare_parameter<double>("ki", 0.0);
        kd_ = this->declare_parameter<double>("kd", 0.15);

        // Input scaling (use -1.0 if sign is flipped)
        input_scale_ = this->declare_parameter<double>("input_scale", -1.0);

        // Integral clamp (anti-windup)
        i_max_ = this->declare_parameter<double>("i_max", 0.5);

        // Steering limits / smoothing
        steer_max_rad_ = this->declare_parameter<double>("steer_max_rad", 0.4);
        steer_rate_limit_rad_s_ = this->declare_parameter<double>("steer_rate_limit_rad_s", 2.0);

        // Speed
        speed_base_ = this->declare_parameter<double>("speed_base", 0.4);               // m/s
        speed_min_ = this->declare_parameter<double>("speed_min", 0.2);                 // m/s
        speed_steer_scale_ = this->declare_parameter<double>("speed_steer_scale", 1.5); // slow down more on turns

        // Timeout for commands
        cmd_timeout_s_ = this->declare_parameter<double>("cmd_timeout_s", 0.5);

        // Publisher & subscriber
        pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(cmd_topic_, 10);

        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            lane_topic_, 10,
            std::bind(&LaneFollowerPID::followLane, this, std::placeholders::_1));

        // State variables
        last_time_ = this->now();
        last_lane_time_ = this->now();
        prev_err_ = 0.0;
        integral_ = 0.0;
        prev_steer_ = 0.0;

        // Command timeout watchdog
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LaneFollowerPID::timerUpdate, this));

        RCLCPP_INFO(this->get_logger(),
                    "LaneFollowerPID: sub=%s pub=%s (kp=%.3f ki=%.3f kd=%.3f)",
                    lane_topic_.c_str(), cmd_topic_.c_str(), kp_, ki_, kd_);
    }

private:
    static double clamp(double x, double lo, double hi)
    {
        return std::max(lo, std::min(hi, x));
    }

    void followLane(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 1e-6)
            dt = 1e-6;
        last_time_ = now;
        last_lane_time_ = now;

        // Lane error (normalized in [-1, 1])
        const double err = static_cast<double>(msg->data) * input_scale_;

        // PID
        integral_ += err * dt;
        integral_ = clamp(integral_, -i_max_, i_max_);

        const double derivative = (err - prev_err_) / dt;
        prev_err_ = err;

        double steer = kp_ * err + ki_ * integral_ + kd_ * derivative;

        // Clamp steer angle
        steer = clamp(steer, -steer_max_rad_, steer_max_rad_);

        // steer rate limiting (smoothness)
        const double max_delta = steer_rate_limit_rad_s_ * dt;
        steer = clamp(steer, prev_steer_ - max_delta, prev_steer_ + max_delta);
        prev_steer_ = steer;

        // Slow down as steering increases
        double speed = speed_base_;
        if (steer_max_rad_ > 1e-6)
        {
            const double turn_frac = std::abs(steer) / steer_max_rad_; // 0..1
            speed = speed_base_ * (1.0 - speed_steer_scale_ * turn_frac);
        }
        speed = clamp(speed, speed_min_, speed_base_);

        // Publish command
        ackermann_msgs::msg::AckermannDriveStamped out;
        out.header.stamp = now;
        out.header.frame_id = "base_link";
        out.drive.steering_angle = steer;
        out.drive.speed = speed;

        pub_->publish(out);
    }

    void timerUpdate()
    {
        // If we stop receiving lane updates, publish a stop command
        const double since = (this->now() - last_lane_time_).seconds();
        if (since <= cmd_timeout_s_)
            return;

        ackermann_msgs::msg::AckermannDriveStamped out;
        out.header.stamp = this->now();
        out.header.frame_id = "base_link";
        out.drive.steering_angle = 0.0;
        out.drive.speed = 0.0;
        pub_->publish(out);
    }

private:
    // Topics
    std::string lane_topic_;
    std::string cmd_topic_;

    // PID params
    double kp_{1.0}, ki_{0.0}, kd_{0.15};
    double input_scale_{1.0};
    double i_max_{0.5};

    // Steering limits
    double steer_max_rad_{0.45};
    double steer_rate_limit_rad_s_{2.0};

    // Speed params
    double speed_base_{1.5};
    double speed_min_{0.6};
    double speed_steer_scale_{1.2};

    // Watchdog
    double cmd_timeout_s_{0.5};

    // State
    rclcpp::Time last_time_;
    rclcpp::Time last_lane_time_;
    double prev_err_{0.0};
    double integral_{0.0};
    double prev_steer_{0.0};

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollowerPID>());
    rclcpp::shutdown();
    return 0;
}
