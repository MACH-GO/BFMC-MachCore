#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>

class AckermannTeleopXbox : public rclcpp::Node
{
public:
    AckermannTeleopXbox() : Node("ackermann_teleop_xbox")
    {
        // Declare parameters
        this->declare_parameter<double>("max_speed", 2.0);
        this->declare_parameter<double>("max_steering_angle", 0.4);
        this->declare_parameter<double>("publish_rate", 20.0);
        this->declare_parameter<int>("speed_axis", 1);     // Left stick Y (RT=5, LT=2 for triggers)
        this->declare_parameter<int>("steering_axis", 3);  // Right stick X (Left stick X=0)
        this->declare_parameter<int>("deadman_button", 4); // LB button
        this->declare_parameter<int>("turbo_button", 5);   // RB button
        this->declare_parameter<double>("turbo_multiplier", 2.0);
        this->declare_parameter<double>("deadzone", 0.1);

        // Get parameters
        max_speed_ = this->get_parameter("max_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        speed_axis_ = this->get_parameter("speed_axis").as_int();
        steering_axis_ = this->get_parameter("steering_axis").as_int();
        deadman_button_ = this->get_parameter("deadman_button").as_int();
        turbo_button_ = this->get_parameter("turbo_button").as_int();
        turbo_multiplier_ = this->get_parameter("turbo_multiplier").as_double();
        deadzone_ = this->get_parameter("deadzone").as_double();

        // Create subscriber and publisher
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&AckermannTeleopXbox::joyCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_cmd", 10);

        // Initialize state
        target_speed_ = 0.0;
        target_steering_ = 0.0;
        deadman_pressed_ = false;
        turbo_pressed_ = false;

        // Publish at fixed rate
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        publish_timer_ = this->create_wall_timer(
            period,
            std::bind(&AckermannTeleopXbox::publishCommand, this));

        RCLCPP_INFO(this->get_logger(), "\nAckermann Teleop Xbox Controller");
        RCLCPP_INFO(this->get_logger(), "--------------------------------");
        RCLCPP_INFO(this->get_logger(), "Left Stick Y  : Forward/Reverse speed");
        RCLCPP_INFO(this->get_logger(), "Right Stick X : Steering");
        RCLCPP_INFO(this->get_logger(), "LB Button     : Deadman switch (hold to enable)");
        RCLCPP_INFO(this->get_logger(), "RB Button     : Turbo mode");
        RCLCPP_INFO(this->get_logger(), "Start Button  : Emergency stop\n");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check if we have enough axes and buttons
        if (msg->axes.size() <= static_cast<size_t>(std::max(speed_axis_, steering_axis_)) ||
            msg->buttons.size() <= static_cast<size_t>(std::max(deadman_button_, turbo_button_)))
        {
            return;
        }

        // Check deadman switch (LB button must be held)
        deadman_pressed_ = msg->buttons[deadman_button_] == 1;

        // Check turbo mode (RB button)
        turbo_pressed_ = msg->buttons[5] == 1;

        // Emergency stop (Start button = button 7)
        if (msg->buttons.size() > 7 && msg->buttons[7] == 1)
        {
            target_speed_ = 0.0;
            target_steering_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP");
            return;
        }

        if (deadman_pressed_)
        {
            // Read axes with deadzone
            double raw_speed = msg->axes[speed_axis_];
            double raw_steering = msg->axes[steering_axis_];

            // Apply deadzone
            raw_speed = applyDeadzone(raw_speed);
            raw_steering = applyDeadzone(raw_steering);

            // Calculate speed (with turbo)
            double speed_multiplier = turbo_pressed_ ? turbo_multiplier_ : 1.0;
            target_speed_ = raw_speed * max_speed_ * speed_multiplier;

            // Calculate steering (invert if needed for natural control)
            target_steering_ = -raw_steering * max_steering_angle_;

            // Clamp values
            target_speed_ = std::clamp(target_speed_, -max_speed_ * speed_multiplier, max_speed_ * speed_multiplier);
            target_steering_ = std::clamp(target_steering_, -max_steering_angle_, max_steering_angle_);
        }
        else
        {
            // Deadman not pressed - stop the vehicle
            target_speed_ = 0.0;
            target_steering_ = 0.0;
        }
    }

    double applyDeadzone(double value)
    {
        if (std::abs(value) < deadzone_)
        {
            return 0.0;
        }
        // Scale the remaining range to full range
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone_) / (1.0 - deadzone_);
    }

    void publishCommand()
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.drive.speed = target_speed_;
        msg.drive.steering_angle = target_steering_;

        publisher_->publish(msg);

        // Log status periodically
        static int counter = 0;
        if (++counter % 20 == 0) // Log once per second at 20Hz
        {
            if (deadman_pressed_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Speed: %.2f | Steering: %.2f | Turbo: %s",
                            target_speed_, target_steering_, turbo_pressed_ ? "ON" : "OFF");
            }
        }
    }

    // Parameters
    double max_speed_;
    double max_steering_angle_;
    double turbo_multiplier_;
    double deadzone_;
    int speed_axis_;
    int steering_axis_;
    int deadman_button_;
    int turbo_button_;

    // State
    double target_speed_;
    double target_steering_;
    bool deadman_pressed_;
    bool turbo_pressed_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannTeleopXbox>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}