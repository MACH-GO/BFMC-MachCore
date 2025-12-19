#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class AckermannController : public rclcpp::Node
{
public:
    AckermannController() : Node("ackermann_controller")
    {
        // Declare parameters
        this->declare_parameter<double>("wheelbase", 0.26);            // meters
        this->declare_parameter<double>("max_steering_angle", 0.52);   // radians (~30 degrees)
        this->declare_parameter<double>("max_speed", 2.0);             // m/s
        this->declare_parameter<double>("min_speed", -2.0);            // m/s (negative for reverse)
        this->declare_parameter<bool>("enable_speed_reduction", true); // Reduce speed in tight turns

        // Get parameters
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        enable_speed_reduction_ = this->get_parameter("enable_speed_reduction").as_bool();

        // Create subscriber for Ackermann commands
        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive_cmd", 10,
            std::bind(&AckermannController::ackermannCallback, this, std::placeholders::_1));

        // Create publisher for Twist commands (to serial node)
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Ackermann Controller initialized");
        RCLCPP_INFO(this->get_logger(), "  Wheelbase: %.3f m", wheelbase_);
        RCLCPP_INFO(this->get_logger(), "  Max steering angle: %.2f rad (%.1f deg)",
                    max_steering_angle_, max_steering_angle_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  Speed range: [%.2f, %.2f] m/s", min_speed_, max_speed_);
    }

private:
    void ackermannCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        // Extract Ackermann command
        double desired_speed = msg->drive.speed;
        double desired_steering_angle = msg->drive.steering_angle;

        // Clamp steering angle to physical limits
        double steering_angle = std::clamp(desired_steering_angle,
                                           -max_steering_angle_,
                                           max_steering_angle_);

        // Clamp speed to limits
        double speed = std::clamp(desired_speed, min_speed_, max_speed_);

        // Reduce speed in tight turns for stability
        if (enable_speed_reduction_)
        {
            double steering_ratio = std::abs(steering_angle) / max_steering_angle_;
            // Reduce speed linearly: full speed at 0 steering, 50% at max steering
            double speed_factor = 1.0 - (0.5 * steering_ratio);
            speed *= speed_factor;
        }

        // Convert Ackermann to Twist
        // For Ackermann steering: angular_z = v * tan(steering_angle) / wheelbase
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = speed;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        if (std::abs(speed) > 1e-6) // Avoid division by zero
        {
            twist_msg.angular.z = speed * std::tan(steering_angle) / wheelbase_;
        }
        else
        {
            twist_msg.angular.z = 0.0;
        }

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;

        // Publish Twist command
        twist_pub_->publish(twist_msg);

        // Log at debug level
        RCLCPP_DEBUG(this->get_logger(),
                     "Ackermann: speed=%.2f m/s, steer=%.2f rad -> Twist: vx=%.2f, wz=%.2f",
                     desired_speed, steering_angle, twist_msg.linear.x, twist_msg.angular.z);
    }

    // Parameters
    double wheelbase_;
    double max_steering_angle_;
    double max_speed_;
    double min_speed_;
    bool enable_speed_reduction_;

    // ROS interfaces
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannController>());
    rclcpp::shutdown();
    return 0;
}