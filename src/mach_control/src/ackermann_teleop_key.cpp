#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <algorithm> // min/max

class AckermannTeleopKeyboard : public rclcpp::Node
{
public:
    AckermannTeleopKeyboard() : Node("ackermann_teleop_keyboard")
    {
        // Declare parameters
        this->declare_parameter<double>("max_speed", 2.0);
        this->declare_parameter<double>("max_steering_angle", 0.52);
        this->declare_parameter<double>("speed_increment", 0.2);
        this->declare_parameter<double>("steering_increment", 0.1);
        this->declare_parameter<double>("publish_rate", 20.0); // Hz

        // Get parameters
        max_speed_ = this->get_parameter("max_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        speed_increment_ = this->get_parameter("speed_increment").as_double();
        steering_increment_ = this->get_parameter("steering_increment").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // Create publisher
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_cmd", 10);

        // Initialize command
        target_speed_ = 0.0;
        target_steering_ = 0.0;

        // Setup terminal for raw input
        setupTerminal();

        std::cout << "\nAckermann Teleop Keyboard\n"
                  << "-------------------------\n"
                  << "w : increase speed\n"
                  << "x : decrease speed\n"
                  << "a : steer left\n"
                  << "d : steer right\n"
                  << "s / SPACE : stop\n"
                  << "ESC or Ctrl+C : quit\n"
                  << std::endl;

        // Publish at fixed rate
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        publish_timer_ = this->create_wall_timer(
            period,
            std::bind(&AckermannTeleopKeyboard::publishCommand, this));

        // Read keyboard faster
        keyboard_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&AckermannTeleopKeyboard::processKeyboard, this));
    }

    ~AckermannTeleopKeyboard()
    {
        restoreTerminal();
    }

private:
    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &original_term_);
        struct termios raw = original_term_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_term_);
    }

    void processKeyboard()
    {
        char c;
        while (read(STDIN_FILENO, &c, 1) > 0)
        {
            bool input_registered = true;

            switch (c)
            {
            // Speed control
            case 'w':
                target_speed_ += speed_increment_;
                target_speed_ = std::min(target_speed_, max_speed_);
                break;

            case 'x':
                target_speed_ -= speed_increment_;
                target_speed_ = std::max(target_speed_, -max_speed_);
                break;

            // Steering control
            case 'a':
                target_steering_ += steering_increment_;
                target_steering_ = std::min(target_steering_, max_steering_angle_);
                break;

            case 'd':
                target_steering_ -= steering_increment_;
                target_steering_ = std::max(target_steering_, -max_steering_angle_);
                break;

            // Stop
            case 's':
            case ' ':
                target_speed_ = 0.0;
                target_steering_ = 0.0;
                break;

            // Quit
            case 27: // ESC
            case 3:  // Ctrl+C
                target_speed_ = 0.0;
                target_steering_ = 0.0;
                publishCommand(); // send stop before shutdown
                rclcpp::shutdown();
                return;

            default:
                input_registered = false; // ignore other keys
                break;
            }

            if (input_registered)
            {
                std::cout
                    << "key='" << (c == ' ' ? "SPACE" : std::string(1, c)) << "' "
                    << "speed=" << target_speed_ << " "
                    << "steer=" << target_steering_
                    << std::endl;
            }
        }
    }

    void publishCommand()
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.drive.speed = target_speed_;
        msg.drive.steering_angle = target_steering_;

        publisher_->publish(msg);
    }

    // Parameters
    double max_speed_;
    double max_steering_angle_;
    double speed_increment_;
    double steering_increment_;

    // Current target values
    double target_speed_;
    double target_steering_;

    // Terminal state
    struct termios original_term_;

    // ROS interfaces
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr keyboard_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AckermannTeleopKeyboard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
