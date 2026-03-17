#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <deque>

class SerialReaderNode : public rclcpp::Node
{
public:
    SerialReaderNode() : Node("serial_reader_node"), serial_fd_(-1), tof_fd_(-1)
    {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("cmd_timeout_ds", 3);
        this->declare_parameter<double>("steer_center_units", 0.0);
        this->declare_parameter<double>("steer_max_units", 207.0);
        this->declare_parameter<double>("steer_max_angle_deg", 23.0);
        this->declare_parameter<double>("control_rate_hz", 50.0);
        this->declare_parameter<double>("speed_scale_units", 500.0);

        // ToF parameters
        this->declare_parameter<std::string>("tof_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("tof_baudrate", 115200);
        this->declare_parameter<double>("tof_brake_threshold_mm", 150.0);

        // Get parameters
        std::string port         = this->get_parameter("port").as_string();
        int         baudrate     = this->get_parameter("baudrate").as_int();
        double      control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();

        cmd_timeout_ds       = this->get_parameter("cmd_timeout_ds").as_int();
        steer_center_units_  = this->get_parameter("steer_center_units").as_double();
        steer_max_units_     = this->get_parameter("steer_max_units").as_double();
        steer_max_angle_deg_ = this->get_parameter("steer_max_angle_deg").as_double();
        speed_scale_units    = this->get_parameter("speed_scale_units").as_double();

        std::string tof_port    = this->get_parameter("tof_port").as_string();
        int         tof_baudrate = this->get_parameter("tof_baudrate").as_int();
        tof_brake_threshold_mm_ = this->get_parameter("tof_brake_threshold_mm").as_double();

        // Steering calculations
        steer_max_angle_rad_ = steer_max_angle_deg_ * M_PI / 180.0;
        steer_units_per_rad_ = steer_max_units_ / steer_max_angle_rad_;

        // Publishers
        publisher_                 = this->create_publisher<std_msgs::msg::String>("serial_data", 10);
        imu_publisher_             = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        tof_left_publisher_        = this->create_publisher<sensor_msgs::msg::Range>("tof/left", 10);
        tof_right_publisher_       = this->create_publisher<sensor_msgs::msg::Range>("tof/right", 10);
        tof_raw_publisher_         = this->create_publisher<std_msgs::msg::String>("serial_data_tof", 10);
        emergency_brake_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "emergency_brake", rclcpp::QoS(1).reliable());

        // Subscribers
        // We both publish and subscribe to /emergency_brake.
        // Publishing: this node asserts true (obstacle) / false (all-clear).
        // Subscribing: an external node writes false to grant permission to resume.
        // Incoming true messages are ignored — only this node triggers the brake.
        emergency_brake_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "emergency_brake", rclcpp::QoS(1).reliable(),
            std::bind(&SerialReaderNode::emergencyBrakeCallback, this, std::placeholders::_1));

        ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive_cmd", 10,
            std::bind(&SerialReaderNode::ackermannCallback, this, std::placeholders::_1));

        // Open main serial port
        if (!openSerial(port, baudrate))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Serial port opened: %s at %d baud", port.c_str(), baudrate);
        sleep(5);
        sendCommand("kl", {"30"});

        // Open ToF serial port
        if (!openTofSerial(tof_port, tof_baudrate))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to open ToF serial port: %s — ToF disabled", tof_port.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "ToF serial port opened: %s at %d baud", tof_port.c_str(), tof_baudrate);
        }

        // Timers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SerialReaderNode::readSerial, this));

        tof_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SerialReaderNode::readTofSerial, this));

        const double period_s = (control_rate_hz_ > 0.0) ? (1.0 / control_rate_hz_) : 0.02;
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(period_s)),
            std::bind(&SerialReaderNode::sendDriveCommand, this));
    }

    ~SerialReaderNode()
    {
        if (serial_fd_ >= 0) close(serial_fd_);
        if (tof_fd_ >= 0)    close(tof_fd_);
    }

private:

    // =========================================================================
    // Serial helpers
    // =========================================================================

    bool openSerial(const std::string &port, int baudrate)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;
        configureTermios(serial_fd_, baudrate);
        return true;
    }

    bool openTofSerial(const std::string &port, int baudrate)
    {
        tof_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (tof_fd_ < 0) return false;
        configureTermios(tof_fd_, baudrate, /*canonical=*/true);
        return true;
    }

    void configureTermios(int fd, int baudrate, bool canonical = false)
    {
        struct termios options;
        tcgetattr(fd, &options);

        speed_t speed = B115200;
        switch (baudrate)
        {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
        }

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |=  CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |=  CREAD | CLOCAL;
        options.c_oflag &= ~OPOST;

        if (canonical)
        {
            options.c_lflag |=  ICANON;
            options.c_lflag &= ~(ECHO | ECHOE | ISIG);
        }
        else
        {
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        }

        tcsetattr(fd, TCSANOW, &options);
    }

    // =========================================================================
    // Read / write
    // =========================================================================

    void readSerial()
    {
        if (serial_fd_ < 0) return;

        char buffer[256];
        int  bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0)
        {
            buffer[bytes_read] = '\0';
            std::string data(buffer, bytes_read);

            std::string              command;
            std::vector<std::string> values;

            if (parseSerialData(data, command, values))
            {
                if (command == "imu" && values.size() >= 6)
                    publishImuData(values);
            }

            auto message = std_msgs::msg::String();
            message.data = data;
            publisher_->publish(message);
            RCLCPP_DEBUG(this->get_logger(), "Received: %s", buffer);
        }
    }

    void readTofSerial()
    {
        if (tof_fd_ < 0) return;

        char buffer[256];
        int  bytes_read = read(tof_fd_, buffer, sizeof(buffer) - 1);
        if (bytes_read <= 0) return;

        buffer[bytes_read] = '\0';
        std::string line(buffer, bytes_read);

        {
            auto msg = std_msgs::msg::String();
            msg.data = line;
            tof_raw_publisher_->publish(msg);
        }

        bool is_blank = std::all_of(line.begin(), line.end(),
                                    [](unsigned char c){ return std::isspace(c); });
        if (is_blank) return;

        std::string              command;
        std::vector<std::string> values;

        if (parseSerialData(line, command, values) &&
            command == "tof" &&
            values.size() >= 2)
        {
            processTofData(values);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "ToF unrecognised line: [%s]", line.c_str());
        }
    }

    bool writeToFd(int fd, const std::string &data)
    {
        if (fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            return false;
        }
        int bytes_written = write(fd, data.c_str(), data.length());
        if (bytes_written < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port (fd=%d)", fd);
            return false;
        }
        if (bytes_written != static_cast<int>(data.length()))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Partial write: %d of %zu bytes", bytes_written, data.length());
        }
        return true;
    }

    bool writeSerial(const std::string &data)    { return writeToFd(serial_fd_, data); }
    bool writeTofSerial(const std::string &data) { return writeToFd(tof_fd_,    data); }

    // =========================================================================
    // Protocol
    // =========================================================================

    bool parseSerialData(const std::string &data,
                         std::string &command,
                         std::vector<std::string> &values)
    {
        if (data.empty() || data[0] != '@') return false;

        size_t colon_pos = data.find(':');
        if (colon_pos == std::string::npos) return false;

        command = data.substr(1, colon_pos - 1);

        size_t end_pos = data.size();
        while (end_pos > colon_pos &&
               std::isspace(static_cast<unsigned char>(data[end_pos - 1])))
            --end_pos;

        std::string values_str = data.substr(colon_pos + 1, end_pos - colon_pos - 1);

        if (values_str.length() >= 2 &&
            values_str.substr(values_str.length() - 2) == ";;")
            values_str = values_str.substr(0, values_str.length() - 2);

        values.clear();
        size_t start = 0, pos = 0;
        while ((pos = values_str.find(';', start)) != std::string::npos)
        {
            values.push_back(values_str.substr(start, pos - start));
            start = pos + 1;
        }
        if (start < values_str.length())
            values.push_back(values_str.substr(start));

        return true;
    }

    bool sendCommand(const std::string &command, const std::vector<std::string> &values)
    {
        std::string message = "#" + command + ":";
        for (size_t i = 0; i < values.size(); ++i)
        {
            message += values[i];
            if (i < values.size() - 1) message += ";";
        }
        message += ";;\r\n";
        RCLCPP_INFO(this->get_logger(), "message %s", message.c_str());
        return writeSerial(message);
    }

    // =========================================================================
    // Emergency brake topic
    // =========================================================================

    void publishEmergencyBrake(bool active)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = active;
        emergency_brake_publisher_->publish(msg);
    }

    // Called when another node publishes to /emergency_brake.
    // Only a false message from an external node is acted on — it grants
    // permission to resume motion immediately. Publishing true is reserved
    // for this node; inbound true messages are silently ignored to prevent
    // feedback loops with our own publications.
    void emergencyBrakeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) return; // ignore — only this node asserts the brake

        std::lock_guard<std::mutex> lock(tof_mutex_);

        if (tof_brake_state_ != BrakeState::BRAKING) return;

        // Snapshot the current distance as a reference baseline. Any subsequent
        // motion that makes this value decrease will re-trigger braking.
        tof_distance_at_resume_mm_ = tof_last_distance_mm_;
        tof_distance_history_.clear();

        // Unblock drive commands immediately — no pre-check required.
        tof_brake_state_ = BrakeState::MONITORING;
        tof_braking_     = false;

        RCLCPP_INFO(this->get_logger(),
                    "External brake-clear received — drive_cmd resumed immediately. "
                    "Monitoring for approach (reference: %.1f mm).",
                    tof_distance_at_resume_mm_);
    }

    // =========================================================================
    // ToF processing
    // =========================================================================

    void processTofData(const std::vector<std::string> &values)
    {
        double left_mm  = 0.0;
        double right_mm = 0.0;

        try
        {
            left_mm  = std::stod(values[0]);
            right_mm = std::stod(values[1]);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse ToF data: %s", e.what());
            return;
        }

        auto stamp = this->now();

        // Publish range messages unconditionally
        {
            auto msg = sensor_msgs::msg::Range();
            msg.header.stamp    = stamp;
            msg.header.frame_id = "tof_left";
            msg.radiation_type  = sensor_msgs::msg::Range::INFRARED;
            msg.field_of_view   = 0.436332f;
            msg.min_range       = 0.03f;
            msg.max_range       = 2.0f;
            msg.range           = static_cast<float>(left_mm / 1000.0);
            tof_left_publisher_->publish(msg);
        }
        {
            auto msg = sensor_msgs::msg::Range();
            msg.header.stamp    = stamp;
            msg.header.frame_id = "tof_right";
            msg.radiation_type  = sensor_msgs::msg::Range::INFRARED;
            msg.field_of_view   = 0.436332f;
            msg.min_range       = 0.03f;
            msg.max_range       = 2.0f;
            msg.range           = static_cast<float>(right_mm / 1000.0);
            tof_right_publisher_->publish(msg);
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "ToF: left=%.1f mm, right=%.1f mm", left_mm, right_mm);

        // Sensor validity: ignore out-of-range / timeout sentinel values
        constexpr double TOF_VALID_MIN_MM =    1.0;
        constexpr double TOF_VALID_MAX_MM = 2000.0;

        bool left_valid  = (left_mm  >= TOF_VALID_MIN_MM && left_mm  <= TOF_VALID_MAX_MM);
        bool right_valid = (right_mm >= TOF_VALID_MIN_MM && right_mm <= TOF_VALID_MAX_MM);

        if (!left_valid || !right_valid)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "ToF invalid reading ignored: left=%.1f mm (valid=%s), "
                                 "right=%.1f mm (valid=%s)",
                                 left_mm,  left_valid  ? "yes" : "no",
                                 right_mm, right_valid ? "yes" : "no");
            return;
        }

        // Use the closer sensor as the single obstacle distance
        const double current_dist_mm = std::min(left_mm, right_mm);

        std::lock_guard<std::mutex> lock(tof_mutex_);

        tof_last_distance_mm_ = current_dist_mm;

        // Maintain a short sliding window for trend smoothing in MONITORING state.
        // 5 samples at ~100 Hz gives ~50 ms of history — enough to reject noise.
        constexpr std::size_t HISTORY_LEN = 5;
        tof_distance_history_.push_back(current_dist_mm);
        if (tof_distance_history_.size() > HISTORY_LEN)
            tof_distance_history_.pop_front();

        // ── State machine ──────────────────────────────────────────────────────
        switch (tof_brake_state_)
        {

        // ── CLEAR ─────────────────────────────────────────────────────────────
        // Normal operation. Trigger brake the moment either sensor crosses the
        // configured threshold.
        case BrakeState::CLEAR:
        {
            if (left_mm < tof_brake_threshold_mm_ || right_mm < tof_brake_threshold_mm_)
            {
                tof_brake_state_ = BrakeState::BRAKING;
                tof_braking_     = true;
                tof_distance_history_.clear();
                publishEmergencyBrake(true);
                sendTofBrake();
                RCLCPP_WARN(this->get_logger(),
                            "ToF obstacle detected! left=%.1f mm, right=%.1f mm — "
                            "braking, waiting for external clearance.",
                            left_mm, right_mm);
            }
            break;
        }

        // ── BRAKING ───────────────────────────────────────────────────────────
        // Hard stop. Keep sending brake commands. Drive commands are blocked.
        // Stays here until an external node publishes false on /emergency_brake,
        // handled in emergencyBrakeCallback() above.
        case BrakeState::BRAKING:
        {
            sendTofBrake();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "ToF brake active — waiting for external clearance "
                                 "(left=%.1f mm, right=%.1f mm)",
                                 left_mm, right_mm);
            break;
        }

        // ── MONITORING ────────────────────────────────────────────────────────
        // Drive commands flow freely. We watch the distance trend to determine
        // whether the vehicle's motion is heading toward the obstacle.
        //
        // Three outcomes:
        //
        //   1. Distance DECREASING (vehicle approaching obstacle)
        //      → re-assert brake immediately, back to BRAKING.
        //
        //   2. Distance INCREASING (vehicle moving away from obstacle)
        //      → clear the history window and continue monitoring.
        //      Once both sensors rise back above the threshold we return to
        //      CLEAR and normal threshold-based protection resumes.
        //
        //   3. Both sensors back ABOVE threshold
        //      → obstacle is no longer a concern; return to CLEAR.
        case BrakeState::MONITORING:
        {
            // ── Exit: obstacle fully cleared ──────────────────────────────
            if (left_mm  >= tof_brake_threshold_mm_ &&
                right_mm >= tof_brake_threshold_mm_)
            {
                tof_brake_state_ = BrakeState::CLEAR;
                tof_braking_     = false;
                tof_distance_history_.clear();
                publishEmergencyBrake(false);
                RCLCPP_INFO(this->get_logger(),
                            "ToF path clear (left=%.1f mm, right=%.1f mm ≥ %.1f mm threshold) — "
                            "returning to normal operation.",
                            left_mm, right_mm, tof_brake_threshold_mm_);
                break;
            }

            // ── Trend check: need a full window before acting ─────────────
            // This prevents a single noisy sample from immediately re-triggering.
            if (tof_distance_history_.size() < HISTORY_LEN) break;

            const double oldest = tof_distance_history_.front();
            const double newest = tof_distance_history_.back();
            const double delta  = newest - oldest; // negative → closer, positive → farther

            // 10 mm hysteresis keeps sensor noise from causing false triggers.
            constexpr double APPROACH_HYSTERESIS_MM = 10.0;

            if (delta < -APPROACH_HYSTERESIS_MM)
            {
                // Vehicle is moving toward the obstacle — stop again.
                tof_brake_state_ = BrakeState::BRAKING;
                tof_braking_     = true;
                tof_distance_history_.clear();
                publishEmergencyBrake(true);
                sendTofBrake();
                RCLCPP_WARN(this->get_logger(),
                            "ToF: drive command is causing approach "
                            "(Δ=%.1f mm, left=%.1f mm, right=%.1f mm) — re-braking.",
                            delta, left_mm, right_mm);
            }
            else if (delta > APPROACH_HYSTERESIS_MM)
            {
                // Vehicle is moving away — reset the window for a fresh reading.
                // Stay in MONITORING until both sensors clear the threshold.
                tof_distance_history_.clear();
                RCLCPP_DEBUG(this->get_logger(),
                             "ToF: moving away from obstacle (Δ=+%.1f mm) — "
                             "continuing until threshold cleared.",
                             delta);
            }
            // else: inside hysteresis band — vehicle stationary or moving laterally;
            // keep monitoring without intervening.

            break;
        }

        } // end switch
    }

    void sendTofBrake()
    {
        const std::string brake_cmd = "#brake:0;;\r\n";
        writeTofSerial(brake_cmd);
        RCLCPP_DEBUG(this->get_logger(), "Sent ToF brake command");
    }

    // =========================================================================
    // IMU
    // =========================================================================

    void eulerToQuaternion(double roll, double pitch, double yaw,
                           double &qx, double &qy, double &qz, double &qw)
    {
        roll  = roll  * M_PI / 180.0;
        pitch = pitch * M_PI / 180.0;
        yaw   = yaw   * M_PI / 180.0;

        double cy = cos(yaw   * 0.5), sy = sin(yaw   * 0.5);
        double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
        double cr = cos(roll  * 0.5), sr = sin(roll  * 0.5);

        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
    }

    void publishImuData(const std::vector<std::string> &values)
    {
        try
        {
            double roll    = std::stod(values[0]);
            double pitch   = std::stod(values[1]);
            double yaw     = std::stod(values[2]);
            double accel_x = std::stod(values[3]);
            double accel_y = std::stod(values[4]);
            double accel_z = std::stod(values[5]);

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp    = this->now();
            imu_msg.header.frame_id = "imu_link";

            eulerToQuaternion(roll, pitch, yaw,
                              imu_msg.orientation.x,
                              imu_msg.orientation.y,
                              imu_msg.orientation.z,
                              imu_msg.orientation.w);

            imu_msg.linear_acceleration.x = accel_x;
            imu_msg.linear_acceleration.y = accel_y;
            imu_msg.linear_acceleration.z = accel_z;

            imu_msg.orientation_covariance[0]         = -1;
            imu_msg.angular_velocity_covariance[0]    = -1;
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

    // =========================================================================
    // Drive control
    // =========================================================================

    void ackermannCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        target_speed_mps_ = msg->drive.speed;
        target_steer_rad_ = msg->drive.steering_angle;
        have_cmd_         = true;
    }

    void sendDriveCommand()
    {
        double v_mps = 0.0, delta_rad = 0.0;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            if (!have_cmd_) return;
            v_mps     = target_speed_mps_;
            delta_rad = target_steer_rad_;
        }

        // Block motion only in BRAKING state (tof_braking_ == true).
        // In MONITORING state tof_braking_ is false so commands pass through
        // unmodified — the trend logic will re-assert if needed.
        {
            std::lock_guard<std::mutex> lock(tof_mutex_);
            if (tof_braking_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "ToF interlock active — drive_cmd speed overridden to 0");
                v_mps = 0.0;
            }
        }

        double speed_units = speed_scale_units * v_mps;
        speed_units = std::clamp(speed_units, -500.0, 500.0);

        delta_rad = std::clamp(delta_rad, -steer_max_angle_rad_, steer_max_angle_rad_);
        double steer_units = steer_center_units_ - steer_units_per_rad_ * delta_rad;
        steer_units = std::clamp(steer_units,
                                 steer_center_units_ - steer_max_units_,
                                 steer_center_units_ + steer_max_units_);

        int speed_i    = static_cast<int>(std::lround(speed_units));
        int steer_i    = static_cast<int>(std::lround(steer_units));
        int timeout_ds = std::max(1, cmd_timeout_ds);

        sendCommand("vcd", {std::to_string(speed_i),
                            std::to_string(steer_i),
                            std::to_string(timeout_ds)});
    }

    // =========================================================================
    // Member variables
    // =========================================================================

    std::mutex cmd_mutex_;

    int serial_fd_;
    int tof_fd_;

    // ── Emergency brake state machine ─────────────────────────────────────────
    //
    //   CLEAR
    //     │  either sensor < threshold
    //     ▼
    //   BRAKING ◄─────────────────────────────────────────────────────────────┐
    //     │  external node publishes false on /emergency_brake               │
    //     │  (drive_cmd unblocked immediately)                               │
    //     ▼                                                                  │
    //   MONITORING ── distance trend < -hysteresis (approaching) ────────────┘
    //     │  both sensors ≥ threshold
    //     ▼
    //   CLEAR
    //
    enum class BrakeState { CLEAR, BRAKING, MONITORING };

    std::mutex         tof_mutex_;
    bool               tof_braking_               = false;
    BrakeState         tof_brake_state_           = BrakeState::CLEAR;
    double             tof_brake_threshold_mm_    = 150.0;
    double             tof_last_distance_mm_      = 9999.0;
    double             tof_distance_at_resume_mm_ = 9999.0;
    std::deque<double> tof_distance_history_;

    // Drive state
    int    cmd_timeout_ds     = 3;
    bool   have_cmd_          = false;
    double target_speed_mps_  = 0.0;
    double target_steer_rad_  = 0.0;

    // Steering parameters
    double steer_center_units_;
    double steer_max_units_;
    double steer_max_angle_deg_;
    double steer_max_angle_rad_;
    double steer_units_per_rad_;
    double speed_scale_units;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_left_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_right_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   tof_raw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     emergency_brake_publisher_;

    // Subscribers
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        emergency_brake_subscriber_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tof_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReaderNode>());
    rclcpp::shutdown();
    return 0;
}
