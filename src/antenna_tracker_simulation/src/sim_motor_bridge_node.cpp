#include <rclcpp/rclcpp.hpp>
#include <antenna_tracker_msgs/msg/motor_command.hpp>
#include <antenna_tracker_msgs/msg/encoder_feedback.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

static constexpr double TWO_PI       = 2.0 * M_PI;
static constexpr double DT           = 0.01;               // 100 Hz integration
static constexpr double EL_MAX_RAD   = M_PI / 2.0;        // 90°
/* Gravity coefficient — must match NMPC model: el_ddot = u_el - GRAVITY_COEFF * cos(el) */
static constexpr double GRAVITY_COEFF = 50.0;              // rad/s²

class SimMotorBridge : public rclcpp::Node {
public:
    SimMotorBridge() : Node("sim_motor_bridge") {
        sub_cmd_ = this->create_subscription<antenna_tracker_msgs::msg::MotorCommand>(
            "/antenna/motor_cmd", rclcpp::SensorDataQoS(),
            std::bind(&SimMotorBridge::cmd_callback, this, std::placeholders::_1));

        pub_az_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/antenna_tracker/joint/azimuth_joint/cmd_vel", 10);
        pub_el_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/antenna_tracker/joint/elevation_joint/cmd_vel", 10);

        pub_encoder_ = this->create_publisher<antenna_tracker_msgs::msg::EncoderFeedback>(
            "/antenna/encoder_feedback", 10);

        // 100 Hz double-integrator timer: acc → vel → pos
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SimMotorBridge::integrate_and_publish, this));

        RCLCPP_INFO(this->get_logger(),
            "SimMotorBridge ready — double-integrator + gravity (%.0f rad/s²) @ 100 Hz",
            GRAVITY_COEFF);
    }

private:
    void cmd_callback(const antenna_tracker_msgs::msg::MotorCommand::SharedPtr msg) {
        if (msg->emergency_stop) {
            az_acc_rads2_ = 0.0;
            el_acc_rads2_ = 0.0;
            return;
        }

        /* NMPC outputs angular acceleration (rad/s²) via frequency_hz field.
         * Do NOT scale by RAD_PER_STEP — the value is already in rad/s². */
        az_acc_rads2_ = msg->az_frequency_hz * (msg->az_direction ? 1.0 : -1.0);
        el_acc_rads2_ = msg->el_frequency_hz * (msg->el_direction ? 1.0 : -1.0);
    }

    void integrate_and_publish() {
        /* AZ: double integrator — az_ddot = u_az (no gravity on azimuth) */
        az_vel_rps_ += az_acc_rads2_ * DT;
        az_pos_rad_ += az_vel_rps_ * DT;
        az_pos_rad_ = std::fmod(az_pos_rad_, TWO_PI);
        if (az_pos_rad_ < 0.0) az_pos_rad_ += TWO_PI;

        /* EL: double integrator with gravity — el_ddot = u_el - GRAVITY_COEFF * cos(el)
         * Matches NMPC model exactly. Without motor input (u_el=0), antenna droops. */
        el_vel_rps_ += (el_acc_rads2_ - GRAVITY_COEFF * std::cos(el_pos_rad_)) * DT;
        el_pos_rad_ += el_vel_rps_ * DT;
        el_pos_rad_ = std::max(0.0, std::min(EL_MAX_RAD, el_pos_rad_));

        /* Clamp velocity at hard limits */
        if ((el_pos_rad_ <= 0.0 && el_vel_rps_ < 0.0) ||
            (el_pos_rad_ >= EL_MAX_RAD && el_vel_rps_ > 0.0)) {
            el_vel_rps_ = 0.0;
        }

        /* Forward velocity to Gazebo for visualisation */
        std_msgs::msg::Float64 az_vel_msg, el_vel_msg;
        az_vel_msg.data = az_vel_rps_;
        el_vel_msg.data = el_vel_rps_;
        pub_az_->publish(az_vel_msg);
        pub_el_->publish(el_vel_msg);

        /* Publish encoder feedback */
        antenna_tracker_msgs::msg::EncoderFeedback feedback;
        feedback.header.stamp = this->now();
        feedback.header.frame_id = "base_link";

        feedback.az_angle_deg    = az_pos_rad_ * 180.0 / M_PI;
        feedback.el_angle_deg    = el_pos_rad_ * 180.0 / M_PI;
        feedback.az_velocity_dps = az_vel_rps_ * 180.0 / M_PI;
        feedback.el_velocity_dps = el_vel_rps_ * 180.0 / M_PI;
        feedback.az_valid = true;
        feedback.el_valid = true;

        pub_encoder_->publish(feedback);
    }

    // Subscriptions / Publishers
    rclcpp::Subscription<antenna_tracker_msgs::msg::MotorCommand>::SharedPtr sub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_az_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_el_;
    rclcpp::Publisher<antenna_tracker_msgs::msg::EncoderFeedback>::SharedPtr pub_encoder_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    double az_pos_rad_{0.0};    // current azimuth position (rad)
    double el_pos_rad_{0.0};    // current elevation position (rad)
    double az_vel_rps_{0.0};    // azimuth velocity (rad/s)
    double el_vel_rps_{0.0};    // elevation velocity (rad/s)
    double az_acc_rads2_{0.0};  // commanded AZ angular acceleration (rad/s²)
    double el_acc_rads2_{0.0};  // commanded EL angular acceleration (rad/s²)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimMotorBridge>());
    rclcpp::shutdown();
    return 0;
}
