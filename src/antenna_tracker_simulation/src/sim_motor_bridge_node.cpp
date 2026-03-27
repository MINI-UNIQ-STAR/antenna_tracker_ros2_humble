#include <rclcpp/rclcpp.hpp>
#include <antenna_tracker_msgs/msg/motor_command.hpp>
#include <antenna_tracker_msgs/msg/encoder_feedback.hpp>
#include <std_msgs/msg/float64.hpp>
#include "antenna_tracker_simulation/sim_motor_model.hpp"

class SimMotorBridge : public rclcpp::Node {
public:
    SimMotorBridge() : Node("sim_motor_bridge") {
        declare_parameter<double>("loop_rate_hz", 100.0);
        declare_parameter<double>("stepper_pulses_per_rev", 200.0);
        declare_parameter<double>("stepper_microsteps", 1.0);
        declare_parameter<double>("gear_ratio", 20.0);
        declare_parameter<double>("motor_max_speed_dps", 180.0);
        declare_parameter<double>("motor_max_accel_dps2", 1800.0);
        declare_parameter<double>("command_deadzone_hz", 18.0);
        declare_parameter<double>("disturbance_dps2", 0.25);
        declare_parameter<double>("command_noise_hz", 0.0);
        declare_parameter<double>("encoder_noise_deg", 0.0);
        declare_parameter<double>("velocity_noise_dps", 0.0);
        declare_parameter<double>("gravity_coeff_rads2", 10.77);
        declare_parameter<double>("az_damping", 10.0);
        // 실측 하드웨어 물리 파라미터
        declare_parameter<double>("load_mass_kg", 0.5);
        declare_parameter<double>("arm_length_m", 0.9);
        declare_parameter<double>("load_inertia_kgm2", 0.41);
        declare_parameter<double>("motor_holding_torque_nm", 1.2);
        declare_parameter<double>("gearbox_efficiency", 0.85);
        // Backlash
        declare_parameter<double>("az_backlash_deg", 0.8);
        declare_parameter<double>("el_backlash_deg", 0.5);
        // Step loss
        declare_parameter<double>("step_loss_accel_threshold_dps2", 2232.0);
        declare_parameter<double>("step_loss_rate", 0.002);

        config_.stepper_pulses_per_rev = get_parameter("stepper_pulses_per_rev").as_double();
        config_.stepper_microsteps = get_parameter("stepper_microsteps").as_double();
        config_.gear_ratio = get_parameter("gear_ratio").as_double();
        config_.motor_max_speed_dps = get_parameter("motor_max_speed_dps").as_double();
        config_.motor_max_accel_dps2 = get_parameter("motor_max_accel_dps2").as_double();
        config_.command_deadzone_hz = get_parameter("command_deadzone_hz").as_double();
        config_.disturbance_dps2 = get_parameter("disturbance_dps2").as_double();
        config_.command_frequency_noise_hz = get_parameter("command_noise_hz").as_double();
        config_.encoder_angle_noise_deg = get_parameter("encoder_noise_deg").as_double();
        config_.encoder_velocity_noise_dps = get_parameter("velocity_noise_dps").as_double();
        config_.gravity_coeff_rads2 = get_parameter("gravity_coeff_rads2").as_double();
        config_.az_damping = get_parameter("az_damping").as_double();
        config_.load_mass_kg = get_parameter("load_mass_kg").as_double();
        config_.arm_length_m = get_parameter("arm_length_m").as_double();
        config_.load_inertia_kgm2 = get_parameter("load_inertia_kgm2").as_double();
        config_.motor_holding_torque_nm = get_parameter("motor_holding_torque_nm").as_double();
        config_.gearbox_efficiency = get_parameter("gearbox_efficiency").as_double();
        config_.az_backlash_deg = get_parameter("az_backlash_deg").as_double();
        config_.el_backlash_deg = get_parameter("el_backlash_deg").as_double();
        config_.step_loss_accel_threshold_dps2 = get_parameter("step_loss_accel_threshold_dps2").as_double();
        config_.step_loss_rate = get_parameter("step_loss_rate").as_double();

        sub_cmd_ = this->create_subscription<antenna_tracker_msgs::msg::MotorCommand>(
            "/antenna/motor_cmd", rclcpp::SensorDataQoS(),
            std::bind(&SimMotorBridge::cmd_callback, this, std::placeholders::_1));

        pub_az_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/antenna_tracker/joint/azimuth_joint/cmd_vel", 10);
        pub_el_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/antenna_tracker/joint/elevation_joint/cmd_vel", 10);

        pub_encoder_ = this->create_publisher<antenna_tracker_msgs::msg::EncoderFeedback>(
            "/antenna/encoder_feedback", 10);

        double loop_rate = get_parameter("loop_rate_hz").as_double();
        auto period = std::chrono::duration<double>(1.0 / loop_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SimMotorBridge::integrate_and_publish, this));

        RCLCPP_INFO(this->get_logger(),
            "SimMotorBridge ready — NEMA23 twin, 200 steps/rev, gear %.1f:1, noise(cmd=%.2fHz, enc=%.2fdeg)",
            config_.gear_ratio, config_.command_frequency_noise_hz, config_.encoder_angle_noise_deg);
    }

private:
    void cmd_callback(const antenna_tracker_msgs::msg::MotorCommand::SharedPtr msg) {
        antenna_tracker_simulation::apply_motor_command(*msg, state_);
    }

    void integrate_and_publish() {
        const auto step = antenna_tracker_simulation::step_sim_motor_model(state_, config_);

        /* Forward velocity to Gazebo for visualisation */
        std_msgs::msg::Float64 az_vel_msg, el_vel_msg;
        az_vel_msg.data = step.az_velocity_command_rps;
        el_vel_msg.data = step.el_velocity_command_rps;
        pub_az_->publish(az_vel_msg);
        pub_el_->publish(el_vel_msg);

        /* Publish encoder feedback */
        auto feedback = step.feedback;
        feedback.header.stamp = this->now();
        feedback.header.frame_id = "base_link";
        pub_encoder_->publish(feedback);
    }

    // Subscriptions / Publishers
    rclcpp::Subscription<antenna_tracker_msgs::msg::MotorCommand>::SharedPtr sub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_az_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_el_;
    rclcpp::Publisher<antenna_tracker_msgs::msg::EncoderFeedback>::SharedPtr pub_encoder_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    antenna_tracker_simulation::SimMotorConfig config_;
    antenna_tracker_simulation::SimMotorState state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimMotorBridge>());
    rclcpp::shutdown();
    return 0;
}
