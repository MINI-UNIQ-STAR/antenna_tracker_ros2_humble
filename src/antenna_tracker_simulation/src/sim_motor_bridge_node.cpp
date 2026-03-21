#include <rclcpp/rclcpp.hpp>
#include <antenna_tracker_msgs/msg/motor_command.hpp>
#include <std_msgs/msg/float64.hpp>

class SimMotorBridge : public rclcpp::Node {
public:
    SimMotorBridge() : Node("sim_motor_bridge") {
        sub_cmd_ = this->create_subscription<antenna_tracker_msgs::msg::MotorCommand>(
            "/antenna/motor_cmd", 10,
            std::bind(&SimMotorBridge::cmd_callback, this, std::placeholders::_1));

        pub_az_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/antenna_tracker/joint/azimuth_joint/cmd_vel", 10);
        pub_el_ = this->create_publisher<std_msgs::msg::Float64>(
            "/model/antenna_tracker/joint/elevation_joint/cmd_vel", 10);
            
        RCLCPP_INFO(this->get_logger(), "Simulated Motor Bridge Initialized.");
    }

private:
    void cmd_callback(const antenna_tracker_msgs::msg::MotorCommand::SharedPtr msg) {
        if (msg->emergency_stop) {
            std_msgs::msg::Float64 zero; zero.data = 0.0;
            pub_az_->publish(zero);
            pub_el_->publish(zero);
            return;
        }

        // Stepper: 3200 steps/rev -> 2*PI rad
        // rad/s = frequency_hz * (2*PI / 3200)
        double rad_per_step = (2.0 * M_PI) / 3200.0;

        std_msgs::msg::Float64 az_vel, el_vel;
        az_vel.data = msg->az_frequency_hz * rad_per_step * (msg->az_direction ? 1.0 : -1.0);
        el_vel.data = msg->el_frequency_hz * rad_per_step * (msg->el_direction ? 1.0 : -1.0);

        // Apply external gear ratio simulation if needed (assume 1:1 for simplicity)
        pub_az_->publish(az_vel);
        pub_el_->publish(el_vel);
    }

    rclcpp::Subscription<antenna_tracker_msgs::msg::MotorCommand>::SharedPtr sub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_az_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_el_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimMotorBridge>());
    rclcpp::shutdown();
    return 0;
}
