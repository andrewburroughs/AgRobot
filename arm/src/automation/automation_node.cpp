#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int step = 0;

class AutomationNode : public rclcpp::Node
{
public:
  AutomationNode()
  : Node("automation_node"), homing_active_(true), timer_started_(false)
  {
    // 1. Create a publisher to talk to your existing motor node
    // Topic name "motor_cmd" must match the 'speed_topic' in your launch file
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("motor_cmd", 10);

    // 2. Control loop running at 10Hz (every 100ms)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&AutomationNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Automation Node Started. Preparing to Home...");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32();

    // --- STATE: HOMING ---
    if (homing_active_) {
        // Capture the start time ONCE when we first enter this state
        if (!timer_started_) {
            start_time_ = this->now();
            timer_started_ = true;
            RCLCPP_INFO(this->get_logger(), "HOMING STARTED: Retracting for 50 seconds...");
        }

        // Calculate how long we have been running
        auto current_time = this->now();
        auto elapsed_duration = current_time - start_time_;
        double elapsed_seconds = elapsed_duration.seconds();

        if (elapsed_seconds < 50.0) {
            // COMMAND: Full Retract (Backwards)
            message.data = -1.0; // Range: -1.0 to 1.0
            
            // Optional: Print status every 5 seconds so you know it's alive
            if ((int)elapsed_seconds % 5 == 0 && (elapsed_seconds - (int)elapsed_seconds) < 0.1) {
                 RCLCPP_INFO(this->get_logger(), "Homing... Time left: %.1f s", 50.0 - elapsed_seconds);
            }
        } 
        else {
            // TIME IS UP
            message.data = 0.0;
            homing_active_ = false; // Disable homing
            RCLCPP_INFO(this->get_logger(), "HOMING COMPLETE. Stopping motor.");
        }
    } 
    // --- STATE: IDLE ---
    else {
        // Keep sending 0.0 to ensure motor stays stopped

        if(step == 0){
            
        }
        message.data = 0.0;
    }

    // Publish the command to the motor node
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  
  // State variables
  bool homing_active_;
  bool timer_started_;
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutomationNode>());
  rclcpp::shutdown();
  return 0;
}