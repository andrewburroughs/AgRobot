#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int step = 0;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon1SpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon2SpeedPublisher;


class AutomationNode : public rclcpp::Node
{
public:
  AutomationNode()
  : Node("automation_node"), homing_active_(true), timer_started_(false)
  {
    // 1. Create a publisher to talk to your existing motor node
    // Topic name "motor_cmd" must match the 'speed_topic' in your launch file
    talon1SpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("talon_1_speed", 10);
    talon2SpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("talon_2_speed", 10);

    // 2. Control loop running at 10Hz (every 100ms)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&AutomationNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Automation Node Started. Preparing to Home...");
  }

private:
  void timer_callback()
  {
    auto talon1Speed = std_msgs::msg::Float32();
    auto talon2Speed = std_msgs::msg::Float32();

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
            talon1Speed.data = -1.0; // Range: -1.0 to 1.0
            talon2Speed.data = -1.0;

            // Optional: Print status every 5 seconds so you know it's alive
            if ((int)elapsed_seconds % 5 == 0 && (elapsed_seconds - (int)elapsed_seconds) < 0.1) {
                 RCLCPP_INFO(this->get_logger(), "Homing... Time left: %.1f s", 50.0 - elapsed_seconds);
            }
            if(elapsed_seconds > 20){
              talon2Speed.data = 0.0;
            }
        } 
        else {
            // TIME IS UP
            talon1Speed.data = 0.0;
            talon2Speed.data = 0.0;
            homing_active_ = false; // Disable homing
            RCLCPP_INFO(this->get_logger(), "HOMING COMPLETE. Stopping motor.");
        }
    } 
    // --- STATE: IDLE ---
    else {
        // Keep sending 0.0 to ensure motor stays stopped

        if(step == 0){
            
        }
        talon1Speed.data = 0.0;
        talon2Speed.data = 0.0;
    }

    // Publish the command to the motor node
    talon1SpeedPublisher->publish(talon1Speed);
    talon2SpeedPublisher->publish(talon2Speed);
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
  talon1SpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_1_speed",1);
  talon2SpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_2_speed",1);

  rclcpp::spin(std::make_shared<AutomationNode>());
  rclcpp::shutdown();
  return 0;
}