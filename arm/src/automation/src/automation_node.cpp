#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

int step = 0;
bool homing_complete = false;
double motor1Position = 0.0;
double motor2Position = 0.0;
double currentDuration = 0.0;
double motor1Speed = 0.0;
double motor2Speed = 0.0;

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
    if (homing_active_ && !homing_complete) {
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
            RCLCPP_INFO(this->get_logger(), "Motor 1 Position: 0.0\"");
            RCLCPP_INFO(this->get_logger(), "Motor 2 Position: 0.0\"");
            start_time_ = this->now();
            homing_complete = true;
            currentDuration = 16;
        }
    } 
    // --- STATE: IDLE ---
    else {
        auto current_time = this->now();
        auto elapsed_duration = current_time - start_time_;
        double elapsed_seconds = elapsed_duration.seconds();
        motor1Position = motor1Speed * elapsed_duration.seconds() * 0.39;
        motor2Position = motor2Speed * elapsed_duration.seconds() * 0.39;
        
        if ((int)elapsed_seconds % 5 == 0 && (elapsed_seconds - (int)elapsed_seconds) < 0.1) {
            RCLCPP_INFO(this->get_logger(), "Running... Time left: %.1f s", currentDuration - elapsed_seconds);
            RCLCPP_INFO(this->get_logger(), "Motor 1 Position: %f", motor1Position);
            RCLCPP_INFO(this->get_logger(), "Motor 2 Position: %f", motor2Position);
        }
        if(step == 0){
            // Extend talon 2
            talon2Speed.data = 1.0;
            motor2Speed = 1.0;
            if(elapsed_seconds > currentDuration){
                step += 1;
                start_time_ = this->now();
                talon2Speed.data = 0.0;
                motor2Speed = 0.0;
                currentDuration = 23;
                return;
            }
        }
        if(step == 1){
            // rotate 45 degrees left
            step += 1;
        }
        if(step == 2){
            // extend talon 1 halfway
            talon1Speed.data = 1.0;
            motor1Speed = 1.0;
            if(elapsed_seconds > currentDuration){
                step += 1;
                start_time_ = this->now();
                talon1Speed.data = 0.0;
                motor1Speed = 0.0;
                currentDuration = 16;
                return;
            }
        }
        if(step == 3){
            // retract talon 2
            talon2Speed.data = -1.0;
            motor2Speed = -1.0;
            if(elapsed_seconds > currentDuration){
                step += 1;
                start_time_ = this->now();
                talon2Speed.data = 0.0;
                motor2Speed = 0.0;
                currentDuration = 16;
                return;
            }
        }
        if(step == 4){
            // extend talon 2
            talon2Speed.data = 1.0;
            motor2Speed = 1.0;
            if(elapsed_seconds > currentDuration){
                step += 1;
                start_time_ = this->now();
                talon2Speed.data = 0.0;
                motor2Speed = 0.0;
                currentDuration = 23;
                return;
            }
        }
        if(step == 5){
            // retract talon 1
            talon1Speed.data = -1.0;
            motor1Speed = -1.0;
            if(elapsed_seconds > currentDuration){
                step += 1;
                start_time_ = this->now();
                talon1Speed.data = 0.0;
                motor1Speed = 0.0;
                return;
            }
        }
        if(step == 6){
            // rotate 45 degrees right
            step += 1;
            currentDuration = 16;
        }
        if(step == 7){
            // retract talon 2
            talon2Speed.data = -1.0;
            motor2Speed = -1.0;
            if(elapsed_seconds > currentDuration){
                step += 1;
                start_time_ = this->now();
                talon2Speed.data = 0.0;
                motor2Speed = 0.0;
                return;
            }
        }
        if(step == 8){
            talon1Speed.data = 0.0;
            motor2Speed = 0.0;
            talon2Speed.data = 0.0;
            motor2Speed = 0.0;
        }
        
    }

    // Publish the command to the motor node
    talon1SpeedPublisher->publish(talon1Speed);
    talon2SpeedPublisher->publish(talon2Speed);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr talon1SpeedPublisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr talon2SpeedPublisher;
  
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