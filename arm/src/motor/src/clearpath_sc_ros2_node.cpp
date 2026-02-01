#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>
#include <mutex>
#include <cmath>

#include "pubSysCls.h"

using namespace sFnd;

// Same helper you used
static bool IsBusPowerLow(INode &node) {
  return node.Status.Power.Value().fld.InBusLoss;
}

class ClearPathJogNode : public rclcpp::Node {
public:
  ClearPathJogNode()
  : Node("clearpath_sc_jog_node")
  {
    init_teknic();
    enable_node();

    // HARD-CODED TOPIC NAME (as requested)
    sub_ = create_subscription<std_msgs::msg::Float32>(
      "motor_speed", 10,
      std::bind(&ClearPathJogNode::speed_callback, this, std::placeholders::_1)
    );

    // Timer drives the “segmented jog”
    timer_ = create_wall_timer(
      std::chrono::milliseconds(TICK_MS),
      std::bind(&ClearPathJogNode::tick, this)
    );

    RCLCPP_INFO(get_logger(),
      "Ready. Subscribed to [motor_speed]. >0 forward, <0 reverse, 0 soft-stop.");
  }

  ~ClearPathJogNode() override {
    shutdown_teknic();
  }

private:
  // ---------- Teknic ----------
  SysManager *mgr_{nullptr};
  INode *node_{nullptr};
  std::mutex mtx_;

  // ---------- ROS ----------
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------- Constants (edit here) ----------
  static constexpr size_t NODE_INDEX = 0;

  static constexpr double ACCEL_RPM_PER_SEC = 50.0; // gentler accel than your example
  static constexpr double VEL_LIM_RPM       = 2.0;  // “slow turn”
  static constexpr int    JOG_STEP_CNTS     = 10;   // small step each segment (tune!)
  static constexpr int    ENABLE_TIMEOUT_MS = 10000;

  static constexpr int    TICK_MS           = 10;   // how often we try to queue next segment
  static constexpr double DEADZONE          = 0.05; // ignore tiny noise

  double last_sent_vel_ = 0.0; 
  // ---------- State ----------
  double cmd_{0.0};          // last commanded motor_speed (-1..1 from keyboard)
  bool   move_in_flight_{false};

private:
  void init_teknic() {
    mgr_ = SysManager::Instance();

    std::vector<std::string> ports;
    SysManager::FindComHubPorts(ports);
    if (ports.empty()) {
      throw std::runtime_error("No SC hubs found");
    }

    mgr_->ComHubPort(0, ports[0].c_str());
    mgr_->PortsOpen(1);

    IPort &port = mgr_->Ports(0);
    if (NODE_INDEX >= port.NodeCount()) {
      throw std::runtime_error("NODE_INDEX out of range for discovered hub");
    }

    node_ = &port.Nodes(NODE_INDEX);

    RCLCPP_INFO(get_logger(),
      "Connected: userID=%s model=%s serial=%d fw=%s",
      node_->Info.UserID.Value(),
      node_->Info.Model.Value(),
      node_->Info.SerialNumber.Value(),
      node_->Info.FirmwareVersion.Value());
  }

  void enable_node() {
    std::lock_guard<std::mutex> lk(mtx_);

    node_->EnableReq(false);
    mgr_->Delay(200);

    node_->Status.AlertsClear();
    node_->Motion.NodeStopClear();
    node_->EnableReq(true);

    double timeout = mgr_->TimeStampMsec() + ENABLE_TIMEOUT_MS;
    while (!node_->Motion.IsReady()) {
      if (mgr_->TimeStampMsec() > timeout) {
        if (IsBusPowerLow(*node_)) {
          throw std::runtime_error("Bus power low (InBusLoss=1). Turn on supply.");
        }
        throw std::runtime_error("Timed out enabling node");
      }
    }

    node_->AccUnit(INode::RPM_PER_SEC);
    node_->VelUnit(INode::RPM);
    node_->Motion.AccLimit = ACCEL_RPM_PER_SEC;
    node_->Motion.VelLimit = VEL_LIM_RPM;

    RCLCPP_INFO(get_logger(), "Node enabled; AccLimit=%.1f RPM/s, VelLimit=%.1f RPM",
                ACCEL_RPM_PER_SEC, VEL_LIM_RPM);
  }

  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_ = msg->data; // store; tick() will act on it
  }

  // Add a new member variable to track the last sent velocity

void tick() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!node_) return;

    // 1. Safety Check: Is the node actually enabled?
    // If a fault occurred (like the one you are seeing), this will be false.
    if (!node_->Motion.IsReady()) {
        // Optional: Read the Alert register to see WHY it crashed
        char alertStr[256];
        node_->Status.Alerts.StateStr(alertStr, 256);

        RCLCPP_ERROR(get_logger(), "Node Not Ready. Alerts: %s", alertStr);
        
        // Attempt to re-enable if it crashed
        node_->Status.AlertsClear();
        node_->Motion.NodeStopClear();
        node_->EnableReq(true);
        return; 
    }

    // 2. Velocity Logic
    // If the command is effectively zero, stop.
    if (std::fabs(cmd_) < DEADZONE) {
        if (std::fabs(last_sent_vel_) > DEADZONE) {
        node_->Motion.MoveVelStart(0); // Stop smoothly
        last_sent_vel_ = 0.0;
        }
        return;
    }

    // 3. Send Velocity Command
    // Only send the command if it has changed to avoid spamming the bus
    double target_vel = (cmd_ > 0) ? VEL_LIM_RPM : -VEL_LIM_RPM;
    
    if (std::abs(target_vel - last_sent_vel_) > 0.1) {
        node_->Motion.MoveVelStart(target_vel);
        last_sent_vel_ = target_vel;
        
        // Note: We do NOT set move_in_flight_ or check MoveIsDone() 
        // because velocity moves run forever until told to stop.
    }
    }

  void shutdown_teknic() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!node_ || !mgr_) return;

    try {
      // Soft stop: stop issuing new segments by zeroing cmd_
      cmd_ = 0.0;

      // Let any in-flight move finish for a short time
      double timeout = mgr_->TimeStampMsec() + 500;
      while (move_in_flight_ && mgr_->TimeStampMsec() < timeout) {
        if (node_->Motion.MoveIsDone()) move_in_flight_ = false;
        mgr_->Delay(10);
      }

      node_->EnableReq(false);
      mgr_->PortsClose();
    } catch (...) {
      // ignore shutdown errors
    }

    node_ = nullptr;
    mgr_ = nullptr;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ClearPathJogNode>());
  } catch (const mnErr &e) {
    fprintf(stderr, "Teknic mnErr: addr=%d err=0x%08x msg=%s\n",
            e.TheAddr, e.ErrorCode, e.ErrorMsg);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
