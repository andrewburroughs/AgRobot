#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>
#include <mutex>
#include <cmath>
#include <cstdint>

#include "pubSysCls.h"

using namespace sFnd;

static bool IsBusPowerLow(INode &node) {
  return node.Status.Power.Value().fld.InBusLoss;
}

class ClearPathJogNode : public rclcpp::Node {
public:
  ClearPathJogNode() : Node("clearpath_sc_jog_node") {
    init_teknic();
    enable_node();

    sub_ = create_subscription<std_msgs::msg::Float32>(
      "motor_speed", 10,
      std::bind(&ClearPathJogNode::speed_callback, this, std::placeholders::_1)
    );

    timer_ = create_wall_timer(
      std::chrono::milliseconds(TICK_MS),
      std::bind(&ClearPathJogNode::tick, this)
    );

    RCLCPP_INFO(get_logger(), "Subscribed to [motor_speed]. >0 fwd, <0 rev, 0 stop.");
  }

  ~ClearPathJogNode() override {
    shutdown_teknic();
  }

private:
  SysManager *mgr_{nullptr};
  INode *node_{nullptr};
  std::mutex mtx_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static constexpr size_t NODE_INDEX = 0;
  static constexpr double ACCEL_RPM_PER_SEC = 50.0;
  static constexpr double VEL_LIM_RPM = 2.0;
  static constexpr int    JOG_STEP_CNTS = 20;     // start larger than 10 for testing
  static constexpr int    ENABLE_TIMEOUT_MS = 10000;

  static constexpr int    TICK_MS = 10;
  static constexpr double DEADZONE = 0.05;

  double cmd_{0.0};
  bool move_in_flight_{false};

  // Throttled diagnostic timestamp
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};

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
      throw std::runtime_error("NODE_INDEX out of range");
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
          throw std::runtime_error("Bus power low (InBusLoss=1).");
        }
        throw std::runtime_error("Timed out enabling node");
      }
      mgr_->Delay(10);
    }

    node_->AccUnit(INode::RPM_PER_SEC);
    node_->VelUnit(INode::RPM);
    node_->Motion.AccLimit = ACCEL_RPM_PER_SEC;
    node_->Motion.VelLimit = VEL_LIM_RPM;

    RCLCPP_INFO(get_logger(), "Enabled. AccLimit=%.1f RPM/s VelLimit=%.1f RPM",
                ACCEL_RPM_PER_SEC, VEL_LIM_RPM);
  }

  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_ = msg->data;
  }

  // Print whatever alert info is available without relying on version-specific bitfields
  void print_diagnostics_locked(const char* context) {
    // throttle to 2 Hz
    auto now = this->get_clock()->now();
    if ((now - last_diag_time_).seconds() < 0.5) return;
    last_diag_time_ = now;

    bool ready = node_->Motion.IsReady();
    bool bus_loss = IsBusPowerLow(*node_);

    // Alerts.Value() is known to exist from your earlier attempts.
    // We’ll print the raw bits as hex. The type varies; cast through uint32_t/uint64_t conservatively.
    auto alerts = node_->Status.Alerts.Value();

  }

  bool ensure_ready_locked() {
    if (IsBusPowerLow(*node_)) {
      print_diagnostics_locked("bus_power_low");
      return false;
    }

    if (node_->Motion.IsReady()) return true;

    print_diagnostics_locked("not_ready_pre_recover");

    // Try recovery (safe calls per your example)
    node_->Status.AlertsClear();
    node_->Motion.NodeStopClear();
    node_->EnableReq(true);

    double timeout = mgr_->TimeStampMsec() + 500;
    while (!node_->Motion.IsReady() && mgr_->TimeStampMsec() < timeout) {
      mgr_->Delay(10);
    }

    if (!node_->Motion.IsReady()) {
      print_diagnostics_locked("not_ready_post_recover");
      return false;
    }

    return true;
  }

  void tick() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!node_) return;

    // Update in-flight state
    if (move_in_flight_) {
      if (!node_->Motion.MoveIsDone()) return;
      move_in_flight_ = false;
    }

    // Stop = don't start new segments
    if (std::fabs(cmd_) < DEADZONE) return;

    // Must be ready before issuing motion
    if (!ensure_ready_locked()) return;

    int dir = (cmd_ > 0.0) ? +1 : -1;
    int delta = dir * JOG_STEP_CNTS;

    try {
      node_->Motion.MoveWentDone();
      node_->Motion.MovePosnStart(delta);
      move_in_flight_ = true;
    } catch (const mnErr &e) {
      // If motion call fails, print diagnostics once and stop issuing segments
      print_diagnostics_locked("MovePosnStart_mnErr");
      cmd_ = 0.0; // soft stop on error
      throw;      // let main print full mnErr too
    }
  }

  void shutdown_teknic() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!node_ || !mgr_) return;

    try {
      cmd_ = 0.0;
      node_->EnableReq(false);
      mgr_->PortsClose();
    } catch (...) {}

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
