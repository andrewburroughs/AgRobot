#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>
#include <mutex>
#include <cmath>
#include <string>
#include <algorithm>

#include "pubSysCls.h"

using namespace sFnd;

static bool IsBusPowerLow(INode &node) {
  return node.Status.Power.Value().fld.InBusLoss;
}

// Your sFoundation: StateStr(char*, size_t) is NON-const, so pass by value.
static std::string AlertsToString(decltype(std::declval<sFnd::INode>().Status.Alerts.Value()) alerts) {
  char buf[512];
  buf[0] = '\0';
  alerts.StateStr(buf, sizeof(buf));
  return std::string(buf);
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

    RCLCPP_INFO(get_logger(),
      "Subscribed to [motor_speed]. >0 fwd, <0 rev, 0 stop. (gentle mode + RMS protection)");
  }

  ~ClearPathJogNode() override {
    shutdown_teknic();
  }

private:
  // -------- Teknic --------
  SysManager *mgr_{nullptr};
  INode *node_{nullptr};
  std::mutex mtx_;

  // -------- ROS --------
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // -------- Tuning (gentle defaults) --------
  static constexpr size_t NODE_INDEX = 0;

  // Make these conservative for a rigid coupler + bearing load
  static constexpr double ACCEL_RPM_PER_SEC = 3.0;  // gentler than 5.0
  static constexpr double VEL_LIM_RPM       = 0.35; // gentler than 0.5

  // Jog segment behavior
  static constexpr int    JOG_STEP_MIN_CNTS = 80;   // start small
  static constexpr int    JOG_STEP_MAX_CNTS = 250;  // cap
  static constexpr int    JOG_STEP_RAMP_PER_TICK = 10;

  static constexpr int    ENABLE_TIMEOUT_MS = 10000;

  // Loop and filtering
  static constexpr int    TICK_MS  = 50;
  static constexpr double DEADZONE = 0.05;

  // Direction-change hold (prevents hard reversals)
  static constexpr double REVERSE_HOLD_SEC = 1.0;

  // Duty-cycle limiting (RMS protection while key is held)
  // run for RUN_SEC, then rest for REST_SEC (repeat while cmd held)
  static constexpr double RUN_SEC  = 1.0;
  static constexpr double REST_SEC = 0.6;

  // Overload cooldown
  static constexpr double RMS_COOLDOWN_SEC = 6.0;

  // -------- State --------
  double cmd_{0.0};
  bool move_in_flight_{false};

  int last_dir_{0};

  int jog_step_current_{JOG_STEP_MIN_CNTS};

  // timers
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time hold_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time rms_cooldown_until_{0, 0, RCL_ROS_TIME};

  // duty-cycle phase
  bool run_phase_{true};
  rclcpp::Time phase_until_{0, 0, RCL_ROS_TIME};

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

    RCLCPP_INFO(get_logger(),
      "Enabled. AccLimit=%.2f RPM/s VelLimit=%.2f RPM",
      ACCEL_RPM_PER_SEC, VEL_LIM_RPM);
  }

  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    cmd_ = msg->data;
  }

  void print_diag_throttled_locked(const char* context, const std::string& alert_str) {
    auto now = this->get_clock()->now();
    if ((now - last_diag_time_).seconds() < 0.5) return;
    last_diag_time_ = now;

    RCLCPP_WARN(get_logger(),
      "[%s] ready=%d bus_loss=%d alerts=\"%s\"",
      context,
      node_->Motion.IsReady() ? 1 : 0,
      IsBusPowerLow(*node_) ? 1 : 0,
      alert_str.c_str());
  }

  bool ensure_ready_locked() {
    if (IsBusPowerLow(*node_)) {
      auto s = AlertsToString(node_->Status.Alerts.Value());
      print_diag_throttled_locked("bus_power_low", s);
      return false;
    }

    if (node_->Motion.IsReady()) return true;

    auto s = AlertsToString(node_->Status.Alerts.Value());
    print_diag_throttled_locked("not_ready", s);

    // If we're in RMS overload shutdown, DON'T thrash the enable line.
    if (s.find("RMSOverloadShutdown") != std::string::npos) {
      rms_cooldown_until_ = this->get_clock()->now() +
                            rclcpp::Duration::from_seconds(RMS_COOLDOWN_SEC);
      // stop commanding
      cmd_ = 0.0;
      jog_step_current_ = JOG_STEP_MIN_CNTS;
      run_phase_ = true;
      phase_until_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(RUN_SEC);

      RCLCPP_ERROR(get_logger(),
        "RMSOverloadShutdown detected. Cooling down %.1fs. Reduce load/misalignment.",
        RMS_COOLDOWN_SEC);
      return false;
    }

    // Attempt recovery for other faults
    node_->Status.AlertsClear();
    node_->Motion.NodeStopClear();
    node_->EnableReq(true);

    double timeout = mgr_->TimeStampMsec() + 500;
    while (!node_->Motion.IsReady() && mgr_->TimeStampMsec() < timeout) {
      mgr_->Delay(10);
    }

    if (!node_->Motion.IsReady()) {
      auto s2 = AlertsToString(node_->Status.Alerts.Value());
      print_diag_throttled_locked("not_ready_post_recover", s2);
      return false;
    }

    return true;
  }

  void update_direction_holds_locked(int dir) {
    auto now = this->get_clock()->now();

    // On direction change, hold (coast/rest) and restart ramp/duty-cycle.
    if (last_dir_ != 0 && dir != last_dir_) {
      hold_until_ = now + rclcpp::Duration::from_seconds(REVERSE_HOLD_SEC);

      jog_step_current_ = JOG_STEP_MIN_CNTS;

      // restart duty-cycle (rest first is often gentler)
      run_phase_ = false;
      phase_until_ = now + rclcpp::Duration::from_seconds(REST_SEC);

      RCLCPP_INFO(get_logger(), "Direction change: holding %.2fs before reversing", REVERSE_HOLD_SEC);
    }

    last_dir_ = dir;
  }

  void update_duty_cycle_locked() {
    auto now = this->get_clock()->now();
    if (phase_until_.nanoseconds() == 0) {
      // initialize
      run_phase_ = true;
      phase_until_ = now + rclcpp::Duration::from_seconds(RUN_SEC);
      return;
    }

    if (now < phase_until_) return;

    // toggle
    run_phase_ = !run_phase_;
    phase_until_ = now + rclcpp::Duration::from_seconds(run_phase_ ? RUN_SEC : REST_SEC);
  }

  void tick() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!node_) return;

    auto now = this->get_clock()->now();

    // Global cooldown after RMSOverloadShutdown
    if (now < rms_cooldown_until_) {
      return;
    }

    // If a move is still executing, wait
    if (move_in_flight_) {
      if (!node_->Motion.MoveIsDone()) return;
      move_in_flight_ = false;
    }

    // Stop = don't start new segments; also reset ramp and duty-cycle
    if (std::fabs(cmd_) < DEADZONE) {
      jog_step_current_ = JOG_STEP_MIN_CNTS;
      last_dir_ = 0;
      // reset duty-cycle
      run_phase_ = true;
      phase_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }

    int dir = (cmd_ > 0.0) ? +1 : -1;

    // Apply direction-change hold logic
    update_direction_holds_locked(dir);
    if (now < hold_until_) return;

    // Duty-cycle limiting to control RMS heating
    update_duty_cycle_locked();
    if (!run_phase_) return;

    // Must be ready before issuing motion
    if (!ensure_ready_locked()) return;

    // Ramp step size up slowly to reduce torque spikes
    jog_step_current_ = std::min(jog_step_current_ + JOG_STEP_RAMP_PER_TICK, JOG_STEP_MAX_CNTS);

    int delta = dir * jog_step_current_;

    try {
      node_->Motion.MoveWentDone();
      node_->Motion.MovePosnStart(delta);
      move_in_flight_ = true;
    } catch (const mnErr &) {
      // Let main print full mnErr, but don't keep slamming the motor
      auto s = AlertsToString(node_->Status.Alerts.Value());
      print_diag_throttled_locked("MovePosnStart_mnErr", s);
      cmd_ = 0.0;
      jog_step_current_ = JOG_STEP_MIN_CNTS;
      throw;
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
