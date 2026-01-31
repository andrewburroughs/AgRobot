#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>
#include <mutex>

#include "pubSysCls.h"

using namespace sFnd;

static bool IsBusPowerLow(INode &node) {
  return node.Status.Power.Value().fld.InBusLoss;
}

class ClearPathKeyboardNode : public rclcpp::Node {
public:
  ClearPathKeyboardNode()
  : Node("clearpath_sc_keyboard_node") {

    init_teknic();
    enable_node();

    // HARD-CODED TOPIC NAME
    sub_ = create_subscription<std_msgs::msg::Float32>(
      "motor_speed", 10,
      std::bind(&ClearPathKeyboardNode::speed_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "ClearPath keyboard node ready. Subscribed to [motor_speed]");
  }

  ~ClearPathKeyboardNode() override {
    shutdown_teknic();
  }

private:
  // ---------- Teknic ----------
  SysManager *mgr_{nullptr};
  INode *node_{nullptr};
  std::mutex mtx_;

  // ---------- ROS ----------
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;

  // ---------- Motion constants ----------
  static constexpr double SPEED_RPM = 2.0;          // slow speed
  static constexpr double ACCEL_RPM_PER_SEC = 100.0;
  static constexpr size_t NODE_INDEX = 0;

  double last_cmd_{0.0};

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
      "Connected to ClearPath node: %s",
      node_->Info.Model.Value());
  }

  void enable_node() {
    std::lock_guard<std::mutex> lk(mtx_);

    node_->EnableReq(false);
    mgr_->Delay(200);

    node_->Status.AlertsClear();
    node_->Motion.NodeStopClear();
    node_->EnableReq(true);

    double timeout = mgr_->TimeStampMsec() + 10000;
    while (!node_->Motion.IsReady()) {
      if (mgr_->TimeStampMsec() > timeout) {
        if (IsBusPowerLow(*node_)) {
          throw std::runtime_error("Bus power low");
        }
        throw std::runtime_error("Enable timeout");
      }
    }

    node_->AccUnit(INode::RPM_PER_SEC);
    node_->VelUnit(INode::RPM);
    node_->Motion.AccLimit = ACCEL_RPM_PER_SEC;
    node_->Motion.VelLimit = SPEED_RPM;

    RCLCPP_INFO(get_logger(), "ClearPath node enabled");
  }

  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);

    const double cmd = msg->data;
    if (cmd == last_cmd_) return;
    last_cmd_ = cmd;

    if (cmd == 0.0) {
      node_->Motion.MoveVelStop();
      RCLCPP_INFO(get_logger(), "Motor STOP");
      return;
    }

    const double rpm = (cmd > 0.0) ? SPEED_RPM : -SPEED_RPM;
    node_->Motion.MoveVelStart(rpm);

    RCLCPP_INFO(get_logger(),
      "Motor RUN %.2f RPM", rpm);
  }

  void shutdown_teknic() {
    std::lock_guard<std::mutex> lk(mtx_);

    if (!node_) return;

    try {
      node_->Motion.MoveVelStop();
      node_->EnableReq(false);
      mgr_->PortsClose();
    } catch (...) {
      // Ignore shutdown errors
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ClearPathKeyboardNode>();
    rclcpp::spin(node);
  } catch (const mnErr &e) {
    fprintf(stderr,
      "Teknic error: addr=%d err=0x%08x msg=%s\n",
      e.TheAddr, e.ErrorCode, e.ErrorMsg);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
