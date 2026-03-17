#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mujoco_pendulum
{

class EffortTestNode : public rclcpp::Node
{
public:
  EffortTestNode()
  : Node("effort_test_node")
  {
    command_topic_ = declare_parameter<std::string>("command_topic", "/effort_controller/commands");
    state_topic_ = declare_parameter<std::string>("state_topic", "/joint_states");
    joint_name_ = declare_parameter<std::string>("joint_name", "hinge");

    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 100.0));
    torque_command_ = declare_parameter<double>("torque_command", 10.0);

    step_mode_ = declare_parameter<bool>("step_mode", false);
    step_after_s_ = std::max(0.0, declare_parameter<double>("step_after_s", 5.0));
    step_torque_ = declare_parameter<double>("step_torque", 20.0);

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic_,
      50,
      std::bind(&EffortTestNode::on_joint_state, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    start_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EffortTestNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "effort_test_node started: base torque=%.3f Nm, step_mode=%s",
      torque_command_,
      step_mode_ ? "true" : "false");
  }

private:
  void on_timer()
  {
    double command = torque_command_;
    if (step_mode_) {
      const double elapsed = (now() - start_time_).seconds();
      if (elapsed >= step_after_s_) {
        command = step_torque_;
      }
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {command};
    pub_->publish(msg);
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_name_);
    if (it == msg->name.end()) {
      return;
    }

    const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (idx >= msg->position.size()) {
      return;
    }

    const double q = msg->position[idx];
    const double qd = idx < msg->velocity.size() ? msg->velocity[idx] : 0.0;
    const double tau = idx < msg->effort.size() ? msg->effort[idx] : 0.0;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "joint=%s q=%.4f qd=%.4f effort=%.4f",
      joint_name_.c_str(),
      q,
      qd,
      tau);
  }

  std::string command_topic_;
  std::string state_topic_;
  std::string joint_name_;

  double publish_rate_hz_{100.0};
  double torque_command_{10.0};
  bool step_mode_{false};
  double step_after_s_{5.0};
  double step_torque_{20.0};

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mujoco_pendulum

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mujoco_pendulum::EffortTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
