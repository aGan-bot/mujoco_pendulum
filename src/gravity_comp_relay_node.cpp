#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mujoco_pendulum
{

class GravityCompRelayNode : public rclcpp::Node
{
public:
  GravityCompRelayNode()
  : Node("gravity_comp_relay_node")
  {
    gain_ = declare_parameter<double>("gain", 1.0);
    gain_vector_ = declare_parameter<std::vector<double>>("gain_vector", std::vector<double>{});
    q_ref_ = declare_parameter<std::vector<double>>("q_ref", {0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0});
    kp_ = declare_parameter<std::vector<double>>("kp", {0.0, 0.0, 18.0, 0.0, 0.0, 0.0});
    kd_ = declare_parameter<std::vector<double>>("kd", {0.0, 0.0, 2.5, 0.0, 0.0, 0.0});
    qd_lpf_alpha_ = declare_parameter<double>("qd_lpf_alpha", 0.2);
    d_term_limit_ = declare_parameter<double>("d_term_limit", 30.0);
    use_shortest_angular_error_ = declare_parameter<bool>("use_shortest_angular_error", false);
    max_torque_ = declare_parameter<double>("max_torque", 200.0);
    max_torque_vector_ = declare_parameter<std::vector<double>>(
      "max_torque_vector", {87.0, 87.0, 52.0, 10.0, 10.0, 10.0});
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/effort_controller/commands");
    bias_topic_ = declare_parameter<std::string>("bias_topic", "/mujoco/bias_torque");
    state_topic_ = declare_parameter<std::string>("state_topic", "/joint_states");
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    q_.assign(joint_names_.size(), 0.0);
    qd_.assign(joint_names_.size(), 0.0);
    qd_filt_.assign(joint_names_.size(), 0.0);

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      bias_topic_,
      10,
      std::bind(&GravityCompRelayNode::on_bias, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic_,
      50,
      std::bind(&GravityCompRelayNode::on_joint_state, this, std::placeholders::_1));
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&GravityCompRelayNode::on_parameters_set, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "gravity_comp_relay_node started (gain=%.3f, joints=%zu, max_torque=%.3f)",
      gain_,
      joint_names_.size(),
      max_torque_);
  }

private:
  static double get_or_default(
    const std::vector<double> & values, size_t i, double fallback)
  {
    if (values.empty()) {
      return fallback;
    }
    if (i < values.size()) {
      return values[i];
    }
    return values.back();
  }

  static double get_torque_limit(
    const std::vector<double> & limits, size_t i, double fallback)
  {
    const double limit = std::abs(get_or_default(limits, i, fallback));
    if (limit > 0.0) {
      return limit;
    }
    return std::abs(fallback);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "gain") {
        gain_ = p.as_double();
      } else if (name == "gain_vector") {
        gain_vector_ = p.as_double_array();
      } else if (name == "q_ref") {
        q_ref_ = p.as_double_array();
      } else if (name == "kp") {
        kp_ = p.as_double_array();
      } else if (name == "kd") {
        kd_ = p.as_double_array();
      } else if (name == "qd_lpf_alpha") {
        qd_lpf_alpha_ = std::clamp(p.as_double(), 0.0, 1.0);
      } else if (name == "d_term_limit") {
        d_term_limit_ = std::max(0.0, p.as_double());
      } else if (name == "use_shortest_angular_error") {
        use_shortest_angular_error_ = p.as_bool();
      } else if (name == "max_torque") {
        max_torque_ = p.as_double();
      } else if (name == "max_torque_vector") {
        max_torque_vector_ = p.as_double_array();
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto & expected_name = joint_names_[i];
      auto it = std::find(msg->name.begin(), msg->name.end(), expected_name);
      if (it == msg->name.end()) {
        continue;
      }
      const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
      if (idx < msg->position.size()) {
        q_[i] = msg->position[idx];
      }
      if (idx < msg->velocity.size()) {
        qd_[i] = msg->velocity[idx];
        if (!has_state_) {
          qd_filt_[i] = qd_[i];
        } else {
          qd_filt_[i] = (1.0 - qd_lpf_alpha_) * qd_filt_[i] + qd_lpf_alpha_ * qd_[i];
        }
      }
    }
    has_state_ = true;
  }

  void on_bias(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!has_state_) {
      return;
    }

    std_msgs::msg::Float64MultiArray cmd;
    const size_t n = std::min(joint_names_.size(), msg->data.size());
    cmd.data.resize(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
      const double gi = get_or_default(gain_vector_, i, gain_);
      const double q_ref_i = get_or_default(q_ref_, i, 0.0);
      const double kp_i = get_or_default(kp_, i, 0.0);
      const double kd_i = get_or_default(kd_, i, 0.0);
      const double tau_limit_i = get_torque_limit(max_torque_vector_, i, max_torque_);
      double e = q_ref_i - q_[i];
      if (use_shortest_angular_error_) {
        e = std::atan2(std::sin(e), std::cos(e));
      }
      const double d_term = std::clamp(-kd_i * qd_filt_[i], -d_term_limit_, d_term_limit_);
      const double u = gi * msg->data[i] + kp_i * e + d_term;
      cmd.data[i] = std::clamp(u, -tau_limit_i, tau_limit_i);
    }

    pub_->publish(cmd);
  }

  double gain_{1.0};
  std::vector<double> gain_vector_;
  std::vector<double> q_ref_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double qd_lpf_alpha_{0.2};
  double d_term_limit_{30.0};
  bool use_shortest_angular_error_{false};
  double max_torque_{200.0};
  std::vector<double> max_torque_vector_;
  std::string command_topic_;
  std::string bias_topic_;
  std::string state_topic_;
  std::vector<std::string> joint_names_;
  std::vector<double> q_;
  std::vector<double> qd_;
  std::vector<double> qd_filt_;
  bool has_state_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace mujoco_pendulum

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mujoco_pendulum::GravityCompRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
