#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mujoco_pendulum
{

class ComputedTorqueNode : public rclcpp::Node
{
public:
  ComputedTorqueNode()
  : Node("computed_torque_node")
  {
    joint_name_ = declare_parameter<std::string>("joint_name", "hinge");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/effort_controller/commands");
    state_topic_ = declare_parameter<std::string>("state_topic", "/joint_states");

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 500.0);
    q_ref_ = declare_parameter<double>("q_ref", 0.0);
    qd_ref_ = declare_parameter<double>("qd_ref", 0.0);
    qdd_ref_ = declare_parameter<double>("qdd_ref", 0.0);

    kp_ = declare_parameter<double>("kp", 30.0);
    kd_ = declare_parameter<double>("kd", 4.0);
    ki_ = declare_parameter<double>("ki", 0.0);
    i_clamp_ = declare_parameter<double>("i_clamp", 2.0);

    inertia_ = declare_parameter<double>("inertia", 1.0);
    damping_ = declare_parameter<double>("damping", 0.05);
    mass_ = declare_parameter<double>("mass", 1.0);
    com_length_ = declare_parameter<double>("com_length", 0.5);
    gravity_ = declare_parameter<double>("gravity", 9.81);
    max_torque_ = declare_parameter<double>("max_torque", 10.0);
    disturbance_tau_ = declare_parameter<double>("disturbance_tau", 0.0);

    use_gravity_ = declare_parameter<bool>("use_gravity_comp", true);
    use_inertia_ = declare_parameter<bool>("use_inertia_comp", true);
    use_damping_ = declare_parameter<bool>("use_damping_comp", true);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      state_topic_,
      50,
      std::bind(&ComputedTorqueNode::on_joint_state, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ComputedTorqueNode::on_control_timer, this));

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&ComputedTorqueNode::on_parameters_set, this, std::placeholders::_1));

    last_control_time_ = now();
    RCLCPP_INFO(get_logger(), "Computed torque node started for joint '%s'", joint_name_.c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "q_ref") {
        q_ref_ = p.as_double();
      } else if (name == "qd_ref") {
        qd_ref_ = p.as_double();
      } else if (name == "qdd_ref") {
        qdd_ref_ = p.as_double();
      } else if (name == "kp") {
        kp_ = p.as_double();
      } else if (name == "kd") {
        kd_ = p.as_double();
      } else if (name == "ki") {
        ki_ = p.as_double();
      } else if (name == "i_clamp") {
        i_clamp_ = p.as_double();
      } else if (name == "max_torque") {
        max_torque_ = p.as_double();
      } else if (name == "mass") {
        mass_ = p.as_double();
      } else if (name == "com_length") {
        com_length_ = p.as_double();
      } else if (name == "gravity") {
        gravity_ = p.as_double();
      } else if (name == "inertia") {
        inertia_ = p.as_double();
      } else if (name == "damping") {
        damping_ = p.as_double();
      } else if (name == "disturbance_tau") {
        disturbance_tau_ = p.as_double();
      } else if (name == "use_gravity_comp") {
        use_gravity_ = p.as_bool();
      } else if (name == "use_inertia_comp") {
        use_inertia_ = p.as_bool();
      } else if (name == "use_damping_comp") {
        use_damping_ = p.as_bool();
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_name_);
    if (it == msg->name.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Joint '%s' not found in /joint_states",
        joint_name_.c_str());
      return;
    }

    const auto idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (idx >= msg->position.size()) {
      return;
    }

    q_ = msg->position[idx];
    if (idx < msg->velocity.size()) {
      qd_ = msg->velocity[idx];
    }

    has_state_ = true;
  }

  void on_control_timer()
  {
    if (!has_state_) {
      return;
    }

    const auto now_time = now();
    const double dt = std::max(1e-4, (now_time - last_control_time_).seconds());
    last_control_time_ = now_time;

    const double e = std::atan2(std::sin(q_ref_ - q_), std::cos(q_ref_ - q_));
    const double edot = qd_ref_ - qd_;

    double tau_ff = 0.0;
    if (use_inertia_) {
      tau_ff += inertia_ * qdd_ref_;
    }
    if (use_damping_) {
      tau_ff += damping_ * qd_;
    }
    if (use_gravity_) {
      tau_ff += mass_ * gravity_ * com_length_ * std::sin(q_);
    }

    const double integral_candidate = std::clamp(integral_error_ + e * dt, -i_clamp_, i_clamp_);
    const double tau_candidate =
      tau_ff + kp_ * e + kd_ * edot + ki_ * integral_candidate - disturbance_tau_;
    const bool windup_positive = (tau_candidate > max_torque_) && (e > 0.0);
    const bool windup_negative = (tau_candidate < -max_torque_) && (e < 0.0);
    if (!(windup_positive || windup_negative)) {
      integral_error_ = integral_candidate;
    }

    const double tau_fb = kp_ * e + kd_ * edot + ki_ * integral_error_;
    // Disturbance emulation: positive disturbance opposes actuator command.
    const double tau = std::clamp(tau_ff + tau_fb - disturbance_tau_, -max_torque_, max_torque_);

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {tau};
    command_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "q=%.3f qd=%.3f tau=%.3f e=%.3f disturbance_tau=%.3f",
      q_,
      qd_,
      tau,
      e,
      disturbance_tau_);
  }

  std::string joint_name_;
  std::string command_topic_;
  std::string state_topic_;

  double control_rate_hz_{500.0};
  double q_ref_{0.0};
  double qd_ref_{0.0};
  double qdd_ref_{0.0};

  double kp_{30.0};
  double kd_{4.0};
  double ki_{0.0};
  double i_clamp_{2.0};

  double inertia_{1.0};
  double damping_{0.05};
  double mass_{1.0};
  double com_length_{0.5};
  double gravity_{9.81};
  double max_torque_{10.0};
  double disturbance_tau_{0.0};

  bool use_gravity_{true};
  bool use_inertia_{true};
  bool use_damping_{true};

  double q_{0.0};
  double qd_{0.0};
  double integral_error_{0.0};
  bool has_state_{false};

  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace mujoco_pendulum

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mujoco_pendulum::ComputedTorqueNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
