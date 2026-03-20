#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace mujoco_pendulum
{

class PinocchioFfHoldNode : public rclcpp::Node
{
public:
  PinocchioFfHoldNode()
  : Node("pinocchio_ff_hold_node")
  {
    const auto default_urdf =
      ament_index_cpp::get_package_share_directory("orion5_humble_description") + "/urdf/orion5.urdf";

    urdf_path_ = declare_parameter<std::string>("urdf_path", default_urdf);
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    q_ref_ = declare_parameter<std::vector<double>>(
      "q_ref", {0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0});
    kp_ = declare_parameter<std::vector<double>>("kp", {4.0, 6.0, 8.0, 1.5, 1.5, 1.0});
    kd_ = declare_parameter<std::vector<double>>("kd", {0.8, 1.2, 1.8, 0.3, 0.3, 0.2});
    qd_lpf_alpha_ = declare_parameter<double>("qd_lpf_alpha", 0.2);
    enable_ref_generator_ = declare_parameter<bool>("enable_ref_generator", true);
    time_sync_enabled_ = declare_parameter<bool>("time_sync_enabled", true);
    joint_max_speed_deg_ = declare_parameter<std::vector<double>>(
      "joint_max_speed_deg", {45.0, 30.0, 40.0, 45.0, 45.0, 45.0});
    joint_max_accel_deg_ = declare_parameter<std::vector<double>>(
      "joint_max_accel_deg", {800.0, 800.0, 800.0, 800.0, 800.0, 800.0});
    torque_rate_limit_vector_ = declare_parameter<std::vector<double>>(
      "torque_rate_limit_vector", {2000.0, 2000.0, 1200.0, 250.0, 250.0, 250.0});
    max_torque_vector_ = declare_parameter<std::vector<double>>(
      "max_torque_vector", {87.0, 87.0, 52.0, 10.0, 10.0, 10.0});
    hold_current_on_start_ = declare_parameter<bool>("hold_current_on_start", true);
    command_topic_ = declare_parameter<std::string>("command_topic", "/effort_controller/commands");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    cmd_debug_topic_ = declare_parameter<std::string>("cmd_debug_topic", "/pinocchio_ff/cmd_torque");
    error_debug_topic_ = declare_parameter<std::string>("error_debug_topic", "/pinocchio_ff/pos_error");
    use_shortest_angular_error_ = declare_parameter<bool>("use_shortest_angular_error", false);
    control_rate_hz_ = std::max(1.0, declare_parameter<double>("control_rate_hz", 500.0));

    try {
      pinocchio::urdf::buildModel(urdf_path_, model_);
      data_ = std::make_unique<pinocchio::Data>(model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(), "Failed to build Pinocchio model from '%s': %s", urdf_path_.c_str(), e.what());
      throw;
    }

    for (const auto & name : joint_names_) {
      const auto id = model_.getJointId(name);
      if (id == 0) {
        RCLCPP_FATAL(get_logger(), "Joint '%s' not found in model", name.c_str());
        throw std::runtime_error("joint not found");
      }
      joint_ids_.push_back(id);
    }

    q_ = pinocchio::neutral(model_);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    q_ref_cmd_ = q_ref_;
    q_ref_dot_cmd_.assign(joint_names_.size(), 0.0);
    qd_filt_.assign(joint_names_.size(), 0.0);
    tau_prev_.assign(joint_names_.size(), 0.0);

    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 20);
    cmd_debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(cmd_debug_topic_, 20);
    err_debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(error_debug_topic_, 20);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      50,
      std::bind(&PinocchioFfHoldNode::on_joint_state, this, std::placeholders::_1));

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&PinocchioFfHoldNode::on_parameters_set, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PinocchioFfHoldNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "pinocchio_ff_hold_node started (rate=%.1fHz, joints=%zu)",
      control_rate_hz_, joint_names_.size());
  }

private:
  static double get_or_last(const std::vector<double> & v, size_t i, double fallback = 0.0)
  {
    if (v.empty()) {
      return fallback;
    }
    if (i < v.size()) {
      return v[i];
    }
    return v.back();
  }

  static double clamp_abs(double x, double limit)
  {
    const double l = std::abs(limit);
    return std::clamp(x, -l, l);
  }

  static double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "q_ref") {
        q_ref_ = p.as_double_array();
      } else if (name == "kp") {
        kp_ = p.as_double_array();
      } else if (name == "kd") {
        kd_ = p.as_double_array();
      } else if (name == "qd_lpf_alpha") {
        qd_lpf_alpha_ = std::clamp(p.as_double(), 0.0, 1.0);
      } else if (name == "enable_ref_generator") {
        enable_ref_generator_ = p.as_bool();
      } else if (name == "time_sync_enabled") {
        time_sync_enabled_ = p.as_bool();
      } else if (name == "joint_max_speed_deg") {
        joint_max_speed_deg_ = p.as_double_array();
      } else if (name == "joint_max_accel_deg") {
        joint_max_accel_deg_ = p.as_double_array();
      } else if (name == "torque_rate_limit_vector") {
        torque_rate_limit_vector_ = p.as_double_array();
      } else if (name == "max_torque_vector") {
        max_torque_vector_ = p.as_double_array();
      } else if (name == "use_shortest_angular_error") {
        use_shortest_angular_error_ = p.as_bool();
      } else if (name == "hold_current_on_start") {
        hold_current_on_start_ = p.as_bool();
      }
    }
    if (std::find_if(
          params.begin(), params.end(),
          [](const rclcpp::Parameter & p) { return p.get_name() == "q_ref"; }) != params.end())
    {
      sync_profile_valid_ = false;
      sync_time_ = 0.0;
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  static double min_time_for_distance(double dist, double v_max, double a_max)
  {
    const double d = std::abs(dist);
    if (d < 1e-12) {
      return 0.0;
    }
    if (v_max <= 0.0 || a_max <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }
    const double t_acc = v_max / a_max;
    const double d_acc = 0.5 * a_max * t_acc * t_acc;
    if (d <= 2.0 * d_acc) {
      return 2.0 * std::sqrt(d / a_max);
    }
    const double d_cruise = d - 2.0 * d_acc;
    return 2.0 * t_acc + d_cruise / v_max;
  }

  void build_sync_profile()
  {
    sync_q_start_ = q_ref_cmd_;
    sync_q_goal_ = q_ref_;
    sync_time_ = 0.0;
    sync_T_ = 0.0;
    sync_v_peak_.assign(joint_names_.size(), 0.0);

    // Common duration from per-joint minimal-time bounds.
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double d = get_or_last(sync_q_goal_, i, 0.0) - get_or_last(sync_q_start_, i, 0.0);
      const double v = deg2rad(get_or_last(joint_max_speed_deg_, i, 0.0));
      const double a = deg2rad(get_or_last(joint_max_accel_deg_, i, 0.0));
      sync_T_ = std::max(sync_T_, min_time_for_distance(d, v, a));
    }

    if (sync_T_ <= 1e-9 || !std::isfinite(sync_T_)) {
      sync_profile_valid_ = false;
      return;
    }

    // For each joint, solve v from d = v*T - v^2/a (symmetric accel/decel + optional cruise).
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double d = std::abs(get_or_last(sync_q_goal_, i, 0.0) - get_or_last(sync_q_start_, i, 0.0));
      const double v_lim = deg2rad(get_or_last(joint_max_speed_deg_, i, 0.0));
      const double a = deg2rad(get_or_last(joint_max_accel_deg_, i, 0.0));
      if (d < 1e-12 || v_lim <= 0.0 || a <= 0.0) {
        sync_v_peak_[i] = 0.0;
        continue;
      }

      const double disc = std::max(0.0, a * a * sync_T_ * sync_T_ - 4.0 * a * d);
      double v = 0.5 * (a * sync_T_ - std::sqrt(disc));
      v = std::clamp(v, 0.0, v_lim);
      sync_v_peak_[i] = v;
    }

    sync_profile_valid_ = true;
  }

  void update_reference_generator_sync(double dt)
  {
    if (!sync_profile_valid_) {
      build_sync_profile();
      if (!sync_profile_valid_) {
        return;
      }
    }

    sync_time_ = std::min(sync_time_ + dt, sync_T_);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double q0 = get_or_last(sync_q_start_, i, 0.0);
      const double q1 = get_or_last(sync_q_goal_, i, q0);
      const double dq = q1 - q0;
      const double sign = (dq >= 0.0) ? 1.0 : -1.0;
      const double d = std::abs(dq);
      const double v = get_or_last(sync_v_peak_, i, 0.0);
      const double a = deg2rad(get_or_last(joint_max_accel_deg_, i, 0.0));

      if (d < 1e-12 || v <= 0.0 || a <= 0.0) {
        q_ref_cmd_[i] = q1;
        q_ref_dot_cmd_[i] = 0.0;
        continue;
      }

      const double t_r = v / a;
      const double t_c = std::max(0.0, sync_T_ - 2.0 * t_r);
      const double t = sync_time_;

      double s = 0.0;
      double sd = 0.0;

      if (t <= t_r) {
        // accel phase
        s = 0.5 * a * t * t;
        sd = a * t;
      } else if (t <= t_r + t_c) {
        // cruise phase
        const double ta = t_r;
        s = 0.5 * a * ta * ta + v * (t - ta);
        sd = v;
      } else if (t <= sync_T_) {
        // decel phase
        const double td = t - (t_r + t_c);
        const double ta = t_r;
        const double s_before = 0.5 * a * ta * ta + v * t_c;
        s = s_before + v * td - 0.5 * a * td * td;
        sd = std::max(0.0, v - a * td);
      } else {
        s = d;
        sd = 0.0;
      }

      q_ref_cmd_[i] = q0 + sign * std::clamp(s, 0.0, d);
      q_ref_dot_cmd_[i] = sign * sd;
    }

    if (sync_time_ >= sync_T_ - 1e-9) {
      sync_profile_valid_ = false;
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        q_ref_cmd_[i] = get_or_last(q_ref_, i, q_ref_cmd_[i]);
        q_ref_dot_cmd_[i] = 0.0;
      }
    }
  }

  void update_reference_generator(double dt)
  {
    if (!enable_ref_generator_) {
      q_ref_cmd_ = q_ref_;
      std::fill(q_ref_dot_cmd_.begin(), q_ref_dot_cmd_.end(), 0.0);
      return;
    }

    if (q_ref_cmd_.size() != joint_names_.size()) {
      q_ref_cmd_.assign(joint_names_.size(), 0.0);
    }
    if (q_ref_dot_cmd_.size() != joint_names_.size()) {
      q_ref_dot_cmd_.assign(joint_names_.size(), 0.0);
    }

    if (time_sync_enabled_) {
      update_reference_generator_sync(dt);
      return;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double q_target = get_or_last(q_ref_, i, 0.0);
      const double q_cmd = get_or_last(q_ref_cmd_, i, q_target);
      const double v_cmd = get_or_last(q_ref_dot_cmd_, i, 0.0);
      const double v_lim = std::max(0.0, deg2rad(get_or_last(joint_max_speed_deg_, i, 0.0)));
      const double a_lim = std::max(0.0, deg2rad(get_or_last(joint_max_accel_deg_, i, 0.0)));

      if (v_lim <= 0.0 || a_lim <= 0.0) {
        q_ref_cmd_[i] = q_target;
        q_ref_dot_cmd_[i] = 0.0;
        continue;
      }

      const double q_err = q_target - q_cmd;
      const double v_des = std::clamp(q_err / dt, -v_lim, v_lim);
      const double dv_max = a_lim * dt;
      const double dv = std::clamp(v_des - v_cmd, -dv_max, dv_max);
      const double v_next = std::clamp(v_cmd + dv, -v_lim, v_lim);
      double q_next = q_cmd + v_next * dt;

      if ((q_target - q_cmd) * (q_target - q_next) <= 0.0) {
        q_next = q_target;
        q_ref_dot_cmd_[i] = 0.0;
      } else {
        q_ref_dot_cmd_[i] = v_next;
      }

      q_ref_cmd_[i] = q_next;
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        continue;
      }
      const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
      const auto & jmodel = model_.joints[joint_ids_[i]];
      if (jmodel.nq() == 1 && idx < msg->position.size()) {
        q_[jmodel.idx_q()] = msg->position[idx];
      }
      if (jmodel.nv() == 1 && idx < msg->velocity.size()) {
        v_[jmodel.idx_v()] = msg->velocity[idx];
        if (!has_state_) {
          qd_filt_[i] = v_[jmodel.idx_v()];
        } else {
          qd_filt_[i] = (1.0 - qd_lpf_alpha_) * qd_filt_[i] + qd_lpf_alpha_ * v_[jmodel.idx_v()];
        }
      }
    }
    has_state_ = true;
  }

  void on_timer()
  {
    std::vector<double> tau_cmd(joint_names_.size(), 0.0);
    std::vector<double> pos_err(joint_names_.size(), 0.0);

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_state_) {
        return;
      }

      if (hold_current_on_start_ && !ref_initialized_) {
        q_ref_.resize(joint_names_.size(), 0.0);
        q_ref_cmd_.resize(joint_names_.size(), 0.0);
        q_ref_dot_cmd_.assign(joint_names_.size(), 0.0);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          const auto & jmodel = model_.joints[joint_ids_[i]];
          if (jmodel.nq() == 1) {
            q_ref_[i] = q_[jmodel.idx_q()];
            q_ref_cmd_[i] = q_ref_[i];
          }
        }
        ref_initialized_ = true;
        RCLCPP_INFO(get_logger(), "q_ref initialized from current pose (bumpless start)");
      }

      const Eigen::VectorXd tau_g = pinocchio::computeGeneralizedGravity(model_, *data_, q_);
      const double dt = 1.0 / control_rate_hz_;
      update_reference_generator(dt);

      for (size_t i = 0; i < joint_names_.size(); ++i) {
        const auto & jmodel = model_.joints[joint_ids_[i]];
        if (jmodel.nv() != 1 || jmodel.nq() != 1) {
          continue;
        }

        const double q_i = q_[jmodel.idx_q()];
        double e = get_or_last(q_ref_cmd_, i, get_or_last(q_ref_, i, 0.0)) - q_i;
        if (use_shortest_angular_error_) {
          e = std::atan2(std::sin(e), std::cos(e));
        }
        pos_err[i] = e;

        const double kp_i = get_or_last(kp_, i, 0.0);
        const double kd_i = get_or_last(kd_, i, 0.0);
        const double tau_ff = tau_g[jmodel.idx_v()];
        const double tau_fb = kp_i * e - kd_i * qd_filt_[i];
        const double tau_sat = clamp_abs(tau_ff + tau_fb, get_or_last(max_torque_vector_, i, 0.0));

        // Torque slew-rate limit to avoid fast oscillatory kicks on low-inertia joints.
        const double tau_rate_limit = std::max(0.0, get_or_last(torque_rate_limit_vector_, i, 0.0));
        const double max_step = tau_rate_limit * dt;
        const double delta = std::clamp(tau_sat - tau_prev_[i], -max_step, max_step);
        tau_cmd[i] = tau_prev_[i] + delta;
        tau_prev_[i] = tau_cmd[i];
      }
    }

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = tau_cmd;
    cmd_pub_->publish(cmd_msg);

    std_msgs::msg::Float64MultiArray dbg_cmd;
    dbg_cmd.data = tau_cmd;
    cmd_debug_pub_->publish(dbg_cmd);

    std_msgs::msg::Float64MultiArray dbg_err;
    dbg_err.data = pos_err;
    err_debug_pub_->publish(dbg_err);
  }

  std::string urdf_path_;
  std::vector<std::string> joint_names_;
  std::vector<double> q_ref_;
  std::vector<double> q_ref_cmd_;
  std::vector<double> q_ref_dot_cmd_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double qd_lpf_alpha_{0.2};
  bool enable_ref_generator_{true};
  bool time_sync_enabled_{true};
  std::vector<double> joint_max_speed_deg_;
  std::vector<double> joint_max_accel_deg_;
  std::vector<double> torque_rate_limit_vector_;
  std::vector<double> max_torque_vector_;
  bool hold_current_on_start_{true};
  std::string command_topic_;
  std::string joint_state_topic_;
  std::string cmd_debug_topic_;
  std::string error_debug_topic_;
  bool use_shortest_angular_error_{false};
  double control_rate_hz_{500.0};

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<pinocchio::JointIndex> joint_ids_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  std::vector<double> qd_filt_;
  std::vector<double> tau_prev_;
  std::vector<double> sync_q_start_;
  std::vector<double> sync_q_goal_;
  std::vector<double> sync_v_peak_;
  double sync_time_{0.0};
  double sync_T_{0.0};
  bool sync_profile_valid_{false};
  bool has_state_{false};
  bool ref_initialized_{false};
  std::mutex mtx_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr err_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace mujoco_pendulum

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mujoco_pendulum::PinocchioFfHoldNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
