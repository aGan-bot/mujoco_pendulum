#include <algorithm>
#include <cmath>
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

class PinocchioGravityNode : public rclcpp::Node
{
public:
  PinocchioGravityNode()
  : Node("pinocchio_gravity_node")
  {
    const auto default_urdf =
      ament_index_cpp::get_package_share_directory("orion5_humble_description") + "/urdf/orion5.urdf";

    urdf_path_ = declare_parameter<std::string>("urdf_path", default_urdf);
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    mujoco_bias_topic_ = declare_parameter<std::string>("mujoco_bias_topic", "/mujoco/bias_torque");
    mujoco_rne_gravity_topic_ = declare_parameter<std::string>(
      "mujoco_rne_gravity_topic", "/mujoco/rne_gravity_torque");
    gravity_topic_ = declare_parameter<std::string>("gravity_topic", "/pinocchio/gravity_torque");
    error_topic_ = declare_parameter<std::string>("error_topic", "/pinocchio/gravity_error");
    bias_topic_ = declare_parameter<std::string>("bias_topic", "/pinocchio/bias_torque");
    bias_error_topic_ = declare_parameter<std::string>("bias_error_topic", "/pinocchio/bias_error");
    rne_gravity_error_topic_ = declare_parameter<std::string>(
      "rne_gravity_error_topic", "/pinocchio/rne_gravity_error");
    log_period_sec_ = declare_parameter<double>("log_period_sec", 1.0);

    try {
      pinocchio::urdf::buildModel(urdf_path_, model_);
      data_ = std::make_unique<pinocchio::Data>(model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to build Pinocchio model from URDF '%s': %s",
        urdf_path_.c_str(),
        e.what());
      throw;
    }

    joint_ids_.clear();
    for (const auto & name : joint_names_) {
      const auto id = model_.getJointId(name);
      if (id == 0) {
        RCLCPP_FATAL(get_logger(), "Joint '%s' not found in URDF model", name.c_str());
        throw std::runtime_error("Joint not found in Pinocchio model");
      }
      joint_ids_.push_back(id);
    }

    q_ = pinocchio::neutral(model_);
    v_ = Eigen::VectorXd::Zero(model_.nv);

    gravity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(gravity_topic_, 20);
    error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(error_topic_, 20);
    bias_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(bias_topic_, 20);
    bias_error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(bias_error_topic_, 20);
    rne_gravity_error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      rne_gravity_error_topic_, 20);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      50,
      std::bind(&PinocchioGravityNode::on_joint_state, this, std::placeholders::_1));

    mujoco_bias_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      mujoco_bias_topic_,
      50,
      std::bind(&PinocchioGravityNode::on_mujoco_bias, this, std::placeholders::_1));
    mujoco_rne_gravity_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      mujoco_rne_gravity_topic_,
      50,
      std::bind(&PinocchioGravityNode::on_mujoco_rne_gravity, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "pinocchio_gravity_node started. urdf=%s, joints=%zu",
      urdf_path_.c_str(),
      joint_names_.size());
  }

private:
  void on_mujoco_bias(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    mujoco_bias_ = msg->data;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::vector<double> tau_g(joint_names_.size(), 0.0);
    std::vector<double> tau_b(joint_names_.size(), 0.0);

    {
      std::lock_guard<std::mutex> lock(mtx_);
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        const auto & expected = joint_names_[i];
        auto it = std::find(msg->name.begin(), msg->name.end(), expected);
        if (it == msg->name.end()) {
          continue;
        }
        const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));

        const auto joint_id = joint_ids_[i];
        const auto & jmodel = model_.joints[joint_id];

        if (jmodel.nq() == 1 && idx < msg->position.size()) {
          q_[jmodel.idx_q()] = msg->position[idx];
        }
        if (jmodel.nv() == 1 && idx < msg->velocity.size()) {
          v_[jmodel.idx_v()] = msg->velocity[idx];
        }
      }

      const Eigen::VectorXd tau_g_full = pinocchio::computeGeneralizedGravity(model_, *data_, q_);
      const Eigen::VectorXd tau_b_full = pinocchio::rnea(model_, *data_, q_, v_, Eigen::VectorXd::Zero(model_.nv));

      for (size_t i = 0; i < joint_names_.size(); ++i) {
        const auto joint_id = joint_ids_[i];
        const auto & jmodel = model_.joints[joint_id];
        if (jmodel.nv() == 1) {
          tau_g[i] = tau_g_full[jmodel.idx_v()];
          tau_b[i] = tau_b_full[jmodel.idx_v()];
        }
      }
    }

    std_msgs::msg::Float64MultiArray g_msg;
    g_msg.data = tau_g;
    gravity_pub_->publish(g_msg);

    std_msgs::msg::Float64MultiArray b_msg;
    b_msg.data = tau_b;
    bias_pub_->publish(b_msg);

    std::vector<double> err;
    std::vector<double> bias_err;
    std::vector<double> rne_gravity_err;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (mujoco_bias_.size() == tau_g.size()) {
        err.resize(tau_g.size(), 0.0);
        bias_err.resize(tau_g.size(), 0.0);
        for (size_t i = 0; i < tau_g.size(); ++i) {
          err[i] = mujoco_bias_[i] - tau_g[i];
          bias_err[i] = mujoco_bias_[i] - tau_b[i];
        }
      }
      if (mujoco_rne_gravity_.size() == tau_g.size()) {
        rne_gravity_err.resize(tau_g.size(), 0.0);
        for (size_t i = 0; i < tau_g.size(); ++i) {
          rne_gravity_err[i] = mujoco_rne_gravity_[i] - tau_g[i];
        }
      }
    }

    if (!err.empty()) {
      std_msgs::msg::Float64MultiArray e_msg;
      e_msg.data = err;
      error_pub_->publish(e_msg);

      std_msgs::msg::Float64MultiArray be_msg;
      be_msg.data = bias_err;
      bias_error_pub_->publish(be_msg);

      if (!rne_gravity_err.empty()) {
        std_msgs::msg::Float64MultiArray rge_msg;
        rge_msg.data = rne_gravity_err;
        rne_gravity_error_pub_->publish(rge_msg);
      }

      const auto now = get_clock()->now();
      if ((now - last_log_time_).seconds() >= log_period_sec_) {
        last_log_time_ = now;
        double g_rms = 0.0;
        double g_max_abs = 0.0;
        double b_rms = 0.0;
        double b_max_abs = 0.0;
        double rg_rms = 0.0;
        double rg_max_abs = 0.0;
        for (const auto & e : err) {
          g_rms += e * e;
          g_max_abs = std::max(g_max_abs, std::abs(e));
        }
        for (const auto & e : bias_err) {
          b_rms += e * e;
          b_max_abs = std::max(b_max_abs, std::abs(e));
        }
        if (!rne_gravity_err.empty()) {
          for (const auto & e : rne_gravity_err) {
            rg_rms += e * e;
            rg_max_abs = std::max(rg_max_abs, std::abs(e));
          }
          rg_rms = std::sqrt(rg_rms / static_cast<double>(rne_gravity_err.size()));
        }
        g_rms = std::sqrt(g_rms / static_cast<double>(err.size()));
        b_rms = std::sqrt(b_rms / static_cast<double>(bias_err.size()));
        if (!rne_gravity_err.empty()) {
          RCLCPP_INFO(
            get_logger(),
            "errors | gravity rms=%.4f max=%.4f | bias rms=%.4f max=%.4f | rne_gravity rms=%.4f max=%.4f",
            g_rms,
            g_max_abs,
            b_rms,
            b_max_abs,
            rg_rms,
            rg_max_abs);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "errors | gravity rms=%.4f max=%.4f | bias rms=%.4f max=%.4f",
            g_rms,
            g_max_abs,
            b_rms,
            b_max_abs);
        }
      }
    }
  }

  void on_mujoco_rne_gravity(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    mujoco_rne_gravity_ = msg->data;
  }

  std::string urdf_path_;
  std::vector<std::string> joint_names_;
  std::string joint_state_topic_;
  std::string mujoco_bias_topic_;
  std::string mujoco_rne_gravity_topic_;
  std::string gravity_topic_;
  std::string error_topic_;
  std::string bias_topic_;
  std::string bias_error_topic_;
  std::string rne_gravity_error_topic_;
  double log_period_sec_{1.0};

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<pinocchio::JointIndex> joint_ids_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;

  std::vector<double> mujoco_bias_;
  std::vector<double> mujoco_rne_gravity_;
  std::mutex mtx_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bias_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bias_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rne_gravity_error_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mujoco_bias_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mujoco_rne_gravity_sub_;

  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace mujoco_pendulum

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mujoco_pendulum::PinocchioGravityNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
