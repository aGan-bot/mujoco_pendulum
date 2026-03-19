#include "mujoco_system.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <iostream>
#include <sstream>
#include <stdexcept>

using hardware_interface::return_type;
using CallbackReturn = hardware_interface::CallbackReturn;

namespace mujoco_pendulum
{

std::string MujocoSystem::resolve_model_path() const
{
  const auto package_share = ament_index_cpp::get_package_share_directory("mujoco_pendulum");
  const auto default_model = package_share + "/mujoco/pendulum.xml";
  const auto it = info_.hardware_parameters.find("mujoco_model");
  if (it == info_.hardware_parameters.end()) {
    return default_model;
  }

  const std::string & value = it->second;
  if (value.empty()) {
    return default_model;
  }
  if (!value.empty() && value[0] == '/') {
    return value;
  }
  if (value == "orion5") {
    return package_share + "/mujoco/orion5.xml";
  }
  if (value == "pendulum") {
    return default_model;
  }
  return package_share + "/mujoco/" + value;
}

CallbackReturn MujocoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  char error[1000] = "Could not load model";
  const auto model_path = resolve_model_path();

  model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);

  if (!model_)
  {
    std::cerr << "MuJoCo model load failed: " << error << std::endl;
    return CallbackReturn::ERROR;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    std::cerr << "MuJoCo data allocation failed" << std::endl;
    return CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_mj_ids_.clear();
  qpos_addrs_.clear();
  dof_addrs_.clear();
  actuator_mj_ids_.clear();

  const size_t n_joints = info_.joints.size();
  if (n_joints == 0) {
    std::cerr << "No joints provided by ros2_control in hardware info." << std::endl;
    return CallbackReturn::ERROR;
  }

  position_.assign(n_joints, 0.0);
  velocity_.assign(n_joints, 0.0);
  effort_.assign(n_joints, 0.0);
  cmd_effort_.assign(n_joints, 0.0);
  bias_effort_.assign(n_joints, 0.0);

  for (size_t i = 0; i < n_joints; ++i) {
    const auto & joint_name = info_.joints[i].name;
    joint_names_.push_back(joint_name);

    const int mj_joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_name.c_str());
    if (mj_joint_id < 0) {
      std::cerr << "Joint '" << joint_name << "' not found in MuJoCo model." << std::endl;
      return CallbackReturn::ERROR;
    }
    joint_mj_ids_.push_back(mj_joint_id);
    qpos_addrs_.push_back(model_->jnt_qposadr[mj_joint_id]);
    dof_addrs_.push_back(model_->jnt_dofadr[mj_joint_id]);

    int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, joint_name.c_str());
    if (actuator_id < 0) {
      if (static_cast<int>(i) < model_->nu) {
        actuator_id = static_cast<int>(i);
      } else {
        std::cerr << "No actuator found for joint '" << joint_name << "'" << std::endl;
        return CallbackReturn::ERROR;
      }
    }
    actuator_mj_ids_.push_back(actuator_id);
  }

  mj_forward(model_, data_);

  std::ostringstream node_name;
  node_name << "mujoco_system_debug_" << reinterpret_cast<uintptr_t>(this);
  debug_node_ = std::make_shared<rclcpp::Node>(node_name.str());
  bias_pub_ = debug_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mujoco/bias_torque", 10);

  std::cout << "Loaded model: " << model_path << " (nq=" << model_->nq
            << ", nv=" << model_->nv << ", nu=" << model_->nu << ")" << std::endl;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i], "position", &position_[i]);
    state_interfaces.emplace_back(joint_names_[i], "velocity", &velocity_[i]);
    state_interfaces.emplace_back(joint_names_[i], "effort", &effort_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(joint_names_[i], "effort", &cmd_effort_[i]);
  }

  return command_interfaces;
}

return_type MujocoSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const int qpos_addr = qpos_addrs_[i];
    const int dof_addr = dof_addrs_[i];
    position_[i] = data_->qpos[qpos_addr];
    velocity_[i] = data_->qvel[dof_addr];
    effort_[i] = data_->qfrc_actuator[dof_addr];
    bias_effort_[i] = data_->qfrc_bias[dof_addr];
  }
  return return_type::OK;
}

return_type MujocoSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const int actuator_id = actuator_mj_ids_[i];
    data_->ctrl[actuator_id] = cmd_effort_[i];
  }
  mj_step(model_, data_);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    bias_effort_[i] = data_->qfrc_bias[dof_addrs_[i]];
  }

  if (bias_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = bias_effort_;
    bias_pub_->publish(msg);
  }

  return return_type::OK;
}

MujocoSystem::~MujocoSystem()
{
  if (data_) {
    mj_deleteData(data_);
  }
  if (model_) {
    mj_deleteModel(model_);
  }
}

} // namespace mujoco_pendulum

PLUGINLIB_EXPORT_CLASS(
  mujoco_pendulum::MujocoSystem,
  hardware_interface::SystemInterface)
