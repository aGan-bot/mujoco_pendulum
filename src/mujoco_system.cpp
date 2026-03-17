#include "mujoco_system.hpp"
#include <pluginlib/class_list_macros.hpp>

using hardware_interface::return_type;
using CallbackReturn = hardware_interface::CallbackReturn;

namespace mujoco_pendulum
{

CallbackReturn MujocoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  char error[1000] = "Could not load model";

  model_ = mj_loadXML(
    "/home/as/humble_ws/src/mujoco_pendulum/mujoco/pendulum.xml",
    0,
    error,
    1000);

  if (!model_)
  {
    std::cout << error << std::endl;
    return CallbackReturn::ERROR;
  }

  data_ = mj_makeData(model_);
  std::cout << "Model nq: " << model_->nq << std::endl;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      "hinge", "position", &position_);

  state_interfaces.emplace_back(
      "hinge", "velocity", &velocity_);

  state_interfaces.emplace_back(
      "hinge", "effort", &effort_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      "hinge", "effort", &cmd_effort_);

  return command_interfaces;
}

return_type MujocoSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  position_ = data_->qpos[0];
  velocity_ = data_->qvel[0];
  effort_ = data_->qfrc_actuator[0];

  // std::cout << "[read] cmd_effort_: " << cmd_effort_ 
  //           << ", pos: " << position_ 
  //           << ", vel: " << velocity_ << std::endl;
  //std::cout << "Model nq: " << model_->nq << std::endl;
  return return_type::OK;
}

return_type MujocoSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{

  data_->ctrl[0] = cmd_effort_;
  mj_step(model_, data_);

  return return_type::OK;
}

MujocoSystem::~MujocoSystem() {
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
}

} // namespace mujoco_pendulum

PLUGINLIB_EXPORT_CLASS(
  mujoco_pendulum::MujocoSystem,
  hardware_interface::SystemInterface)
