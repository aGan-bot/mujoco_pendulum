#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mujoco_pendulum
{

class MujocoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  ~MujocoSystem();

private:
  std::string resolve_model_path() const;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_mj_ids_;
  std::vector<int> qpos_addrs_;
  std::vector<int> dof_addrs_;
  std::vector<int> actuator_mj_ids_;

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> cmd_effort_;
  std::vector<double> bias_effort_;
  std::vector<double> rne_gravity_effort_;

  rclcpp::Node::SharedPtr debug_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bias_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rne_gravity_pub_;

  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;
};

} 
