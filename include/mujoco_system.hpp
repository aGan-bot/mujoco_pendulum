#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"

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
 
  ~MujocoSystem();  // Destructor
private:

  double position_ = 0.0;
  double velocity_ = 0.0;
  double effort_ = 0.0;

  double cmd_effort_ = 0.0;
    // MuJoCo
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
};

}