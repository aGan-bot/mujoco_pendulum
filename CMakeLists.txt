cmake_minimum_required(VERSION 3.8)
project(mujoco_pendulum)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# -----------------------------
#  Kütüphane oluşturma
# -----------------------------
add_library(mujoco_pendulum_system SHARED
  src/mujoco_system.cpp
)

ament_target_dependencies(
  mujoco_pendulum_system
  rclcpp
  ament_index_cpp
  hardware_interface
  pluginlib
)

# include yolları
target_include_directories(
  mujoco_pendulum_system PUBLIC
  include
  /opt/ros/humble/opt/mujoco_vendor/include
)

# MuJoCo kütüphanesini linkle
target_link_libraries(
  mujoco_pendulum_system
  /opt/ros/humble/opt/mujoco_vendor/lib/libmujoco.so
)

add_executable(computed_torque_node
  src/computed_torque_node.cpp
)

add_executable(effort_test_node
  src/effort_test_node.cpp
)

add_executable(gravity_comp_relay_node
  src/gravity_comp_relay_node.cpp
)

add_executable(pinocchio_gravity_node
  src/pinocchio_gravity_node.cpp
)

add_executable(pinocchio_ff_hold_node
  src/pinocchio_ff_hold_node.cpp
)

ament_target_dependencies(
  computed_torque_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(
  effort_test_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(
  gravity_comp_relay_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(
  pinocchio_gravity_node
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)

ament_target_dependencies(
  pinocchio_ff_hold_node
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)

target_link_libraries(
  pinocchio_gravity_node
  pinocchio::pinocchio
)

target_link_libraries(
  pinocchio_ff_hold_node
  pinocchio::pinocchio
)

# RPATH ayarı (plugin runtime için)
set_target_properties(mujoco_pendulum_system PROPERTIES
  BUILD_RPATH "$ORIGIN/../lib"
  INSTALL_RPATH "$ORIGIN"
)

# -----------------------------
#  Plugin tanımı
# -----------------------------
pluginlib_export_plugin_description_file(
  hardware_interface
  mujoco_pendulum_plugin.xml
)

# -----------------------------
#  Install ayarları
# -----------------------------
install(
  TARGETS mujoco_pendulum_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS computed_torque_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS effort_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS gravity_comp_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS pinocchio_gravity_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS pinocchio_ff_hold_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf mujoco
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES mujoco_pendulum_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
#  Paket tanımı
# -----------------------------
ament_package()
