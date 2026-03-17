cmake_minimum_required(VERSION 3.8)
project(mujoco_pendulum)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

# -----------------------------
#  Kütüphane oluşturma
# -----------------------------
add_library(mujoco_pendulum_system SHARED
  src/mujoco_system.cpp
)

ament_target_dependencies(
  mujoco_pendulum_system
  rclcpp
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

ament_target_dependencies(
  computed_torque_node
  rclcpp
  sensor_msgs
  std_msgs
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
