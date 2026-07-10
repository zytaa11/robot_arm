cmake_minimum_required(VERSION 3.8)
project(robot_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/usb_serial_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${hardware_interface_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 导出硬件插件
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugins.xml)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
