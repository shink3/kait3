cmake_minimum_required(VERSION 3.8)
project(kait3_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  emergency_stop_controller 
  SHARED
  src/emergency_stop_controller.cpp
)

target_include_directories(
  emergency_stop_controller
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(emergency_stop_controller PRIVATE cxx_std_17)

target_link_libraries(
  emergency_stop_controller
  PRIVATE
  rcl::rcl
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  controller_interface::controller_interface
  realtime_tools::realtime_tools
  pluginlib::pluginlib
  ${std_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(
  controller_interface
  kait3_controller_plugins.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS emergency_stop_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
