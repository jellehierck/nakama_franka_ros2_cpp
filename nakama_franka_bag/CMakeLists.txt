cmake_minimum_required(VERSION 3.8)
project(nakama_franka_bag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Export a compile_commands.json file for the clangd language server
# Alternative: remove this line and use `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(nakama_franka_interfaces REQUIRED)

add_executable(franka_state_bag_recorder src/franka_state_bag_recorder.cpp)
target_include_directories(franka_state_bag_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(franka_state_bag_recorder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  franka_state_bag_recorder
  "example_interfaces"
  "rclcpp"
  "rosbag2_cpp"
  "std_msgs"
  "sensor_msgs"
  "franka_msgs"
  "nakama_franka_interfaces"
)

install(TARGETS franka_state_bag_recorder
  DESTINATION lib/${PROJECT_NAME})

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
