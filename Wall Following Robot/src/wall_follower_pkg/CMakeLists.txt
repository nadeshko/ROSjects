cmake_minimum_required(VERSION 3.5)
project(wall_follower_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(findwall_server src/findwall_server.cpp)
ament_target_dependencies(findwall_server rclcpp geometry_msgs sensor_msgs custom_interfaces)

add_executable(odomrecord_server src/odomrecord_server.cpp)
ament_target_dependencies(odomrecord_server rclcpp rclcpp_action geometry_msgs nav_msgs custom_interfaces)

add_executable(wall_follower src/wall_follower.cpp)
ament_target_dependencies(wall_follower rclcpp rclcpp_action geometry_msgs sensor_msgs custom_interfaces)

install(TARGETS
   wall_follower
   findwall_server 
   odomrecord_server
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
