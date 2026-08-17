cmake_minimum_required(VERSION 3.8)
project(fr3_moveit_api_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(fr3_interfaces REQUIRED)


add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)
install(TARGETS 
  test_moveit
  DESTINATION lib/${PROJECT_NAME}/
)

add_executable(named_target src/named_target)
ament_target_dependencies(named_target 
rclcpp
moveit_ros_planning_interface
)
install(TARGETS named_target
DESTINATION lib/${PROJECT_NAME}/
) 



add_executable(joints_target src/joints_target)
ament_target_dependencies(joints_target 
rclcpp
moveit_ros_planning_interface
fr3_interfaces
)
install(TARGETS joints_target
DESTINATION lib/${PROJECT_NAME}/)
ament_package()
