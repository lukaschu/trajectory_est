cmake_minimum_required(VERSION 3.8)
project(trajectory_est)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# custom dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(custom_messages REQUIRED) 

# my executables
add_executable(trajectory_est src/traj_estimator_main.cpp)

add_library(${PROJECT_NAME}_NODE src/traj_estimator_node.cpp src/kalman_filter.cpp)

# dep.
ament_target_dependencies(${PROJECT_NAME}_NODE rclcpp std_msgs geometry_msgs tf2_ros mocap4r2_msgs Eigen3 yaml-cpp custom_messages)

install(TARGETS 
  trajectory_est
  ${PROJECT_NAME}_NODE
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

target_include_directories(${PROJECT_NAME}_NODE
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
    )

target_link_libraries(${PROJECT_NAME}_NODE Eigen3::Eigen yaml-cpp)

target_link_libraries(trajectory_est ${PROJECT_NAME}_NODE)

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
