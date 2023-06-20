cmake_minimum_required(VERSION 3.8)
project(particle_filter)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)# Find Eigen package
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(generate_parameter_library REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${sensor_msgs_INCLUDE_DIRS})

set(dependencies1
        rclcpp
        visualization_msgs
        eigen3_cmake_module
        Eigen3
        sensory_msgs

        )
set(dependencies2
        eigen3_cmake_module
        Eigen3

        )

# Add executable
add_executable(particle_filter src/particle_filter.cpp)
ament_target_dependencies(particle_filter ${dependencies2})

add_executable(particle_filter_node src/particle_filter_node.cpp)
ament_target_dependencies(particle_filter_node ${dependencies1})

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

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS
        particle_filter
        particle_filter_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
