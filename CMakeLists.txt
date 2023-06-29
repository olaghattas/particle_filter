cmake_minimum_required(VERSION 3.8)
project(particle_filter)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(collision REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fcl REQUIRED)
find_package(CUDA REQUIRED)
find_package(ros2-tiny-cuda-nn REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(colmap_srv REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(detection_msgs REQUIRED)

set(dependencies1
        rclcpp
        visualization_msgs
        eigen3_cmake_module
        Eigen3
        sensor_msgs
        ament_index_cpp
        geometry_msgs
        cv_bridge
        detection_msgs
        )

# Add executable
add_library(particle_filter src/particle_filter.cpp)
ament_target_dependencies(particle_filter ${dependencies1})
target_link_libraries(particle_filter ${collision_LIBRARIES})
target_link_libraries(particle_filter ${FCL_LIBRARIES})
target_link_libraries(particle_filter ${ros2-tiny-cuda-nn_LIBRARIES})


target_include_directories(particle_filter PUBLIC
        ${CUDA_INCLUDE_DIRS}
        ${ros2-tiny-cuda-nn_INCLUDE_DIRS}
        ${collision_INCLUDE_DIRS}
        )

add_executable(particle_filter_node src/particle_filter_node.cpp)
ament_target_dependencies(particle_filter_node ${dependencies1})
target_link_libraries(particle_filter_node particle_filter)
ament_target_dependencies(particle_filter_node colmap_srv)
ament_target_dependencies(particle_filter_node ros2-tiny-cuda-nn)
target_link_libraries(particle_filter_node ${collision_LIBRARIES})
target_link_libraries(particle_filter_node ${ros2-tiny-cuda-nn_LIBRARIES})

target_include_directories(particle_filter PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

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



#cmake_minimum_required(VERSION 3.8)
#project(particle_filter)
#
#if (NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif ()
#
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()
#
## find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(visualization_msgs REQUIRED)# Find Eigen package
#find_package(eigen3_cmake_module REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(collision REQUIRED)
##find_package(generate_parameter_library REQUIRED)
#find_package(sensor_msgs REQUIRED)
#include_directories(${sensor_msgs_INCLUDE_DIRS})
#find_package(fcl REQUIRED)
#find_package(CUDA REQUIRED)
#find_package(ros2-tiny-cuda-nn REQUIRED)
#
#set(dependencies1
#        rclcpp
#        visualization_msgs
#        eigen3_cmake_module
#        Eigen3
#        sensor_msgs
#
#        )
#set(dependencies2
#        eigen3_cmake_module
#        Eigen3
#
#        )
#
## Add executable
#add_library(particle_filter src/particle_filter.cpp)
#ament_target_dependencies(particle_filter ${dependencies2})
#target_link_libraries(particle_filter ${collision_LIBRARIES})
#target_link_libraries(particle_filter ${ros2-tiny-cuda-nn_LIBRARIES})
#
#target_include_directories(particle_filter PUBLIC ${CUDA_INCLUDE_DIRS})
#target_include_directories(particle_filter PUBLIC ${ros2-tiny-cuda-nn_INCLUDE_DIR})
#target_include_directories(particle_filter PUBLIC ${collision_INCLUDE_DIR})
#target_link_libraries(particle_filter collision)
#
#
#add_executable(particle_filter_node src/particle_filter_node.cpp)
#ament_target_dependencies(particle_filter_node ${dependencies1})
#target_link_libraries(particle_filter_node particle_filter)
##target_link_libraries(particle_filter_node collision)
#
##if(BUILD_TESTING)
##  add_executable(particle_filter_test tests/test.cpp)
##  ament_target_dependencies(particle_filter_test ${dependencies2})
##  target_link_libraries(particle_filter_test particle_filter)
##
##endif()
#
#include_directories(
#        include
#        ${EIGEN3_INCLUDE_DIRS}
#)
#install(TARGETS
#
#        particle_filter
#        particle_filter_node
#        DESTINATION lib/${PROJECT_NAME}
#        )
#
#ament_package()
