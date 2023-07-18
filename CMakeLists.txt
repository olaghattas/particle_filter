cmake_minimum_required(VERSION 3.8 )
project(particle_filter)
enable_language(CUDA)
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
find_package(collision_lib REQUIRED)
#find_package(collision REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fcl REQUIRED)
find_package(CUDA REQUIRED)
find_package(ros2-tiny-cuda-nn REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(detection_msgs REQUIRED)
#find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)



set(dependencies1
        rclcpp
        visualization_msgs
        sensor_msgs
        ament_index_cpp
        geometry_msgs
        cv_bridge
        detection_msgs
#        collision
        collision_lib
        tf2_ros
        )

# Add executable
add_library(particle_filter src/particle_filter.cpp)
ament_target_dependencies(particle_filter ${dependencies1})
#target_link_libraries(particle_filter  collision_lib)
target_link_libraries(particle_filter ${FCL_LIBRARIES})
#target_link_libraries(particle_filter collision)
ament_target_dependencies(particle_filter ros2-tiny-cuda-nn)
target_link_libraries(particle_filter ${ros2-tiny-cuda-nn_LIBRARIES})
target_include_directories(particle_filter PUBLIC ${EIGEN3_INCLUDE_DIRS})


target_include_directories(particle_filter PUBLIC
        ${CUDA_INCLUDE_DIRS}
        ${ros2-tiny-cuda-nn_INCLUDE_DIR}
        )

if (BUILD_TESTING)
  add_subdirectory(tests)


endif ()

add_executable(particle_filter_node src/particle_filter_node.cpp)
ament_target_dependencies(particle_filter_node ${dependencies1})
#target_link_libraries(particle_filter  collision_lib)
#target_link_libraries(particle_filter_node particle_filter)
ament_target_dependencies(particle_filter_node ros2-tiny-cuda-nn)
#target_link_libraries(particle_filter_node ${collision_LIBRARIES})
target_link_libraries(particle_filter_node ${ros2-tiny-cuda-nn_LIBRARIES})
target_include_directories(particle_filter_node PUBLIC ${EIGEN3_INCLUDE_DIRS})


add_executable(yaml_node src/example_yaml.cpp)
target_include_directories(yaml_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(yaml_node PUBLIC ${YAML_CPP_INCLUDE_DIRS})

# Link the libraries
target_link_libraries(yaml_node
        ${YAML_CPP_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        )

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

#target_include_directories(particle_filter_node PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
target_include_directories(particle_filter_node PUBLIC
        ${CUDA_INCLUDE_DIRS}
        ${ros2-tiny-cuda-nn_INCLUDE_DIR}
#        ${collision_INCLUDE_DIR}
        )

target_include_directories(particle_filter PUBLIC
        ${CUDA_INCLUDE_DIRS}
        ${ros2-tiny-cuda-nn_INCLUDE_DIR}
#        ${collision_INCLUDE_DIR}
        )

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
        particle_filter
#        particle_filter_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()

