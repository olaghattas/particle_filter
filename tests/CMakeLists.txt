list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(gtest)
find_package(ament_index_cpp REQUIRED)

add_executable(test_tf_extrinsics
        test_tf_extrinsics.cpp
        )

find_package(GTest REQUIRED)
target_link_libraries(test_tf_extrinsics ${GTEST_LIBRARIES} particle_filter)
ament_target_dependencies(test_tf_extrinsics
        ament_index_cpp
        rclcpp
        sensor_msgs
        geometry_msgs
        )