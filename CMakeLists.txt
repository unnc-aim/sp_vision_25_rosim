cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# ROS2 Dependencies - STANDARD AMENT PACKAGE STRUCTURE
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(sp_msgs QUIET)

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)

target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)


# sim_sentry (Gazebo/ROS2 simulation) does not require sp_msgs
add_executable(sim_sentry src/sim_sentry.cpp)
target_link_libraries(sim_sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
ament_target_dependencies(sim_sentry rclcpp std_msgs sensor_msgs cv_bridge rosidl_typesupport_cpp ament_index_cpp rmoss_interfaces vision_interfaces)


# sentry* and some tests require sp_msgs (nav interface)
if(sp_msgs_FOUND)
  add_executable(sentry src/sentry.cpp)
  add_executable(sentry_bp src/sentry_bp.cpp)
  add_executable(sentry_debug src/sentry_debug.cpp)
  add_executable(sentry_multithread src/sentry_multithread.cpp)

  target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
  target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
  target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
  target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)

  foreach(target sentry sentry_bp sentry_debug sentry_multithread)
    ament_target_dependencies(${target} rclcpp std_msgs sensor_msgs cv_bridge rosidl_typesupport_cpp sp_msgs)
  endforeach()

  add_executable(topic_loop_test tests/topic_loop_test.cpp)
  target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
  ament_target_dependencies(topic_loop_test rclcpp std_msgs sensor_msgs cv_bridge rosidl_typesupport_cpp sp_msgs)

  # ROS2 utility tests (depend on io::ROS2 -> sp_msgs)
  add_executable(publish_test tests/publish_test.cpp)
  add_executable(subscribe_test tests/subscribe_test.cpp)
  target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
  target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
  foreach(target publish_test subscribe_test)
    ament_target_dependencies(${target} rclcpp std_msgs sensor_msgs cv_bridge rosidl_typesupport_cpp sp_msgs)
  endforeach()
else()
  message(WARNING "sp_msgs not found: skipping sentry* and topic_loop_test (nav interface)")
endif()


# Install rules
install(TARGETS
  sim_sentry
  DESTINATION lib/${PROJECT_NAME}
)

if(sp_msgs_FOUND)
  install(TARGETS
    sentry
    sentry_bp
    sentry_debug
    sentry_multithread
    publish_test
    subscribe_test
    topic_loop_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# NOTE: ament_cmake symlink install doesn't support FILES_MATCHING;
# using it forces fallback to CMake's native install() and avoids recursion.
install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs
  FILES_MATCHING PATTERN "*"
)
install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
  FILES_MATCHING PATTERN "*"
)

ament_package()
