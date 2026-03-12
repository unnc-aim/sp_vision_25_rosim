cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(cv_bridge QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(rmoss_interfaces QUIET)
find_package(vision_interfaces QUIET)
find_package(sp_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT sensor_msgs_FOUND OR NOT cv_bridge_FOUND OR NOT rosidl_typesupport_cpp_FOUND)
  message(WARNING "ROS2 core not found, skipping ROS2/simulation code.")
  # 允许在无 ROS2 环境下正常编译（仿真相关构造函数会在运行时提示）
  target_compile_definitions(io PUBLIC SP_VISION_HAS_ROS2=0)
else()
  message(STATUS "ROS2 core found, compiling simulation code.")
  target_compile_definitions(io PUBLIC SP_VISION_HAS_ROS2=1)

  # 仿真(ROS2)相关的文件
  target_sources(io PRIVATE sim_camera.cpp sim_board.cpp)
  include_directories(${std_msgs_INCLUDE_DIRS})
  ament_target_dependencies(io rclcpp std_msgs sensor_msgs cv_bridge)

  # rmoss_interfaces 用于 Gazebo 仿真控制（GimbalCmd, ShootCmd）
  if(rmoss_interfaces_FOUND)
    message(STATUS "rmoss_interfaces found, enabling Gazebo control.")
    ament_target_dependencies(io rmoss_interfaces)
  else()
    message(WARNING "rmoss_interfaces not found, Gazebo control will not work")
  endif()

  # vision_interfaces 用于发布自瞄状态（AutoAim）
  if(vision_interfaces_FOUND)
    message(STATUS "vision_interfaces found, enabling AutoAim publishing.")
    ament_target_dependencies(io vision_interfaces)
  else()
    message(WARNING "vision_interfaces not found, AutoAim publishing will not work")
  endif()

  # 导航接口（依赖 sp_msgs）
  if(sp_msgs_FOUND)
    message(STATUS "sp_msgs found, compiling nav interface code.")
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)
    ament_target_dependencies(io sp_msgs)
  else()
    message(WARNING "sp_msgs not found, skipping nav interface code in io/ros2")
  endif()
endif()
