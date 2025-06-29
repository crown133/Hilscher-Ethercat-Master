cmake_minimum_required(VERSION 3.8)
project(libertech_interface_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/msg/* ${CMAKE_CURRENT_SOURCE_DIR}/srv/* )
rosidl_generate_interfaces(${PROJECT_NAME}
        msg/HybridJointState.msg
        msg/ActuatorCmds.msg
        msg/MotionCommands.msg
        msg/TouchSensor.msg
        srv/SimulationReset.srv
        srv/InitHardware.srv
        srv/GaitSwitch.srv
        DEPENDENCIES ament_cmake std_msgs builtin_interfaces geometry_msgs sensor_msgs 
        )

ament_package()

