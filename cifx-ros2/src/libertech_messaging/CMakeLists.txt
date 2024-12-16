cmake_minimum_required(VERSION 3.8)
project(libertech_messaging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  Eigen3
  sensor_msgs
  libertech_interface_msgs
  ethercat_device_configurator
  elmo_ethercat_sdk
  yaml-cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")  # 如果是 ARM64 架构
    link_directories(libs/arm64)               # 指定 ARM64 架构下的库目录
else()                                         # 否则（如果是 x86_64 架构）
    link_directories(libs/x86_64)             # 指定 x86_64 架构下的库目录
endif()


add_library(head_interface SHARED
  src/HeadInterface.cpp
)
# ament_target_dependencies(${PROJECT_NAME}_sdk
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )
target_link_libraries(head_interface controlcan.so pthread)


add_executable(demo src/demo.cpp)
# ament_target_dependencies(${PROJECT_NAME}_sdk
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )
target_link_libraries(demo head_interface)

add_executable(head_node src/head_node.cpp)
ament_target_dependencies(head_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(head_node head_interface)

add_executable(elmo_node src/elmo_node.cpp)
ament_target_dependencies(elmo_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(elmo_node
  -pthread
  stdc++fs
  yaml-cpp
)

add_executable(elmo_test_node src/elmo_test_node.cpp)
ament_target_dependencies(elmo_test_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(elmo_test_node
  -pthread
  stdc++fs
  yaml-cpp
)

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY 
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS demo head_node elmo_node elmo_test_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/joint_state_merge.py
  scripts/test_hybrid_joint_cmd_pub.py
  scripts/hybrid_joint_cmd_dynamic_pub.py
  scripts/motor_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()


