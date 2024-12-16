cmake_minimum_required(VERSION 3.8)
project(message_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include(CMakeDependentOption)
cmake_dependent_option(MELO_USE_COUT "Use std::cout" OFF
                       "DEFINED ENV{ROS_DISTRO}" ON)
set(MELO_USE_COUT ON)
message(${MELO_USE_COUT})
# configure_file(
#   include/message_logger/log/log_messages_backend_config.hpp.in
#   ${CMAKE_INSTALL_PREFIX}/include/message_logger/log/log_messages_backend_config.hpp @ONLY
# )
configure_file(
  include/message_logger/log/log_messages_backend_config.hpp.in
  ${CMAKE_INSTALL_PREFIX}/include/message_logger/message_logger/log/log_messages_backend_config.hpp @ONLY
)

# Set the CMake module path to include the 'cmake' directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# Include the custom cmake file
include(message_logger)

if(NOT MELO_USE_COUT)
  message(WARNING "Building message_logger using ROS2.")
  find_package(rclcpp REQUIRED)
  set(dependencies
    rclcpp
  )
else()
  message(STATUS "Building message_logger using std::cout.")
  set(dependencies)
endif()

if(DEFINED MELO_FUNCTION_PRINTS)
  add_compile_definitions(MELO_FUNCTION_PRINTS)
endif()

add_compile_options(-std=c++11 -fPIC -Wall)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/time/Time.cpp
  src/time/TimeStd.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/EmptyTests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      gtest_main
    )
  endif()

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(test_${PROJECT_NAME})
  endif()
endif()

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()