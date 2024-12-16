cmake_minimum_required(VERSION 3.8)
project(soem_interface_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  message_logger
  soem
  soem_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/ExampleSlave.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(
  ${PROJECT_NAME}_1
  src/example_slave.cpp
)
ament_target_dependencies(${PROJECT_NAME}_1
  ${dependencies}
)
target_link_libraries(
  ${PROJECT_NAME}_1
  ${PROJECT_NAME}
)

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS 
  ${PROJECT_NAME}_1
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


