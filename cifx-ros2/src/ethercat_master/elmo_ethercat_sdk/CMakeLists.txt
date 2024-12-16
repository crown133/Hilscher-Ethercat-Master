cmake_minimum_required(VERSION 3.8)
project(elmo_ethercat_sdk)

## Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(dependencies
  message_logger
  ethercat_sdk_master
  yaml-cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########
include_directories(
  include
  SYSTEM
)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/Elmo.cpp
  src/${PROJECT_NAME}/Configuration.cpp
  src/${PROJECT_NAME}/ConfigurationParser.cpp
  src/${PROJECT_NAME}/Reading.cpp
  src/${PROJECT_NAME}/Command.cpp
  src/${PROJECT_NAME}/Controlword.cpp
  src/${PROJECT_NAME}/Statusword.cpp
  src/${PROJECT_NAME}/DriveState.cpp
  src/${PROJECT_NAME}/PdoTypeEnum.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

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
