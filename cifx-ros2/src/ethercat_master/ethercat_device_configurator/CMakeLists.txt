cmake_minimum_required(VERSION 3.8)
project(ethercat_device_configurator)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(anydrive QUIET)
find_package(elmo_ethercat_sdk QUIET)
find_package(maxon_epos_ethercat_sdk QUIET)
find_package(rokubimini_rsl_ethercat QUIET)

if(anydrive_FOUND)
  add_definitions(-D_ANYDRIVE_FOUND_)
  set(dependencies
    ${dependencies}
    anydrive
  )
endif()
if(rokubimini_rsl_ethercat_FOUND)
  add_definitions(-D_ROKUBI_FOUND_)
  set(dependencies
    ${dependencies}
    rokubimini_rsl_ethercat
  )
endif()
if(elmo_ethercat_sdk_FOUND)
  add_definitions(-D_ELMO_FOUND_)
  set(dependencies
    ${dependencies}
    elmo_ethercat_sdk
  )
endif()
if(maxon_epos_ethercat_sdk_FOUND)
  add_definitions(-D_MAXON_FOUND_)
  set(dependencies
    ${dependencies}
    maxon_epos_ethercat_sdk
  )
endif()
foreach(sdk ${dependencies})
  message("Found EtherCAT device sdk: " ${sdk})
endforeach()

set(dependencies
  ${dependencies}
  yaml-cpp
)

include_directories(
  include
)

file(GLOB_RECURSE SRC_FILES ./src/*.c*)
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(
  ${PROJECT_NAME}
  stdc++fs
)

add_executable(
  standalone
  src/standalone.cpp
)
ament_target_dependencies(standalone
  ${dependencies}
)
target_link_libraries(
  standalone
  ${PROJECT_NAME}
  -pthread
  stdc++fs
  yaml-cpp
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
install(
  TARGETS standalone
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
