cmake_minimum_required (VERSION 2.8)
project(cifx_examples)
set(PROJECT_VERSION, 2.0.0.0)

add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)

find_package(catkin REQUIRED)
find_package(cifx)

set(CATKIN_PACKAGE_DEPENDENCIES 
		pthread 
		rt
)

catkin_package(
  INCLUDE_DIRS
    
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
	SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_executable(cifxsample cifxsample/cifxlinuxsample.c)
target_include_directories(cifxsample BEFORE PUBLIC cifxsample/)
target_link_libraries (cifxsample cifx ${CATKIN_PACKAGE_DEPENDENCIES})
install(TARGETS cifxsample 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB CIFXTESTCONSOLE_SRC cifXTestConsole/*.c* cifXTestConsole/shared/*.c*)
add_executable(cifxtestconsole ${CIFXTESTCONSOLE_SRC})
target_include_directories(cifxtestconsole BEFORE PUBLIC cifXTestConsole cifXTestConsole/shared)
target_link_libraries (cifxtestconsole cifx ${CATKIN_PACKAGE_DEPENDENCIES})
install(TARGETS cifxtestconsole 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(TCPSERVERDIR "cifXTCPServer/")
include(${CMAKE_SOURCE_DIR}/cifXTCPServer/CMakeLists.txt)

file(GLOB SPISAMPLE_SRC SPISample/*.c)
add_executable(cifxspisample ${SPISAMPLE_SRC})
target_compile_options(cifxspisample PUBLIC "-Wno-format-overflow")
target_include_directories(cifxspisample BEFORE PUBLIC SPISample/)
target_link_libraries (cifxspisample cifx ${CATKIN_PACKAGE_DEPENDENCIES})
install(TARGETS cifxspisample 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB LOADABLEMODULES_SRC LoadModules/*.c*)
add_executable(cifxloadablemodules ${LOADABLEMODULES_SRC})
message(STATUS ${CIFX_HEADER})
target_include_directories(cifxloadablemodules PUBLIC LoadModules)
target_link_libraries (cifxloadablemodules cifx ${CATKIN_PACKAGE_DEPENDENCIES})
install(TARGETS cifxloadablemodules 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (NOT LINK_NETXTRANSPORT)
	file(GLOB ISASAMPLE_SRC ISASample/*.c*)
	add_executable(cifxisasample ${ISASAMPLE_SRC})
	target_include_directories(cifxisasample BEFORE PUBLIC ISASample/)
	target_link_libraries (cifxisasample cifx ${CATKIN_PACKAGE_DEPENDENCIES})
	install(TARGETS cifxisasample 
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif (NOT LINK_NETXTRANSPORT)
