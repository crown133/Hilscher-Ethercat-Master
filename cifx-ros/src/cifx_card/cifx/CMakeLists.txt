cmake_minimum_required(VERSION 2.8.12)

project(cifx)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
	roscpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)


set(DRVDIR "${CMAKE_CURRENT_SOURCE_DIR}/")

set(src_dir "${DRVDIR}/src/cifx")
set(tk_dir "${DRVDIR}/include/cifx/Toolkit")
set(src_tk_dir "${tk_dir}/Source")
set(src_tap_dir "${DRVDIR}/include/cifx/netx_tap")

# file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "${src_dir}/*.c" "${src_tk_dir}/*.c")
include_directories(
	include
	include/cifx
	include/cifx/Toolkit/
	include/cifx/Toolkit/Source
	include/cifx/Toolkit/Common/cifXAPI
	include/cifx/Toolkit/Common/HilscherDefinitions
	${catkin_INCLUDE_DIRS}
	/usr/include/
)

set(drv_libs "-lpthread -lrt")

add_definitions(-DVERSION_INFO="\"${DRVVERSION}\"")

option(DEBUG "Build Library debug messages" OFF)
option(DISABLE_PCI "Disable use of PCI" OFF)
option(TIME "Enable device time setting during start-up" OFF)
option(DMA "Compile driver with dma support" ON)
option(NO_MINSLEEP "Disable minimum sleep time" OFF)
option(VIRTETH "Enables virtual ethernet interface support" OFF)
option(HWIF "Enables the toolkit's Hardware Function Interface (e.g. for SPI)" OFF)
option(PLUGIN "Enables support of device plugins and enables hardware function interface (see HWIF)" OFF)
option(SPM_PLUGIN "Build spm-plugin and enables support of device plugins (see PLUGIN)" OFF)

if(DEBUG)
	add_definitions(-DDEBUG)
endif(DEBUG)
if(DISABLE_PCI)
	add_definitions(-DCIFX_TOOLKIT_DISABLEPCI)
else(DISABLE_PCI)
	set(drv_libs "${drv_libs} -lpciaccess")
	set(REQUIRED_PACKAGES pciaccess)
	if (NOT PCIACCESS_LIBS)
		set(PCIACCESS_LIBS -lpciaccess)
	endif(NOT PCIACCESS_LIBS)
	if (PCIACCESS_CFLAGS)
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${PCIACCESS_CFLAGS}")
	endif (PCIACCESS_CFLAGS)
endif(DISABLE_PCI)
if(TIME)
	add_definitions(-DCIFX_TOOLKIT_TIME)
endif(TIME)
if(DMA)
	add_definitions(-DCIFX_TOOLKIT_DMA)
endif(DMA)
if(NO_MINSLEEP)
	add_definitions(-DNO_MIN_SLEEP)
endif(NO_MINSLEEP)
if(VIRTETH)
	add_definitions(-DCIFXETHERNET)
	include_directories(${src_tap_dir})
	file(GLOB NETXTAP_SOURCES ${src_tap_dir}/*.c)
	set_source_files_properties(${src_tk_dir}/cifXFunctions.c PROPERTIES COMPILE_FLAGS -DAPIRENAME)
endif(VIRTETH)
if(HWIF OR PLUGIN OR SPM_PLUGIN)
	add_definitions(-DCIFX_DRV_HWIF)
endif(HWIF OR PLUGIN OR SPM_PLUGIN)
if(PLUGIN OR SPM_PLUGIN)
	add_definitions(-DCIFX_PLUGIN_SUPPORT)
	set(PLUGIN_LIBS "-ldl")
	set(drv_libs "${drv_libs} ${PLUGIN_LIBS}")
endif(PLUGIN OR SPM_PLUGIN)

# check endianess
include (TestBigEndian)
TEST_BIG_ENDIAN(IS_BIG_ENDIAN)
if(IS_BIG_ENDIAN)
	message(STATUS "BIG_ENDIAN")
	add_definitions(-DCIFX_TOOLKIT_BIGENDIAN)
else(IS_BIG_ENDIAN)
	message(STATUS "LITTLE_ENDIAN")
endif(IS_BIG_ENDIAN)

# configure pkgconfig file
configure_file( ${DRVDIR}/cifx.pc.in
		${CMAKE_CURRENT_BINARY_DIR}/cifx.pc @ONLY)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
	include
	include/cifx
	include/cifx/Toolkit/
	include/cifx/Toolkit/Source
	include/cifx/Toolkit/Common/cifXAPI
	include/cifx/Toolkit/Common/HilscherDefinitions
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# link and create shared & static library
add_library(${PROJECT_NAME} ${SOURCES} ${NETXTAP_SOURCES})
add_library(${PROJECT_NAME}_static STATIC ${SOURCES} ${NETXTAP_SOURCES})

target_link_libraries(${PROJECT_NAME} ${drv_libs} ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME}_static PROPERTIES OUTPUT_NAME cifx)


# install resources
# install(TARGETS ${PROJECT_NAME} DESTINATION ${dest_dir})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_static
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
# file(GLOB INSTALL_HEADERS
#      "${DRVDIR}/Toolkit/Common/cifXAPI/*.h"
#      "${DRVDIR}/Toolkit/Common/HilscherDefinitions/*.h"
#      "${DRVDIR}/cifxlinux.h" "${DRVDIR}/Toolkit/Source/cifXEndianess.h")

# install(FILES ${INSTALL_HEADERS}
#   	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/cifx)

install(DIRECTORY include/
  	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if (VIRTETH)
	install(FILES ${DRVDIR}/include/cifx/netx_tap/netx_tap.h DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/cifx)
endif()

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cifx.pc DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig)

if (SPM_PLUGIN)
	set (CIFX_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/")
	include(${CMAKE_CURRENT_SOURCE_DIR}/../plugins/netx-spm/CMakeLists.txt)
endif(SPM_PLUGIN)
