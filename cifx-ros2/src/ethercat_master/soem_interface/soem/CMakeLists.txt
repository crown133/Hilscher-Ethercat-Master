cmake_minimum_required(VERSION 3.9)
project(soem)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

if(WIN32)
  set(OS "win32")
else()
  set(OS "linux")
endif()

file(GLOB SOEM_COMMON_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/SOEM/soem/*.h")
file(GLOB SOEM_OSAL_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/${OS}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/*.h")
file(GLOB SOEM_OSHW_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/SOEM/oshw/${OS}/*.h")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# SOEM is, by default, built with position-dependent code as a static library.
# To be able to include SOEM in a dynamic library (default ROS convention),
# it needs to be built with position-independent code.
# Thus, we add -fPIC here.
# This allows us to still include this in any shared libraries we create.
add_compile_options(-fPIC)
# removed any pre-defined `WIN32_LEAN_AND_MEAN`.
# Otherwise, many symbols will be reported as missing.
remove_definitions(-DWIN32_LEAN_AND_MEAN)
add_subdirectory(SOEM)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#install all Headers
install(
  FILES ${SOEM_COMMON_HEADERS} ${SOEM_OSAL_HEADERS} ${SOEM_OSHW_HEADERS}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
