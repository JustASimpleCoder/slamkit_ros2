cmake_minimum_required(VERSION 3.22.1)
project(slamkit_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(SLAMKITSDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk/slamkit/")
set(SLAMKITSDK_ROS_PATH "./sdk")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -w
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

file(GLOB SLAMKITSDK_SRC
  "${SLAMKITSDK_PATH}/src/arch/linux/*.cpp"
  "${SLAMKITSDK_PATH}/src/hal/*.cpp"
  "${SLAMKITSDK_PATH}/src/*.cpp"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})

include_directories(
  ${SLAMKITSDK_PATH}/include
  ${SLAMKITSDK_PATH}/src
  ${SLAMKITSDK_ROS_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(slamkitNode src/node.cpp ${SLAMKITSDK_SRC})
ament_target_dependencies(slamkitNode
  rclcpp
  std_srvs
  sensor_msgs
)
target_link_libraries(slamkitNode  ${LIBUSB_LIBRARIES})

add_executable(slamkitNodeClient src/client.cpp)
target_link_libraries(slamkitNodeClient ${LIBUSB_LIBRARIES})
ament_target_dependencies(slamkitNodeClient
  rclcpp
  std_srvs
  sensor_msgs
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS slamkitNode slamkitNodeClient
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  file(GLOB_RECURSE EXCLUDE_FILES
    "${SLAMKITSDK_PATH}/src/arch/linux/*.cpp"
    "${SLAMKITSDK_PATH}/src/arch/linux/*.hpp"
    "${SLAMKITSDK_PATH}/src/arch/linux/*.h"
    "${SLAMKITSDK_PATH}/src/arch/macOS/*.cpp"
    "${SLAMKITSDK_PATH}/src/arch/macOS/*.h"
    "${SLAMKITSDK_PATH}/src/arch/macOS/*.hpp"
    "${SLAMKITSDK_PATH}/src/arch/win32/*.cpp"
    "${SLAMKITSDK_PATH}/src/arch/win32/*.hpp"
    "${SLAMKITSDK_PATH}/src/arch/win32/*.h"
    "${SLAMKITSDK_PATH}/src/hal/*.cpp"
    "${SLAMKITSDK_PATH}/src/*.cpp"
    "${SLAMKITSDK_PATH}/src/*.h"
    "${SLAMKITSDK_PATH}/*.cpp"
    "${SLAMKITSDK_PATH}/include/*.h"
    "${SLAMKITSDK_PATH}LICENSE.txt"
    "${SLAMKITSDK_PATH}README.md"
  )

  set(AMENT_LINT_AUTO_EXCLUDE_DIRS "${SLAMKITSDK_ROS_PATH}")
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
    ${EXCLUDE_FILES}
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()