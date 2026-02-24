cmake_minimum_required(VERSION 3.5)
project(slamkit_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(SLAMKITSDK_PATH "./sdk/slamkit/")
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

FILE(GLOB SLAMKITSDK_SRC 
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
  ${PROJECT_SOURCE_DIR}/include
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
  find_package(ament_cmake_cpplint REQUIRED)
  set(ament_cmake_cppcheck_FOUND REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  set(AMENT_LINT_AUTO_EXCLUDE_DIRS "${SLAMKITSDK_PATH}")
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
    ${SLAMKITSDK_SRC}
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()