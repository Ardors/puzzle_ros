cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(
  include
)
ament_export_include_directories(include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  sensor_msgs
  interfaces
)

add_executable(${PROJECT_NAME} src/vision.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_compile_features(vision PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS vision
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
