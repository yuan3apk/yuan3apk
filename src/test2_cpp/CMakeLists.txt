cmake_minimum_required(VERSION 3.8)
project(test2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
add_executable(raw_image2 src/raw_image2.cpp)
ament_target_dependencies(raw_image2 rclcpp sensor_msgs cv_bridge)
target_link_libraries(raw_image2 
  ${OpenCV_LIBS} 
)
add_executable(process src/process.cpp)
ament_target_dependencies(process rclcpp sensor_msgs cv_bridge)
target_link_libraries(process 
  ${OpenCV_LIBS} 
)
install(TARGETS
  raw_image2
  process
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
