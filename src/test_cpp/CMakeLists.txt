cmake_minimum_required(VERSION 3.5)
project(test_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_ros2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
set(SQLite3_ROOT "/usr/lib/x86_64-linux-gnu") 
find_package(SQLite3 REQUIRED)

add_executable(raw_image src/raw_image.cpp)
ament_target_dependencies(raw_image rclcpp sensor_msgs cv_bridge example_ros2_interfaces)
target_link_libraries(raw_image 
  ${OpenCV_LIBS} 
  ${SQLite3_LIBRARIES} 
  #SQLite::SQLite3 # 使用 SQLite3 的别名
)

add_executable(image src/image.cpp)
ament_target_dependencies(image rclcpp sensor_msgs cv_bridge example_ros2_interfaces)
target_link_libraries(image 
  ${OpenCV_LIBS} 
  ${SQLite3_LIBRARIES} 
#  SQLite::SQLite3
)

add_executable(process_image1 src/process_image1.cpp)
ament_target_dependencies(process_image1 rclcpp sensor_msgs cv_bridge example_ros2_interfaces)
target_link_libraries(process_image1 
  ${OpenCV_LIBS} 
  ${SQLite3_LIBRARIES} 
  #SQLite::SQLite3
)

add_executable(process_image2 src/process_image2.cpp)
ament_target_dependencies(process_image2 rclcpp sensor_msgs cv_bridge example_ros2_interfaces)
target_link_libraries(process_image2 
  ${OpenCV_LIBS} 
  ${SQLite3_LIBRARIES} 
  #SQLite::SQLite3
)

# 安装可执行文件
install(TARGETS
  raw_image
  image
  process_image1
  process_image2
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
