cmake_minimum_required(VERSION 3.8)
project(drone_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(drone_follow src/1_drone_follow.cpp)
ament_target_dependencies(drone_follow rclcpp geometry_msgs sensor_msgs cv_bridge)
target_link_libraries(drone_follow ${OpenCV_LIBS})

install(TARGETS
drone_follow
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
