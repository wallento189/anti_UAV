cmake_minimum_required(VERSION 3.8)
project(autoaim_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(autoaim_interfaces REQUIRED)

# Attitude pairing is pure logic, kept ROS-free so it can be unit tested.
add_library(autoaim_tracker_core SHARED
  src/attitude_buffer.cpp
)
target_include_directories(autoaim_tracker_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(autoaim_tracker_core PUBLIC cxx_std_17)

add_library(${PROJECT_NAME} SHARED
  src/tracker.cpp
  src/tracker_node.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  autoaim_interfaces
)
target_link_libraries(${PROJECT_NAME} autoaim_tracker_core ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN TrackerNode::TrackerNode
  EXECUTABLE tracker_node
)

install(TARGETS ${PROJECT_NAME} autoaim_tracker_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_attitude_buffer test/test_attitude_buffer.cpp)
  target_link_libraries(test_attitude_buffer autoaim_tracker_core)
endif()

ament_package()
