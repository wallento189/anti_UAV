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
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN TrackerNode::TrackerNode
  EXECUTABLE tracker_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
