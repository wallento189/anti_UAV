cmake_minimum_required(VERSION 3.8)
project(autoaim_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoaim_interfaces REQUIRED)

# Protocol codec is kept independent of ROS so it can be unit-tested directly.
add_library(autoaim_serial_protocol SHARED
  src/protocol.cpp
)
target_include_directories(autoaim_serial_protocol PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(autoaim_serial_protocol PUBLIC cxx_std_17)

add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp
  src/serial_node.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} autoaim_serial_protocol)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  autoaim_interfaces
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN SerialNode::SerialNode
  EXECUTABLE serial_node
)

install(TARGETS ${PROJECT_NAME} autoaim_serial_protocol
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
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_protocol autoaim_serial_protocol)
endif()

ament_package()
