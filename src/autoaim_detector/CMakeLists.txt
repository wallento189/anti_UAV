cmake_minimum_required(VERSION 3.8)
project(autoaim_detector)

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
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(autoaim_interfaces REQUIRED)

# Pure ON/OFF backend switches. ON means the SDK must be present, otherwise
# configuration fails fast. Defaults match the x86 development machine
# (OpenVINO available, Horizon BPU absent); on an RDK board configure with
# -DAUTOAIM_WITH_HORIZON=ON -DAUTOAIM_WITH_OPENVINO=OFF.
option(AUTOAIM_WITH_OPENVINO "Enable OpenVINO backend for ONNX models (GPU/CPU)" ON)
option(AUTOAIM_WITH_HORIZON "Enable Horizon BPU backend for .bin models" OFF)

if(NOT AUTOAIM_WITH_OPENVINO AND NOT AUTOAIM_WITH_HORIZON)
  message(FATAL_ERROR "At least one inference backend must be enabled")
endif()

if(AUTOAIM_WITH_OPENVINO)
  find_package(OpenVINO QUIET COMPONENTS runtime)
  if(NOT OpenVINO_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(OPENVINO QUIET IMPORTED_TARGET openvino)
    endif()
  endif()
  if(NOT TARGET openvino::runtime AND NOT TARGET PkgConfig::OPENVINO)
    message(FATAL_ERROR
      "AUTOAIM_WITH_OPENVINO=ON but OpenVINO was not found. Install OpenVINO "
      "or configure with -DAUTOAIM_WITH_OPENVINO=OFF")
  endif()
endif()

if(AUTOAIM_WITH_HORIZON)
  find_path(HORIZON_INCLUDE_DIR NAMES dnn/hb_dnn.h
    HINTS ${HORIZON_SDK_ROOT} $ENV{HORIZON_SDK_ROOT}
    PATHS /usr/include/hobot /usr/local/include/hobot)
  find_library(HORIZON_DNN_LIB NAMES dnn
    HINTS ${HORIZON_SDK_ROOT} $ENV{HORIZON_SDK_ROOT}
    PATH_SUFFIXES lib/hobot lib/hbbpu lib)
  find_library(HORIZON_HBRT_LIB NAMES hbrt
    HINTS ${HORIZON_SDK_ROOT} $ENV{HORIZON_SDK_ROOT}
    PATH_SUFFIXES lib/hobot lib/hbbpu lib)
  find_library(HORIZON_HBIPC_LIB NAMES hbipc
    HINTS ${HORIZON_SDK_ROOT} $ENV{HORIZON_SDK_ROOT}
    PATH_SUFFIXES lib/hobot lib/hbbpu lib)
  if(NOT HORIZON_INCLUDE_DIR OR NOT HORIZON_DNN_LIB OR NOT HORIZON_HBRT_LIB)
    message(FATAL_ERROR
      "AUTOAIM_WITH_HORIZON=ON but the Horizon hbDNN SDK was not found. "
      "Set -DHORIZON_SDK_ROOT=<sdk root>")
  endif()
endif()

set(DETECTOR_SOURCES
  src/detector_node.cpp
  src/infer_engine.cpp
  src/preprocess.cpp
  src/postprocess.cpp
)

if(AUTOAIM_WITH_OPENVINO)
  list(APPEND DETECTOR_SOURCES src/backend_openvino.cpp)
endif()
if(AUTOAIM_WITH_HORIZON)
  list(APPEND DETECTOR_SOURCES src/backend_horizon.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${DETECTOR_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  $<$<BOOL:${AUTOAIM_WITH_OPENVINO}>:AUTOAIM_WITH_OPENVINO=1>
  $<$<BOOL:${AUTOAIM_WITH_HORIZON}>:AUTOAIM_WITH_HORIZON=1>)

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

if(AUTOAIM_WITH_OPENVINO)
  if(TARGET openvino::runtime)
    target_link_libraries(${PROJECT_NAME} openvino::runtime)
  else()
    target_link_libraries(${PROJECT_NAME} PkgConfig::OPENVINO)
  endif()
endif()

if(AUTOAIM_WITH_HORIZON)
  target_include_directories(${PROJECT_NAME} PRIVATE ${HORIZON_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${HORIZON_DNN_LIB} ${HORIZON_HBRT_LIB})
  if(HORIZON_HBIPC_LIB)
    target_link_libraries(${PROJECT_NAME} ${HORIZON_HBIPC_LIB})
  endif()
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN DetectorNode::DetectorNode
  EXECUTABLE detector_node
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

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/model)
  install(DIRECTORY model
    DESTINATION share/${PROJECT_NAME})
endif()

message(STATUS "autoaim_detector backends: OpenVINO=${AUTOAIM_WITH_OPENVINO}, Horizon=${AUTOAIM_WITH_HORIZON}")

ament_package()
