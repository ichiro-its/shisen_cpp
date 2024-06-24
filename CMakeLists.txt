cmake_minimum_required(VERSION 3.5)

project(shisen_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(keisan REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shisen_interfaces REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

# Find gRPC installation
# Looks for gRPCConfig.cmake file installed by gRPC's cmake installation.
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/camera/node/camera_node.cpp"
  "src/${PROJECT_NAME}/camera/provider/camera_config_provider.cpp"
  "src/${PROJECT_NAME}/camera/provider/image_provider.cpp"
  "src/${PROJECT_NAME}/config/grpc/config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_get_capture_setting.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_get_image.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_save_capture_setting.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_set_capture_setting.cpp"
  "src/${PROJECT_NAME}/utility/capture_setting.cpp"
  "src/${PROJECT_NAME}/utility/interface.cpp"
  "src/${PROJECT_NAME}/node/shisen_cpp_node.cpp"
  "src/${PROJECT_NAME}/viewer/consumer/image_consumer.cpp"
  "src/${PROJECT_NAME}/viewer/node/viewer_node.cpp"
)

add_library(${PROJECT_NAME}_exported SHARED
  "src/${PROJECT_NAME}/camera/node/camera_node.cpp"
  "src/${PROJECT_NAME}/camera/provider/camera_config_provider.cpp"
  "src/${PROJECT_NAME}/camera/provider/image_provider.cpp"
  "src/${PROJECT_NAME}/utility/capture_setting.cpp"
  "src/${PROJECT_NAME}/utility/interface.cpp"
  "src/${PROJECT_NAME}/viewer/consumer/image_consumer.cpp"
  "src/${PROJECT_NAME}/viewer/node/viewer_node.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(${PROJECT_NAME}_exported PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  keisan
  OpenCV
  rclcpp
  shisen_interfaces
  jitsuyo
  sensor_msgs
  cv_bridge
  std_msgs
  gRPC)

ament_target_dependencies(${PROJECT_NAME}_exported
  keisan
  OpenCV
  rclcpp
  shisen_interfaces
  jitsuyo
  sensor_msgs
  cv_bridge
  std_msgs)

target_link_libraries(${PROJECT_NAME}
  gRPC::grpc++_reflection
  gRPC::grpc++
)

install(DIRECTORY "include" DESTINATION ".")

install(DIRECTORY
  launch
  DESTINATION "share/${PROJECT_NAME}")

install(TARGETS ${PROJECT_NAME}
EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "include")

install(TARGETS ${PROJECT_NAME}_exported
EXPORT export_${PROJECT_NAME}_exported
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "include")

target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

add_executable(camera "src/shisen_cpp_main.cpp")
target_include_directories(camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera ${PROJECT_NAME})

add_executable(viewer "src/viewer_main.cpp")
target_include_directories(viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(viewer ${PROJECT_NAME})

install(TARGETS camera viewer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    "test/capture_setting_test.cpp"
    "test/compile_test.cpp"
    "test/emptiable_test.cpp")

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(keisan OpenCV rclcpp shisen_interfaces sensor_msgs cv_bridge std_msgs)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME}_exported)

ament_package()
