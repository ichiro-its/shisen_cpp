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
find_package(keisan REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shisen_interfaces REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

add_library(${PROJECT_NAME} SHARED
  "src/camera/node/camera_node.cpp"
  "src/camera/provider/camera_config_provider.cpp"
  "src/camera/provider/image_provider.cpp"
  "src/utility/capture_setting.cpp"
  "src/utility/interface.cpp"
  "src/utility/mat_image.cpp"
  "src/node/shisen_cpp_node.cpp"
  "src/node/shisen_cpp_viewer_node.cpp"
  "src/viewer/consumer/image_consumer.cpp"
  "src/viewer/node/viewer_node.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  keisan OpenCV rclcpp shisen_interfaces)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(camera "src/shisen_cpp_camera_main.cpp")
target_include_directories(camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera ${PROJECT_NAME})

add_executable(viewer "src/shisen_cpp_viewer_main.cpp")
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
    "test/emptiable_test.cpp"
    "test/mat_image_test.cpp")

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(keisan OpenCV rclcpp shisen_interfaces)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
