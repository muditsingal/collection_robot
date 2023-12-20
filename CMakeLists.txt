cmake_minimum_required(VERSION 3.8)
project(collection_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
set(PACKAGE_DEPENDENCIES
  ament_cmake
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_msgs
  nav_msgs
  tf2_ros
  image_transport
  cv_bridge
  rclcpp
  rclpy
  tf2
  OpenCV
  gazebo_msgs
  rosidl_default_generators
)

# find dependencies
foreach(dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(
  include/${PROJECT_NAME}
)

add_executable(collector_node src/main.cpp src/object_detector.cpp src/handle_trash.cpp)
ament_target_dependencies(collector_node ${PACKAGE_DEPENDENCIES})


install(TARGETS
  collector_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Add gtest here
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # un-comment the following lines skip linters (TRUE = skip that test)
  # set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  # ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${PACKAGE_DEPENDENCIES})
  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
