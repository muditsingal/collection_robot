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
  apriltag_ros
  tf2_ros
  apriltag_msgs
  apriltag
  image_transport
  cv_bridge
  rclcpp
  rclpy
)

# find dependencies
foreach(dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

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
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${PACKAGE_DEPENDENCIES})
  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
