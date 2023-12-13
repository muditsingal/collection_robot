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

add_executable(move_robot src/move_robot.cpp)
ament_target_dependencies(move_robot ${PACKAGE_DEPENDENCIES})
target_include_directories(move_robot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(object_detect src/object_detect.cpp)
ament_target_dependencies(object_detect ${PACKAGE_DEPENDENCIES})
target_include_directories(object_detect
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(plan_path src/plan_path.cpp)
ament_target_dependencies(plan_path ${PACKAGE_DEPENDENCIES})
target_include_directories(plan_path
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  move_robot
  object_detect
  plan_path
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
