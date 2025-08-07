cmake_minimum_required(VERSION 3.8)
project(graspnet_ros)

# Set C++ standard (for possible C++ code)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# List your service files here
set(SRV_FILES
  srv/UpdateInterestMap.srv
)

# Generate message/service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs std_msgs
)

# Export dependencies for consumers
ament_export_dependencies(rosidl_default_runtime)

# Install Python script files
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/graspnet_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/graspnet_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/graspnet-baseline
  DESTINATION share/${PROJECT_NAME}/scripts
)



# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare this as aament package
ament_package()
