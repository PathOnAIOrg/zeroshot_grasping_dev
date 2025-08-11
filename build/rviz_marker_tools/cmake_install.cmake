# Install script for directory: /home/pathonai/ros2_ws/src/opensource_dev/mycobot_ros2/moveit_task_constructor/rviz_marker_tools

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pathonai/ros2_ws/src/opensource_dev/install/rviz_marker_tools")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/pathonai/ros2_ws/src/opensource_dev/build/rviz_marker_tools/ament_cmake_symlink_install/ament_cmake_symlink_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_marker_tools.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_marker_tools.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_marker_tools.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/pathonai/ros2_ws/src/opensource_dev/build/rviz_marker_tools/librviz_marker_tools.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_marker_tools.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_marker_tools.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_marker_tools.so"
         OLD_RPATH "/home/pathonai/ros2_jazzy/install/visualization_msgs/lib:/home/pathonai/ros2_jazzy/install/tf2_ros/lib:/home/pathonai/ros2_jazzy/install/tf2/lib:/home/pathonai/ros2_jazzy/install/message_filters/lib:/home/pathonai/ros2_jazzy/install/rclcpp_action/lib:/home/pathonai/ros2_jazzy/install/rclcpp/lib:/home/pathonai/ros2_jazzy/install/libstatistics_collector/lib:/home/pathonai/ros2_jazzy/install/rosgraph_msgs/lib:/home/pathonai/ros2_jazzy/install/statistics_msgs/lib:/home/pathonai/ros2_jazzy/install/rcl_action/lib:/home/pathonai/ros2_jazzy/install/rcl/lib:/home/pathonai/ros2_jazzy/install/rcl_interfaces/lib:/home/pathonai/ros2_jazzy/install/rcl_yaml_param_parser/lib:/home/pathonai/ros2_jazzy/install/tracetools/lib:/home/pathonai/ros2_jazzy/install/rcl_logging_interface/lib:/home/pathonai/ros2_jazzy/install/rmw_implementation/lib:/home/pathonai/ros2_jazzy/install/ament_index_cpp/lib:/home/pathonai/ros2_jazzy/install/type_description_interfaces/lib:/home/pathonai/ros2_jazzy/install/tf2_msgs/lib:/home/pathonai/ros2_jazzy/install/action_msgs/lib:/home/pathonai/ros2_jazzy/install/unique_identifier_msgs/lib:/home/pathonai/ros2_jazzy/install/sensor_msgs/lib:/home/pathonai/ros2_jazzy/install/geometry_msgs/lib:/home/pathonai/ros2_jazzy/install/std_msgs/lib:/home/pathonai/ros2_jazzy/install/service_msgs/lib:/home/pathonai/ros2_jazzy/install/builtin_interfaces/lib:/home/pathonai/ros2_jazzy/install/rosidl_typesupport_fastrtps_c/lib:/home/pathonai/ros2_jazzy/install/rosidl_typesupport_introspection_cpp/lib:/home/pathonai/ros2_jazzy/install/rosidl_typesupport_introspection_c/lib:/home/pathonai/ros2_jazzy/install/rosidl_typesupport_fastrtps_cpp/lib:/home/pathonai/ros2_jazzy/install/fastcdr/lib:/home/pathonai/ros2_jazzy/install/rmw/lib:/home/pathonai/ros2_jazzy/install/rosidl_dynamic_typesupport/lib:/home/pathonai/ros2_jazzy/install/rosidl_typesupport_cpp/lib:/home/pathonai/ros2_jazzy/install/rosidl_typesupport_c/lib:/home/pathonai/ros2_jazzy/install/rosidl_runtime_c/lib:/home/pathonai/ros2_jazzy/install/rcpputils/lib:/home/pathonai/ros2_jazzy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_marker_tools.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_marker_tools/cmake/rviz_marker_toolsTargetsExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_marker_tools/cmake/rviz_marker_toolsTargetsExport.cmake"
         "/home/pathonai/ros2_ws/src/opensource_dev/build/rviz_marker_tools/CMakeFiles/Export/cd5f87cf1320cec0899e0f87948c90f5/rviz_marker_toolsTargetsExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_marker_tools/cmake/rviz_marker_toolsTargetsExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_marker_tools/cmake/rviz_marker_toolsTargetsExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_marker_tools/cmake" TYPE FILE FILES "/home/pathonai/ros2_ws/src/opensource_dev/build/rviz_marker_tools/CMakeFiles/Export/cd5f87cf1320cec0899e0f87948c90f5/rviz_marker_toolsTargetsExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_marker_tools/cmake" TYPE FILE FILES "/home/pathonai/ros2_ws/src/opensource_dev/build/rviz_marker_tools/CMakeFiles/Export/cd5f87cf1320cec0899e0f87948c90f5/rviz_marker_toolsTargetsExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/pathonai/ros2_ws/src/opensource_dev/build/rviz_marker_tools/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
