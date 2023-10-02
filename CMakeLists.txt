cmake_minimum_required(VERSION 3.10)
project(Bonxai)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fno-omit-frame-pointer -g")

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(PCL REQUIRED COMPONENTS common io)

find_package(benchmark QUIET)
find_package(octomap QUIET)
find_package(LZ4 QUIET)
find_package(Eigen3 QUIET)

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" ON)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()


####################################################
add_subdirectory(bonxai_core)
add_subdirectory(bonxai_map)

find_package(ros_environment QUIET)
if(NOT ros_environment_FOUND)
  message(STATUS "ROS environment not found, skipping ROS build")
  return()
endif()

set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

  project(bonxai_ros)

  find_package(ament_cmake REQUIRED)

  # Add colcon-specific configuration here
  message(STATUS "Building with colcon")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(bonxai_ros/include 3rdparty)

  ament_auto_add_library(bonxai_server SHARED
    bonxai_ros/src/bonxai_server.ros2.cpp
  )

  target_link_libraries(bonxai_server
    bonxai_map
    ${PCL_LIBRARIES}
  )

  rclcpp_components_register_node(bonxai_server
    PLUGIN "bonxai_server::BonxaiServer"
    EXECUTABLE bonxai_server_node
  )

  ament_auto_package(
    INSTALL_TO_SHARE
    bonxai_ros/launch
    bonxai_ros/rviz
    bonxai_ros/params
  )

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

  project(bonxai_ros)

  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    message_filters
    nodelet
    pcl_conversions
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
  )

  # Add catkin-specific configuration here
  message(STATUS "Building with catkin")

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if your package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
    INCLUDE_DIRS
      bonxai_ros/include
      3rdparty
    LIBRARIES bonxai_server
    CATKIN_DEPENDS
      message_filters
      pcl_conversions
      roscpp
      sensor_msgs
      std_srvs
      tf2
      tf2_eigen
      tf2_geometry_msgs
      tf2_ros
      visualization_msgs
  )

  ###########
  ## Build ##
  ###########

  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(
    bonxai_ros/include
    3rdparty
    ${catkin_INCLUDE_DIRS}
  )

  ## Declare a C++ library
  add_library(bonxai_server
    bonxai_ros/src/bonxai_server.ros1.cpp
  )

  ## Add cmake target dependencies of the executable
  ## same as for the library above
  add_dependencies(bonxai_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  ## Specify libraries to link a library or executable target against
  target_link_libraries(bonxai_server
    bonxai_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  #############
  ## Install ##
  #############

  ## Mark executables and/or libraries for installation
  install(TARGETS bonxai_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  ## Mark cpp header files for installation
  install(DIRECTORY bonxai_ros/include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*ros2*" EXCLUDE
  )

  ## Mark other files for installation (e.g. launch and bag files, etc.)
  install(FILES
    nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  install(DIRECTORY
    bonxai_ros/launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*ros2*" EXCLUDE
  )
  install(DIRECTORY
    bonxai_ros/params
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*ros2*" EXCLUDE
  )
endif()
