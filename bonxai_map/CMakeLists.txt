#######################
# general configuration
#######################
cmake_minimum_required(VERSION 3.10)
project(bonxai_map VERSION 0.2.0 LANGUAGES CXX)

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

find_package(PCL QUIET REQUIRED COMPONENTS common io)

find_package(benchmark QUIET)
find_package(Eigen3 QUIET REQUIRED)

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" ON)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()

include_directories(include)

add_library(${PROJECT_NAME} STATIC
    src/pcl_utils.cpp
    src/probabilistic_map.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${PCL_DEFINITIONS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
    bonxai_core
    Eigen3::Eigen)

####################################################

if(benchmark_FOUND AND octomap_FOUND)
    add_subdirectory(benchmark)
endif()
