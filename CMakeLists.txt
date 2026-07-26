cmake_minimum_required(VERSION 3.0.2)
project(shinji VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

option(USE_UNORDERED_DENSE "Use ankerl::unordered_dense container" ON)
option(SMALL_GICP_STD_PTR "Use std::shared_ptr in small_gicp" OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  tf2_ros
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem container)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found, enabling parallelization")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "OpenMP not found.")
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES shinji
 CATKIN_DEPENDS
)

if(USE_UNORDERED_DENSE)
  add_definitions(-DUSE_UNORDERED_DENSE)
  message(STATUS "Using ankerl::unordered_dense container")
endif()

if(SMALL_GICP_STD_PTR)
  add_definitions(-DSMALL_GICP_STD_PTR)
  message(STATUS "Using std::shared_ptr in small_gicp")
endif()

set(TEASER_CACHE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/teaserpp")
set(TEASER_LIBRARIES
  ${TEASER_CACHE_DIR}/build/devel/lib/libpmc.so
  ${TEASER_CACHE_DIR}/build/devel/lib/libteaser_features.so
  ${TEASER_CACHE_DIR}/build/devel/lib/libteaser_io.so
  ${TEASER_CACHE_DIR}/build/devel/lib/libteaser_registration.so
)

add_library(shinji
  src/shinji/config_server.cpp
  src/shinji/globalmap_server.cpp
  src/shinji/matching_cost_evaluater.cpp
  src/shinji/shinji.cpp
  src/shinji/utility.cpp
)
target_include_directories(shinji PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  thirdparty
  thirdparty/teaserpp/teaser/include
)

add_library(shinji_nodelet apps/shinji_nodelet.cpp)
target_link_libraries(shinji_nodelet
  shinji
  ${catkin_LIBRARIES}
  ${TEASER_LIBRARIES}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)