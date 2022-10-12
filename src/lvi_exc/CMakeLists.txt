cmake_minimum_required(VERSION 3.2)
project(lvi_exc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")
set(CMAKE_BUILD_TYPE "RELEASE")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2 -mavx")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosbag
  geometry_msgs
  nav_msgs
  velodyne_msgs
  ndt_omp
  camera_model
  tf
  pcl_ros
  cv_bridge
  orb_slam2
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time iostreams)
# set(PANGOLIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-pangolin")
# find_package(Pangolin REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Kontiki)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
      ${Boost_LIBRARIES}
      ${catkin_LIBRARIES}
      # ${Pangolin_LIBRARIES}
      ${OpenCV_LIBS}
      glog
)

catkin_package(
   INCLUDE_DIRS include
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs velodyne_msgs
#  DEPENDS system_lib
)

include_directories( 
        "/usr/local/include"
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Kontiki/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/eth_atag
)

# file(GLOB_RECURSE KONKITI_HDRS "thirdparty/Kontiki/include/*.h")
file(GLOB_RECURSE KONKITI_SRCS "thirdparty/Kontiki/include/kontiki/sfm/*.cc")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "src/*.cpp")
file(GLOB_RECURSE APRIL_TAG_SRCS_CC "src/eth_atag/*.cc")

add_library(lvi_exc_lib ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS} ${APRIL_TAG_SRCS_CC} ${KONKITI_SRCS})

target_link_libraries(lvi_exc_lib ${thirdparty_libraries})


add_executable(write_orb_slam_results test/write_orb_slam_results.cpp)
target_link_libraries(write_orb_slam_results lvi_exc_lib ${thirdparty_libraries})

add_executable(lvi_initialize_surfel_orb test/lvi_initialize_surfel_orb.cpp)
target_link_libraries(lvi_initialize_surfel_orb lvi_exc_lib ${thirdparty_libraries})

