cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_image_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
    camera_model
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(pointcloud_image_fusion
    src/pointcloud_image_fusion_node.cpp
    src/parameters.cpp
    src/pointcloud_image_fusion.cpp
    )

target_link_libraries(pointcloud_image_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
