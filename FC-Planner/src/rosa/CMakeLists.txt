cmake_minimum_required(VERSION 2.8.3)
project(rosa)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    traj_utils
    roscpp
    std_msgs
    quadrotor_msgs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosa
  CATKIN_DEPENDS traj_utils
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS})

add_library( rosa
    src/rosa_main.cpp
    src/Extra_Del.cpp
    )

target_link_libraries( rosa
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  

add_executable(rosa_exec
    src/rosa_exec.cpp
    src/rosa_main.cpp
    src/Extra_Del.cpp
    )

target_link_libraries( rosa_exec
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )