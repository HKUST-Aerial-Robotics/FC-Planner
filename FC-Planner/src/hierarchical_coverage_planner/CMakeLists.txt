cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD)
project(hierarchical_coverage_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    rosa
    traj_utils
    roscpp
    std_msgs
    plan_env
    active_perception
    path_searching
    quadrotor_msgs
    viewpoint_manager
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hcplanner
  CATKIN_DEPENDS rosa traj_utils plan_env
  DEPENDS OpenCV
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${CGAL_LIBRARIES})

add_library( hcplanner
    src/hcplanner.cpp
    src/hcsolver.cpp
    src/hctraj.cpp
    src/hcopp.cpp
    )

target_link_libraries( hcplanner
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OMPL_LIBRARIES}
    )  

add_executable(hcplanner_exec
    src/hcplanner.cpp
    src/hcsolver.cpp
    src/hctraj.cpp
    src/hcopp.cpp
    src/hcplanner_exec.cpp
    )

target_link_libraries( hcplanner_exec
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OMPL_LIBRARIES}
    )