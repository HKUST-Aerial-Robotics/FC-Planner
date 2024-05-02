cmake_minimum_required(VERSION 2.8.3)
project(viewpoint_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    rosa
    traj_utils
    plan_env
    active_perception
    quadrotor_msgs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES viewpoint_manager skeleton_viewpoint
  CATKIN_DEPENDS traj_utils plan_env
  DEPENDS system_lib
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

add_library( viewpoint_manager
    src/viewpoint_manager.cpp
)

target_link_libraries( viewpoint_manager
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  

add_library( skeleton_viewpoint
    src/skeleton_viewpoint.cpp
)

target_link_libraries( viewpoint_manager
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  

add_executable(viewpoint_manager_node
    src/viewpoint_manager_node.cpp
)

target_link_libraries(viewpoint_manager_node
    viewpoint_manager
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

add_executable(skeleton_viewpoint_node
    src/skeleton_viewpoint_node.cpp
    src/skeleton_viewpoint.cpp
    src/viewpoint_manager.cpp
)

target_link_libraries(skeleton_viewpoint_node
    skeleton_viewpoint
    viewpoint_manager
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )