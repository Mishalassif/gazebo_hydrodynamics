cmake_minimum_required(VERSION 2.8.3)
project(gazebo_hydrodynamics)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    gazebo_ros_control
    roscpp
    )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED
             Eigen3 REQUIRED)

include_directories( ./src ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES auv_simulator
    #  CATKIN_DEPENDS auv_msgs gazebo_ros gazebo_ros_control roscpp
    DEPENDS gazebo_ros  # system_lib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set( SRC_DIR src)

set( plugins
    virtual_mass_plugin
    viscous_damping_plugin )

foreach (src ${plugins})
    add_library(${src} SHARED ${SRC_DIR}/${src}.cpp)
    target_link_libraries(${src} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} 
        ${Boost_LIBRARIES} ${SDFormat_LIBRARIES})
endforeach (src ${plugins})

