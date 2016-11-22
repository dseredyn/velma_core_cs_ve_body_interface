cmake_minimum_required(VERSION 2.8.3)
project(velma_low_level_interface)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  diagnostic_msgs
  velma_low_level_interface_msgs
  barrett_hand_controller_msgs
  eigen_conversions
  kdl_conversions cmake_modules
  common_interfaces
  kuka_lwr_fri
  )

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package( rtt-typekit )

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

orocos_component(velma_low_level_components
    src/velma_low_level_interface.cpp
    src/velma_lli_status_ports.cpp
    src/velma_lli_command_ports.cpp
    src/velma_lli_real_effector_status.cpp
    src/velma_lli_real_effector_commands.cpp
)

#                    src/velma_lli_lo_test.cpp src/velma_lli_hi_test.cpp src/velma_lli_test_generator.cpp
#                    src/velma_lli_test_error.cpp)

target_link_libraries(velma_low_level_components
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES})


add_library(velma_low_level_interface src/velma_lli_status_ports.cpp src/velma_lli_command_ports.cpp)

target_link_libraries(velma_low_level_interface ${catkin_LIBRARIES})
#add_dependencies(velma_low_level_interface ${PROJECT_NAME}_gencpp)

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  LIBRARIES velma_low_level_interface
  CATKIN_DEPENDS rtt_ros geometry_msgs std_msgs message_runtime rtt_roscomm common_interfaces
)

#orocos_install_headers(
#  DIRECTORY include/${PROJECT_NAME}/
#)
