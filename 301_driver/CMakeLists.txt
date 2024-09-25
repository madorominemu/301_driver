cmake_minimum_required(VERSION 2.8.3)
project(bulldog_driver)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    controller_manager
    hardware_interface
    diagnostic_updater
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_updater hardware_interface roscpp sensor_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(bulldog_node src/bulldog_node.cpp src/bulldog_diagnostics.cpp src/bulldog_hw.cpp src/motor_controller.cpp  src/uart_display.cpp)
target_link_libraries(bulldog_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
    TARGETS
        bulldog_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
