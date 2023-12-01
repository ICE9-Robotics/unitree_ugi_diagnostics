cmake_minimum_required(VERSION 3.1)
project(unitree_diagnostics VERSION 0.1)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    sensor_msgs
    geometry_msgs
    nmea_msgs
    unitree_legged_msgs
    unitree_diagnostics_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/diagnostics.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(diagnostics_node src/diagnostics_node.cpp)
target_link_libraries(diagnostics_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
