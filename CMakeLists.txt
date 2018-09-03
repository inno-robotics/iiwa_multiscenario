cmake_minimum_required(VERSION 2.8.3)
project(iiwa_multiscenario)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  geometry_msgs
  roscpp
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

set(MAIN_SOURCES
    # basic
    src/kinematics/homogenous.cpp
    # ros communication + kuka specific
    src/kuka/ros_iiwa_link.cpp
    src/kuka/iiwa_kinematics.cpp
    # finite state machine
    src/fsm/collision_detection.cpp
    src/fsm/collision_reactions.cpp
    src/fsm/main_task.cpp
    src/fsm/task_dispatcher.cpp
)

# check basic reactions
add_executable(basic_reactions src/basic_reactions.cpp ${MAIN_SOURCES})
add_dependencies(basic_reactions iiwa_msgs_generate_messages_cpp)
target_link_libraries(basic_reactions ${catkin_LIBRARIES})
